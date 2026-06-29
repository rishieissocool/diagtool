"""
engine.py — lifecycle + orchestration shared by the CLI and the GUI.

The Engine:
  * loads the robot map from TeamControl's ipconfig.yaml (same IPs/ports the
    real program uses) and DiagTool's own diag_settings.yaml thresholds,
  * owns the live sources (vision, telemetry) and the Commander,
  * runs individual diagnostics or a full sweep, gathering results,
  * builds the calibration doc + report and writes them to an output folder.

Diagnostics are blocking (they poll/sleep). The CLI runs them directly; the
GUI runs Engine methods on a worker thread. `log`, `progress` and `stop`
callbacks let either front-end stream output and abort.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path

import yaml

from . import bridge, safety
from .sources import VisionSource, TelemetrySource
from .commander import Commander
from .diagnostics import DiagContext, RobotRef, DEFAULTS, BY_NAME, ALL_DIAGNOSTICS
from . import calibrator, report


DIAGTOOL_DIR = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = DIAGTOOL_DIR / "output"
SETTINGS_FILE = DIAGTOOL_DIR / "diag_settings.yaml"


@dataclass
class RobotInfo:
    is_yellow: bool
    robot_id: int          # shellID — used for both command and vision lookup
    ip: str
    port: int
    key: str               # ipconfig letter (A..F)
    grsim_id: int | None = None  # grSimID from ipconfig (for the grSim packet)
    mode: str = "real"     # "real" -> RobotCommand to ip:port; "grsim" -> grSim

    @property
    def label(self) -> str:
        return f"{'Y' if self.is_yellow else 'B'}{self.robot_id}"

    @property
    def is_real(self) -> bool:
        return not (self.ip.startswith("127.") or self.ip == "localhost")

    def ref(self) -> RobotRef:
        return RobotRef(self.is_yellow, self.robot_id, self.ip, self.port)


def default_mode_for(ip: str) -> str:
    """A loopback robot is almost certainly running in grSim, so default it to the
    grSim command path; a real LAN address defaults to the RobotFramework path."""
    return "grsim" if (ip.startswith("127.") or ip == "localhost") else "real"


def resolve_field_half(vision_size, setting_len, setting_wid, config_half):
    """Most-conservative real field half-extents -> (half_len, half_wid, source).

    Prefers real dimensions (SSL-Vision geometry, or an explicit override in
    diag_settings.yaml) over TeamControl's field_config constants, which can be
    wrong for a given arena. When several real sources are present, the SMALLER
    on each axis wins, so the arena never exceeds the true field.

      vision_size  : (length_mm, width_mm) from SSL-Vision, or None
      setting_len  : explicit field_length_mm, or None
      setting_wid  : explicit field_width_mm, or None
      config_half  : (half_len, half_wid) fallback from field_config
    """
    lens, wids, srcs = [], [], []
    if vision_size and vision_size[0] > 0 and vision_size[1] > 0:
        lens.append(vision_size[0] / 2.0)
        wids.append(vision_size[1] / 2.0)
        srcs.append("vision")
    if setting_len and setting_wid and float(setting_len) > 0 and float(setting_wid) > 0:
        lens.append(float(setting_len) / 2.0)
        wids.append(float(setting_wid) / 2.0)
        srcs.append("settings")
    if lens:
        return min(lens), min(wids), "+".join(srcs)
    return float(config_half[0]), float(config_half[1]), "field_config"


def ip_conflicts(robots) -> dict:
    """Real robots that share an IP -> {ip: [labels]}.

    Two robots on one address is a config bug: commands for one label are
    delivered to whatever single robot actually answers at that IP (or to
    nothing), which looks exactly like "no motion / no telemetry". Loopback
    (127.x) is ignored — sharing it is normal for sim entries.
    """
    by_ip: dict[str, list[str]] = {}
    for r in robots:
        if getattr(r, "is_real", False):
            by_ip.setdefault(r.ip, []).append(r.label)
    return {ip: sorted(set(lbls)) for ip, lbls in by_ip.items() if len(set(lbls)) > 1}


def load_settings() -> dict:
    s = dict(DEFAULTS)
    try:
        if SETTINGS_FILE.is_file():
            data = yaml.safe_load(SETTINGS_FILE.read_text(encoding="utf-8")) or {}
            for k, v in data.items():
                if k in s and v is not None:
                    s[k] = v
    except Exception:
        pass
    return s


class Engine:
    def __init__(self, settings: dict | None = None):
        self.config = bridge.get_config()
        self.settings = settings or load_settings()

        self.vision = VisionSource(port=int(self.config.vision[1]))
        self.telemetry = TelemetrySource(self.config)
        self.commander: Commander | None = None
        self._started = False

        # grSim command address (ip, port) from ipconfig — where grSim-mode
        # robots are driven.
        self.grsim_addr = self._read_grsim_addr()

        # The robot inventory is owned (not rebuilt from config on every call) so
        # IP/port/target edits made in the UI persist and the same RobotInfo
        # objects stay valid across the app.
        self._inventory: list[RobotInfo] = self._build_inventory()

        # Field size is resolved (vision geometry / settings / config) into the
        # arena limits; refreshed live once vision sends a geometry packet.
        self._field_source = None
        self.lim = self._build_limits()

    # -- robot inventory --
    def _read_grsim_addr(self) -> tuple[str, int] | None:
        addr = getattr(self.config, "grSim_addr", None)
        try:
            if addr:
                return (str(addr[0]), int(addr[1]))
        except Exception:
            pass
        return None

    def _build_inventory(self) -> list[RobotInfo]:
        out: list[RobotInfo] = []
        for is_yellow, team in ((True, getattr(self.config, "yellow", {})),
                                (False, getattr(self.config, "blue", {}))):
            for key, d in (team or {}).items():
                sid = d.get("shellID")
                if sid is None:
                    continue
                ip = str(d.get("ip", "127.0.0.1"))
                gid = d.get("grSimID")
                out.append(RobotInfo(
                    bool(is_yellow), int(sid), ip,
                    int(d.get("port", 50514)), str(key),
                    grsim_id=(int(gid) if gid is not None else None),
                    mode=default_mode_for(ip)))
        return out

    def robots(self, real_only: bool = False) -> list[RobotInfo]:
        if real_only:
            return [r for r in self._inventory if r.is_real]
        return list(self._inventory)

    def find_robot(self, label: str) -> RobotInfo | None:
        for r in self._inventory:
            if r.label.lower() == label.lower():
                return r
        return None

    # -- live reconfiguration --
    def set_robot_target(self, label: str, ip: str | None = None,
                         port: int | None = None, mode: str | None = None) -> RobotInfo | None:
        """Change a robot's IP / port / target (real|grsim) and apply it live.

        Re-registers the commander target so the very next command stream uses
        the new address/protocol. Returns the updated RobotInfo (the same object
        the rest of the app holds), or None if the label is unknown.
        """
        r = self.find_robot(label)
        if r is None:
            return None
        if ip is not None:
            r.ip = str(ip).strip()
        if port is not None:
            r.port = int(port)
        if mode is not None:
            r.mode = "grsim" if str(mode).lower().startswith("g") else "real"
        if self.commander is not None:
            self.commander.register(r.is_yellow, r.robot_id, r.ip, r.port,
                                    mode=r.mode, grsim_id=r.grsim_id)
        return r

    def reload_config(self) -> dict:
        """Re-read ipconfig.yaml from disk and apply it live.

        Updates each known robot's ip/port/grsim_id in place (so labels stay
        stable for the UI) and re-registers commander targets. Returns a summary
        of what changed.
        """
        self.config = bridge.get_config()
        self.grsim_addr = self._read_grsim_addr()
        fresh = {r.label: r for r in self._build_inventory()}
        changed, added, removed = [], [], []
        have = {r.label for r in self._inventory}
        for label, nr in fresh.items():
            cur = self.find_robot(label)
            if cur is None:
                self._inventory.append(nr)
                added.append(label)
            else:
                if (cur.ip, cur.port) != (nr.ip, nr.port):
                    changed.append(label)
                cur.ip, cur.port, cur.grsim_id = nr.ip, nr.port, nr.grsim_id
                cur.mode = nr.mode
        removed = [l for l in have if l not in fresh]
        if self.commander is not None:
            self.commander.set_grsim_addr(self.grsim_addr)
            for r in self._inventory:
                self.commander.register(r.is_yellow, r.robot_id, r.ip, r.port,
                                        mode=r.mode, grsim_id=r.grsim_id)
        return {"changed": changed, "added": added, "removed": removed}

    def save_config(self, path: Path | None = None) -> Path:
        """Write the current inventory's IP/port back into ipconfig.yaml.

        Loads the raw YAML, updates each robot's ip/port by its team+letter so
        every other key is preserved, and writes it back. Returns the path.
        """
        cfg_path = Path(path) if path else bridge.local_config_path("ipconfig.yaml")
        raw = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
        for r in self._inventory:
            team = "yellow" if r.is_yellow else "blue"
            section = raw.setdefault(team, {})
            entry = section.setdefault(r.key, {})
            entry["ip"] = r.ip
            entry["port"] = int(r.port)
        cfg_path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
        return cfg_path

    def ip_conflicts(self) -> dict:
        """Real robots sharing an IP (see module-level ip_conflicts())."""
        return ip_conflicts(self.robots())

    # -- field geometry --
    def _build_limits(self) -> safety.Limits:
        vs = self.vision.field_size() if self.vision else None
        half_len, half_wid, src = resolve_field_half(
            vs, self.settings.get("field_length_mm"),
            self.settings.get("field_width_mm"), bridge.get_field_geometry())
        self._field_source = src
        return safety.limits(
            boundary_inset=float(self.settings.get("boundary_inset_mm",
                                                   DEFAULTS["boundary_inset_mm"])),
            brake_zone=float(self.settings.get("brake_zone_mm",
                                               DEFAULTS["brake_zone_mm"])),
            half_len=half_len, half_wid=half_wid)

    def refresh_field_geometry(self) -> bool:
        """Rebuild the arena from the latest field size (vision/settings).

        Returns True if the field size changed (e.g. SSL-Vision geometry just
        arrived); pushes the new limits to the commander so driving uses them.
        """
        new_lim = self._build_limits()
        if (new_lim.half_len != self.lim.half_len or
                new_lim.half_wid != self.lim.half_wid):
            self.lim = new_lim
            if self.commander is not None:
                self.commander.set_limits(new_lim)
            return True
        return False

    def field_info(self) -> dict:
        lim = self.lim
        return {
            "length_mm": round(lim.half_len * 2, 1),
            "width_mm": round(lim.half_wid * 2, 1),
            "source": self._field_source,
            "arena_half_len_mm": round(lim.arena_half_len, 1),
            "arena_half_wid_mm": round(lim.arena_half_wid, 1),
        }

    def probe_robot(self, robot: RobotInfo, seconds: float = 5.0,
                    move_speed: float = 0.0, log=None) -> dict:
        """Is the robot actually there? Stream a direct command and listen.

        Sends a harmless zero heartbeat (or a gentle body-forward nudge if
        move_speed > 0) straight to the robot's ip:port — exactly like the real
        dispatcher — and watches for (a) telemetry coming back and (b) vision
        motion. Separates "wrong IP / robot off" (silent) from "reachable but
        frozen" (replies but won't move, e.g. SAFE-mode wheel-math).
        """
        log = log or (lambda *_: None)
        if not self._started:
            self.start()
        is_y, rid = robot.is_yellow, robot.robot_id

        conflict = self.ip_conflicts().get(robot.ip)
        if conflict:
            log(f"  [!] {robot.ip} is shared by {', '.join(conflict)} — a "
                "command for one of them is acted on by whichever robot answers "
                "at that IP. Give each robot a unique IP in ipconfig.yaml.")

        self.commander.reset_stats(is_y, rid)
        pkt0 = self.telemetry.status().get("packets", 0)
        s0 = self.vision.get_pose_sample(is_y, rid, max_age=1.0)
        max_move = 0.0
        end = time.time() + seconds
        while time.time() < end:
            self.commander.set_velocity(is_y, rid, vx=move_speed, vy=0.0, w=0.0,
                                        frame="body", safe=False)
            time.sleep(0.05)
            s = self.vision.get_pose_sample(is_y, rid, max_age=1.0)
            if s0 is not None and s is not None:
                max_move = max(max_move, math.hypot(s.x - s0.x, s.y - s0.y))
        self.commander.stop_robot(is_y, rid)

        st = self.telemetry.status()
        rs = self.telemetry.robot_status(is_y, rid)
        cs = self.commander.stats(is_y, rid)
        telem_pkts = st.get("packets", 0) - pkt0
        vis = self.vision.get_pose_sample(is_y, rid, max_age=1.0) is not None

        if cs.get("send_errors", 0) and not cs.get("sends", 0):
            verdict = ("SEND FAILED — the OS rejected every packet "
                       f"({cs.get('last_error')}). The IP in ipconfig.yaml is "
                       "wrong/unroutable. Fix it and check you're on the robot LAN.")
        elif telem_pkts > 0:
            if move_speed > 0 and max_move >= 12.0:
                verdict = (f"ALIVE & MOVING — telemetry returning ({telem_pkts} "
                           f"pkts) and vision saw {max_move:.0f} mm of motion. "
                           "Comms are fine.")
            else:
                verdict = (f"ALIVE but did not move — telemetry IS returning "
                           f"({telem_pkts} pkts), so the robot receives commands. "
                           "Motion is frozen on the robot: SAFE-mode wheel-math / "
                           "W_LIMIT or wheel units. Fix RobotFramework, not DiagTool.")
        else:
            verdict = (f"SILENT — PC sent {cs.get('sends', 0)} commands with no "
                       "errors, but the robot never replied and "
                       f"{'vision sees it' if vis else 'vision does NOT see it'}. "
                       "Nothing is answering at this IP: robot off, RobotFramework "
                       "not running, wrong IP, or wrong port. Ping it and verify "
                       "ipconfig.yaml.")

        result = {
            "robot": robot.label, "ip": robot.ip, "port": robot.port,
            "robot_id": rid, "seconds": seconds, "move_speed": move_speed,
            "telemetry_packets": telem_pkts, "telemetry_seen": rs.get("seen", False),
            "vision_visible": vis, "vision_motion_mm": round(max_move, 1),
            "commands_sent": cs.get("sends", 0),
            "send_errors": cs.get("send_errors", 0),
            "last_error": cs.get("last_error"),
            "ip_shared_with": conflict or [],
            "verdict": verdict,
        }
        log(f"  {verdict}")
        return result

    # -- lifecycle --
    def start(self) -> None:
        if self._started:
            return
        self.vision.start()
        self.telemetry.start()
        self.commander = Commander(
            self.vision,
            sender_ip=getattr(self.config, "robot_ip", None),
            send_hz=float(self.settings.get("send_hz", 50.0)),
            pose_max_age=float(self.settings.get("pose_max_age_s",
                                                 DEFAULTS["pose_max_age_s"])),
            drive_grace=float(self.settings.get("drive_grace_s",
                                                DEFAULTS["drive_grace_s"])),
            lim=self.lim,
            blind_speed=float(self.settings.get("direct_blind_speed_ms",
                                                DEFAULTS["direct_blind_speed_ms"])),
            safety_reaction_s=float(self.settings.get("safety_reaction_s",
                                                      DEFAULTS["safety_reaction_s"])),
            safety_decel_mm_s2=float(self.settings.get("safety_decel_mm_s2",
                                                       DEFAULTS["safety_decel_mm_s2"])),
            safety_factor=float(self.settings.get("safety_factor",
                                                  DEFAULTS["safety_factor"])),
            safety_max_speed_mm_s=float(self.settings.get("safety_max_speed_mm_s",
                                                          DEFAULTS["safety_max_speed_mm_s"])),
            grsim_addr=self.grsim_addr,
        )
        # register every robot so the commander can drive any of them, each with
        # its target protocol (real ip:port, or the grSim command address)
        for r in self.robots():
            self.commander.register(r.is_yellow, r.robot_id, r.ip, r.port,
                                    mode=r.mode, grsim_id=r.grsim_id)
        self.commander.start()
        self._started = True

    def stop(self) -> None:
        if self.commander:
            self.commander.stop()
        self.telemetry.stop()
        self.vision.stop()
        self._started = False

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()

    # -- status --
    def source_status(self) -> dict:
        return {"vision": self.vision.status(),
                "telemetry": self.telemetry.status()}

    def _context(self) -> DiagContext:
        return DiagContext(self.vision, self.telemetry, self.commander,
                           self.settings, self.lim)

    # -- running tests --
    def run_diagnostic(self, robot: RobotInfo, name: str,
                       log=None, progress=None, stop=None) -> dict:
        log = log or (lambda *_: None)
        progress = progress or (lambda *_: None)
        stop = stop or (lambda: False)
        cls = BY_NAME.get(name)
        if cls is None:
            return {"error": f"unknown diagnostic '{name}'"}
        if not self._started:
            self.start()
        diag = cls(self._context())
        log(f"== {diag.title} :: {robot.label} ==")
        t0 = time.time()
        try:
            res = diag.run(robot.ref(), log, progress, stop)
        except Exception as e:
            from .diagnostics import StopRequested
            if isinstance(e, StopRequested):
                log("  [aborted]")
                res = {"aborted": True}
            else:
                res = {"error": f"{type(e).__name__}: {e}"}
                log(f"  [error] {res['error']}")
        finally:
            if self.commander:
                self.commander.stop_robot(robot.is_yellow, robot.robot_id)
        res["_seconds"] = round(time.time() - t0, 2)
        progress(1.0, "done")
        return res

    def run_sweep(self, robots: list[RobotInfo], test_names: list[str] | None = None,
                  log=None, progress=None, stop=None,
                  output_dir=None) -> dict:
        log = log or (lambda *_: None)
        progress = progress or (lambda *_: None)
        stop = stop or (lambda: False)
        test_names = test_names or [d.name for d in ALL_DIAGNOSTICS]
        if not self._started:
            self.start()

        all_results: dict[str, dict] = {}
        total = max(len(robots) * len(test_names), 1)
        done = 0
        for r in robots:
            all_results.setdefault(r.label, {})
            for name in test_names:
                if stop():
                    log("Sweep aborted.")
                    break

                def _p(frac, text="", _n=name, _d=done):
                    progress((_d + max(0.0, min(1.0, frac))) / total,
                             f"{r.label}:{_n} {text}")

                res = self.run_diagnostic(r, name, log=log, progress=_p, stop=stop)
                all_results[r.label][name] = res
                done += 1
                progress(done / total, f"{r.label}:{name} done")
            if stop():
                break

        cal = calibrator.build_calibration(all_results)
        st = self.source_status()
        rep = report.build_report(
            all_results, cal, st["vision"], st["telemetry"],
            meta={"robots": [r.label for r in robots], "tests": test_names,
                  "ip_conflicts": ip_conflicts(robots)})

        out_dir = Path(output_dir) if output_dir else (
            DEFAULT_OUTPUT / time.strftime("%Y%m%d_%H%M%S"))
        paths = report.write_report(out_dir, rep)
        rep["_paths"] = paths
        log(f"\nReport written:\n  {paths['txt']}\n  {paths['json']}")
        return rep
