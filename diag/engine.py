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

    @property
    def label(self) -> str:
        return f"{'Y' if self.is_yellow else 'B'}{self.robot_id}"

    @property
    def is_real(self) -> bool:
        return not (self.ip.startswith("127.") or self.ip == "localhost")

    def ref(self) -> RobotRef:
        return RobotRef(self.is_yellow, self.robot_id, self.ip, self.port)


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
        self.lim = safety.limits()

        self.vision = VisionSource(port=int(self.config.vision[1]))
        self.telemetry = TelemetrySource(self.config)
        self.commander: Commander | None = None
        self._started = False

    # -- robot inventory --
    def robots(self, real_only: bool = False) -> list[RobotInfo]:
        out: list[RobotInfo] = []
        for is_yellow, team in ((True, getattr(self.config, "yellow", {})),
                                (False, getattr(self.config, "blue", {}))):
            for key, d in (team or {}).items():
                sid = d.get("shellID")
                if sid is None:
                    continue
                info = RobotInfo(bool(is_yellow), int(sid),
                                 str(d.get("ip", "127.0.0.1")),
                                 int(d.get("port", 50514)), str(key))
                if real_only and not info.is_real:
                    continue
                out.append(info)
        return out

    def find_robot(self, label: str) -> RobotInfo | None:
        for r in self.robots():
            if r.label.lower() == label.lower():
                return r
        return None

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
        )
        # register every robot so the commander can drive any of them
        for r in self.robots():
            self.commander.register(r.is_yellow, r.robot_id, r.ip, r.port)
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
            meta={"robots": [r.label for r in robots], "tests": test_names})

        out_dir = Path(output_dir) if output_dir else (
            DEFAULT_OUTPUT / time.strftime("%Y%m%d_%H%M%S"))
        paths = report.write_report(out_dir, rep)
        rep["_paths"] = paths
        log(f"\nReport written:\n  {paths['txt']}\n  {paths['json']}")
        return rep
