"""
diagnostics.py — the measurement battery.

Each Diagnostic measures one facet of the control pipeline and returns a
result dict (always JSON-serialisable). Tests are written so they:

  * never drive a robot they can't see (Commander enforces this too),
  * always drive toward open field and stop before the safety margin,
  * always stop the robot in a finally block,
  * are abortable mid-run via the `stop()` predicate.

The headline number is command_latency: time from issuing a velocity command
to the robot visibly starting to move. That captures the *whole* felt delay
(our send cadence + UDP + the robot's 20 ms read loop + 30 ms recv timeout +
motor spin-up + vision latency). vision_health reports the vision-latency
component separately so it can be subtracted.

Units: positions mm (world frame), velocities m/s for commands, mm/s when
measured from vision and converted as noted, angles rad unless said otherwise.
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass

from . import safety
from .metrics import summarize


# ── Defaults (overridable via diag_settings.yaml) ─────────────────────────
DEFAULTS = {
    "settle_s": 1.0,            # stop-and-wait before/after a motion
    "onset_disp_mm": 12.0,      # displacement that counts as "started moving"
    "onset_angle_rad": 0.05,    # rotation that counts as "started turning"
    "moving_mm_s": 90.0,        # speed above which the robot is "moving"
    "stopped_mm_s": 35.0,       # speed below which the robot is "stopped"
    "vel_window_s": 0.22,       # window for instantaneous velocity estimate
    "poll_s": 0.004,            # vision poll period inside loops
    "pose_max_age_s": 0.30,     # ignore vision poses older than this
    "drive_grace_s": 0.5,       # keep driving on the last good pose this long
                                # after vision goes stale (vs zeroing instantly)
    "latency_direct_send": False,  # latency/rotation tests stream the command
                                # straight to the robot (no vision gate), like
                                # the real dispatcher — set True if vision is
                                # unreliable and you just need the link tested
    "latency_trials": 6,
    "stop_trials": 5,
    "angular_trials": 4,
    "speed_passes": 4,
    "test_speed_ms": 0.12,      # commanded linear speed for motion tests — LOW
                                # on purpose: a miscalibrated robot can move
                                # several x faster than commanded, so keep the
                                # real speed (and coast) small near walls
    "test_w_rads": 0.20,        # commanded angular speed for rotation test
    # --- high-speed / motion tests (face-forward, closed-loop heading) ---
    # These drive the robot FORWARD (body +x) toward its heading rather than
    # omni-strafing in world frame, so it points the way it's going and can run
    # at full speed. Commands are streamed closed-loop every poll tick (not set
    # once), and the stream rate is raised to fast_send_hz for the duration so
    # the motion is smooth, not stepwise. Only run these once a robot is
    # calibrated (speed_scale ~ 1.0).
    "fast_send_hz": 120.0,      # command stream rate during motion tests (Hz)
    "heading_kp": 3.5,          # yaw gain: rad/s of turn per rad of heading error
    "heading_tol_rad": 0.087,   # ~5 deg: "facing the target" once within this
    "face_speed_gate_rad": 0.35,# above this heading error, ease off forward speed
                                # (cos-taper) so the robot tracks its line instead
                                # of veering while it is still turning to face
    "shuttle_speed_ms": 1.0,    # commanded speed for the max-speed shuttle test
                                # (m/s) -- much higher than test_speed_ms on
                                # purpose; only run this once a robot is already
                                # calibrated (speed_scale ~ 1.0)
    "shuttle_legs": 6,          # one-way legs per shuttle run (3 round trips)
    "straight_line_speed_ms": 0.6,  # speed for the straight-line tracking test
    "straight_line_trials": 4,      # runs (alternating direction)
    "accel_speed_ms": 1.0,          # target speed for the acceleration profile
    "accel_trials": 3,              # acceleration-profile runs
    "heading_targets_deg": [90.0, 180.0, -90.0, 0.0],  # heading-step test targets
    "heading_hold_s": 0.6,          # steady-hold window after each heading step
    "waypoint_speed_ms": 0.6,       # cruise speed for the go-to-point test
    "waypoint_tol_mm": 60.0,        # "arrived" radius for the go-to-point test
    "waypoint_cycles": 2,           # there-and-back cycles for the go-to-point test
    "spin_trials": 4,           # pure-spin (in-place rotation) calibration trials
    "spin_w_rads": 0.5,         # commanded angular speed for the spin calibration
                                # (clamped to MAX_W); higher = cleaner w_scale, but
                                # a robot with a low SAFE-mode W limit may freeze
    "spin_seconds": 1.2,        # steady-spin measurement window per trial (s)
    "spin_drift_warn_mm": 120.0,# flag a robot whose centre wanders more than this
                                # while spinning in place (wheel/encoder cal off)
    "accel_settle_s": 0.5,      # ignore this much accel before steady measure
    "stop_buffer_mm": 350.0,    # stop a run this far before the arena edge
    "max_run_s": 4.0,           # hard cap on any single run (s)
    "max_travel_mm": 800.0,     # hard cap on how far a robot may travel per run
    "health_window_s": 6.0,     # observation window for health tests
    # --- field geometry (mm); None -> use SSL-Vision geometry / field_config ---
    "field_length_mm": None,       # explicit field length override (x, goal-goal)
    "field_width_mm": None,        # explicit field width override (y, touchlines)
    # --- arena / wall safety (smaller = robots stay further from walls) ---
    "boundary_inset_mm": 250.0,    # shrink the drive arena this far inside the
                                   # keep-off margin (robots never leave it)
    "brake_zone_mm": 300.0,        # decel ramp distance before the arena edge
    "direct_blind_speed_ms": 0.25, # speed cap for direct/jog with no vision
    # --- custom test zone (world mm); None -> use the full symmetric arena ---
    # Set these to restrict testing to part of the field (e.g. one half at a comp).
    # Usually set by dragging a box on the field view (persisted to test_zone.yaml),
    # but can be pinned here. Clamped to the keep-off-walls safe box either way.
    "test_zone_x_min_mm": None,    # zone min x (length axis, goal-to-goal)
    "test_zone_x_max_mm": None,    # zone max x
    "test_zone_y_min_mm": None,    # zone min y (width axis, touchline-to-touchline)
    "test_zone_y_max_mm": None,    # zone max y
    # --- predictive emergency stop (uses MEASURED vision velocity) ---
    # Catches a robot that moves faster than commanded or coasts/drifts: it is
    # cut the moment its real stopping distance would breach the arena. Tuned
    # conservatively from measured coast (~1.2 m) and latency (~0.5 s).
    "safety_reaction_s": 0.6,      # assumed react-to-stop delay (>= cmd latency)
    "safety_decel_mm_s2": 250.0,   # assumed braking decel (lower = safer/earlier)
    "safety_factor": 1.5,          # extra margin on the stopping distance
    "safety_max_speed_mm_s": 600.0,# hard cap: stop if actually moving faster
    # --- battery gauge (telemetry voltage -> %); UI only, no effect on driving ---
    "battery_full_v": 25.2,        # voltage shown as 100% (6S LiPo: 6 x 4.20 V)
    "battery_empty_v": 19.8,       # voltage shown as 0%   (6S LiPo: 6 x 3.30 V)
    "battery_warn_pct": 40.0,      # gauge turns amber at/below this %
    "battery_crit_pct": 15.0,      # gauge turns red at/below this %
    # --- robots gallery ---
    "robot_photo_dir": None,       # folder of <LABEL>.png robot photos (e.g. Y0.png);
                                   # None -> diag/ui/assets/robots. Missing photos
                                   # fall back to a drawn placeholder.
}


class StopRequested(Exception):
    pass


@dataclass(frozen=True)
class RobotRef:
    is_yellow: bool
    robot_id: int
    ip: str
    port: int

    @property
    def label(self) -> str:
        return f"{'Y' if self.is_yellow else 'B'}{self.robot_id}"


@dataclass
class DiagContext:
    vision: object        # VisionSource
    telemetry: object     # TelemetrySource
    commander: object     # Commander
    settings: dict
    lim: safety.Limits


# ── orientation / motion helpers ──────────────────────────────────────────

def _unwrap(prev_cont: float, prev_raw: float, raw: float) -> float:
    """Continuous angle update given previous continuous & raw, and new raw."""
    d = raw - prev_raw
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return prev_cont + d


class MotionTracker:
    """Rolling pose buffer -> instantaneous world velocity & angular rate."""

    def __init__(self, window_s: float):
        self.window_s = window_s
        self._buf: deque[tuple[float, float, float, float]] = deque()  # t,x,y,ocont
        self._prev_raw_o: float | None = None
        self._cont_o: float = 0.0

    def add(self, t: float, x: float, y: float, o_raw: float) -> None:
        if self._prev_raw_o is None:
            self._cont_o = o_raw
        else:
            self._cont_o = _unwrap(self._cont_o, self._prev_raw_o, o_raw)
        self._prev_raw_o = o_raw
        self._buf.append((t, x, y, self._cont_o))
        cutoff = t - self.window_s
        while len(self._buf) > 2 and self._buf[0][0] < cutoff:
            self._buf.popleft()

    def _slope(self, idx: int) -> float:
        n = len(self._buf)
        if n < 2:
            return 0.0
        t0 = self._buf[0][0]
        ts = [row[0] - t0 for row in self._buf]
        vs = [row[idx] for row in self._buf]
        mt = sum(ts) / n
        mv = sum(vs) / n
        denom = sum((t - mt) ** 2 for t in ts)
        if denom <= 1e-9:
            return 0.0
        num = sum((ts[i] - mt) * (vs[i] - mv) for i in range(n))
        return num / denom

    def velocity(self) -> tuple[float, float]:
        return self._slope(1), self._slope(2)   # mm/s

    def speed(self) -> float:
        vx, vy = self.velocity()
        return math.hypot(vx, vy)

    def omega(self) -> float:
        return self._slope(3)                    # rad/s

    @property
    def ready(self) -> bool:
        return len(self._buf) >= 3 and (self._buf[-1][0] - self._buf[0][0]) > 0.05


# ── Base diagnostic ───────────────────────────────────────────────────────

class Diagnostic:
    name = "base"
    title = "Base diagnostic"
    drives_robot = True

    def __init__(self, ctx: DiagContext):
        self.ctx = ctx
        self.v = ctx.vision
        self.t = ctx.telemetry
        self.cmd = ctx.commander
        self.lim = ctx.lim

    def s(self, key: str):
        return self.ctx.settings.get(key, DEFAULTS[key])

    # to be overridden
    def run(self, robot: RobotRef, log, progress, stop) -> dict:
        raise NotImplementedError

    # -- shared helpers --
    def _pose(self, robot: RobotRef):
        return self.v.get_pose_sample(robot.is_yellow, robot.robot_id,
                                      max_age=self.s("pose_max_age_s"))

    def _check(self, stop):
        if stop():
            raise StopRequested()

    def _wait_visible(self, robot, log, timeout=4.0) -> bool:
        end = time.perf_counter() + timeout
        while time.perf_counter() < end:
            if self._pose(robot) is not None:
                return True
            time.sleep(0.05)
        log(f"  [!] {robot.label} not visible in vision after {timeout:.0f}s")
        return False

    def _settle(self, robot, log, stop, dur=None):
        dur = self.s("settle_s") if dur is None else dur
        self.cmd.stop_robot(robot.is_yellow, robot.robot_id)
        end = time.perf_counter() + dur
        while time.perf_counter() < end:
            self._check(stop)
            time.sleep(0.02)

    def _drive_world(self, robot, vx, vy, w=0.0):
        self.cmd.set_velocity(robot.is_yellow, robot.robot_id,
                              vx=vx, vy=vy, w=w, frame="world")

    # -- send instrumentation (so "no motion" is diagnosable) --
    def _reset_cmd_stats(self, robot):
        fn = getattr(self.cmd, "reset_stats", None)
        if fn:
            fn(robot.is_yellow, robot.robot_id)

    def _cmd_stats(self, robot) -> dict:
        fn = getattr(self.cmd, "stats", None)
        return fn(robot.is_yellow, robot.robot_id) if fn else {}

    def _explain_no_motion(self, robot) -> str:
        """Compact reason string appended to a 'no motion' log line.

        Tells you whether the PC actually streamed commands (robot's fault) or
        zeroed/failed them (vision-gate, safety, or a bad robot address).
        """
        st = self._cmd_stats(robot)
        if not st:
            return ""
        parts = [f"sent={st.get('sends', '?')}",
                 f"no_pose={st.get('zeroed_no_pose', 0)}",
                 f"safety={st.get('zeroed_safety', 0)}",
                 f"send_err={st.get('send_errors', 0)}"]
        ls = st.get("last_sent")
        if ls:
            parts.append(f"last_vel=({ls[0]:.2f},{ls[1]:.2f},{ls[2]:.2f})")
        if st.get("last_error"):
            parts.append(f"err={st['last_error']}")
        return "  [" + " ".join(parts) + "]"

    def _dir_to_open(self, pose):
        """World unit vector pointing into open space (toward the test-zone centre)."""
        x, y, _ = pose
        cx, cy = self.lim.arena_cx, self.lim.arena_cy
        dx, dy = cx - x, cy - y
        d = math.hypot(dx, dy)
        if d < 250:
            return self._open_x_dir(pose)[:2]   # already central; head along x
        return (dx / d, dy / d)

    def _open_x_dir(self, pose):
        """Drive direction along the long (x) axis toward open space, and the room
        that way (mm). Always points toward the zone centre, so it never reverses
        toward a wall (or out of an off-centre test zone) the way a naive flip can."""
        x = pose[0]
        ux = 1.0 if x <= self.lim.arena_cx else -1.0   # tie at centre -> +x
        return ux, 0.0, self._room_ahead(pose, ux, 0.0)

    def _emergency_seen(self, robot) -> bool:
        return bool(self._cmd_stats(robot).get("zeroed_emergency", 0))

    def _room_ahead(self, pose, ux, uy) -> float:
        x, y, _ = pose
        xlo, xhi, ylo, yhi = self.lim.arena_bounds()
        ts = []
        if ux > 1e-6:
            ts.append((xhi - x) / ux)
        elif ux < -1e-6:
            ts.append((xlo - x) / ux)
        if uy > 1e-6:
            ts.append((yhi - y) / uy)
        elif uy < -1e-6:
            ts.append((ylo - y) / uy)
        ts = [t for t in ts if t > 0]
        return min(ts) if ts else 0.0

    def _poll_until(self, robot, predicate, timeout, stop, tracker=None):
        """Poll vision until predicate(sample, tracker) is truthy or timeout.

        Returns (sample_at_trigger, triggered_bool). Feeds `tracker` if given.
        """
        end = time.perf_counter() + timeout
        last_fn = None
        poll = self.s("poll_s")
        while time.perf_counter() < end:
            self._check(stop)
            s = self._pose(robot)
            if s is not None and s.frame_number != last_fn:
                last_fn = s.frame_number
                if tracker is not None:
                    tracker.add(s.t_perf, s.x, s.y, s.o)
                if predicate(s, tracker):
                    return s, True
            time.sleep(poll)
        return self._pose(robot), False

    # -- closed-loop heading / forward-drive helpers (motion tests) -----------
    #
    # The routine calibration tests set a world-frame velocity once and let the
    # Commander stream it. The high-speed motion tests instead POINT the robot
    # the way it is going and drive it FORWARD (body +x), re-issuing the command
    # every poll tick so heading is corrected continuously (smooth, not
    # stepwise) — and they raise the stream rate for the duration.

    def _boost_send_rate(self):
        """Raise the command stream rate for a fast test; return a restore fn.

        No-op on stand-in commanders that don't expose the setter (e.g. the
        self-test's fake), so callers can always use it in a try/finally.
        """
        setter = getattr(self.cmd, "set_send_hz", None)
        getter = getattr(self.cmd, "get_send_hz", None)
        if not setter or not getter:
            return lambda: None
        prev = getter()
        setter(float(self.s("fast_send_hz")))
        return lambda: setter(prev)

    def _steer_forward(self, pose, ux, uy, speed):
        """Body-frame (vx_forward, w) that yaws the robot to FACE world dir
        (ux, uy) and drives forward at `speed`.

        The forward speed is cos-tapered while the heading error is large (past
        face_speed_gate_rad) so the robot tracks its intended line instead of
        launching off at an angle before it has finished turning to face it.
        Returns (vx_forward, w, heading_err_rad).
        """
        o = pose[2]
        err = _ang_diff(math.atan2(uy, ux), o)
        w = safety.clamp(float(self.s("heading_kp")) * err,
                         -self.lim.max_w, self.lim.max_w)
        gate = float(self.s("face_speed_gate_rad"))
        aim = 1.0 if abs(err) <= gate else max(0.0, math.cos(err))
        return speed * aim, w, err

    def _drive_forward_body(self, robot, vx_forward, w):
        """Stream one forward (body +x) command with yaw. Safe pipeline on."""
        self.cmd.set_velocity(robot.is_yellow, robot.robot_id,
                              vx=vx_forward, vy=0.0, w=w,
                              frame="body", safe=True)

    def _rotate_to_heading(self, robot, target_o, stop, timeout=3.0):
        """Spin in place until facing world heading `target_o` (within
        heading_tol_rad) or timeout. Wall-safe (pure rotation). Returns
        (reached_bool, final_heading_err_rad)."""
        tol = float(self.s("heading_tol_rad"))
        kp = float(self.s("heading_kp"))
        end = time.perf_counter() + timeout
        err = math.pi
        while time.perf_counter() < end:
            self._check(stop)
            s = self._pose(robot)
            if s is not None:
                err = _ang_diff(target_o, s.o)
                if abs(err) <= tol:
                    self.cmd.stop_robot(robot.is_yellow, robot.robot_id)
                    return True, err
                w = safety.clamp(kp * err, -self.lim.max_w, self.lim.max_w)
                self.cmd.set_velocity(robot.is_yellow, robot.robot_id,
                                      vx=0.0, vy=0.0, w=w,
                                      frame="world", safe=True)
            time.sleep(self.s("poll_s"))
        self.cmd.stop_robot(robot.is_yellow, robot.robot_id)
        return False, err


# ── Vision health ─────────────────────────────────────────────────────────

class VisionHealthDiagnostic(Diagnostic):
    name = "vision_health"
    title = "Vision health & latency"
    drives_robot = False

    def run(self, robot, log, progress, stop) -> dict:
        win = self.s("health_window_s")
        log(f"Observing vision for {win:.0f}s (robot held still)...")
        # measure stationary pose noise for the selected robot
        xs, ys, os_ = [], [], []
        end = time.perf_counter() + win
        last_fn = None
        while time.perf_counter() < end:
            self._check(stop)
            s = self._pose(robot)
            if s is not None and s.frame_number != last_fn:
                last_fn = s.frame_number
                xs.append(s.x); ys.append(s.y); os_.append(s.o)
            progress(1.0 - (end - time.perf_counter()) / win, "sampling vision")
            time.sleep(0.005)

        st = self.v.status()
        noise = {
            "x_mm": summarize(_centered(xs), "mm"),
            "y_mm": summarize(_centered(ys), "mm"),
            "o_deg": summarize([math.degrees(o) for o in _centered(os_)], "deg"),
            "samples": len(xs),
        }
        log(f"  fps={st['fps']:.1f}  drops={st['drops']} dupes={st['dupes']}")
        if st["sent_latency_ms"]["n"]:
            log(f"  vision latency (recv - t_sent): "
                f"mean={st['sent_latency_ms']['mean']:.1f} ms")
        if noise["samples"] > 3:
            log(f"  stationary pose noise: x std={noise['x_mm']['std']:.2f}mm "
                f"y std={noise['y_mm']['std']:.2f}mm")
        return {"status": st, "pose_noise": noise}


def _centered(xs: list[float]) -> list[float]:
    if not xs:
        return []
    m = sum(xs) / len(xs)
    return [x - m for x in xs]


# ── Telemetry health ──────────────────────────────────────────────────────

class TelemetryHealthDiagnostic(Diagnostic):
    name = "telemetry_health"
    title = "Onboard telemetry health & rate"
    drives_robot = False

    def run(self, robot, log, progress, stop) -> dict:
        win = max(self.s("health_window_s"), 6.0)
        log(f"Listening for onboard telemetry for {win:.0f}s...")
        end = time.perf_counter() + win
        while time.perf_counter() < end:
            self._check(stop)
            progress(1.0 - (end - time.perf_counter()) / win, "listening")
            time.sleep(0.05)

        st = self.t.status()
        rs = self.t.robot_status(robot.is_yellow, robot.robot_id)
        log(f"  total telemetry rate={st['rate_hz']:.2f} Hz  "
            f"packets={st['packets']} parse_errors={st['parse_errors']}")
        if rs.get("seen"):
            log(f"  {robot.label}: rate={rs['rate_hz']:.2f} Hz "
                f"age={_fmt_age(rs['age_s'])} voltage={rs.get('voltage')}")
        else:
            log(f"  {robot.label}: no telemetry received")
        return {"global": st, "robot": rs}


def _fmt_age(a):
    return "—" if a is None else f"{a:.2f}s"


# ── Command -> motion latency ─────────────────────────────────────────────

class CommandLatencyDiagnostic(Diagnostic):
    name = "command_latency"
    title = "Command -> motion latency"

    def run(self, robot, log, progress, stop) -> dict:
        if not self._wait_visible(robot, log):
            return {"error": "robot not visible"}
        trials = int(self.s("latency_trials"))
        speed = float(self.s("test_speed_ms"))
        onset = float(self.s("onset_disp_mm"))
        direct = bool(self.s("latency_direct_send"))
        if direct:
            log("  [direct send] streaming command straight to the robot "
                "(no vision gate / wall guard) — like the real dispatcher.")
        lat_ms, no_motion = [], 0
        try:
            for i in range(trials):
                self._check(stop)
                progress(i / trials, f"trial {i + 1}/{trials}")
                self._settle(robot, log, stop)
                s0 = self._pose(robot)
                if s0 is None:
                    no_motion += 1
                    continue
                ux, uy = self._dir_to_open(s0.pose())
                p0 = (s0.x, s0.y)

                self._reset_cmd_stats(robot)
                t_cmd = time.perf_counter()
                if direct:
                    # body-frame forward nudge, sent straight through
                    self.cmd.set_velocity(robot.is_yellow, robot.robot_id,
                                          vx=speed, vy=0.0, w=0.0,
                                          frame="body", safe=False)
                else:
                    self._drive_world(robot, ux * speed, uy * speed)

                def moved(s, _):
                    return math.hypot(s.x - p0[0], s.y - p0[1]) >= onset

                s_on, ok = self._poll_until(robot, moved, timeout=1.6, stop=stop)
                self.cmd.stop_robot(robot.is_yellow, robot.robot_id)
                if ok and s_on is not None:
                    dt = (s_on.t_perf - t_cmd) * 1000.0
                    lat_ms.append(dt)
                    log(f"  trial {i + 1}: latency={dt:.0f} ms")
                else:
                    no_motion += 1
                    log(f"  trial {i + 1}: NO MOTION within timeout"
                        + self._explain_no_motion(robot))
        finally:
            self._settle(robot, log, stop, dur=0.5)

        res = {
            "latency_ms": summarize(lat_ms, "ms"),
            "trials": trials,
            "no_motion_trials": no_motion,
            "commanded_speed_ms": speed,
            "direct_send": direct,
            "send_stats": self._cmd_stats(robot),
            "note": ("Includes vision latency; subtract vision_health "
                     "sent_latency to approximate actuation-only delay. If "
                     "no_motion: 'sent>0 no_pose=0 send_err=0' points at the "
                     "robot (SAFE-mode/comms); 'no_pose>0' at vision; "
                     "'send_err>0' at a bad robot IP in ipconfig.yaml."),
        }
        return res


# ── Stop latency + coast distance ─────────────────────────────────────────

class StopLatencyDiagnostic(Diagnostic):
    name = "stop_latency"
    title = "Stop latency & coast distance"

    def run(self, robot, log, progress, stop) -> dict:
        if not self._wait_visible(robot, log):
            return {"error": "robot not visible"}
        trials = int(self.s("stop_trials"))
        speed = float(self.s("test_speed_ms"))
        moving = float(self.s("moving_mm_s"))
        stopped = float(self.s("stopped_mm_s"))
        lat_ms, coast_mm, skipped = [], [], 0
        try:
            for i in range(trials):
                self._check(stop)
                progress(i / trials, f"trial {i + 1}/{trials}")
                self._settle(robot, log, stop)
                s0 = self._pose(robot)
                if s0 is None:
                    skipped += 1
                    continue
                # Drive along the long axis toward open field — NEVER reverse
                # toward a wall. Skip the trial if there isn't clear room.
                ux, uy, room = self._open_x_dir(s0.pose())
                need = self.s("stop_buffer_mm") + 200.0
                if room < need:
                    skipped += 1
                    log(f"  trial {i + 1}: only {room:.0f} mm of clear room "
                        f"(need {need:.0f}) — skipped for safety")
                    continue
                self._reset_cmd_stats(robot)
                self._drive_world(robot, ux * speed, uy * speed)

                tr = MotionTracker(self.s("vel_window_s"))

                def is_moving(s, t):
                    return t is not None and t.ready and t.speed() >= moving

                s_mv, ok = self._poll_until(robot, is_moving,
                                            timeout=self.s("max_run_s"),
                                            stop=stop, tracker=tr)
                if self._emergency_seen(robot):
                    skipped += 1
                    self.cmd.stop_robot(robot.is_yellow, robot.robot_id)
                    log(f"  trial {i + 1}: boundary guard stopped the robot "
                        f"(moving too fast / drifting) — skipped"
                        + self._explain_no_motion(robot))
                    self._settle(robot, log, stop)
                    continue
                if not ok:
                    skipped += 1
                    self.cmd.stop_robot(robot.is_yellow, robot.robot_id)
                    log(f"  trial {i + 1}: never reached moving speed")
                    continue

                t_stop = time.perf_counter()
                p_stop = (s_mv.x, s_mv.y)
                self.cmd.stop_robot(robot.is_yellow, robot.robot_id)

                tr2 = MotionTracker(self.s("vel_window_s"))

                def is_stopped(s, t):
                    return t is not None and t.ready and t.speed() <= stopped

                s_halt, ok2 = self._poll_until(robot, is_stopped, timeout=2.5,
                                               stop=stop, tracker=tr2)
                if ok2 and s_halt is not None:
                    dt = (s_halt.t_perf - t_stop) * 1000.0
                    dist = math.hypot(s_halt.x - p_stop[0], s_halt.y - p_stop[1])
                    lat_ms.append(dt)
                    coast_mm.append(dist)
                    log(f"  trial {i + 1}: stop_latency={dt:.0f} ms "
                        f"coast={dist:.0f} mm")
                else:
                    skipped += 1
                    log(f"  trial {i + 1}: did not settle in time")
                self._settle(robot, log, stop)
        finally:
            self._settle(robot, log, stop, dur=0.5)

        return {
            "stop_latency_ms": summarize(lat_ms, "ms"),
            "coast_distance_mm": summarize(coast_mm, "mm"),
            "trials": trials,
            "skipped": skipped,
            "commanded_speed_ms": speed,
        }


# ── Linear speed scale + drift ────────────────────────────────────────────

class SpeedScaleDiagnostic(Diagnostic):
    name = "speed_scale"
    title = "Linear speed scale & drift"

    def run(self, robot, log, progress, stop) -> dict:
        if not self._wait_visible(robot, log):
            return {"error": "robot not visible"}
        passes = int(self.s("speed_passes"))
        speed = float(self.s("test_speed_ms"))
        ratios, drifts, heads, actuals, sent_mags = [], [], [], [], []
        sign = 1
        try:
            for i in range(passes):
                self._check(stop)
                progress(i / passes, f"pass {i + 1}/{passes}")
                self._settle(robot, log, stop)
                s0 = self._pose(robot)
                if s0 is None:
                    continue
                # primary axis = x (most room); flip if no room that way
                ux, uy = float(sign), 0.0
                if self._room_ahead(s0.pose(), ux, uy) < self.s("stop_buffer_mm") + 600:
                    sign = -sign
                    ux = float(sign)
                self._drive_world(robot, ux * speed, uy * speed)

                # let it reach steady speed
                time.sleep(self.s("accel_settle_s"))
                self._check(stop)
                s_start = self._pose(robot)
                if s_start is None:
                    self.cmd.stop_robot(robot.is_yellow, robot.robot_id)
                    continue
                t_start = s_start.t_perf
                p_start = (s_start.x, s_start.y, s_start.o)

                # drive until near arena edge, travel cap, or time cap
                end = time.perf_counter() + self.s("max_run_s")
                max_travel = float(self.s("max_travel_mm"))
                s_last = s_start
                while time.perf_counter() < end:
                    self._check(stop)
                    s = self._pose(robot)
                    if s is not None:
                        s_last = s
                        if self._room_ahead(s.pose(), ux, uy) < self.s("stop_buffer_mm"):
                            break
                        if math.hypot(s.x - p_start[0], s.y - p_start[1]) >= max_travel:
                            break
                    ls = self.cmd.last_sent(robot.is_yellow, robot.robot_id)
                    if ls:
                        sent_mags.append(math.hypot(ls[0], ls[1]))
                    time.sleep(self.s("poll_s"))

                self.cmd.stop_robot(robot.is_yellow, robot.robot_id)

                dt = s_last.t_perf - t_start
                dx = s_last.x - p_start[0]
                dy = s_last.y - p_start[1]
                dist = math.hypot(dx, dy)
                if dt < 0.2 or dist < 50:
                    log(f"  pass {i + 1}: too little motion, skipped")
                    sign = -sign
                    continue

                actual = (dist / dt) / 1000.0          # m/s
                ratio = actual / speed if speed > 0 else 0.0
                # perpendicular drift relative to intended direction
                along = dx * ux + dy * uy
                perp = abs(dx * (-uy) + dy * ux)
                drift_per_m = perp / (dist / 1000.0)
                head_deg = math.degrees(abs(_ang_diff(s_last.o, p_start[2])))
                head_per_m = head_deg / (dist / 1000.0)

                ratios.append(ratio)
                actuals.append(actual)
                drifts.append(drift_per_m)
                heads.append(head_per_m)
                log(f"  pass {i + 1}: cmd={speed:.2f} actual={actual:.3f} m/s "
                    f"(ratio={ratio:.3f}) drift={drift_per_m:.1f} mm/m "
                    f"head={head_per_m:.1f} deg/m")
                sign = -sign
                self._settle(robot, log, stop, dur=0.4)
        finally:
            self._settle(robot, log, stop, dur=0.5)

        return {
            "commanded_speed_ms": speed,
            "actual_speed_ms": summarize(actuals, "m/s"),
            "speed_ratio": summarize(ratios, ""),
            "sent_speed_ms": summarize(sent_mags, "m/s"),
            "lateral_drift_mm_per_m": summarize(drifts, "mm/m"),
            "heading_drift_deg_per_m": summarize(heads, "deg/m"),
            "passes": passes,
        }


def _ang_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


# ── Angular: rotation latency + w scale ───────────────────────────────────

class AngularDiagnostic(Diagnostic):
    name = "angular"
    title = "Rotation latency & angular speed scale"

    def run(self, robot, log, progress, stop) -> dict:
        if not self._wait_visible(robot, log):
            return {"error": "robot not visible"}
        trials = int(self.s("angular_trials"))
        w_cmd = min(float(self.s("test_w_rads")), self.lim.max_w)
        onset = float(self.s("onset_angle_rad"))
        direct = bool(self.s("latency_direct_send"))
        lat_ms, scales, no_motion = [], [], 0
        sign = 1
        try:
            for i in range(trials):
                self._check(stop)
                progress(i / trials, f"trial {i + 1}/{trials}")
                self._settle(robot, log, stop)
                s0 = self._pose(robot)
                if s0 is None:
                    no_motion += 1
                    continue
                o0 = s0.o
                w_target = sign * w_cmd

                self._reset_cmd_stats(robot)
                t_cmd = time.perf_counter()
                self.cmd.set_velocity(robot.is_yellow, robot.robot_id,
                                      vx=0, vy=0, w=w_target, frame="world",
                                      safe=not direct)

                def turned(s, _):
                    return abs(_ang_diff(s.o, o0)) >= onset

                s_on, ok = self._poll_until(robot, turned, timeout=1.6, stop=stop)
                if ok and s_on is not None:
                    lat = (s_on.t_perf - t_cmd) * 1000.0
                    lat_ms.append(lat)
                    # measure steady angular rate over a window
                    tr = MotionTracker(0.6)
                    end = time.perf_counter() + 0.8
                    last_fn = None
                    while time.perf_counter() < end:
                        self._check(stop)
                        s = self._pose(robot)
                        if s is not None and s.frame_number != last_fn:
                            last_fn = s.frame_number
                            tr.add(s.t_perf, s.x, s.y, s.o)
                        time.sleep(self.s("poll_s"))
                    actual_w = tr.omega() if tr.ready else 0.0
                    scale = abs(actual_w) / w_cmd if w_cmd > 0 else 0.0
                    scales.append(scale)
                    log(f"  trial {i + 1}: rot_latency={lat:.0f} ms "
                        f"actual_w={actual_w:.3f} rad/s (scale={scale:.3f})")
                else:
                    no_motion += 1
                    log(f"  trial {i + 1}: NO ROTATION within timeout"
                        + self._explain_no_motion(robot))
                self.cmd.stop_robot(robot.is_yellow, robot.robot_id)
                sign = -sign
        finally:
            self._settle(robot, log, stop, dur=0.5)

        return {
            "commanded_w_rads": w_cmd,
            "rotation_latency_ms": summarize(lat_ms, "ms"),
            "w_scale": summarize(scales, ""),
            "trials": trials,
            "no_motion_trials": no_motion,
            "direct_send": direct,
            "send_stats": self._cmd_stats(robot),
        }


# ── Spin-in-place calibration (pure rotation, no translation) ──────────────

class SpinCalibrationDiagnostic(Diagnostic):
    """Pure spin in place: command rotation only (vx=vy=0) and measure how well
    the robot turns without translating.

    Reports, per robot:
      * w_scale          actual / commanded angular speed (cleaner than the
                         angular test's, measured over a longer steady window),
      * spin_latency_ms  time from the spin command to first rotation,
      * center_drift_mm  how far the robot's CENTRE wandered while spinning — a
                         well-calibrated omni robot spins about its centre, so
                         this should be ~0; a large value means the wheel radii
                         / encoder scales are mismatched (the robot "walks" while
                         turning). This is the headline precision number.

    Safety: the command is streamed through the same wall-safe Commander as every
    other test. Rotation itself can't drive a robot into a wall; if a miscalibrated
    robot *translates* while spinning, the Commander's predictive emergency stop
    (on MEASURED vision velocity) cuts it the instant its real stopping distance
    would breach the arena/test-zone — so the trial is aborted, not the robot.
    """

    name = "spin"
    title = "Spin-in-place calibration (pure rotation)"

    def run(self, robot, log, progress, stop) -> dict:
        if not self._wait_visible(robot, log):
            return {"error": "robot not visible"}
        trials = int(self.s("spin_trials"))
        w_cmd = min(float(self.s("spin_w_rads")), self.lim.max_w)
        spin_s = float(self.s("spin_seconds"))
        onset = float(self.s("onset_angle_rad"))
        warn_mm = float(self.s("spin_drift_warn_mm"))
        scales, lat_ms, drifts, abs_w, no_motion, guard_trips = [], [], [], [], 0, 0
        sign = 1
        try:
            for i in range(trials):
                self._check(stop)
                progress(i / trials, f"spin {i + 1}/{trials}")
                self._settle(robot, log, stop)
                s0 = self._pose(robot)
                if s0 is None:
                    no_motion += 1
                    continue
                x0, y0, o0 = s0.x, s0.y, s0.o
                w_target = sign * w_cmd

                self._reset_cmd_stats(robot)
                t_cmd = time.perf_counter()
                # pure spin: world-frame rotation only, full safety pipeline on
                self.cmd.set_velocity(robot.is_yellow, robot.robot_id,
                                      vx=0.0, vy=0.0, w=w_target,
                                      frame="world", safe=True)

                def turned(s, _):
                    return abs(_ang_diff(s.o, o0)) >= onset

                s_on, ok = self._poll_until(robot, turned, timeout=1.6, stop=stop)
                if not ok:
                    no_motion += 1
                    self.cmd.stop_robot(robot.is_yellow, robot.robot_id)
                    log(f"  spin {i + 1}: NO ROTATION within timeout"
                        + self._explain_no_motion(robot))
                    sign = -sign
                    continue
                lat = (s_on.t_perf - t_cmd) * 1000.0
                lat_ms.append(lat)

                # steady spin window: track angular rate and centre drift
                tr = MotionTracker(0.6)
                max_drift = 0.0
                last_fn = None
                tripped = False
                end = time.perf_counter() + spin_s
                while time.perf_counter() < end:
                    self._check(stop)
                    s = self._pose(robot)
                    if s is not None and s.frame_number != last_fn:
                        last_fn = s.frame_number
                        tr.add(s.t_perf, s.x, s.y, s.o)
                        max_drift = max(max_drift, math.hypot(s.x - x0, s.y - y0))
                    if self._emergency_seen(robot):
                        tripped = True
                        break
                    time.sleep(self.s("poll_s"))
                self.cmd.stop_robot(robot.is_yellow, robot.robot_id)

                actual_w = tr.omega() if tr.ready else 0.0
                scale = abs(actual_w) / w_cmd if w_cmd > 0 else 0.0
                scales.append(scale)
                abs_w.append(abs(actual_w))
                drifts.append(max_drift)
                if tripped:
                    guard_trips += 1
                    log(f"  spin {i + 1}: boundary guard tripped — robot "
                        f"TRANSLATED while spinning (drift {max_drift:.0f} mm); "
                        "wheel/encoder calibration is off")
                else:
                    flag = "  [!] drifts" if max_drift > warn_mm else ""
                    log(f"  spin {i + 1}: w_scale={scale:.3f} "
                        f"actual_w={actual_w:.3f} rad/s  centre_drift="
                        f"{max_drift:.0f} mm  latency={lat:.0f} ms{flag}")
                sign = -sign
                self._settle(robot, log, stop, dur=0.4)
        finally:
            self._settle(robot, log, stop, dur=0.5)

        return {
            "commanded_w_rads": w_cmd,
            "spin_seconds": spin_s,
            "w_scale": summarize(scales, ""),
            "actual_w_rads": summarize(abs_w, "rad/s"),
            "spin_latency_ms": summarize(lat_ms, "ms"),
            "center_drift_mm": summarize(drifts, "mm"),
            "trials": trials,
            "no_motion_trials": no_motion,
            "boundary_guard_trips": guard_trips,
            "note": ("center_drift_mm ~0 = spins cleanly about its centre; a large "
                     "drift (or a boundary-guard trip) means the robot translates "
                     "while turning — mismatched wheel radii / encoder scales."),
        }


# ── Max-speed shuttle (continuous side-to-side top-speed run) ─────────────

class SpeedShuttleDiagnostic(Diagnostic):
    """Continuous max-speed shuttle: bounce the robot back and forth along the
    test zone's long (goal-to-goal) axis at a much higher commanded speed than
    the calibration battery uses (shuttle_speed_ms, default 1.0 m/s) -- to see
    the actual top speed and behaviour under real load instead of the low,
    deliberately-safe speed_scale probe.

    The robot FACES the way it is going: at each turnaround it rotates to point
    down the new direction and then drives FORWARD (body +x) at full speed,
    which is how a real robot moves fastest and most stably. The forward
    command is streamed closed-loop every poll tick (heading corrected
    continuously, so it stays smooth rather than stepwise) and the command
    stream rate is raised to fast_send_hz for the duration.

    Same wall-safe pipeline as every other test: velocities go through the
    Commander with safe=True, so the arena brake/hard-stop and the predictive
    emergency stop (driven by MEASURED vision velocity, so it adapts to
    whatever speed the robot is actually reaching, not just what was
    commanded) still protect it. On top of that each leg reverses on its own,
    well before the edge, at a buffer scaled from the commanded speed using
    the same reaction+coast model as the emergency stop -- so a healthy run
    turns around gracefully and rarely needs the hard stop at all; that's
    only a backstop for a mis-scaled/misbehaving robot.

    If the test zone is too short for even one clean leg at the requested
    speed, the run refuses up front rather than hammering the emergency stop
    every leg -- lower shuttle_speed_ms or widen the test zone.
    """

    name = "speed_shuttle"
    title = "Max-speed shuttle (faces forward)"

    def _turn_buffer_mm(self, speed_ms: float) -> float:
        """Distance-to-edge at which a leg reverses on its own -- generous vs.
        the Commander's own emergency-stop margin (_stop_distance) so a
        healthy robot turns around gracefully instead of relying on it."""
        speed_mm_s = speed_ms * 1000.0
        reaction = float(self.s("safety_reaction_s"))
        decel = max(float(self.s("safety_decel_mm_s2")), 1.0)
        factor = float(self.s("safety_factor"))
        coast = (speed_mm_s * speed_mm_s) / (2.0 * decel)
        return 1.25 * factor * (reaction * speed_mm_s + coast) + float(self.s("stop_buffer_mm"))

    def run(self, robot, log, progress, stop) -> dict:
        if not self._wait_visible(robot, log):
            return {"error": "robot not visible"}
        speed = float(self.s("shuttle_speed_ms"))
        legs = max(1, int(self.s("shuttle_legs")))
        turn_buffer = self._turn_buffer_mm(speed)
        max_leg_s = float(self.s("max_run_s"))

        s0 = self._pose(robot)
        if s0 is None:
            return {"error": "robot not visible"}
        ux, uy, room = self._open_x_dir(s0.pose())
        min_room = turn_buffer + 300.0
        if room < min_room:
            return {"error": f"test zone too short for a {speed:.2f} m/s shuttle "
                             f"(has {room:.0f} mm of room ahead, needs >= "
                             f"{min_room:.0f} mm to stop safely from that speed -- "
                             "position the robot nearer one end of the test zone "
                             "before starting so it has the full length ahead, "
                             "or lower shuttle_speed_ms)"}

        log(f"  shuttle: {speed:.2f} m/s x {legs} legs (faces forward), "
            f"turn buffer {turn_buffer:.0f} mm, room {room:.0f} mm")
        peaks, ratios, heading_errs, trip_legs = [], [], [], 0
        sign = 1
        self._reset_cmd_stats(robot)
        restore_rate = self._boost_send_rate()
        try:
            for leg in range(legs):
                self._check(stop)
                progress(leg / legs, f"leg {leg + 1}/{legs}")
                lux, luy = ux * sign, uy * sign
                # 1) point the robot down the leg before blasting off
                self._rotate_to_heading(robot, math.atan2(luy, lux), stop,
                                        timeout=2.5)

                # 2) closed-loop forward drive, re-issued every tick
                tr = MotionTracker(0.4)
                last_fn = None
                leg_peak = 0.0
                leg_head_err = 0.0
                tripped = False
                end = time.perf_counter() + max_leg_s
                while time.perf_counter() < end:
                    self._check(stop)
                    s = self._pose(robot)
                    if s is not None:
                        if s.frame_number != last_fn:
                            last_fn = s.frame_number
                            tr.add(s.t_perf, s.x, s.y, s.o)
                            if tr.ready:
                                leg_peak = max(leg_peak, tr.speed())
                            if self._room_ahead(s.pose(), lux, luy) < turn_buffer:
                                break
                        vx_f, w, herr = self._steer_forward(s.pose(), lux, luy, speed)
                        leg_head_err = max(leg_head_err, abs(herr))
                        self._drive_forward_body(robot, vx_f, w)
                    if self._emergency_seen(robot):
                        tripped = True
                        break
                    time.sleep(self.s("poll_s"))
                self.cmd.stop_robot(robot.is_yellow, robot.robot_id)

                if tripped:
                    trip_legs += 1
                    log(f"  leg {leg + 1}: emergency stop tripped -- still "
                        "accelerating into its own stopping distance; widen "
                        "the test zone or lower shuttle_speed_ms")
                else:
                    ratio = (leg_peak / 1000.0) / speed if speed > 0 else 0.0
                    peaks.append(leg_peak)
                    ratios.append(ratio)
                    heading_errs.append(math.degrees(leg_head_err))
                    log(f"  leg {leg + 1}: peak={leg_peak / 1000.0:.2f} m/s "
                        f"(cmd={speed:.2f}, ratio={ratio:.2f}) "
                        f"max_heading_err={math.degrees(leg_head_err):.1f} deg")
                sign = -sign
                self._settle(robot, log, stop, dur=0.3)
        finally:
            restore_rate()
            self.cmd.stop_robot(robot.is_yellow, robot.robot_id)
            self._settle(robot, log, stop, dur=0.5)

        return {
            "commanded_speed_ms": speed,
            "legs": legs,
            "turn_buffer_mm": round(turn_buffer, 0),
            "peak_speed_ms": summarize([p / 1000.0 for p in peaks], "m/s"),
            "speed_ratio": summarize(ratios, ""),
            "max_heading_err_deg": summarize(heading_errs, "deg"),
            "emergency_trip_legs": trip_legs,
        }


# ── Straight-line tracking (facing forward) ───────────────────────────────

class StraightLineDiagnostic(Diagnostic):
    """Drive the robot FORWARD in a straight line and measure how straight it
    actually goes: lateral deviation off the intended line and heading drift.

    Points the robot down the open axis, then drives forward (body +x) at
    straight_line_speed with closed-loop heading, sampling the path. The
    lateral deviation of each sample is its perpendicular distance from the
    ideal straight line through the start point along the intended heading;
    we report peak and RMS deviation per run plus heading drift. Wall-safe
    (arena guard + emergency stop as always)."""

    name = "straight_line"
    title = "Straight-line tracking (facing forward)"

    def run(self, robot, log, progress, stop) -> dict:
        if not self._wait_visible(robot, log):
            return {"error": "robot not visible"}
        speed = float(self.s("straight_line_speed_ms"))
        trials = max(1, int(self.s("straight_line_trials")))
        buffer_mm = float(self.s("stop_buffer_mm")) + 400.0
        max_run_s = float(self.s("max_run_s"))
        max_travel = float(self.s("max_travel_mm"))

        peak_dev, rms_dev, head_drift, lengths = [], [], [], []
        sign = 1
        restore_rate = self._boost_send_rate()
        try:
            for i in range(trials):
                self._check(stop)
                progress(i / trials, f"run {i + 1}/{trials}")
                self._settle(robot, log, stop)
                s0 = self._pose(robot)
                if s0 is None:
                    continue
                ux, uy, room = self._open_x_dir(s0.pose())
                ux, uy = ux * sign, uy * sign
                if self._room_ahead(s0.pose(), ux, uy) < buffer_mm + 200.0:
                    sign = -sign
                    ux, uy = -ux, -uy
                heading = math.atan2(uy, ux)
                self._rotate_to_heading(robot, heading, stop, timeout=2.5)

                s_start = self._pose(robot)
                if s_start is None:
                    continue
                p0 = (s_start.x, s_start.y)
                devs = []
                max_abs_dev = 0.0
                max_head = 0.0
                end = time.perf_counter() + max_run_s
                while time.perf_counter() < end:
                    self._check(stop)
                    s = self._pose(robot)
                    if s is not None:
                        dx, dy = s.x - p0[0], s.y - p0[1]
                        along = dx * ux + dy * uy
                        perp = dx * (-uy) + dy * ux        # signed lateral offset
                        devs.append(perp)
                        max_abs_dev = max(max_abs_dev, abs(perp))
                        max_head = max(max_head, abs(_ang_diff(s.o, heading)))
                        if along >= max_travel or \
                                self._room_ahead(s.pose(), ux, uy) < buffer_mm:
                            break
                        vx_f, w, _ = self._steer_forward(s.pose(), ux, uy, speed)
                        self._drive_forward_body(robot, vx_f, w)
                    if self._emergency_seen(robot):
                        break
                    time.sleep(self.s("poll_s"))
                self.cmd.stop_robot(robot.is_yellow, robot.robot_id)

                s_end = self._pose(robot)
                length = 0.0
                if s_end is not None:
                    length = (s_end.x - p0[0]) * ux + (s_end.y - p0[1]) * uy
                if devs and length > 100.0:
                    rms = math.sqrt(sum(d * d for d in devs) / len(devs))
                    peak_dev.append(max_abs_dev)
                    rms_dev.append(rms)
                    head_drift.append(math.degrees(max_head))
                    lengths.append(length)
                    log(f"  run {i + 1}: length={length:.0f} mm  peak_dev="
                        f"{max_abs_dev:.1f} mm  rms_dev={rms:.1f} mm  "
                        f"heading_drift={math.degrees(max_head):.1f} deg")
                else:
                    log(f"  run {i + 1}: too little motion, skipped")
                sign = -sign
                self._settle(robot, log, stop, dur=0.3)
        finally:
            restore_rate()
            self.cmd.stop_robot(robot.is_yellow, robot.robot_id)
            self._settle(robot, log, stop, dur=0.5)

        return {
            "commanded_speed_ms": speed,
            "trials": trials,
            "run_length_mm": summarize(lengths, "mm"),
            "peak_lateral_dev_mm": summarize(peak_dev, "mm"),
            "rms_lateral_dev_mm": summarize(rms_dev, "mm"),
            "heading_drift_deg": summarize(head_drift, "deg"),
        }


# ── Acceleration profile (0 -> target speed) ──────────────────────────────

class AccelProfileDiagnostic(Diagnostic):
    """Measure how quickly the robot gets up to speed. Faces forward, commands
    accel_speed from rest, and records the time and distance to reach fractions
    of the commanded speed plus the peak acceleration. Wall-safe."""

    name = "accel_profile"
    title = "Acceleration profile (0 -> target)"

    def run(self, robot, log, progress, stop) -> dict:
        if not self._wait_visible(robot, log):
            return {"error": "robot not visible"}
        speed = float(self.s("accel_speed_ms"))
        trials = max(1, int(self.s("accel_trials")))
        target_mm_s = speed * 1000.0
        buffer_mm = float(self.s("stop_buffer_mm")) + 400.0
        max_run_s = float(self.s("max_run_s"))
        max_travel = float(self.s("max_travel_mm"))
        fracs = (0.5, 0.9)

        t50, t90, d90, peak_acc, reached = [], [], [], [], 0
        sign = 1
        restore_rate = self._boost_send_rate()
        try:
            for i in range(trials):
                self._check(stop)
                progress(i / trials, f"run {i + 1}/{trials}")
                self._settle(robot, log, stop)
                s0 = self._pose(robot)
                if s0 is None:
                    continue
                ux, uy, room = self._open_x_dir(s0.pose())
                ux, uy = ux * sign, uy * sign
                if self._room_ahead(s0.pose(), ux, uy) < buffer_mm + 200.0:
                    sign = -sign
                    ux, uy = -ux, -uy
                heading = math.atan2(uy, ux)
                self._rotate_to_heading(robot, heading, stop, timeout=2.5)

                s_start = self._pose(robot)
                if s_start is None:
                    continue
                p0 = (s_start.x, s_start.y)
                t0 = time.perf_counter()
                tr = MotionTracker(0.2)
                last_fn = None
                prev_v = 0.0
                prev_t = t0
                hit = {0.5: None, 0.9: None}
                trial_peak_acc = 0.0
                end = t0 + max_run_s
                while time.perf_counter() < end:
                    self._check(stop)
                    s = self._pose(robot)
                    if s is not None:
                        if s.frame_number != last_fn:
                            last_fn = s.frame_number
                            tr.add(s.t_perf, s.x, s.y, s.o)
                            if tr.ready:
                                v = tr.speed()
                                now = s.t_perf
                                if now > prev_t:
                                    acc = (v - prev_v) / (now - prev_t)
                                    trial_peak_acc = max(trial_peak_acc, acc)
                                prev_v, prev_t = v, now
                                for f in fracs:
                                    if hit[f] is None and v >= f * target_mm_s:
                                        hit[f] = (now - t0,
                                                  math.hypot(s.x - p0[0], s.y - p0[1]))
                            dist = math.hypot(s.x - p0[0], s.y - p0[1])
                            if hit[0.9] is not None or dist >= max_travel or \
                                    self._room_ahead(s.pose(), ux, uy) < buffer_mm:
                                break
                        vx_f, w, _ = self._steer_forward(s.pose(), ux, uy, speed)
                        self._drive_forward_body(robot, vx_f, w)
                    if self._emergency_seen(robot):
                        break
                    time.sleep(self.s("poll_s"))
                self.cmd.stop_robot(robot.is_yellow, robot.robot_id)

                if hit[0.5] is not None:
                    t50.append(hit[0.5][0] * 1000.0)
                if hit[0.9] is not None:
                    reached += 1
                    t90.append(hit[0.9][0] * 1000.0)
                    d90.append(hit[0.9][1])
                if trial_peak_acc > 0:
                    peak_acc.append(trial_peak_acc)
                log(f"  run {i + 1}: t50={_ms(hit[0.5])}  t90={_ms(hit[0.9])}  "
                    f"peak_acc={trial_peak_acc / 1000.0:.2f} m/s^2")
                sign = -sign
                self._settle(robot, log, stop, dur=0.3)
        finally:
            restore_rate()
            self.cmd.stop_robot(robot.is_yellow, robot.robot_id)
            self._settle(robot, log, stop, dur=0.5)

        return {
            "commanded_speed_ms": speed,
            "trials": trials,
            "reached_90pct": reached,
            "t_to_50pct_ms": summarize(t50, "ms"),
            "t_to_90pct_ms": summarize(t90, "ms"),
            "dist_to_90pct_mm": summarize(d90, "mm"),
            "peak_accel_ms2": summarize([a / 1000.0 for a in peak_acc], "m/s^2"),
        }


# ── Heading step response (rotate & hold) ─────────────────────────────────

class HeadingHoldDiagnostic(Diagnostic):
    """Step the robot's heading to a sequence of target angles and measure how
    fast and cleanly it gets there: settle time, overshoot, and steady-state
    error over a hold window. Pure in-place rotation, so it is inherently
    wall-safe. Targets are world headings (deg) from heading_targets_deg."""

    name = "heading_hold"
    title = "Heading step response (rotate & hold)"

    def run(self, robot, log, progress, stop) -> dict:
        if not self._wait_visible(robot, log):
            return {"error": "robot not visible"}
        targets = list(self.s("heading_targets_deg"))
        tol = float(self.s("heading_tol_rad"))
        kp = float(self.s("heading_kp"))
        hold_s = float(self.s("heading_hold_s"))

        settle_ms, overshoot_deg, steady_deg, missed = [], [], [], 0
        restore_rate = self._boost_send_rate()
        try:
            for i, tdeg in enumerate(targets):
                self._check(stop)
                progress(i / max(1, len(targets)), f"target {i + 1}/{len(targets)}")
                target_o = math.radians(float(tdeg))
                self._settle(robot, log, stop, dur=0.3)

                s0 = self._pose(robot)
                if s0 is None:
                    missed += 1
                    log(f"  target {tdeg:.0f} deg: robot not visible")
                    continue
                # sign of the initial error = the direction we're rotating; an
                # overshoot is the robot swinging PAST the target (opposite sign)
                approach = 1.0 if _ang_diff(target_o, s0.o) >= 0 else -1.0

                t0 = time.perf_counter()
                settled_at = None
                peak_past = 0.0
                hold_errs = []
                # one continuous loop: rotate toward the target, note when it
                # first enters the tolerance band (settle time), keep holding for
                # hold_s while recording steady error, and track the furthest it
                # swings beyond the target (overshoot) throughout.
                end = t0 + max(2.5, float(self.s("max_run_s"))) + hold_s
                while time.perf_counter() < end:
                    self._check(stop)
                    s = self._pose(robot)
                    if s is not None:
                        err = _ang_diff(target_o, s.o)
                        peak_past = max(peak_past, -err * approach)
                        if settled_at is None and abs(err) <= tol:
                            settled_at = time.perf_counter() - t0
                        if settled_at is not None:
                            hold_errs.append(abs(err))
                            if (time.perf_counter() - t0) - settled_at >= hold_s:
                                break
                        w = safety.clamp(kp * err, -self.lim.max_w, self.lim.max_w)
                        self.cmd.set_velocity(robot.is_yellow, robot.robot_id,
                                              vx=0.0, vy=0.0, w=w,
                                              frame="world", safe=True)
                    time.sleep(self.s("poll_s"))
                self.cmd.stop_robot(robot.is_yellow, robot.robot_id)

                if settled_at is None:
                    missed += 1
                    log(f"  target {tdeg:.0f} deg: did NOT settle within timeout")
                    continue
                settle = settled_at * 1000.0
                over = math.degrees(max(0.0, peak_past))
                steady = math.degrees(sum(hold_errs) / len(hold_errs)) \
                    if hold_errs else 0.0
                settle_ms.append(settle)
                overshoot_deg.append(over)
                steady_deg.append(steady)
                log(f"  target {tdeg:.0f} deg: settle={settle:.0f} ms  "
                    f"overshoot={over:.1f} deg  steady_err={steady:.2f} deg")
        finally:
            restore_rate()
            self.cmd.stop_robot(robot.is_yellow, robot.robot_id)

        return {
            "targets_deg": targets,
            "settle_ms": summarize(settle_ms, "ms"),
            "overshoot_deg": summarize(overshoot_deg, "deg"),
            "steady_error_deg": summarize(steady_deg, "deg"),
            "missed_targets": missed,
        }


# ── Go-to-point (position control) ────────────────────────────────────────

class WaypointDiagnostic(Diagnostic):
    """Drive the robot to a point and stop on it, there and back. Measures how
    accurately it arrives: final position error, overshoot past the target, and
    settle time. Faces forward while cruising and eases off as it nears the
    target (proportional approach). Wall-safe."""

    name = "waypoint"
    title = "Go-to-point (position control)"

    def run(self, robot, log, progress, stop) -> dict:
        if not self._wait_visible(robot, log):
            return {"error": "robot not visible"}
        speed = float(self.s("waypoint_speed_ms"))
        tol = float(self.s("waypoint_tol_mm"))
        cycles = max(1, int(self.s("waypoint_cycles")))
        max_run_s = float(self.s("max_run_s"))

        s0 = self._pose(robot)
        if s0 is None:
            return {"error": "robot not visible"}
        # two points along the long axis, inside the arena with margin
        xlo, xhi, ylo, yhi = self.lim.arena_bounds()
        cy = self.lim.arena_cy
        margin = float(self.s("stop_buffer_mm")) + 300.0
        ax = xlo + margin
        bx = xhi - margin
        if bx - ax < 400.0:
            return {"error": "test zone too short for a go-to-point run "
                             "(need ~1 m of length) -- widen the test zone"}
        pa, pb = (ax, cy), (bx, cy)
        log(f"  waypoints A=({pa[0]:.0f},{pa[1]:.0f}) B=({pb[0]:.0f},{pb[1]:.0f})  "
            f"tol={tol:.0f} mm")

        final_err, overshoot, settle_ms, misses = [], [], [], 0
        restore_rate = self._boost_send_rate()
        try:
            legs = [pb, pa] * cycles
            for i, target in enumerate(legs):
                self._check(stop)
                progress(i / len(legs), f"leg {i + 1}/{len(legs)}")
                t0 = time.perf_counter()
                arrived_at = None
                min_dist = 1e9
                closed_in = False
                end = t0 + max_run_s
                while time.perf_counter() < end:
                    self._check(stop)
                    s = self._pose(robot)
                    if s is not None:
                        dx, dy = target[0] - s.x, target[1] - s.y
                        dist = math.hypot(dx, dy)
                        min_dist = min(min_dist, dist)
                        if dist <= tol:
                            arrived_at = time.perf_counter()
                            break
                        if dist <= 2 * tol:
                            closed_in = True
                        ux, uy = dx / dist, dy / dist
                        # ease off within ~1.5 * (stopping room) of the target
                        approach = min(1.0, dist / max(200.0, 3.0 * tol))
                        vx_f, w, _ = self._steer_forward(
                            s.pose(), ux, uy, speed * approach)
                        self._drive_forward_body(robot, vx_f, w)
                    if self._emergency_seen(robot):
                        break
                    time.sleep(self.s("poll_s"))
                self.cmd.stop_robot(robot.is_yellow, robot.robot_id)

                # let it coast/settle briefly, then read the final error
                self._settle(robot, log, stop, dur=0.4)
                s_end = self._pose(robot)
                if s_end is not None:
                    ferr = math.hypot(target[0] - s_end.x, target[1] - s_end.y)
                    final_err.append(ferr)
                    # overshoot = how far the closest approach undershot vs how
                    # far the final rest is past target (settle wobble)
                    overshoot.append(max(0.0, ferr - min_dist))
                    if arrived_at is not None:
                        settle_ms.append((arrived_at - t0) * 1000.0)
                    else:
                        misses += 1
                    tag = "" if arrived_at is not None else "  [did not reach tol]"
                    log(f"  leg {i + 1}: final_err={ferr:.1f} mm  "
                        f"closest={min_dist:.1f} mm{tag}")
        finally:
            restore_rate()
            self.cmd.stop_robot(robot.is_yellow, robot.robot_id)
            self._settle(robot, log, stop, dur=0.4)

        return {
            "waypoint_a": [round(pa[0], 0), round(pa[1], 0)],
            "waypoint_b": [round(pb[0], 0), round(pb[1], 0)],
            "tolerance_mm": tol,
            "final_error_mm": summarize(final_err, "mm"),
            "overshoot_mm": summarize(overshoot, "mm"),
            "settle_ms": summarize(settle_ms, "ms"),
            "missed_legs": misses,
        }


def _ms(hit) -> str:
    return "—" if hit is None else f"{hit[0] * 1000.0:.0f} ms"


# ── Registry ──────────────────────────────────────────────────────────────

ALL_DIAGNOSTICS = [
    VisionHealthDiagnostic,
    TelemetryHealthDiagnostic,
    CommandLatencyDiagnostic,
    StopLatencyDiagnostic,
    SpeedScaleDiagnostic,
    AngularDiagnostic,
]

# High-speed / motion tests. Deliberately NOT in ALL_DIAGNOSTICS, so the
# routine Diagnostics-tab battery / Full Sweep / self-test / per-robot
# calibration never silently pick up a full-speed, face-forward run. Exposed by
# name (engine.run_diagnostic(...)) and via their own GUI buttons.
MOTION_DIAGNOSTICS = [
    StraightLineDiagnostic,
    SpeedShuttleDiagnostic,
    AccelProfileDiagnostic,
    HeadingHoldDiagnostic,
    WaypointDiagnostic,
]

# Ordered list of motion-test names for the GUI (title looked up via BY_NAME).
MOTION_TESTS = [d.name for d in MOTION_DIAGNOSTICS]

BY_NAME = {d.name: d for d in ALL_DIAGNOSTICS}
BY_NAME[SpinCalibrationDiagnostic.name] = SpinCalibrationDiagnostic
for _d in MOTION_DIAGNOSTICS:
    BY_NAME[_d.name] = _d

# Canonical test order for a full per-robot calibration (used by the
# Auto-Calibrate tab and engine.run_auto_calibration).
CALIBRATION_TESTS = [
    "vision_health", "telemetry_health", "command_latency",
    "stop_latency", "speed_scale", "angular", "spin",
]
