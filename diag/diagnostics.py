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
    "latency_trials": 6,
    "stop_trials": 5,
    "angular_trials": 4,
    "speed_passes": 4,
    "test_speed_ms": 0.30,      # commanded linear speed for motion tests
    "test_w_rads": 0.25,        # commanded angular speed for rotation test
    "accel_settle_s": 0.5,      # ignore this much accel before steady measure
    "stop_buffer_mm": 350.0,    # stop a run this far before the safety margin
    "max_run_s": 6.0,           # hard cap on any single run
    "health_window_s": 6.0,     # observation window for health tests
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

    def _dir_to_open(self, pose):
        """World unit vector pointing into open field (toward centre)."""
        x, y, _ = pose
        d = math.hypot(x, y)
        if d < 250:
            return (1.0, 0.0)   # already central; pick +x (most room)
        return (-x / d, -y / d)

    def _room_ahead(self, pose, ux, uy) -> float:
        x, y, _ = pose
        xs = self.lim.half_len - self.lim.safe_margin
        ys = self.lim.half_wid - self.lim.safe_margin
        ts = []
        if ux > 1e-6:
            ts.append((xs - x) / ux)
        elif ux < -1e-6:
            ts.append((-xs - x) / ux)
        if uy > 1e-6:
            ts.append((ys - y) / uy)
        elif uy < -1e-6:
            ts.append((-ys - y) / uy)
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

                t_cmd = time.perf_counter()
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
                    log(f"  trial {i + 1}: NO MOTION within timeout")
        finally:
            self._settle(robot, log, stop, dur=0.5)

        res = {
            "latency_ms": summarize(lat_ms, "ms"),
            "trials": trials,
            "no_motion_trials": no_motion,
            "commanded_speed_ms": speed,
            "note": ("Includes vision latency; subtract vision_health "
                     "sent_latency to approximate actuation-only delay."),
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
                ux, uy = self._dir_to_open(s0.pose())
                if self._room_ahead(s0.pose(), ux, uy) < self.s("stop_buffer_mm") + 400:
                    ux, uy = -ux, -uy  # not enough room ahead; go the other way
                self._drive_world(robot, ux * speed, uy * speed)

                tr = MotionTracker(self.s("vel_window_s"))

                def is_moving(s, t):
                    return t is not None and t.ready and t.speed() >= moving

                s_mv, ok = self._poll_until(robot, is_moving,
                                            timeout=self.s("max_run_s"),
                                            stop=stop, tracker=tr)
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

                # drive until near margin or time cap, sampling sent magnitude
                end = time.perf_counter() + self.s("max_run_s")
                s_last = s_start
                while time.perf_counter() < end:
                    self._check(stop)
                    s = self._pose(robot)
                    if s is not None:
                        s_last = s
                        if self._room_ahead(s.pose(), ux, uy) < self.s("stop_buffer_mm"):
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

                t_cmd = time.perf_counter()
                self.cmd.set_velocity(robot.is_yellow, robot.robot_id,
                                      vx=0, vy=0, w=w_target, frame="world")

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
                    log(f"  trial {i + 1}: NO ROTATION within timeout")
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
        }


# ── Registry ──────────────────────────────────────────────────────────────

ALL_DIAGNOSTICS = [
    VisionHealthDiagnostic,
    TelemetryHealthDiagnostic,
    CommandLatencyDiagnostic,
    StopLatencyDiagnostic,
    SpeedScaleDiagnostic,
    AngularDiagnostic,
]

BY_NAME = {d.name: d for d in ALL_DIAGNOSTICS}
