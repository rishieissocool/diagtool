"""
commander.py — wall-safe, continuous command streamer.

The robot stops its wheels if it doesn't receive a command within ~30 ms, so
DiagTool (like TeamControl's dispatcher) must *stream* commands continuously
while a test runs. This Commander does that on a background thread, and on
every send:

  1. looks up the robot's latest vision pose,
  2. runs the velocity through safety.py (clamp to MAX_SPEED/MAX_W,
     wall-brake, outward-guard),
  3. builds a RobotCommand and sends it with TeamControl's Sender — the exact
     same wire format the robots already understand,
  4. records the *actually sent* velocity so diagnostics measure scale against
     what was transmitted, not merely what was requested.

In *safe* mode (default) a robot whose pose is unknown is sent a zero command —
it will never drive a robot it can't see (that's how you hit a wall) — but a
brief vision dropout is bridged with the last good pose (`drive_grace`) so a
single stale frame no longer kills the whole command stream. In *direct* mode
(`safe=False`) the command is streamed straight to the robot's ip:port exactly
like the real dispatcher, with no vision gate, for testing the command link
itself. Every send is counted (sends / no-pose / safety / errors) so a "no
motion" result can be traced to its cause.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

from . import bridge, safety
from .metrics import RateMeter
from .sources import VisionSource


@dataclass
class Target:
    is_yellow: bool
    robot_id: int
    ip: str
    port: int
    enabled: bool = False
    frame: str = "world"        # "world" | "body"
    safe: bool = True           # False -> send straight through (like the
                                # real dispatcher), no vision gate / wall guard
    vx: float = 0.0
    vy: float = 0.0
    w: float = 0.0
    kick: int = 0
    dribble: int = 0
    # readback of what was actually transmitted last tick
    last_sent: tuple[float, float, float] = (0.0, 0.0, 0.0)
    last_blocked: bool = False
    rate: RateMeter = field(default_factory=lambda: RateMeter(window=120))
    # last known-good pose, for grace driving across brief vision dropouts
    _last_pose: tuple | None = None
    _last_pose_t: float = 0.0
    last_pose_age: float | None = None
    # per-target send instrumentation (so "no motion" is diagnosable)
    sends: int = 0
    send_errors: int = 0
    zeroed_no_pose: int = 0
    zeroed_safety: int = 0
    last_error: str | None = None


class Commander:
    def __init__(self, vision: VisionSource, sender_ip: str | None = None,
                 send_hz: float = 50.0, pose_max_age: float = 0.4,
                 drive_grace: float = 0.5):
        Sender, RobotCommand = bridge.get_command_classes()
        self._RobotCommand = RobotCommand
        self._sender = Sender(device_ip=sender_ip) if sender_ip else Sender(device_ip=None)
        self._vision = vision
        self._period = 1.0 / send_hz
        self._pose_max_age = pose_max_age
        # keep driving on the last good pose for this long after vision goes
        # stale, instead of instantly zeroing the command stream
        self._drive_grace = max(0.0, float(drive_grace))
        self._lim = safety.limits()

        self._targets: dict[tuple[bool, int], Target] = {}
        self._lock = threading.Lock()
        self._run = threading.Event()
        self._thread: threading.Thread | None = None

    # -- lifecycle --
    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._run.set()
        self._thread = threading.Thread(
            target=self._loop, name="Commander", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self.stop_all()
        # flush a few explicit zero commands so robots halt promptly
        for _ in range(5):
            self._tick()
            time.sleep(0.01)
        self._run.clear()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    # -- target management --
    def register(self, is_yellow: bool, robot_id: int, ip: str, port: int) -> Target:
        key = (bool(is_yellow), int(robot_id))
        with self._lock:
            t = Target(bool(is_yellow), int(robot_id), ip, int(port))
            self._targets[key] = t
            return t

    def enable(self, is_yellow: bool, robot_id: int, enabled: bool = True) -> None:
        with self._lock:
            t = self._targets.get((bool(is_yellow), int(robot_id)))
            if t:
                t.enabled = enabled
                if not enabled:
                    t.vx = t.vy = t.w = 0.0
                    t.kick = t.dribble = 0

    def set_velocity(self, is_yellow: bool, robot_id: int,
                     vx: float = 0.0, vy: float = 0.0, w: float = 0.0,
                     frame: str = "world", kick: int = 0, dribble: int = 0,
                     safe: bool = True) -> None:
        with self._lock:
            t = self._targets.get((bool(is_yellow), int(robot_id)))
            if t:
                t.frame = frame
                t.safe = bool(safe)
                t.vx, t.vy, t.w = float(vx), float(vy), float(w)
                t.kick, t.dribble = int(kick), int(dribble)
                t.enabled = True

    def stop_robot(self, is_yellow: bool, robot_id: int) -> None:
        self.set_velocity(is_yellow, robot_id, 0, 0, 0)

    def stop_all(self) -> None:
        with self._lock:
            for t in self._targets.values():
                t.vx = t.vy = t.w = 0.0
                t.kick = t.dribble = 0

    def last_sent(self, is_yellow: bool, robot_id: int):
        with self._lock:
            t = self._targets.get((bool(is_yellow), int(robot_id)))
            return t.last_sent if t else None

    def stats(self, is_yellow: bool, robot_id: int) -> dict:
        """Snapshot of what the streamer is actually doing for this robot.

        Lets a diagnostic explain a 'no motion' result: did the PC send real
        commands (so the robot is the problem), or did it zero them for lack of
        a vision pose / safety, or fail to send at all (bad IP / unreachable)?
        """
        with self._lock:
            t = self._targets.get((bool(is_yellow), int(robot_id)))
            if not t:
                return {}
            return {
                "sends": t.sends,
                "send_errors": t.send_errors,
                "zeroed_no_pose": t.zeroed_no_pose,
                "zeroed_safety": t.zeroed_safety,
                "last_sent": t.last_sent,
                "last_blocked": t.last_blocked,
                "last_error": t.last_error,
                "last_pose_age": t.last_pose_age,
                "send_rate_hz": round(t.rate.rate_hz, 1),
                "ip": t.ip,
                "port": t.port,
            }

    def reset_stats(self, is_yellow: bool, robot_id: int) -> None:
        with self._lock:
            t = self._targets.get((bool(is_yellow), int(robot_id)))
            if t:
                t.sends = t.send_errors = 0
                t.zeroed_no_pose = t.zeroed_safety = 0
                t.last_error = None

    # -- thread body --
    def _loop(self) -> None:
        next_t = time.perf_counter()
        while self._run.is_set():
            self._tick()
            next_t += self._period
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.perf_counter()   # fell behind; resync

    def _tick(self) -> None:
        with self._lock:
            targets = [t for t in self._targets.values() if t.enabled]
        for t in targets:
            self._send_one(t)

    def _send_one(self, t: Target) -> None:
        now = time.perf_counter()
        sample = self._vision.get_pose_sample(t.is_yellow, t.robot_id, max_age=None)
        if sample is not None:
            pose = sample.pose()
            age = max(0.0, now - sample.t_perf)
            with self._lock:
                t._last_pose = pose
                t._last_pose_t = sample.t_perf
                t.last_pose_age = age
            fresh = age <= self._pose_max_age
        else:
            with self._lock:
                t.last_pose_age = None
            fresh = False

        # Pose used for the safety transform: the fresh one, or — across a brief
        # vision dropout — the last good one (grace), else none.
        if fresh:
            drive_pose = t._last_pose
        elif t._last_pose is not None and (now - t._last_pose_t) <= self._drive_grace:
            drive_pose = t._last_pose
        else:
            drive_pose = None

        want_motion = bool(t.vx or t.vy or t.w)
        reason = None
        blocked = False

        if not t.safe:
            # DIRECT mode — send the command straight to the robot exactly like
            # the real dispatcher (no vision gate, no wall guard). Magnitude is
            # still capped to MAX_SPEED / MAX_W so a typo can't launch it.
            vx_r, vy_r, w = safety.clamp_velocity(t.vx, t.vy, t.w, self._lim)
        elif drive_pose is not None:
            if t.frame == "body":
                vx_r, vy_r, w, blocked = safety.safe_body_velocity(
                    drive_pose, t.vx, t.vy, t.w, self._lim)
            else:
                vx_r, vy_r, w, blocked = safety.safe_world_velocity(
                    drive_pose, t.vx, t.vy, t.w, self._lim)
            if blocked and want_motion:
                reason = "safety"
        else:
            # Safe mode with no usable pose -> never drive a robot we can't see.
            vx_r = vy_r = w = 0.0
            blocked = True
            if want_motion:
                reason = "no_pose"

        cmd = self._RobotCommand(
            robot_id=t.robot_id, vx=vx_r, vy=vy_r, w=w,
            kick=t.kick, dribble=t.dribble, isYellow=t.is_yellow)
        try:
            self._sender.send(cmd, t.ip, t.port)
            sent_ok, err = True, None
        except Exception as e:                      # bad address / unreachable
            sent_ok, err = False, f"{type(e).__name__}: {e}"

        with self._lock:
            t.last_sent = (vx_r, vy_r, w)
            t.last_blocked = blocked
            if sent_ok:
                t.sends += 1
            else:
                t.send_errors += 1
                t.last_error = err
            if reason == "no_pose":
                t.zeroed_no_pose += 1
            elif reason == "safety":
                t.zeroed_safety += 1
            t.rate.tick()
