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

If a robot's pose is unknown/stale, the Commander sends a zero command (it
will never drive a robot it can't see — that's how you hit a wall).
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
    vx: float = 0.0
    vy: float = 0.0
    w: float = 0.0
    kick: int = 0
    dribble: int = 0
    # readback of what was actually transmitted last tick
    last_sent: tuple[float, float, float] = (0.0, 0.0, 0.0)
    last_blocked: bool = False
    rate: RateMeter = field(default_factory=lambda: RateMeter(window=120))


class Commander:
    def __init__(self, vision: VisionSource, sender_ip: str | None = None,
                 send_hz: float = 50.0, pose_max_age: float = 0.4):
        Sender, RobotCommand = bridge.get_command_classes()
        self._RobotCommand = RobotCommand
        self._sender = Sender(device_ip=sender_ip) if sender_ip else Sender(device_ip=None)
        self._vision = vision
        self._period = 1.0 / send_hz
        self._pose_max_age = pose_max_age
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
                     frame: str = "world", kick: int = 0, dribble: int = 0) -> None:
        with self._lock:
            t = self._targets.get((bool(is_yellow), int(robot_id)))
            if t:
                t.frame = frame
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
        pose = self._vision.get_pose(t.is_yellow, t.robot_id,
                                     max_age=self._pose_max_age)
        if pose is None:
            # Can't see the robot -> never drive it. Send a stop.
            vx_r = vy_r = w = 0.0
            blocked = True
        else:
            if t.frame == "body":
                vx_r, vy_r, w, blocked = safety.safe_body_velocity(
                    pose, t.vx, t.vy, t.w, self._lim)
            else:
                vx_r, vy_r, w, blocked = safety.safe_world_velocity(
                    pose, t.vx, t.vy, t.w, self._lim)

        cmd = self._RobotCommand(
            robot_id=t.robot_id, vx=vx_r, vy=vy_r, w=w,
            kick=t.kick, dribble=t.dribble, isYellow=t.is_yellow)
        try:
            self._sender.send(cmd, t.ip, t.port)
        except Exception:
            pass
        with self._lock:
            t.last_sent = (vx_r, vy_r, w)
            t.last_blocked = blocked
            t.rate.tick()
