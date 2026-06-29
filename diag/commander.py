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

Whenever the robot's pose is known, EVERY command — safe or direct — is run
through the arena brake + hard stop, so a robot we can see can never be driven
out of bounds or into a wall. In *safe* mode (default) a robot whose pose is
unknown is sent a zero command (it will never drive a robot it can't see) — but
a brief vision dropout is bridged with the last good pose (`drive_grace`). In
*direct* mode (`safe=False`) the command is streamed to the robot's ip:port like
the real dispatcher even with no pose, for testing the command link — capped to
a slow blind speed since it can't be arena-guarded. Every send is counted
(sends / no-pose / safety / errors) so a "no motion" result can be traced.
"""

from __future__ import annotations

import math
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
    mode: str = "real"          # "real" -> RobotCommand to ip:port (RobotFramework)
                                # "grsim" -> grSim protobuf to the grSim address
    grsim_id: int | None = None  # id used inside the grSim packet (defaults to robot_id)
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
    zeroed_emergency: int = 0
    last_error: str | None = None
    # measured velocity from vision (mm/s, lightly smoothed) — drives the
    # predictive emergency stop, which is immune to speed_scale / drift bugs
    meas_vx: float = 0.0
    meas_vy: float = 0.0
    _vref_x: float = 0.0
    _vref_y: float = 0.0
    _vref_t: float = 0.0
    _vref_frame: int = -1


class Commander:
    def __init__(self, vision: VisionSource, sender_ip: str | None = None,
                 send_hz: float = 50.0, pose_max_age: float = 0.4,
                 drive_grace: float = 0.5, lim: safety.Limits | None = None,
                 blind_speed: float = 0.25, safety_reaction_s: float = 0.6,
                 safety_decel_mm_s2: float = 250.0, safety_factor: float = 1.5,
                 safety_max_speed_mm_s: float = 600.0,
                 grsim_addr: tuple[str, int] | None = None):
        Sender, RobotCommand = bridge.get_command_classes()
        self._RobotCommand = RobotCommand
        self._sender = Sender(device_ip=sender_ip) if sender_ip else Sender(device_ip=None)
        # grSim command path (built lazily; needs protobuf). When set, targets in
        # "grsim" mode are driven by sending a grSim packet here instead of a
        # RobotCommand to their ip:port.
        self._grsim_addr = (str(grsim_addr[0]), int(grsim_addr[1])) if grsim_addr else None
        self._grsim_factory = None
        self._grsim_error: str | None = None
        self._vision = vision
        self._period = 1.0 / send_hz
        self._pose_max_age = pose_max_age
        # keep driving on the last good pose for this long after vision goes
        # stale, instead of instantly zeroing the command stream
        self._drive_grace = max(0.0, float(drive_grace))
        # speed cap for direct sends when we have NO pose (can't arena-guard)
        self._blind_speed = max(0.0, float(blind_speed))
        # arena-aware limits (shared with the engine/diagnostics)
        self._lim = lim or safety.limits()
        # predictive emergency-stop model (uses MEASURED vision velocity):
        #   stop_dist(v) = factor * (reaction * v + v^2 / (2*decel))
        # tuned conservative from real coast/latency measurements.
        self._safety_reaction = max(0.0, float(safety_reaction_s))
        self._safety_decel = max(1.0, float(safety_decel_mm_s2))
        self._safety_factor = max(1.0, float(safety_factor))
        self._safety_max_speed = max(0.0, float(safety_max_speed_mm_s))

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

    def set_limits(self, lim: safety.Limits) -> None:
        """Swap in new arena limits (e.g. after vision reports the field size).
        Takes effect on the next send."""
        self._lim = lim

    def set_grsim_addr(self, addr: tuple[str, int] | None) -> None:
        """Point the grSim command path at a new (ip, port) (e.g. after reload)."""
        self._grsim_addr = (str(addr[0]), int(addr[1])) if addr else None

    def _grsim_packet_factory(self):
        """Lazily import grSimPacketFactory; cache the result (or the error)."""
        if self._grsim_factory is None and self._grsim_error is None:
            try:
                self._grsim_factory = bridge.get_grsim_factory()
            except Exception as e:
                self._grsim_error = f"{type(e).__name__}: {e}"
        return self._grsim_factory

    # -- target management --
    def register(self, is_yellow: bool, robot_id: int, ip: str, port: int,
                 mode: str = "real", grsim_id: int | None = None) -> Target:
        key = (bool(is_yellow), int(robot_id))
        with self._lock:
            t = Target(bool(is_yellow), int(robot_id), ip, int(port),
                       mode=str(mode), grsim_id=grsim_id)
            self._targets[key] = t
            return t

    def set_target_mode(self, is_yellow: bool, robot_id: int, mode: str) -> None:
        """Switch a robot between 'real' (RobotCommand→ip:port) and 'grsim'."""
        with self._lock:
            t = self._targets.get((bool(is_yellow), int(robot_id)))
            if t:
                t.mode = str(mode)

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
                "zeroed_emergency": t.zeroed_emergency,
                "meas_speed_mm_s": round(math.hypot(t.meas_vx, t.meas_vy), 1),
                "last_sent": t.last_sent,
                "last_blocked": t.last_blocked,
                "last_error": t.last_error,
                "last_pose_age": t.last_pose_age,
                "send_rate_hz": round(t.rate.rate_hz, 1),
                "ip": t.ip,
                "port": t.port,
                "mode": t.mode,
            }

    def reset_stats(self, is_yellow: bool, robot_id: int) -> None:
        with self._lock:
            t = self._targets.get((bool(is_yellow), int(robot_id)))
            if t:
                t.sends = t.send_errors = 0
                t.zeroed_no_pose = t.zeroed_safety = t.zeroed_emergency = 0
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

    def _stop_distance(self, speed_mm_s: float) -> float:
        """Conservative distance (mm) to fully stop from `speed_mm_s`, including
        command-latency reaction and coast (decel model), times a safety factor."""
        coast = (speed_mm_s * speed_mm_s) / (2.0 * self._safety_decel)
        return self._safety_factor * (self._safety_reaction * speed_mm_s + coast)

    def _emergency_stop(self, pose, vx: float, vy: float) -> bool:
        """True if the robot's MEASURED motion is unsafe and must be cut now.

        Independent of the commanded velocity, so it catches a robot that moves
        far faster than commanded (speed_scale bug) or drifts sideways: if it is
        moving too fast outright, or its real stopping distance would carry it
        past the arena edge on either axis, stop.
        """
        lim = self._lim
        if self._safety_max_speed > 0 and math.hypot(vx, vy) > self._safety_max_speed:
            return True
        x, y = pose[0], pose[1]
        for p, v, bound in ((x, vx, lim.arena_half_len), (y, vy, lim.arena_half_wid)):
            if v > 5.0:
                room = bound - p
            elif v < -5.0:
                room = bound + p
            else:
                continue
            if room <= 0.0 or self._stop_distance(abs(v)) >= room:
                return True
        return False

    def _update_velocity(self, t: Target, sample) -> None:
        """Estimate the robot's world-frame velocity (mm/s) from vision frames."""
        fn = int(sample.frame_number)
        if fn == t._vref_frame:
            return
        if t._vref_t > 0.0:
            dt = sample.t_perf - t._vref_t
            if dt > 1e-3:
                nvx = (sample.x - t._vref_x) / dt
                nvy = (sample.y - t._vref_y) / dt
                a = 0.5                           # light smoothing
                t.meas_vx = a * nvx + (1 - a) * t.meas_vx
                t.meas_vy = a * nvy + (1 - a) * t.meas_vy
        t._vref_x, t._vref_y = sample.x, sample.y
        t._vref_t, t._vref_frame = sample.t_perf, fn

    def _send_one(self, t: Target) -> None:
        now = time.perf_counter()
        sample = self._vision.get_pose_sample(t.is_yellow, t.robot_id, max_age=None)
        if sample is not None:
            pose = sample.pose()
            age = max(0.0, now - sample.t_perf)
            with self._lock:
                self._update_velocity(t, sample)
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

        if drive_pose is not None:
            # Vision available -> ALWAYS apply the arena brake + hard stop, even
            # in direct/jog mode, so a robot we can see is never driven out of
            # bounds or into a wall.
            if t.frame == "body":
                vx_r, vy_r, w, blocked = safety.safe_body_velocity(
                    drive_pose, t.vx, t.vy, t.w, self._lim)
            else:
                vx_r, vy_r, w, blocked = safety.safe_world_velocity(
                    drive_pose, t.vx, t.vy, t.w, self._lim)
            if blocked and want_motion:
                reason = "safety"
            # Predictive emergency stop on MEASURED motion — overrides the above
            # and is what actually saves the robot when it moves far faster than
            # commanded or drifts toward a wall.
            if self._emergency_stop(drive_pose, t.meas_vx, t.meas_vy):
                vx_r = vy_r = w = 0.0
                blocked = True
                reason = "emergency"
        elif not t.safe:
            # DIRECT mode, no pose: stream the command (comms/link test) but cap
            # it to a slow blind speed — we can't arena-guard what we can't see.
            vx_r, vy_r, w = safety.clamp_velocity(t.vx, t.vy, t.w, self._lim)
            spd = math.hypot(vx_r, vy_r)
            if self._blind_speed > 0.0 and spd > self._blind_speed:
                s = self._blind_speed / spd
                vx_r, vy_r = vx_r * s, vy_r * s
        else:
            # Safe mode with no usable pose -> never drive a robot we can't see.
            vx_r = vy_r = w = 0.0
            blocked = True
            if want_motion:
                reason = "no_pose"

        # Transmit. Two wire formats, same body-frame velocity:
        #   * "grsim" -> a grSim protobuf packet to the grSim command address
        #     (the simulator's robots understand this),
        #   * "real"  -> a RobotCommand string to the robot's ip:port (exactly
        #     what RobotFramework parses).
        try:
            if t.mode == "grsim" and self._grsim_addr is not None:
                factory = self._grsim_packet_factory()
                if factory is None:
                    raise RuntimeError(self._grsim_error or "grSim factory unavailable")
                pkt = factory.robot_command(
                    robot_id=(t.grsim_id if t.grsim_id is not None else t.robot_id),
                    vx=vx_r, vy=vy_r, w=w,
                    kick=bool(t.kick), dribble=bool(t.dribble),
                    isYellow=t.is_yellow)
                self._sender.send(pkt.SerializeToString(),
                                  self._grsim_addr[0], self._grsim_addr[1])
            else:
                cmd = self._RobotCommand(
                    robot_id=t.robot_id, vx=vx_r, vy=vy_r, w=w,
                    kick=t.kick, dribble=t.dribble, isYellow=t.is_yellow)
                self._sender.send(cmd, t.ip, t.port)
            sent_ok, err = True, None
        except Exception as e:                      # bad address / unreachable / no protobuf
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
            elif reason == "emergency":
                t.zeroed_emergency += 1
            t.rate.tick()
