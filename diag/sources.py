"""
sources.py — live, threaded data sources.

VisionSource     : joins the SSL vision multicast (reusing TeamControl's
                   Vision socket + protobuf decode) and keeps the latest pose
                   per robot, plus frame-rate / jitter / drop / vision-latency
                   stats.

TelemetrySource  : binds the onboard-telemetry UDP port (50513) and parses
                   each packet with TeamControl's own parse_packet, while
                   timestamping every packet so we can measure the telemetry
                   rate (currently ~1 Hz) and its age — a prime delay suspect.

Both are start()/stop() daemons that never raise out of their thread; failures
are captured in `.error` and surfaced through `status()`.
"""

from __future__ import annotations

import socket
import threading
import time
from dataclasses import dataclass, field

from . import bridge
from .metrics import RateMeter, summarize


# ── Vision ────────────────────────────────────────────────────────────────

@dataclass
class PoseSample:
    x: float                # mm, world frame
    y: float                # mm
    o: float                # rad, orientation
    confidence: float
    frame_number: int
    t_perf: float           # local perf_counter() at receive
    t_wall: float           # local time.time() at receive
    t_capture: float        # vision-reported capture unix time (s)
    t_sent: float           # vision-reported sent unix time (s)

    def pose(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.o)


class VisionSource:
    """Threaded SSL-vision receiver with per-robot pose + timing stats."""

    def __init__(self, port: int = 10006, real_life_cameras: int = 1):
        self.port = port
        self.cameras = real_life_cameras
        self._run = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

        self._poses: dict[tuple[bool, int], PoseSample] = {}
        self._frames = RateMeter(window=480)
        self._last_frame_no: int | None = None
        self._drops = 0
        self._dupes = 0
        self._field_length: int | None = None   # mm, from SSL-Vision geometry
        self._field_width: int | None = None     # mm
        self._sent_latency: list[float] = []      # ms, recv_wall - t_sent
        self._capture_latency: list[float] = []    # ms, recv_wall - t_capture
        self.error: str | None = None
        self._recv = None

    # -- lifecycle --
    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._run.set()
        self._thread = threading.Thread(
            target=self._loop, name="VisionSource", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._run.clear()
        if self._recv is not None:
            try:
                self._recv.close()
            except Exception:
                pass
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    # -- thread body --
    def _loop(self) -> None:
        try:
            Vision, _Frame, _Robot = bridge.get_vision_classes()
        except bridge.MissingDependency as e:
            self.error = str(e)
            return
        try:
            self._recv = Vision(is_running=self._run, port=self.port)
        except Exception as e:  # multicast join failed, etc.
            self.error = f"vision socket init failed: {e}"
            return

        while self._run.is_set():
            try:
                pkt = self._recv.listen()
            except Exception as e:
                self.error = f"vision listen error: {e}"
                time.sleep(0.05)
                continue
            if pkt is None:
                continue
            try:
                if pkt.HasField("detection"):
                    self._on_detection(pkt.detection)
                if pkt.HasField("geometry"):
                    self._on_geometry(pkt.geometry)
            except Exception:
                # malformed packet — ignore, keep listening
                continue

    def _on_geometry(self, geo) -> None:
        f = getattr(geo, "field", None)
        if f is None:
            return
        fl = int(getattr(f, "field_length", 0) or 0)
        fw = int(getattr(f, "field_width", 0) or 0)
        if fl > 0 and fw > 0:
            with self._lock:
                self._field_length = fl
                self._field_width = fw

    def _on_detection(self, det) -> None:
        t_perf = time.perf_counter()
        t_wall = time.time()
        frame_no = int(getattr(det, "frame_number", 0))
        t_capture = float(getattr(det, "t_capture", 0.0) or 0.0)
        t_sent = float(getattr(det, "t_sent", 0.0) or 0.0)

        with self._lock:
            self._frames.tick(t_perf)

            if self._last_frame_no is not None:
                gap = frame_no - self._last_frame_no
                if gap > 1:
                    self._drops += (gap - 1)
                elif gap == 0:
                    self._dupes += 1
            self._last_frame_no = frame_no

            if t_sent > 0:
                self._sent_latency.append((t_wall - t_sent) * 1000.0)
                if len(self._sent_latency) > 480:
                    self._sent_latency.pop(0)
            if t_capture > 0:
                self._capture_latency.append((t_wall - t_capture) * 1000.0)
                if len(self._capture_latency) > 480:
                    self._capture_latency.pop(0)

            for is_yellow, robots in ((True, det.robots_yellow),
                                      (False, det.robots_blue)):
                for r in robots:
                    self._poses[(is_yellow, int(r.robot_id))] = PoseSample(
                        x=float(r.x), y=float(r.y), o=float(r.orientation),
                        confidence=float(getattr(r, "confidence", 0.0)),
                        frame_number=frame_no,
                        t_perf=t_perf, t_wall=t_wall,
                        t_capture=t_capture, t_sent=t_sent,
                    )

    # -- readers --
    def get_pose_sample(self, is_yellow: bool, robot_id: int,
                        max_age: float | None = None) -> PoseSample | None:
        with self._lock:
            s = self._poses.get((bool(is_yellow), int(robot_id)))
        if s is None:
            return None
        if max_age is not None and (time.perf_counter() - s.t_perf) > max_age:
            return None
        return s

    def get_pose(self, is_yellow: bool, robot_id: int,
                 max_age: float | None = None):
        s = self.get_pose_sample(is_yellow, robot_id, max_age)
        return s.pose() if s else None

    def visible_robots(self) -> list[tuple[bool, int]]:
        with self._lock:
            return sorted(self._poses.keys())

    def field_size(self) -> tuple[int, int] | None:
        """(field_length_mm, field_width_mm) from SSL-Vision geometry, or None
        if no geometry packet has been received yet."""
        with self._lock:
            if self._field_length and self._field_width:
                return (self._field_length, self._field_width)
        return None

    @property
    def fps(self) -> float:
        return self._frames.rate_hz

    def status(self) -> dict:
        with self._lock:
            return {
                "available": self.error is None and self._frames.total > 0,
                "error": self.error,
                "fps": round(self._frames.rate_hz, 2),
                "frames_total": self._frames.total,
                "drops": self._drops,
                "dupes": self._dupes,
                "interval_ms": self._frames.interval_stats("ms"),
                "sent_latency_ms": summarize(self._sent_latency, "ms"),
                "capture_latency_ms": summarize(self._capture_latency, "ms"),
                "visible": len(self._poses),
                "age_s": self._frames.age(),
                "field_length": self._field_length,
                "field_width": self._field_width,
            }


# ── Telemetry ───────────────────────────────────────────────────────────--

@dataclass
class TelemetryState:
    obs: object = None                 # latest OnboardObservation
    rate: RateMeter = field(default_factory=lambda: RateMeter(window=120))


class TelemetrySource:
    """Threaded onboard-telemetry receiver (port 50513).

    Reuses TeamControl's parse_packet + build_ip_map so the wire format and
    IP->robot attribution match the real program exactly, but timestamps every
    packet itself so we can measure telemetry rate / jitter / age per robot.
    """

    def __init__(self, config, port: int = 50513, bind_addr: str = "0.0.0.0"):
        self.port = port
        self.bind_addr = bind_addr
        self._run = threading.Event()
        self._thread: threading.Thread | None = None
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()

        _R, _S, parse_packet, build_ip_map = bridge.get_onboard_classes()
        self._parse = parse_packet
        self._ip_map = build_ip_map(config) if config is not None else {}

        self._by_robot: dict[tuple[bool, int], TelemetryState] = {}
        self._all = RateMeter(window=240)
        self.packets = 0
        self.parse_errors = 0
        self.unattributed = 0
        self.error: str | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.bind_addr, self.port))
            s.settimeout(0.5)
            self._sock = s
        except OSError as e:
            self.error = f"telemetry bind failed on {self.bind_addr}:{self.port} — {e}"
            return
        self._run.set()
        self._thread = threading.Thread(
            target=self._loop, name="TelemetrySource", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._run.clear()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _loop(self) -> None:
        while self._run.is_set():
            try:
                data, addr = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            t_perf = time.perf_counter()
            obs = self._parse(data)
            if obs is None:
                self.parse_errors += 1
                continue
            obs.recv_ts = time.time()
            if obs.robot_id < 0:
                m = self._ip_map.get(addr[0])
                if m is not None:
                    obs.is_yellow = bool(m[0])
                    obs.robot_id = int(m[1])
            if obs.robot_id < 0:
                self.unattributed += 1
                continue

            key = (bool(obs.is_yellow), int(obs.robot_id))
            with self._lock:
                st = self._by_robot.get(key)
                if st is None:
                    st = TelemetryState()
                    self._by_robot[key] = st
                st.obs = obs
                st.rate.tick(t_perf)
                self._all.tick(t_perf)
                self.packets += 1

    def get(self, is_yellow: bool, robot_id: int):
        with self._lock:
            st = self._by_robot.get((bool(is_yellow), int(robot_id)))
            return st.obs if st else None

    def robot_status(self, is_yellow: bool, robot_id: int) -> dict:
        with self._lock:
            st = self._by_robot.get((bool(is_yellow), int(robot_id)))
            if st is None:
                return {"seen": False}
            obs = st.obs
            return {
                "seen": True,
                "rate_hz": round(st.rate.rate_hz, 3),
                "age_s": st.rate.age(),
                "interval_ms": st.rate.interval_stats("ms"),
                "voltage": getattr(obs, "voltage", None) if obs else None,
                "ball": getattr(obs, "found", None) if obs else None,
                "robot_ts_ms": getattr(obs, "robot_ts_ms", None) if obs else None,
            }

    def status(self) -> dict:
        with self._lock:
            robots = sorted(self._by_robot.keys())
            return {
                "available": self.error is None,
                "error": self.error,
                "packets": self.packets,
                "parse_errors": self.parse_errors,
                "unattributed": self.unattributed,
                "rate_hz": round(self._all.rate_hz, 3),
                "interval_ms": self._all.interval_stats("ms"),
                "robots_seen": [f"{'Y' if y else 'B'}{i}" for (y, i) in robots],
            }
