"""Parsed onboard ball observation + packet decoder."""

from dataclasses import dataclass


@dataclass
class OnboardObservation:
    """A single onboard-camera ball reading.

    found       : True if the robot's camera sees an orange blob this frame.
    px, py      : Pixel centroid in the image (origin top-left).
    radius      : Enclosing-circle radius in pixels (rough distance proxy).
    bearing     : Horizontal bearing from camera optical axis in radians,
                  positive = right of center. Combine with robot heading
                  to get a world-frame bearing to the ball.
    confidence  : 0..1 roundness score; reject readings below ~0.3.
    robot_ts_ms : Timestamp from the robot's steady_clock (ms).
    recv_ts     : Wall-clock time.time() when the PC received the packet.
    robot_id    : Robot that produced the packet (if inferable).
    is_yellow   : Robot's team color (if inferable).
    """

    found: bool = False
    px: float = 0.0
    py: float = 0.0
    radius: float = 0.0
    bearing: float = 0.0
    confidence: float = 0.0
    robot_ts_ms: int = 0
    recv_ts: float = 0.0
    robot_id: int = -1
    is_yellow: bool = True


# Map the robot's human-readable keys (lower-cased, whitespace
# collapsed) to our canonical short keys.
_KEY_ALIASES = {
    "robot state": "state",
    "state": "state",
    "battery voltage": "voltage",
    "voltage": "voltage",
    "ball detection": "ball",
    "ball detected": "ball",
    "ball": "ball",
    "pixel x": "px",
    "pixel_x": "px",
    "px": "px",
    "pixel y": "py",
    "pixel_y": "py",
    "py": "py",
    "radius": "radius",
    "r": "radius",
    "bearing": "bearing",
    "confidence": "confidence",
    "conf": "confidence",
    "timestamp": "ts_ms",
    "ts_ms": "ts_ms",
    "robot id": "robot_id",
    "robot_id": "robot_id",
    "id": "robot_id",
    "yellow": "yellow",
    "is yellow": "yellow",
    "is_yellow": "yellow",
    "team": "team",
}


def _normalize_key(raw):
    k = " ".join(raw.strip().lower().split())  # collapse whitespace
    return _KEY_ALIASES.get(k, k)


def _coerce(key, value):
    v = value.strip() if isinstance(value, str) else value
    try:
        if key == "ball":
            # accept "1"/"0", "true"/"false", "yes"/"no", "detected"/"none"
            if isinstance(v, str):
                low = v.lower()
                if low in ("1", "true", "yes", "detected", "found", "on", "active"):
                    return True
                if low in ("0", "false", "no", "none", "off", "lost", "absent"):
                    return False
                return bool(int(float(v)))
            return bool(v)
        if key in ("px", "py", "radius", "bearing", "confidence", "voltage"):
            return float(v)
        if key in ("ts_ms", "robot_id"):
            return int(float(v))
        if key == "yellow":
            if isinstance(v, str):
                low = v.lower()
                if low in ("yellow", "1", "true", "y"):
                    return True
                if low in ("blue", "0", "false", "b"):
                    return False
                return bool(int(float(v)))
            return bool(v)
        if key == "team":
            if isinstance(v, str):
                low = v.lower()
                if "yellow" in low:
                    return True
                if "blue" in low:
                    return False
    except (ValueError, TypeError):
        return None
    return v


def parse_packet(payload):
    """Parse a RobotFramework telemetry packet.

    Accepts both:
      `state=active,voltage=22.8,ball=1,px=160,py=120,bearing=0.1,conf=0.9`
      `Robot State: Active, Battery Voltage: 22.8, Ball Detection: 1, ...`

    Tokens are `,`-separated. Within a token the key/value separator may
    be either `=` or `:`. Unknown keys are ignored. Returns an
    `OnboardObservation` with missing fields left at defaults, or None
    if the payload has no recognisable key/value pairs at all.
    """
    if isinstance(payload, (bytes, bytearray)):
        try:
            payload = payload.decode("utf-8", errors="replace")
        except Exception:
            return None
    if not isinstance(payload, str):
        return None
    if "=" not in payload and ":" not in payload:
        return None

    kv = {}
    for token in payload.strip().split(","):
        if "=" in token:
            k, v = token.split("=", 1)
        elif ":" in token:
            k, v = token.split(":", 1)
        else:
            continue
        key = _normalize_key(k)
        parsed = _coerce(key, v)
        if parsed is not None:
            kv[key] = parsed

    if not kv:
        return None

    obs = OnboardObservation()
    if "ball" in kv:
        obs.found = bool(kv["ball"])
    obs.px = float(kv.get("px", 0.0))
    obs.py = float(kv.get("py", 0.0))
    obs.radius = float(kv.get("radius", 0.0))
    obs.bearing = float(kv.get("bearing", 0.0))
    obs.confidence = float(kv.get("confidence", 0.0))
    obs.robot_ts_ms = int(kv.get("ts_ms", 0))
    if "robot_id" in kv:
        obs.robot_id = int(kv["robot_id"])
    if "yellow" in kv:
        obs.is_yellow = bool(kv["yellow"])
    elif "team" in kv:
        obs.is_yellow = bool(kv["team"])
    return obs
