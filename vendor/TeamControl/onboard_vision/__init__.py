"""Receiver for per-robot onboard ball-detection telemetry.

Each robot's RobotFramework sends a UDP packet with its latest camera
observation (ball presence, pixel coords, bearing, confidence). This
module parses those packets and exposes the latest reading per robot
through `OnboardObservationStore`.

Public API:
  OnboardObservation  — parsed reading + receive timestamp
  OnboardReceiver     — background UDP listener thread
  OnboardObservationStore — thread-safe per-robot snapshot store
  parse_packet        — stateless key=value parser
"""

from .observation import OnboardObservation, parse_packet
from .store import OnboardObservationStore
from .receiver import OnboardReceiver


def build_ip_map(preset):
    """Map robot IP → (is_yellow, shell_id) from a Config preset."""
    mapping = {}
    for is_yellow, team_dict in ((True, getattr(preset, "yellow", None)),
                                 (False, getattr(preset, "blue", None))):
        if not team_dict:
            continue
        for _key, r in team_dict.items():
            ip = r.get("ip")
            sid = r.get("shellID")
            if ip and sid is not None:
                mapping[ip] = (bool(is_yellow), int(sid))
    return mapping


__all__ = [
    "OnboardObservation",
    "OnboardObservationStore",
    "OnboardReceiver",
    "parse_packet",
    "build_ip_map",
]
