"""
bridge.py — locate and import the 2026-TeamControl package without modifying it.

DiagTool deliberately re-uses TeamControl's already-tested code instead of
re-implementing UDP framing, the exact wire format the robots expect, the
vision protobuf decode, and the movement/limit constants. That guarantees we
measure and drive the robots the *same way* the real program does.

We never import the whole TeamControl package eagerly, because some of it
(the `ui` and `network.ssl_sockets` modules) pulls in heavy optional deps
(PySide6, protobuf). Instead callers import the specific helpers they need
through the thin accessors below, which raise a clear, actionable error if a
dependency is missing.

Resolution order for the TeamControl source root:
    1. $TEAMCONTROL_SRC environment variable, if set.
    2. A sibling "2026-TeamControl/src" next to the diagtool folder.
    3. Walk upward from here looking for "2026-TeamControl/src".
"""

from __future__ import annotations

import os
import sys
from pathlib import Path


class TeamControlNotFound(RuntimeError):
    """Raised when the 2026-TeamControl source tree cannot be located."""


class MissingDependency(RuntimeError):
    """Raised when an optional dependency (protobuf, PySide6) is absent."""


_SRC_CACHE: Path | None = None


def _candidate_roots() -> list[Path]:
    here = Path(__file__).resolve()
    roots: list[Path] = []

    env = os.environ.get("TEAMCONTROL_SRC")
    if env:
        roots.append(Path(env))

    # diagtool/diag/bridge.py -> parents[1] = diagtool, parents[2] = testing
    for parent in list(here.parents)[1:6]:
        roots.append(parent / "2026-TeamControl" / "src")

    return roots


def teamcontrol_src() -> Path:
    """Return the path to 2026-TeamControl/src, adding it to sys.path once."""
    global _SRC_CACHE
    if _SRC_CACHE is not None:
        return _SRC_CACHE

    for cand in _candidate_roots():
        if (cand / "TeamControl" / "__init__.py").is_file():
            cand = cand.resolve()
            if str(cand) not in sys.path:
                sys.path.insert(0, str(cand))
            _SRC_CACHE = cand
            return cand

    tried = "\n  ".join(str(c) for c in _candidate_roots())
    raise TeamControlNotFound(
        "Could not find 2026-TeamControl/src. Set the TEAMCONTROL_SRC "
        "environment variable to its path, or keep diagtool next to the "
        "2026-TeamControl folder.\nLooked in:\n  " + tried
    )


def ensure_on_path() -> None:
    """Idempotently make `import TeamControl` work."""
    teamcontrol_src()


# ── Lightweight imports (no protobuf / no Qt) ─────────────────────────────

def get_config(filename: str = "ipconfig.yaml"):
    """TeamControl.utils.yaml_config.Config — robot IPs/ports + vision config."""
    ensure_on_path()
    from TeamControl.utils.yaml_config import Config
    return Config(config_filename=filename)


def get_command_classes():
    """Return (Sender, RobotCommand) — exactly what the dispatcher uses."""
    ensure_on_path()
    from TeamControl.network.sender import Sender
    from TeamControl.network.robot_command import RobotCommand
    return Sender, RobotCommand


def get_constants():
    """Return the TeamControl.robot.constants module (limits + geometry)."""
    ensure_on_path()
    from TeamControl.robot import constants
    return constants


def get_movement_helpers():
    """Return ball_nav (wall_brake, clamp, move_toward) + transform_cords."""
    ensure_on_path()
    from TeamControl.robot import ball_nav
    from TeamControl.world import transform_cords
    return ball_nav, transform_cords


# ── Heavy / optional imports (guarded) ────────────────────────────────────

def get_vision_classes():
    """Return (Vision, Frame). Requires the `protobuf` runtime."""
    ensure_on_path()
    try:
        from TeamControl.network.ssl_sockets import Vision
        from TeamControl.SSL.vision.frame import Frame
        from TeamControl.SSL.vision.robots import Robot
    except ModuleNotFoundError as e:  # almost always google.protobuf
        raise MissingDependency(
            f"Vision decoding needs the 'protobuf' package ({e}). "
            "Install it into the same environment TeamControl uses:\n"
            "    pip install protobuf"
        ) from e
    return Vision, Frame, Robot


def get_onboard_classes():
    """Return (OnboardReceiver, OnboardObservationStore, parse_packet, build_ip_map)."""
    ensure_on_path()
    from TeamControl.onboard_vision import (
        OnboardReceiver,
        OnboardObservationStore,
        parse_packet,
        build_ip_map,
    )
    return OnboardReceiver, OnboardObservationStore, parse_packet, build_ip_map


def have_protobuf() -> bool:
    try:
        import google.protobuf  # noqa: F401
        return True
    except Exception:
        return False


def have_pyside6() -> bool:
    try:
        import PySide6  # noqa: F401
        return True
    except Exception:
        return False
