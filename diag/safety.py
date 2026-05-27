"""
safety.py — wall-aware velocity limiting, using TeamControl's own values.

The robots cannot hit walls. To guarantee that while still driving them hard
enough to calibrate, we reuse the *exact* limits and helpers the real program
uses:

  * MAX_SPEED / MAX_W              from TeamControl.robot.constants
  * field geometry (HALF_LEN ...)  from TeamControl.robot.constants
  * wall_brake() / clamp()         from TeamControl.robot.ball_nav

On top of those we add a hard *outward-velocity guard*: if a robot is within
the safety margin of a boundary and the commanded world-frame velocity points
further out, that outward component is removed entirely (not just scaled).
wall_brake only scales magnitude, so the guard is what actually prevents a
fast robot from coasting through the margin into a wall.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from . import bridge

_C = None          # cached constants module
_NAV = None        # cached ball_nav module


def _consts():
    global _C, _NAV
    if _C is None:
        _C = bridge.get_constants()
        _NAV, _ = bridge.get_movement_helpers()
    return _C


@dataclass(frozen=True)
class Limits:
    max_speed: float        # m/s  (linear)
    max_w: float            # rad/s (angular)
    half_len: float         # mm   (field half length, +/-x)
    half_wid: float         # mm   (field half width,  +/-y)
    robot_radius: float     # mm
    field_margin: float     # mm   (TeamControl's keep-off-walls margin)

    @property
    def safe_margin(self) -> float:
        """Distance from a wall (to robot *centre*) we never knowingly cross."""
        return self.field_margin + self.robot_radius


def limits() -> Limits:
    c = _consts()
    return Limits(
        max_speed=float(c.MAX_SPEED),
        max_w=float(c.MAX_W),
        half_len=float(c.HALF_LEN),
        half_wid=float(c.HALF_WID),
        robot_radius=float(c.ROBOT_RADIUS),
        field_margin=float(c.FIELD_MARGIN),
    )


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def clamp_velocity(vx: float, vy: float, w: float,
                   lim: Limits | None = None) -> tuple[float, float, float]:
    """Clamp linear speed to MAX_SPEED (preserving direction) and |w| to MAX_W."""
    lim = lim or limits()
    speed = math.hypot(vx, vy)
    if speed > lim.max_speed and speed > 0:
        s = lim.max_speed / speed
        vx, vy = vx * s, vy * s
    w = clamp(w, -lim.max_w, lim.max_w)
    return vx, vy, w


def nearest_wall_dist(x: float, y: float, lim: Limits | None = None) -> float:
    """Distance (mm) from robot centre to the nearest field boundary."""
    lim = lim or limits()
    return min(lim.half_len - abs(x), lim.half_wid - abs(y))


def in_safe_zone(x: float, y: float, lim: Limits | None = None,
                 extra: float = 0.0) -> bool:
    """True if (x, y) is inside the field minus the safety margin."""
    lim = lim or limits()
    m = lim.safe_margin + extra
    return abs(x) <= (lim.half_len - m) and abs(y) <= (lim.half_wid - m)


def wall_brake(x: float, y: float, vx: float, vy: float) -> tuple[float, float]:
    """Isotropic slow-down near walls — TeamControl's ball_nav.wall_brake."""
    _consts()
    return _NAV.wall_brake(x, y, vx, vy)


def guard_outward_world(x: float, y: float, vx_w: float, vy_w: float,
                        lim: Limits | None = None) -> tuple[float, float]:
    """Zero any world-frame velocity component pushing further past the margin.

    `vx_w, vy_w` are WORLD-frame velocities. If the robot centre is already
    within `safe_margin` of a wall on an axis and the velocity on that axis
    points outward, that component is removed.
    """
    lim = lim or limits()
    m = lim.safe_margin
    if x > lim.half_len - m and vx_w > 0:
        vx_w = 0.0
    elif x < -(lim.half_len - m) and vx_w < 0:
        vx_w = 0.0
    if y > lim.half_wid - m and vy_w > 0:
        vy_w = 0.0
    elif y < -(lim.half_wid - m) and vy_w < 0:
        vy_w = 0.0
    return vx_w, vy_w


def safe_world_velocity(pose, vx_w: float, vy_w: float, w: float,
                        lim: Limits | None = None):
    """Full safety pipeline for a WORLD-frame velocity intent.

    pose = (x, y, theta). Returns (vx_robot, vy_robot, w, blocked) where the
    linear velocity has been: outward-guarded, magnitude-clamped, wall-braked,
    then rotated into the robot body frame (omni-drive). `blocked` is True if
    the guard removed velocity the caller asked for (i.e. we're at a wall).
    """
    lim = lim or limits()
    x, y, theta = pose

    gx, gy = guard_outward_world(x, y, vx_w, vy_w, lim)
    blocked = (gx != vx_w) or (gy != vy_w)

    gx, gy, w = clamp_velocity(gx, gy, w, lim)
    gx, gy = wall_brake(x, y, gx, gy)

    # World -> robot body frame (rotate by -theta).
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    vx_r = gx * cos_t + gy * sin_t
    vy_r = -gx * sin_t + gy * cos_t
    return vx_r, vy_r, w, blocked


def safe_body_velocity(pose, vx_r: float, vy_r: float, w: float,
                       lim: Limits | None = None):
    """Safety pipeline for a velocity already expressed in the ROBOT body frame.

    Used by raw step tests. We rotate the body intent to world to apply the
    directional wall guard, then rotate the (possibly trimmed) result back.
    """
    lim = lim or limits()
    x, y, theta = pose
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    # body -> world
    vx_w = vx_r * cos_t - vy_r * sin_t
    vy_w = vx_r * sin_t + vy_r * cos_t
    return safe_world_velocity((x, y, theta), vx_w, vy_w, w, lim)
