"""
safety.py — wall-aware velocity limiting, using TeamControl's own values.

The robots cannot hit walls. To guarantee that while still driving them hard
enough to calibrate, we reuse the *exact* limits and helpers the real program
uses:

  * MAX_SPEED / MAX_W              from TeamControl.robot.constants
  * field half-extents             via bridge.get_field_geometry()
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
    boundary_inset: float = 0.0   # mm extra inset -> a tighter "arena"
    brake_zone: float = 400.0     # mm before the arena edge where braking starts

    @property
    def safe_margin(self) -> float:
        """Distance from a wall (to robot *centre*) we never knowingly cross."""
        return self.field_margin + self.robot_radius

    @property
    def arena_half_len(self) -> float:
        """+/-x half-extent of the drive arena (robots are kept inside this)."""
        return max(0.0, self.half_len - self.safe_margin - self.boundary_inset)

    @property
    def arena_half_wid(self) -> float:
        """+/-y half-extent of the drive arena."""
        return max(0.0, self.half_wid - self.safe_margin - self.boundary_inset)


def limits(boundary_inset: float = 0.0, brake_zone: float = 400.0,
           half_len: float | None = None, half_wid: float | None = None) -> Limits:
    """Build the limit set. `half_len`/`half_wid` (mm) override the field size
    (e.g. from SSL-Vision geometry); when omitted, TeamControl's field_config
    constants are used."""
    c = _consts()
    if half_len is None or half_wid is None:
        gl, gw = bridge.get_field_geometry()
        half_len = gl if half_len is None else half_len
        half_wid = gw if half_wid is None else half_wid
    return Limits(
        max_speed=float(c.MAX_SPEED),
        max_w=float(c.MAX_W),
        half_len=float(half_len),
        half_wid=float(half_wid),
        robot_radius=float(c.ROBOT_RADIUS),
        field_margin=float(c.FIELD_MARGIN),
        boundary_inset=max(0.0, float(boundary_inset)),
        brake_zone=max(0.0, float(brake_zone)),
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
    """True if (x, y) is inside the drive arena (field minus margin minus inset)."""
    lim = lim or limits()
    return abs(x) <= (lim.arena_half_len - extra) and \
        abs(y) <= (lim.arena_half_wid - extra)


def wall_brake(x: float, y: float, vx: float, vy: float) -> tuple[float, float]:
    """Isotropic slow-down near walls — TeamControl's ball_nav.wall_brake."""
    _consts()
    return _NAV.wall_brake(x, y, vx, vy)


def _brake_axis(p: float, v: float, bound: float, brake: float) -> float:
    """Ramp the *outward* velocity on one axis to 0 as the robot nears +/-bound.

    Inward motion is never touched (the robot can always head back to centre).
    Beyond the boundary the outward component is fully removed (hard stop).
    """
    if v > 0:
        room = bound - p
    elif v < 0:
        room = bound + p
    else:
        return v
    if room <= 0.0:
        return 0.0                       # at/over the arena edge: no outward motion
    if brake > 0.0 and room < brake:
        return v * (room / brake)        # linear decel into the boundary
    return v


def limit_to_arena(x: float, y: float, vx_w: float, vy_w: float,
                   lim: Limits | None = None) -> tuple[float, float]:
    """Brake + hard-stop a WORLD-frame velocity at the (tighter) arena edge.

    Replaces the old instant outward-guard: instead of only zeroing velocity at
    the margin (after which the robot still coasts), this ramps speed down over
    `brake_zone` mm so the robot is already crawling when it reaches the arena
    line, and zeroes any outward component at/past it.
    """
    lim = lim or limits()
    vx_w = _brake_axis(x, vx_w, lim.arena_half_len, lim.brake_zone)
    vy_w = _brake_axis(y, vy_w, lim.arena_half_wid, lim.brake_zone)
    return vx_w, vy_w


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
    linear velocity has been: magnitude-clamped, then braked + hard-stopped at
    the arena boundary (decel ramp over brake_zone), then rotated into the robot
    body frame (omni-drive). `blocked` is True when the arena guard removed a
    meaningful chunk of the command (i.e. we're at / heading into a wall).
    """
    lim = lim or limits()
    x, y, theta = pose

    # cap magnitude / angular, then brake + hard-stop at the arena boundary
    gx, gy, w = clamp_velocity(vx_w, vy_w, w, lim)
    in_mag = math.hypot(gx, gy)
    bx, by = limit_to_arena(x, y, gx, gy, lim)
    out_mag = math.hypot(bx, by)
    # "blocked" = the arena guard removed a meaningful chunk of a real command
    blocked = in_mag > 1e-6 and out_mag < 0.5 * in_mag

    # World -> robot body frame (rotate by -theta).
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    vx_r = bx * cos_t + by * sin_t
    vy_r = -bx * sin_t + by * cos_t
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
