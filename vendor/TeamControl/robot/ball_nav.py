"""
Shared ball physics and pathfinding utilities.

Every robot module (goalie, striker, navigator, team) imports from here
instead of defining its own copies.  One source of truth.

Ball physics:
  - predict_ball       — position after dt seconds with friction
  - ball_velocity      — velocity from timestamped history
  - update_ball_history — append to history buffer

Movement:
  - clamp              — value clamping
  - move_toward        — deceleration-ramp movement toward a local target
  - sanitize_field_target — offset out-of-field movement targets inward
  - apply_boundary_braking — dynamic speed braking near/outside the field
                             edge, including the goal-post no-go zone
  - rotation_compensate — pre-rotate velocity during turning
  - wrap_angle         — wrap an angle into (-pi, pi]

Shared motion rules (apply to every motion controller -- RobotMotionController,
Movement.py, and the functions in this file -- not just one):
  - FieldGeometryCache   — detect a change in live field bounds/defence area
  - clamp_for_role       — goalie stays in the penalty box, non-goalie stays out
  - regulate_speed_to_target — stateless never-overshoot speed cap
  - predict_position     — one-step lookahead from current velocity

Pathfinding:
  - compute_arc_nav    — arc approach to get behind the ball
"""

import json
import math
import os
from typing import Tuple, Optional, List

from TeamControl.planner.voronoi_dijkstra import is_in_penalty_box
from TeamControl.robot import constants as C
from TeamControl.robot.constants import (
    ROBOT_RADIUS,
    FRICTION,
    BALL_HISTORY_SIZE,
    LOOP_RATE,
)
from TeamControl.world.field_config import (
    DEFENCE_X_MM,
    DEFENCE_Y_MM,
    FIELD_X_MIN,
    FIELD_X_MAX,
    FIELD_Y_MIN,
    FIELD_Y_MAX,
    GOAL_HALF_WIDTH_MM,
    GOAL_DEPTH_MM,
    VORONOI_BOUNDARY_DECEL_ZONE_MM,
    VORONOI_BOUNDARY_HARD_STOP_MM,
    VORONOI_BOUNDARY_NEAR_SPEED_SCALE,
    VORONOI_OUT_OF_FIELD_SPEED_SCALE,
)


# ═══════════════════════════════════════════════════════════════════
#  CALIBRATION — loaded from calibration.json, applied in move_toward
# ═══════════════════════════════════════════════════════════════════

_CAL_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__), os.pardir, os.pardir, os.pardir,
    "calibration.json"))

_cal = {"speed_scale": 1.0}


def _reload_calibration():
    """Reload calibration values from disk.  Called at import time
    and when the calibration UI applies new values."""
    global _cal
    try:
        with open(_CAL_PATH, "r") as f:
            data = json.load(f)
        _cal["speed_scale"] = float(data.get("speed_scale", 1.0))
    except (FileNotFoundError, json.JSONDecodeError, ValueError, TypeError):
        _cal["speed_scale"] = 1.0


_reload_calibration()


# ═══════════════════════════════════════════════════════════════════
#  UTILITIES
# ═══════════════════════════════════════════════════════════════════

def clamp(v, lo, hi):
    """Clamp v to [lo, hi]."""
    return max(lo, min(hi, v))


def wrap_angle(a: float) -> float:
    """Wrap any angle (radians) into the shortest equivalent in (-pi, pi]."""
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    if a <= -math.pi:
        a += 2.0 * math.pi
    return a


class FieldGeometryCache:
    """Detects a change in field geometry (size mismatch awareness).

    Every motion controller (RobotMotionController, Movement.py, the
    ball_nav-based navigators) should hold one instance of this per robot
    and call `.refresh()` once per tick. Field geometry is now fixed to the
    static constants in field_config.py; this class will report changed=True
    only on the very first call, and False thereafter. It is kept for
    API compatibility with callers that react to geometry changes.
    """

    def __init__(self):
        self.bounds = None
        self.defence = None

    def refresh(self) -> bool:
        """Update the cache; return True if geometry changed since last call."""
        bounds = (float(FIELD_X_MIN), float(FIELD_X_MAX), float(FIELD_Y_MIN), float(FIELD_Y_MAX))
        defence = (float(DEFENCE_X_MM), float(DEFENCE_Y_MM), float(GOAL_HALF_WIDTH_MM), float(GOAL_DEPTH_MM))
        changed = bounds != self.bounds or defence != self.defence
        self.bounds = bounds
        self.defence = defence
        return changed


def is_target_in_field_box(target, margin=ROBOT_RADIUS):
    """Return whether a world-frame target is inside the inset field box.

    Uses the static field constants from field_config.py.
    See FieldGeometryCache for callers that want to detect a *change* in bounds.
    """
    if target is None:
        return False
    x_min, x_max, y_min, y_max = float(FIELD_X_MIN), float(FIELD_X_MAX), float(FIELD_Y_MIN), float(FIELD_Y_MAX)
    return (
        x_min + margin <= target[0] <= x_max - margin
        and y_min + margin <= target[1] <= y_max - margin
    )


def sanitize_field_target(target, margin=ROBOT_RADIUS, reject_outside=False):
    """Return an allowed world-frame movement target.

    Targets outside the inset field box are offset inward by default. Callers
    that prefer to ignore invalid targets can pass ``reject_outside=True``.
    """
    if target is None or is_target_in_field_box(target, margin):
        return target
    if reject_outside:
        return None

    x_min, x_max, y_min, y_max = float(FIELD_X_MIN), float(FIELD_X_MAX), float(FIELD_Y_MIN), float(FIELD_Y_MAX)
    return (
        clamp(target[0], x_min + margin, x_max - margin),
        clamp(target[1], y_min + margin, y_max - margin),
    )


def clamp_for_role(target_xy, is_goalie, margin=ROBOT_RADIUS,
                    own_goal_positive_side=None, own_goal_x=None):
    """Penalty-box rule: goalie stays in, non-goalie stays out.

    Shared by every motion controller -- the same push-out-along-x logic
    that used to live only in voronoi_game_navigator.py's
    `_clamp_out_of_own_box`, now reusable everywhere.

    Args:
        target_xy: (x, y) world-frame target.
        is_goalie: this robot's role.
        margin:    clearance kept from both the goal line and the box's
                    far (most-advanced) edge.
        own_goal_positive_side: which end is this robot's own goal
            (True = +x side, False = -x side). A robot's goal side is
            fixed by its identity/team, not by where the target happens
            to be -- pass this when known (e.g. from `cache.team.us_positive`)
            rather than relying on the "nearest end" fallback below.
        own_goal_x: the exact x of this robot's own goal line, if the
            caller already knows it precisely (e.g. goalie.py's
            `sign * FIELD_LENGTH_MM / 2`). Falls back to the static
            field edge (FIELD_X_MAX / FIELD_X_MIN) if omitted.

    Returns:
        (x, y) target, clamped for the given role.
    """
    tx, ty = float(target_xy[0]), float(target_xy[1])

    if is_goalie:
        # Goalie must stay inside its own box -- clamp x to the
        # max-advance distance (DEFENCE_X_MM - 50mm margin) from its own end line.
        x_min, x_max = float(FIELD_X_MIN), float(FIELD_X_MAX)
        max_advance = float(DEFENCE_X_MM) - 50.0
        if own_goal_positive_side is None:
            # Fallback: guess from whichever end is nearest the target.
            own_goal_positive_side = abs(tx - x_max) < abs(tx - x_min)
        if own_goal_x is not None:
            goal_line_x = own_goal_x
        else:
            goal_line_x = x_max if own_goal_positive_side else x_min
        if own_goal_positive_side:
            tx = min(tx, goal_line_x - margin)
            tx = max(tx, goal_line_x - max_advance)
        else:
            tx = max(tx, goal_line_x + margin)
            tx = min(tx, goal_line_x + max_advance)
        return tx, ty

    # Non-goalie: push the target out of whichever own-side penalty box
    # it's currently in, if any.
    for positive_side in (True, False):
        if is_in_penalty_box((tx, ty), positive_side=positive_side, margin=margin):
            defence_x = float(DEFENCE_X_MM)
            x_min, x_max = float(FIELD_X_MIN), float(FIELD_X_MAX)
            if positive_side:
                tx = min(tx, x_max - defence_x - margin)
            else:
                tx = max(tx, x_min + defence_x + margin)
            break
    return tx, ty


def compute_out_of_bounds_clearance(exit_xy, clearance_mm=500.0):
    """Where to retreat to when the ball has left the field at exit_xy.

    Finds whichever boundary edge exit_xy is past (or nearest to) via
    the static field constants, then pushes exit_xy `clearance_mm` back into the
    field along that edge's normal -- this single point ends up
    simultaneously >= clearance_mm from the ball's exit position AND
    >= clearance_mm inside the crossed line, since the push is exactly
    perpendicular to it.

    Args:
        exit_xy: (x, y) world-frame position where the ball left the field
            (e.g. WorldModel.possible_ball_left_field_pos_mm).
        clearance_mm: minimum distance to keep from both the ball and the
            crossed line (500mm = SSL's standard ball clearance distance).

    Returns:
        (x, y) world-frame point satisfying both clearances.
    """
    ex, ey = float(exit_xy[0]), float(exit_xy[1])
    x_min, x_max, y_min, y_max = float(FIELD_X_MIN), float(FIELD_X_MAX), float(FIELD_Y_MIN), float(FIELD_Y_MAX)

    # How far past each edge -- the biggest value is the edge that was
    # actually crossed (the others are negative: still inside on that axis).
    past = {
        "x_min": x_min - ex,
        "x_max": ex - x_max,
        "y_min": y_min - ey,
        "y_max": ey - y_max,
    }
    edge = max(past, key=past.get)

    if edge == "x_min":
        return (x_min + clearance_mm, clamp(ey, y_min, y_max))
    if edge == "x_max":
        return (x_max - clearance_mm, clamp(ey, y_min, y_max))
    if edge == "y_min":
        return (clamp(ex, x_min, x_max), y_min + clearance_mm)
    return (clamp(ex, x_min, x_max), y_max - clearance_mm)


def compute_ball_clearance_target(robot_xy, ball_xy, clearance_mm=500.0):
    """Point >= clearance_mm from the ball, straight back along robot->ball.

    Used to give a possessing enemy room instead of immediately pushing
    in on the ball (e.g. a brief "back off" phase before a steal attempt).
    Unlike compute_out_of_bounds_clearance (clearance from a boundary
    *line*), this is clearance from a *point* -- same clearance_mm idea,
    different geometry.

    Returns robot_xy unchanged if already >= clearance_mm away (nothing to
    do -- caller should just hold position, not drive anywhere).
    """
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    bx, by = float(ball_xy[0]), float(ball_xy[1])
    dx, dy = rx - bx, ry - by
    dist = math.hypot(dx, dy)
    if dist >= clearance_mm:
        return (rx, ry)
    if dist < 1e-6:
        ux, uy = 1.0, 0.0  # degenerate: robot exactly on the ball, pick a side
    else:
        ux, uy = dx / dist, dy / dist
    return (bx + ux * clearance_mm, by + uy * clearance_mm)


# ═══════════════════════════════════════════════════════════════════
#  BALL PHYSICS
# ═══════════════════════════════════════════════════════════════════

def predict_ball(pos, vel, dt):
    """Predict ball position after *dt* seconds with linear friction.

    Args:
        pos: (x, y) current ball position in mm
        vel: (vx, vy) current ball velocity in mm/s
        dt:  seconds to simulate forward

    Returns:
        (x, y) predicted position, clamped to field.
    """
    bx, by = pos
    vx, vy = vel
    t, step = 0.0, 0.02
    while t < dt:
        s = min(step, dt - t)
        bx += vx * s
        by += vy * s
        f = max(1.0 - FRICTION * s, 0.0)
        vx *= f
        vy *= f
        t += s
        if math.hypot(vx, vy) < 30:
            break
    x_min, x_max, y_min, y_max = float(FIELD_X_MIN), float(FIELD_X_MAX), float(FIELD_Y_MIN), float(FIELD_Y_MAX)
    return (clamp(bx, x_min, x_max),
            clamp(by, y_min, y_max))


def ball_velocity(history):
    """Estimate ball velocity from timestamped history.

    Args:
        history: list of (timestamp, x, y) tuples

    Returns:
        (vx, vy, speed) in mm/s.
    """
    if len(history) < 2:
        return 0.0, 0.0, 0.0
    dt = history[-1][0] - history[0][0]
    if dt < 0.02:
        return 0.0, 0.0, 0.0
    vx = (history[-1][1] - history[0][1]) / dt
    vy = (history[-1][2] - history[0][2]) / dt
    return vx, vy, math.hypot(vx, vy)


def update_ball_history(history, now, ball, last_ball_xy,
                        max_size=BALL_HISTORY_SIZE):
    """Append ball position to history if it moved.

    Returns the new last_ball_xy value.
    """
    if last_ball_xy is None or ball != last_ball_xy:
        history.append((now, ball[0], ball[1]))
        if len(history) > max_size:
            history.pop(0)
    return ball


# ═══════════════════════════════════════════════════════════════════
#  MOVEMENT
# ═══════════════════════════════════════════════════════════════════

def move_toward(rel, speed, ramp_dist=350.0, stop_dist=10.0, min_speed=0.06,
                 regulate=True, accel=C.LINEAR_AMAX):
    """Move toward a robot-local point with linear deceleration ramp.

    Speed is scaled by the calibration factor from calibration.json
    so commanded speed matches actual robot speed.

    Args:
        rel:       (x, y) target in robot frame
        speed:     cruise speed (m/s fraction)
        ramp_dist: start decelerating at this distance (mm)
        stop_dist: stop moving below this distance (mm)
        min_speed: floor speed during deceleration
        regulate:  apply the never-overshoot stopping-distance cap on top
                   of the ramp (see regulate_speed_to_target) -- stateless,
                   safe for moving targets, on by default.
        accel:     braking capability used by the regulation cap (m/s^2).

    Returns:
        (vx, vy) velocity in robot frame.
    """
    d = math.hypot(rel[0], rel[1])
    if d < stop_dist:
        return 0.0, 0.0
    # Apply calibration: if robot runs slow, scale > 1 boosts commanded speed
    cal_scale = _cal.get("speed_scale", 1.0)
    if cal_scale > 0.01:
        speed = speed / cal_scale
    if d < ramp_dist:
        t = (d - stop_dist) / max(ramp_dist - stop_dist, 1.0)
        speed = max(speed * t, min_speed)
    if regulate:
        # Never-overshoot wins over the min_speed floor above -- a hard
        # physical safety cap takes priority over an artificial "don't
        # crawl too slowly" floor.
        speed = regulate_speed_to_target(d, speed, accel)
    return (rel[0] / d) * speed, (rel[1] / d) * speed


def wall_brake(rx, ry, vx, vy, *args, **kwargs):
    """Deprecated compatibility shim; sanitize movement targets instead."""
    return vx, vy


def apply_boundary_braking(current_pos, vx, vy, max_speed=C.MAX_SPEED):
    """Dynamic speed braking near/outside the field boundary.

    Robot-frame (vx, vy) in, robot-frame (vx, vy) out -- the boundary math
    itself works in world frame, so this rotates in by the robot's heading,
    applies three safety stages, then rotates back:

      1. Decel zone (inside, within VORONOI_BOUNDARY_DECEL_ZONE_MM of an
         edge): cap speed to a safe-stopping max, ramping from
         VORONOI_BOUNDARY_NEAR_SPEED_SCALE at the wall up to full speed at
         the zone's outer edge.
      2. Out-of-field (already past an edge): scale down to
         VORONOI_OUT_OF_FIELD_SPEED_SCALE -- a slow crawl back in.
      3. Hard stop: zero whichever velocity component points further into
         a boundary within VORONOI_BOUNDARY_HARD_STOP_MM, regardless of
         which stage above fired -- the final guarantee the robot never
         drives further out.
      4. Goal-post zone: past the end line within |y| < GOAL_HW +
         ROBOT_RADIUS of center -- zero the component driving further into
         the physical goal structure.

    Args:
        current_pos: (x, y, theta) robot pose in world frame.
        vx, vy:      desired velocity in robot frame (m/s).
        max_speed:   speed the decel-zone ramp scales against.

    Returns:
        (vx, vy) braked velocity, robot frame.
    """
    rx, ry, theta = float(current_pos[0]), float(current_pos[1]), float(current_pos[2])
    cos_t, sin_t = math.cos(theta), math.sin(theta)

    # Robot frame → world frame so boundary checks use world-axis distances
    vx_w = vx * cos_t - vy * sin_t
    vy_w = vx * sin_t + vy * cos_t

    x_min, x_max, y_min, y_max = float(FIELD_X_MIN), float(FIELD_X_MAX), float(FIELD_Y_MIN), float(FIELD_Y_MAX)
    dist_to_boundary = min(
        rx - x_min, x_max - rx,
        ry - y_min, y_max - ry,
    )

    if dist_to_boundary < 0:
        # Outside field: slow crawl back in
        vx_w *= VORONOI_OUT_OF_FIELD_SPEED_SCALE
        vy_w *= VORONOI_OUT_OF_FIELD_SPEED_SCALE
    elif dist_to_boundary < VORONOI_BOUNDARY_DECEL_ZONE_MM:
        # Linear ramp: full speed at zone edge → NEAR_SPEED_SCALE at the wall
        t = dist_to_boundary / VORONOI_BOUNDARY_DECEL_ZONE_MM
        v_max = max_speed * (
            VORONOI_BOUNDARY_NEAR_SPEED_SCALE
            + t * (1.0 - VORONOI_BOUNDARY_NEAR_SPEED_SCALE)
        )
        speed = math.hypot(vx_w, vy_w)
        if speed > v_max and speed > 0.0:
            scale = v_max / speed
            vx_w *= scale
            vy_w *= scale

    # Hard stop: zero the component pointing toward any boundary the robot
    # is within VORONOI_BOUNDARY_HARD_STOP_MM of. Fires regardless of which
    # speed-cap stage is active -- the final guarantee against crossing.
    if rx - x_min < VORONOI_BOUNDARY_HARD_STOP_MM and vx_w < 0.0:
        vx_w = 0.0
    if x_max - rx < VORONOI_BOUNDARY_HARD_STOP_MM and vx_w > 0.0:
        vx_w = 0.0
    if ry - y_min < VORONOI_BOUNDARY_HARD_STOP_MM and vy_w < 0.0:
        vy_w = 0.0
    if y_max - ry < VORONOI_BOUNDARY_HARD_STOP_MM and vy_w > 0.0:
        vy_w = 0.0

    # Goal-post zone: the physical goal sits just past the end line, inside
    # |y| < GOAL_HW + ROBOT_RADIUS. Once the robot is out there, zero the
    # x-component that would drive it further into the goal structure --
    # a hard physical obstacle, not just a soft boundary.
    goal_post_half_width = float(GOAL_HALF_WIDTH_MM) + ROBOT_RADIUS
    if abs(ry) < goal_post_half_width:
        if rx < x_min and vx_w < 0.0:
            vx_w = 0.0
        if rx > x_max and vx_w > 0.0:
            vx_w = 0.0

    # World frame → robot frame
    vx = vx_w * cos_t + vy_w * sin_t
    vy = -vx_w * sin_t + vy_w * cos_t
    return vx, vy


def regulate_speed_to_target(dist_mm, speed_mps, accel=C.LINEAR_AMAX):
    """Stateless never-overshoot speed cap.

    v_max = sqrt(2 * accel * dist) -- the speed from which the robot can
    brake to a stop exactly at the target using `accel`. Recomputed fresh
    from the *current* distance/speed every call, with no memory of
    previous ticks -- unlike a PD derivative term, this never differentiates
    the target's own motion, so it's safe to use even when the target
    (e.g. the ball) is moving. Same formula already used for field-boundary
    braking, applied here to the target instead of the field edge.

    Args:
        dist_mm:   current distance to target (mm).
        speed_mps: desired speed (m/s).
        accel:     braking capability (m/s^2).

    Returns:
        speed_mps, capped so the robot can still stop in time.
    """
    if dist_mm <= 0.0 or speed_mps <= 0.0:
        return speed_mps
    v_max = math.sqrt(2.0 * accel * (dist_mm / 1000.0))
    return min(speed_mps, v_max)


def predict_position(current_pos, vx, vy, dt):
    """One-step lookahead: where will the robot be after dt at this velocity?

    Args:
        current_pos: (x, y, theta) world frame.
        vx, vy:      robot-frame velocity (m/s).
        dt:          seconds to look ahead (measured response time, not a
                     fixed constant -- see rule on robot responsiveness).

    Returns:
        (x, y) predicted world-frame position.
    """
    rx, ry, theta = float(current_pos[0]), float(current_pos[1]), float(current_pos[2])
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    vx_w = vx * cos_t - vy * sin_t
    vy_w = vx * sin_t + vy * cos_t
    return (rx + vx_w * dt * 1000.0, ry + vy_w * dt * 1000.0)


def rotation_compensate(vx, vy, w, dt=LOOP_RATE):
    """Pre-rotate velocity so the world-frame path stays on target
    despite simultaneous rotation.

    Uses midpoint approximation: rotate by -w*dt/2.
    """
    if abs(w) < 0.01:
        return vx, vy
    half_rot = -w * dt * 0.5
    cos_r = math.cos(half_rot)
    sin_r = math.sin(half_rot)
    return vx * cos_r - vy * sin_r, vx * sin_r + vy * cos_r


# ═══════════════════════════════════════════════════════════════════
#  ARC-BASED APPROACH — get behind the ball without looping
# ═══════════════════════════════════════════════════════════════════

def compute_arc_nav(
    robot_xy: Tuple[float, float],
    ball: Tuple[float, float],
    aim: Tuple[float, float],
    behind_dist: float,
    avoid_radius: float,
    committed_side: Optional[int],
) -> Tuple[Tuple[float, float], int, bool]:
    """Compute a navigation target for approaching behind the ball.

    Uses arc-based pathing with committed-side hysteresis to prevent
    oscillation / looping.

    Args:
        robot_xy:       (x, y) robot position in world frame
        ball:           (x, y) ball position
        aim:            (x, y) aim target (usually inside opponent goal)
        behind_dist:    how far behind the ball to line up
        avoid_radius:   minimum clearance from ball while arcing
        committed_side: +1 / -1 from previous tick, or None on first call

    Returns:
        (nav_target, updated_committed_side, robot_is_behind_ball)
    """
    # Unit vector ball → aim
    ba_dx = aim[0] - ball[0]
    ba_dy = aim[1] - ball[1]
    ba_d = max(math.hypot(ba_dx, ba_dy), 1.0)
    ux, uy = ba_dx / ba_d, ba_dy / ba_d

    # Perpendicular (90° CCW of aim direction)
    px, py = -uy, ux

    # Behind-ball point
    behind = (ball[0] - ux * behind_dist, ball[1] - uy * behind_dist)

    # Robot relative to ball, decomposed along aim axis
    rbx = robot_xy[0] - ball[0]
    rby = robot_xy[1] - ball[1]
    along = rbx * ux + rby * uy          # >0 → aim-side (wrong side)
    perp  = rbx * px + rby * py          # lateral offset
    d_ball = math.hypot(rbx, rby)

    # ── Do we need to arc? ──────────────────────────────────
    need_arc = (along > -behind_dist * 0.15) and (d_ball < avoid_radius * 3.0)
    if d_ball < avoid_radius * 0.9 and along > -behind_dist * 0.5:
        need_arc = True

    if not need_arc:
        side = committed_side if committed_side is not None else (1 if perp >= 0 else -1)
        return sanitize_field_target(behind), side, True

    # ── Commit to a side with strong hysteresis ─────────────
    HYSTERESIS = avoid_radius * 0.6

    if committed_side is None:
        if abs(perp) < 80:
            y_max = float(FIELD_Y_MAX)
            if abs(robot_xy[1]) > y_max - 350:
                committed_side = -1 if robot_xy[1] > 0 else 1
            else:
                committed_side = 1 if perp >= 0 else -1
        else:
            committed_side = 1 if perp > 0 else -1
    else:
        if committed_side == 1 and perp < -HYSTERESIS:
            committed_side = -1
        elif committed_side == -1 and perp > HYSTERESIS:
            committed_side = 1

    # ── Compute arc waypoint ──────────────────────────────
    wrong_ratio = clamp(
        (along + behind_dist * 0.3) / max(avoid_radius * 2.0, 1.0),
        0.0, 1.0,
    )

    arc_r = avoid_radius * 1.3

    bk_x, bk_y = -ux, -uy
    sd_x, sd_y = px * committed_side, py * committed_side

    blend = wrong_ratio * 0.85
    dir_x = bk_x * (1.0 - blend) + sd_x * blend
    dir_y = bk_y * (1.0 - blend) + sd_y * blend
    dir_d = max(math.hypot(dir_x, dir_y), 1e-9)
    dir_x /= dir_d
    dir_y /= dir_d

    nav = sanitize_field_target(
        (ball[0] + dir_x * arc_r, ball[1] + dir_y * arc_r)
    )
    return nav, committed_side, False
