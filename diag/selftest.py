"""
selftest.py — offline, hardware-free self-test for DiagTool.

Runs WITHOUT robots, vision, telemetry, or a network connection, so you can
verify the tool is healthy from anywhere (e.g. when you're not on the team's
field network). Nothing is ever transmitted to a robot.

It exercises, in layers:

  * metrics        — stats / rate / jitter / Welford math
  * calibrator     — results -> calibration values + warnings
  * report         — root-cause confirm() rules, ranking, render + write
  * diagnostics    — motion math (MotionTracker, angle unwrap, geometry)
  * safety         — wall-aware velocity limiting (synthetic + real constants)
  * config/engine  — ipconfig parsing + robot inventory
  * wiring         — Engine.start()/stop() on local sockets (tolerant)
  * simulated sweep — the REAL Diagnostic classes driven against a fake
                      vision+commander robot model, with shrunk timings, so the
                      whole measurement pipeline runs end-to-end offline.

Each check is PASS / FAIL / SKIP. SKIP means an optional dependency or the
environment was missing (not a defect) — e.g. protobuf absent, or a UDP port
already in use. The process exits non-zero only when something FAILs.

Run:
    python run_diag.py --cli selftest
    python run_diag.py --cli selftest --no-sim     # skip the ~8s sim sweep
    python -m diag.selftest
"""

from __future__ import annotations

import json
import math
import tempfile
import time
from pathlib import Path

from . import bridge, safety
from . import report as report_mod
from . import metrics as metrics_mod
from .metrics import summarize, fmt_stat, RateMeter, Welford
from .calibrator import summarize_robot, teamcontrol_block, build_calibration
from .diagnostics import (
    MotionTracker, _unwrap, _ang_diff, _centered,
    DiagContext, RobotRef, BY_NAME, DEFAULTS,
)
from .sources import PoseSample


# ── tiny test framework (no pytest dependency) ────────────────────────────

class SkipTest(Exception):
    """Raise to mark a check SKIP (env/optional-dep missing, not a defect)."""


_TESTS = []   # list of (section, name, tier, fn)


def _test(section: str, tier=1):
    def deco(fn):
        _TESTS.append((section, fn.__name__, tier, fn))
        return fn
    return deco


def approx(a, b, tol: float = 1e-6) -> bool:
    return abs(float(a) - float(b)) <= tol


def _need_movement():
    """Skip a check unless TeamControl's real movement constants import."""
    try:
        safety.limits()
    except bridge.TeamControlNotFound as e:
        raise SkipTest(f"TeamControl not found: {e}")
    except bridge.MissingDependency as e:
        raise SkipTest(str(e))
    except ImportError as e:
        raise SkipTest(f"dependency missing: {e}")


# A synthetic limit set so the pure-math safety + geometry checks need no
# TeamControl import at all (matches the real SSL field shape closely enough).
_SYNTH_LIM = safety.Limits(
    max_speed=1.0, max_w=1.8, half_len=2000.0, half_wid=1500.0,
    robot_radius=90.0, field_margin=300.0,
)

# A custom (drag-set) test zone: roughly the +x half of the field. zone_x_max is
# deliberately past the safe box (1610) to prove it gets clamped. safe box here
# is +/-1610 (x) by +/-1110 (y); the zone resolves to x[0,1610] y[-600,600].
_ZONE_LIM = safety.Limits(
    max_speed=1.0, max_w=1.8, half_len=2000.0, half_wid=1500.0,
    robot_radius=90.0, field_margin=300.0,
    zone_x_min=0.0, zone_x_max=9999.0, zone_y_min=-600.0, zone_y_max=600.0)


# ══════════════════════════════════════════════════════════════════════════
# 1. metrics
# ══════════════════════════════════════════════════════════════════════════

@_test("metrics")
def summarize_empty():
    s = summarize([])
    assert s["n"] == 0
    assert s["mean"] is None and s["median"] is None and s["std"] is None
    assert s["unit"] == ""


@_test("metrics")
def summarize_basic():
    s = summarize([1, 2, 3, 4, 5], "ms")
    assert s["n"] == 5
    assert approx(s["mean"], 3.0)
    assert approx(s["median"], 3.0)
    assert s["min"] == 1 and s["max"] == 5
    assert approx(s["std"], math.sqrt(2.0))           # population std of 1..5
    assert s["unit"] == "ms"


@_test("metrics")
def summarize_filters_none_and_nan():
    s = summarize([1.0, None, float("nan"), 3.0])
    assert s["n"] == 2 and approx(s["mean"], 2.0)


@_test("metrics")
def percentile_edges():
    xs = list(range(11))                              # 0..10
    assert approx(metrics_mod._percentile(xs, 0.0), 0.0)
    assert approx(metrics_mod._percentile(xs, 100.0), 10.0)
    assert approx(metrics_mod._percentile(xs, 50.0), 5.0)
    assert approx(metrics_mod._percentile([7.0], 95.0), 7.0)


@_test("metrics")
def fmt_stat_text():
    assert fmt_stat(summarize([])) == "no samples"
    out = fmt_stat(summarize([1.0, 2.0, 3.0], "ms"))
    assert "mean=2.00" in out and "ms" in out
    scaled = fmt_stat(summarize([1.0, 2.0, 3.0], "s"), scale=1000.0, digits=0)
    assert "mean=2000" in out.replace("ms", "") or "mean=2000" in scaled


@_test("metrics")
def ratemeter_rate_and_age():
    rm = RateMeter()
    base = 100.0
    for i in range(11):
        rm.tick(base + i * 0.1)                       # 10 Hz, last ts = base+1.0
    assert rm.total == 11
    assert approx(rm.rate_hz, 10.0, 1e-3)
    ist = rm.interval_stats("ms")
    assert approx(ist["mean"], 100.0, 0.5)            # 0.1 s == 100 ms
    assert approx(rm.age(now=base + 1.0 + 5.0), 5.0)


@_test("metrics")
def ratemeter_empty():
    rm = RateMeter()
    assert rm.rate_hz == 0.0
    assert rm.age() is None and rm.last_ts is None


@_test("metrics")
def welford_matches_population_std():
    w = Welford()
    for x in (2, 4, 4, 4, 5, 5, 7, 9):
        w.add(x)
    assert approx(w.mean, 5.0)
    assert approx(w.std, 2.0)


# ══════════════════════════════════════════════════════════════════════════
# 2. calibrator
# ══════════════════════════════════════════════════════════════════════════

def _good_results() -> dict:
    return {
        "speed_scale": {
            "speed_ratio": {"mean": 1.0},
            "actual_speed_ms": {"mean": 0.30},
            "lateral_drift_mm_per_m": {"mean": 5.0},
            "heading_drift_deg_per_m": {"mean": 2.0},
        },
        "angular": {
            "w_scale": {"mean": 1.0},
            "rotation_latency_ms": {"mean": 120.0},
        },
        "stop_latency": {
            "coast_distance_mm": {"mean": 40.0},
            "stop_latency_ms": {"mean": 80.0},
        },
        "command_latency": {"latency_ms": {"mean": 130.0}},
    }


@_test("calibrator")
def summarize_robot_clean():
    out = summarize_robot("Y1", _good_results())
    assert out["robot"] == "Y1"
    assert out["speed_scale"] == 1.0
    assert out["command_latency_ms"] == 130.0
    assert out["stop_overshoot_mm"] == 40.0
    assert out["warnings"] == []


@_test("calibrator")
def summarize_robot_flags_unit_bug():
    res = _good_results()
    res["speed_scale"]["speed_ratio"]["mean"] = 6.28      # rad/s vs rev/s factor
    out = summarize_robot("Y1", res)
    assert any("speed_scale" in w for w in out["warnings"])


@_test("calibrator")
def summarize_robot_flags_no_motion():
    res = _good_results()
    res["command_latency"] = {"latency_ms": {"mean": None},
                              "no_motion_trials": 6, "trials": 6}
    out = summarize_robot("Y1", res)
    assert any("NO motion" in w for w in out["warnings"])


@_test("calibrator")
def teamcontrol_block_shape():
    out = summarize_robot("Y1", _good_results())
    blk = teamcontrol_block(out)
    assert blk["speed_scale"] == 1.0
    assert "lateral_drift_per_m" in blk and "stop_overshoot_mm" in blk
    assert "_diagtool_extras" in blk
    # None-safety: a robot with no data must still yield usable defaults
    empty = teamcontrol_block(summarize_robot("Z9", {}))
    assert empty["speed_scale"] == 1.0 and empty["stop_overshoot_mm"] == 0.0


@_test("calibrator")
def build_calibration_indexes_by_label():
    cal = build_calibration({"Y1": _good_results(), "B0": {}})
    assert set(cal["robots"]) == {"Y1", "B0"}
    assert set(cal["teamcontrol_format"]) == {"Y1", "B0"}


# ══════════════════════════════════════════════════════════════════════════
# 3. report (root-cause rules + render/write)
# ══════════════════════════════════════════════════════════════════════════

@_test("report")
def confirm_rules():
    assert report_mod._telemetry_confirm({"telemetry_rate_hz": 1.0}) == "confirmed"
    assert report_mod._telemetry_confirm({"telemetry_rate_hz": 60.0}) == "cleared"
    assert report_mod._telemetry_confirm({}) == "suspect"

    assert report_mod._latency_confirm({"max_cmd_latency_ms": 300.0}) == "confirmed"
    assert report_mod._latency_confirm({"max_cmd_latency_ms": 50.0}) == "suspect"

    assert report_mod._speed_scale_confirm({"speed_scale": 6.28}) == "confirmed"
    assert report_mod._speed_scale_confirm({"speed_scale": 1.0}) == "cleared"
    assert report_mod._speed_scale_confirm({}) == "suspect"

    assert report_mod._angular_freeze_confirm({"angular_no_motion": True}) == "confirmed"
    assert report_mod._angular_freeze_confirm({"angular_no_motion": False}) == "suspect"


@_test("report")
def evidence_distillation():
    ev = report_mod.build_evidence(
        {"Y1": _good_results()},
        {"fps": 60.0}, {"rate_hz": 1.0})
    assert approx(ev["telemetry_rate_hz"], 1.0)
    assert approx(ev["vision_fps"], 60.0)
    assert approx(ev["speed_scale"], 1.0)
    assert approx(ev["max_cmd_latency_ms"], 130.0)


@_test("report")
def suspects_ranked_confirmed_first():
    ev = {"telemetry_rate_hz": 1.0, "max_cmd_latency_ms": 300.0,
          "speed_scale": 6.28, "angular_no_motion": True}
    out = report_mod.analyze_suspects(ev)
    assert len(out) == len(report_mod.SUSPECTS)
    rank = {"confirmed": 0, "suspect": 1, "cleared": 2}
    order = [rank[s["status"]] for s in out]
    assert order == sorted(order), "suspects not sorted confirmed-first"
    assert out[0]["status"] == "confirmed"


@_test("report")
def build_and_render_report():
    all_results = {"Y1": _good_results()}
    cal = build_calibration(all_results)
    vs = {"fps": 60.0, "drops": 0, "dupes": 0, "visible": 1,
          "sent_latency_ms": summarize([10.0, 12.0, 11.0], "ms")}
    ts = {"rate_hz": 1.0, "packets": 5, "robots_seen": ["Y1"]}
    rep = report_mod.build_report(all_results, cal, vs, ts,
                                  meta={"robots": ["Y1"]})
    txt = report_mod.render_text(rep)
    assert "DIAGTOOL REPORT" in txt
    assert "Y1" in txt
    assert "speed_scale" in txt


@_test("report")
def write_report_round_trips():
    all_results = {"Y1": _good_results()}
    cal = build_calibration(all_results)
    rep = report_mod.build_report(all_results, cal, {"fps": 60.0}, {"rate_hz": 1.0},
                                  meta={"robots": ["Y1"]})
    tmp = Path(tempfile.mkdtemp(prefix="diagtool_selftest_"))
    try:
        paths = report_mod.write_report(tmp, rep)
        for key in ("json", "txt", "calibration"):
            assert Path(paths[key]).is_file(), f"missing {key}"
        loaded = json.loads(Path(paths["json"]).read_text(encoding="utf-8"))
        assert loaded["meta"]["robots"] == ["Y1"]
        assert "suspected_delay_sources" in loaded
    finally:
        for p in tmp.glob("*"):
            p.unlink()
        tmp.rmdir()


# ══════════════════════════════════════════════════════════════════════════
# 4. diagnostics — motion math & geometry (pure)
# ══════════════════════════════════════════════════════════════════════════

@_test("diagnostics-math")
def angle_unwrap_handles_wrap():
    # raw step from +3.1 to -3.1 is physically +0.083 rad, not -6.2
    cont = _unwrap(3.1, 3.1, -3.1)
    assert approx(cont, 3.1 + (-6.2 + 2 * math.pi))


@_test("diagnostics-math")
def ang_diff_shortest():
    assert approx(_ang_diff(0.1, -0.1), 0.2)
    assert approx(abs(_ang_diff(math.pi - 0.05, -math.pi + 0.05)), 0.1)


@_test("diagnostics-math")
def centered_sums_to_zero():
    out = _centered([10.0, 12.0, 14.0])
    assert approx(sum(out), 0.0)


@_test("diagnostics-math")
def motion_tracker_linear_velocity():
    tr = MotionTracker(1.0)
    for i in range(31):                               # 0.5 s @ 60 Hz, 300 mm/s +x
        t = i / 60.0
        tr.add(t, 300.0 * t, 0.0, 0.0)
    assert tr.ready
    vx, vy = tr.velocity()
    assert approx(vx, 300.0, 1.0)
    assert approx(vy, 0.0, 1.0)
    assert approx(tr.speed(), 300.0, 1.0)


@_test("diagnostics-math")
def motion_tracker_angular_rate():
    tr = MotionTracker(1.0)
    for i in range(31):                               # 0.5 rad/s
        t = i / 60.0
        tr.add(t, 0.0, 0.0, 0.5 * t)
    assert tr.ready
    assert approx(tr.omega(), 0.5, 0.01)


@_test("diagnostics-math")
def open_direction_and_room():
    ctx = DiagContext(None, None, None, {}, _SYNTH_LIM)
    diag = BY_NAME["command_latency"](ctx)
    # near centre -> picks +x (most room)
    ux, uy = diag._dir_to_open((0.0, 0.0, 0.0))
    assert approx(ux, 1.0) and approx(uy, 0.0)
    # offset toward +x,+y -> points back toward centre
    ux, uy = diag._dir_to_open((1000.0, 1000.0, 0.0))
    assert ux < 0 and uy < 0
    # room ahead in +x from centre = half_len - safe_margin
    room = diag._room_ahead((0.0, 0.0, 0.0), 1.0, 0.0)
    assert approx(room, _SYNTH_LIM.half_len - _SYNTH_LIM.safe_margin)


@_test("diagnostics-math")
def open_direction_and_room_follow_custom_zone():
    # with an off-centre test zone, "open" is the ZONE centre (800, 0), and room
    # is measured to the ZONE edges, not the field/symmetric-arena edges.
    ctx = DiagContext(None, None, None, {}, _ZONE_LIM)
    diag = BY_NAME["command_latency"](ctx)
    assert approx(_ZONE_LIM.arena_cx, 805.0) and approx(_ZONE_LIM.arena_cy, 0.0)
    # near the zone's low-x edge -> head +x, into the zone
    ux, uy = diag._dir_to_open((50.0, 0.0, 0.0))
    assert ux > 0
    ux, _, room = diag._open_x_dir((50.0, 0.0, 0.0))
    assert approx(ux, 1.0) and approx(room, 1610.0 - 50.0)
    # near the zone's high-x edge -> head back -x toward centre
    ux2, _, _ = diag._open_x_dir((1500.0, 0.0, 0.0))
    assert approx(ux2, -1.0)
    # room from zone centre to its +x edge
    assert approx(diag._room_ahead((805.0, 0.0, 0.0), 1.0, 0.0), 1610.0 - 805.0)


# ══════════════════════════════════════════════════════════════════════════
# 5. safety — pure (synthetic limits, no TeamControl) ...
# ══════════════════════════════════════════════════════════════════════════

@_test("safety")
def clamp_scalar():
    assert safety.clamp(5.0, 0.0, 1.0) == 1.0
    assert safety.clamp(-5.0, 0.0, 1.0) == 0.0
    assert safety.clamp(0.5, 0.0, 1.0) == 0.5


@_test("safety")
def clamp_velocity_caps_speed_and_w():
    vx, vy, w = safety.clamp_velocity(3.0, 0.0, 10.0, _SYNTH_LIM)
    assert approx(math.hypot(vx, vy), _SYNTH_LIM.max_speed)
    assert approx(w, _SYNTH_LIM.max_w)
    # below limits passes through, direction preserved
    vx, vy, w = safety.clamp_velocity(0.3, 0.4, 0.5, _SYNTH_LIM)
    assert approx(math.hypot(vx, vy), 0.5) and approx(w, 0.5)


@_test("safety")
def outward_guard_zeroes_only_outward():
    lim = _SYNTH_LIM
    x_near = lim.half_len - lim.safe_margin + 1.0
    gx, gy = safety.guard_outward_world(x_near, 0.0, 1.0, 0.0, lim)
    assert gx == 0.0                                   # pushing further out -> removed
    gx, gy = safety.guard_outward_world(x_near, 0.0, -1.0, 0.0, lim)
    assert gx == -1.0                                  # pulling back in -> allowed
    gx, gy = safety.guard_outward_world(0.0, 0.0, 1.0, 1.0, lim)
    assert gx == 1.0 and gy == 1.0                     # mid-field -> untouched


@_test("safety")
def safe_zone_and_wall_distance():
    lim = _SYNTH_LIM
    assert safety.in_safe_zone(0.0, 0.0, lim) is True
    assert safety.in_safe_zone(lim.half_len, 0.0, lim) is False
    assert approx(safety.nearest_wall_dist(0.0, 0.0, lim),
                  min(lim.half_len, lim.half_wid))


# A tighter arena: 300mm extra inset, 200mm brake zone.
_ARENA_LIM = safety.Limits(
    max_speed=1.0, max_w=1.8, half_len=2000.0, half_wid=1500.0,
    robot_radius=90.0, field_margin=300.0, boundary_inset=300.0, brake_zone=200.0)


@_test("safety")
def arena_inset_shrinks_drive_zone():
    lim = _ARENA_LIM
    # arena = half - safe_margin(390) - inset(300)
    assert approx(lim.arena_half_len, 2000.0 - 390.0 - 300.0)   # 1310
    assert approx(lim.arena_half_wid, 1500.0 - 390.0 - 300.0)   # 810
    assert safety.in_safe_zone(1300.0, 0.0, lim) is True
    assert safety.in_safe_zone(1320.0, 0.0, lim) is False        # inside old margin,
    assert safety.in_safe_zone(1320.0, 0.0, _SYNTH_LIM) is True  # but arena stops it


@_test("safety")
def arena_brakes_then_hard_stops_outward():
    lim = _ARENA_LIM
    ax = lim.arena_half_len                       # 1310
    # well inside: full speed
    vx, vy = safety.limit_to_arena(0.0, 0.0, 0.30, 0.0, lim)
    assert approx(vx, 0.30) and approx(vy, 0.0)
    # halfway into the 200mm brake zone -> ~half speed
    vx, _ = safety.limit_to_arena(ax - 100.0, 0.0, 0.30, 0.0, lim)
    assert approx(vx, 0.30 * 0.5, 1e-6)
    # at/over the arena edge, pushing further out -> zero
    vx, _ = safety.limit_to_arena(ax + 50.0, 0.0, 0.30, 0.0, lim)
    assert vx == 0.0
    # but inward (back toward centre) is never restricted
    vx, _ = safety.limit_to_arena(ax + 50.0, 0.0, -0.30, 0.0, lim)
    assert approx(vx, -0.30)


@_test("safety")
def arena_pipeline_blocks_outside_inset():
    _need_movement()
    lim = safety.limits(boundary_inset=500.0, brake_zone=400.0)
    # a robot sitting beyond the arena, commanded straight at the near wall
    x = lim.arena_half_len + 1.0
    vx_r, vy_r, _, blocked = safety.safe_world_velocity((x, 0.0, 0.0),
                                                        lim.max_speed, 0.0, 0.0, lim)
    assert blocked is True
    assert approx(math.hypot(vx_r, vy_r), 0.0, 1e-9)


@_test("safety")
def custom_zone_clamps_to_safe_box():
    lim = _ZONE_LIM
    assert lim.has_custom_zone
    xlo, xhi, ylo, yhi = lim.arena_bounds()
    assert approx(xlo, 0.0)
    assert approx(xhi, 1610.0)          # zone_x_max 9999 clamped to the safe box
    assert approx(ylo, -600.0) and approx(yhi, 600.0)
    assert approx(lim.arena_cx, 805.0) and approx(lim.arena_cy, 0.0)   # (0+1610)/2


@_test("safety")
def custom_zone_in_safe_zone_is_rectangular():
    lim = _ZONE_LIM
    assert safety.in_safe_zone(800.0, 0.0, lim) is True
    assert safety.in_safe_zone(-50.0, 0.0, lim) is False     # left of the zone
    assert safety.in_safe_zone(50.0, 0.0, lim) is True
    assert safety.in_safe_zone(800.0, 700.0, lim) is False    # above the zone
    # the SAME point is fine under the default symmetric arena
    assert safety.in_safe_zone(-50.0, 0.0, _SYNTH_LIM) is True


@_test("safety")
def custom_zone_brakes_and_hard_stops_at_its_edges():
    lim = _ZONE_LIM                                  # x[0,1610], brake_zone 400
    # well inside, headed +x -> full speed
    vx, _ = safety.limit_to_arena(800.0, 0.0, 0.30, 0.0, lim)
    assert approx(vx, 0.30)
    # 100 mm inside the +x edge -> braked to a quarter
    vx, _ = safety.limit_to_arena(1510.0, 0.0, 0.30, 0.0, lim)
    assert approx(vx, 0.30 * (100.0 / 400.0), 1e-6)
    # just left of (outside) the low-x edge, pushing further out -> hard zero
    vx, _ = safety.limit_to_arena(-10.0, 0.0, -0.30, 0.0, lim)
    assert vx == 0.0
    # but heading back into the zone is never restricted
    vx, _ = safety.limit_to_arena(-10.0, 0.0, 0.30, 0.0, lim)
    assert approx(vx, 0.30)


# ── safety against the REAL TeamControl constants (needs vendored tree) ────

@_test("safety")
def real_limits_are_sane():
    _need_movement()
    lim = safety.limits()
    assert lim.max_speed > 0 and lim.max_w > 0
    assert lim.half_len > 0 and lim.half_wid > 0
    assert lim.safe_margin == lim.field_margin + lim.robot_radius
    assert lim.has_custom_zone is False                 # no zone by default


@_test("safety")
def limits_factory_carries_and_clamps_zone():
    _need_movement()
    lim = safety.limits(half_len=2000.0, half_wid=1500.0,
                        zone_x_min=0.0, zone_x_max=9999.0,
                        zone_y_min=-300.0, zone_y_max=300.0)
    assert lim.has_custom_zone
    xlo, xhi, ylo, yhi = lim.arena_bounds()
    assert approx(xlo, 0.0) and approx(xhi, lim.safe_half_len)   # clamped to box
    assert approx(ylo, -300.0) and approx(yhi, 300.0)


@_test("safety")
def real_pipeline_blocks_at_wall():
    _need_movement()
    lim = safety.limits()
    # mid-field, modest command -> not blocked, magnitude preserved under rotation
    vx_r, vy_r, w, blocked = safety.safe_world_velocity((0.0, 0.0, 0.0),
                                                        0.2, 0.0, 0.0, lim)
    assert blocked is False
    assert math.hypot(vx_r, vy_r) <= lim.max_speed + 1e-6
    vx_r, vy_r, *_ = safety.safe_world_velocity((0.0, 0.0, math.pi / 2),
                                                0.2, 0.0, 0.0, lim)
    assert approx(math.hypot(vx_r, vy_r), 0.2, 1e-6)
    # at the +x wall pushing further out -> blocked
    x_wall = lim.half_len - lim.safe_margin + 1.0
    *_, blocked2 = safety.safe_world_velocity((x_wall, 0.0, 0.0),
                                              0.2, 0.0, 0.0, lim)
    assert blocked2 is True


# ══════════════════════════════════════════════════════════════════════════
# 6. config / engine inventory
# ══════════════════════════════════════════════════════════════════════════

@_test("config/engine")
def robotinfo_label_and_real():
    from .engine import RobotInfo
    r = RobotInfo(True, 1, "192.168.1.7", 50514, "B")
    assert r.label == "Y1" and r.is_real is True
    r2 = RobotInfo(False, 0, "127.0.0.1", 50514, "A")
    assert r2.label == "B0" and r2.is_real is False
    assert RobotInfo(True, 2, "localhost", 50514, "C").is_real is False


@_test("config/engine")
def settings_merge_with_defaults():
    from .engine import load_settings
    s = load_settings()
    # load_settings() seeds from DEFAULTS then overlays diag_settings.yaml,
    # so every default key must survive the merge.
    for key in DEFAULTS:
        assert key in s, f"missing default {key}"
    # values that diag_settings.yaml sets are real and sane after the overlay
    assert s["latency_trials"] >= 1
    assert s["test_speed_ms"] > 0


@_test("config/engine")
def field_geometry_resolution():
    from .engine import resolve_field_half
    cfg = (2000.0, 1500.0)                              # wrong field_config
    # vision reports the true field -> used
    hl, hw, src = resolve_field_half((4500, 2230), None, None, cfg)
    assert approx(hl, 2250.0) and approx(hw, 1115.0) and src == "vision"
    # no vision, explicit settings -> used
    hl, hw, src = resolve_field_half(None, 4500, 2230, cfg)
    assert approx(hl, 2250.0) and approx(hw, 1115.0) and src == "settings"
    # both present -> the SMALLER (most conservative) per axis wins
    hl, hw, src = resolve_field_half((5000, 3000), 4500, 2230, cfg)
    assert approx(hl, 2250.0) and approx(hw, 1115.0), (hl, hw)
    assert "vision" in src and "settings" in src
    # neither real source -> field_config fallback
    hl, hw, src = resolve_field_half(None, None, None, cfg)
    assert approx(hl, 2000.0) and approx(hw, 1500.0) and src == "field_config"
    # a zero/garbage vision size is ignored
    hl, hw, src = resolve_field_half((0, 0), 4500, 2230, cfg)
    assert src == "settings"


@_test("config/engine")
def ip_conflict_detection():
    from .engine import RobotInfo, ip_conflicts
    rs = [
        RobotInfo(True, 3, "192.168.1.4", 50514, "D"),    # Y3
        RobotInfo(False, 1, "192.168.1.4", 50514, "B"),   # B1  -> conflict
        RobotInfo(True, 1, "192.168.1.7", 50514, "B"),    # Y1  -> unique
        RobotInfo(False, 0, "127.0.0.1", 50514, "A"),     # loopback -> ignored
        RobotInfo(True, 0, "127.0.0.1", 50514, "A"),      # loopback -> ignored
    ]
    c = ip_conflicts(rs)
    assert c == {"192.168.1.4": ["B1", "Y3"]}, c          # only the real clash


@_test("config/engine")
def engine_parses_ipconfig_inventory():
    from .engine import Engine
    try:
        eng = Engine()
    except bridge.MissingDependency as e:
        raise SkipTest(str(e))
    except (bridge.TeamControlNotFound, ImportError) as e:
        raise SkipTest(f"dependency missing: {e}")
    try:
        robots = eng.robots()
        assert len(robots) >= 1
        labels = {r.label for r in robots}
        assert "Y1" in labels                          # yellow B, shellID 1
        real = eng.robots(real_only=True)
        assert all(r.is_real for r in real)
        assert eng.find_robot("Y1") is not None
        assert eng.find_robot("does-not-exist") is None
    finally:
        eng.stop()


# ══════════════════════════════════════════════════════════════════════════
# 7. wiring — Engine lifecycle on local sockets (tolerant: SKIP on env issues)
# ══════════════════════════════════════════════════════════════════════════

@_test("wiring", tier="net")
def engine_start_stop_no_transmit():
    from .engine import Engine
    try:
        eng = Engine()
    except Exception as e:                             # noqa: BLE001 — env-dependent
        raise SkipTest(f"engine construct: {type(e).__name__}: {e}")
    try:
        eng.start()
        time.sleep(0.2)
        st = eng.source_status()
        assert "vision" in st and "telemetry" in st
        # No targets were enabled, so the Commander transmits nothing.
        assert eng.commander is not None
    except OSError as e:
        raise SkipTest(f"socket unavailable here: {e}")
    finally:
        try:
            eng.stop()
        except Exception:
            pass


# ══════════════════════════════════════════════════════════════════════════
# 7b. commander — the actual send path (real Commander, recording sender)
# ══════════════════════════════════════════════════════════════════════════

class _RecordingSender:
    """Stands in for TeamControl's Sender — records sends, can fake failures."""

    def __init__(self):
        self.sent = []
        self.fail = False

    def send(self, msg, ip, port):
        if self.fail:
            raise OSError("network unreachable")
        self.sent.append((str(msg), ip, port))


class _PoseVision:
    """Minimal VisionSource stand-in returning a fixed (or no) pose."""

    def __init__(self, sample):
        self._s = sample

    def get_pose_sample(self, is_yellow, robot_id, max_age=None):
        return self._s

    def get_pose(self, is_yellow, robot_id, max_age=None):
        return self._s.pose() if self._s else None


def _make_commander(vision):
    from .commander import Commander
    try:
        cmd = Commander(vision, sender_ip=None, send_hz=50.0,
                        pose_max_age=0.30, drive_grace=0.0)
    except bridge.MissingDependency as e:
        raise SkipTest(str(e))
    except (bridge.TeamControlNotFound, ImportError) as e:
        raise SkipTest(f"dependency missing: {e}")
    rec = _RecordingSender()
    cmd._sender = rec
    return cmd, rec


def _fresh_pose():
    now = time.perf_counter()
    return PoseSample(x=0.0, y=0.0, o=0.0, confidence=1.0, frame_number=1,
                      t_perf=now, t_wall=time.time(),
                      t_capture=time.time(), t_sent=time.time())


@_test("commander")
def commander_drives_with_fresh_pose():
    cmd, rec = _make_commander(_PoseVision(_fresh_pose()))
    cmd.register(True, 1, "127.0.0.1", 50514)
    cmd.set_velocity(True, 1, vx=0.30, vy=0.0, w=0.0, frame="world", safe=True)
    cmd._send_one(cmd._targets[(True, 1)])
    assert rec.sent, "nothing was sent"
    st = cmd.stats(True, 1)
    assert st["sends"] == 1 and st["send_errors"] == 0
    assert math.hypot(st["last_sent"][0], st["last_sent"][1]) > 0.1


@_test("commander")
def commander_safe_zeroes_without_pose():
    cmd, rec = _make_commander(_PoseVision(None))
    cmd.register(True, 1, "127.0.0.1", 50514)
    cmd.set_velocity(True, 1, vx=0.30, vy=0.0, w=0.0, frame="world", safe=True)
    cmd._send_one(cmd._targets[(True, 1)])
    st = cmd.stats(True, 1)
    assert st["last_sent"] == (0.0, 0.0, 0.0), "drove a robot it can't see"
    assert st["zeroed_no_pose"] == 1
    assert rec.sent, "should still send an explicit stop"


@_test("commander")
def commander_direct_sends_without_pose():
    # direct mode mirrors the real dispatcher: command goes through with no pose,
    # but is capped to the slow blind speed (can't arena-guard what we can't see)
    cmd, rec = _make_commander(_PoseVision(None))   # blind_speed default 0.25
    cmd.register(True, 1, "127.0.0.1", 50514)
    cmd.set_velocity(True, 1, vx=0.20, vy=0.0, w=0.0, frame="body", safe=False)
    cmd._send_one(cmd._targets[(True, 1)])
    st = cmd.stats(True, 1)
    assert st["sends"] == 1 and st["zeroed_no_pose"] == 0
    assert approx(st["last_sent"][0], 0.20, 1e-6), st["last_sent"]    # under cap -> passes
    # an over-cap command is throttled to the blind speed
    cmd.set_velocity(True, 1, vx=0.90, vy=0.0, w=0.0, frame="body", safe=False)
    cmd._send_one(cmd._targets[(True, 1)])
    assert approx(cmd.stats(True, 1)["last_sent"][0], 0.25, 1e-6)


@_test("commander")
def commander_counts_send_errors():
    cmd, rec = _make_commander(_PoseVision(_fresh_pose()))
    rec.fail = True                                    # simulate a bad robot IP
    cmd.register(True, 1, "203.0.113.9", 50514)
    cmd.set_velocity(True, 1, vx=0.30, vy=0.0, w=0.0, frame="world", safe=True)
    cmd._send_one(cmd._targets[(True, 1)])
    st = cmd.stats(True, 1)
    assert st["send_errors"] == 1 and st["sends"] == 0
    assert st["last_error"] and "OSError" in st["last_error"]


@_test("commander")
def commander_set_limits_retightens_arena():
    # when vision reports a smaller field, new limits must bound driving at once
    cmd, rec = _make_commander(_PoseVision(_fresh_pose()))
    cmd.register(True, 1, "127.0.0.1", 50514)
    # a tiny arena whose edge is right at the robot (x=0): all outward x removed
    tight = safety.Limits(max_speed=1.0, max_w=1.8, half_len=490.0, half_wid=490.0,
                          robot_radius=90.0, field_margin=400.0,
                          boundary_inset=0.0, brake_zone=1.0)
    assert approx(tight.arena_half_len, 0.0)           # half - margin(490) = 0
    cmd.set_limits(tight)
    cmd.set_velocity(True, 1, vx=0.30, vy=0.0, w=0.0, frame="world", safe=True)
    cmd._send_one(cmd._targets[(True, 1)])
    st = cmd.stats(True, 1)
    assert approx(st["last_sent"][0], 0.0, 1e-9), st["last_sent"]   # clamped to arena


def _moving_pose(x, y, vx_mm_s, vy_mm_s, frame, dt=1 / 60.0):
    """A pose whose position implies the given velocity vs the previous frame."""
    now = time.perf_counter()
    return PoseSample(x=x, y=y, o=0.0, confidence=1.0, frame_number=frame,
                      t_perf=now, t_wall=time.time(),
                      t_capture=time.time(), t_sent=time.time())


@_test("commander")
def emergency_stops_robot_moving_too_fast():
    # commander with a tiny arena; robot measured moving fast toward +x edge
    cmd, rec = _make_commander(_PoseVision(None))
    tight = safety.Limits(max_speed=2.0, max_w=3.0, half_len=2250.0, half_wid=1115.0,
                          robot_radius=90.0, field_margin=300.0,
                          boundary_inset=250.0, brake_zone=300.0)
    cmd.set_limits(tight)
    cmd.register(True, 1, "127.0.0.1", 50514)
    t = cmd._targets[(True, 1)]
    # feed two frames 1/60 s apart implying ~760 mm/s in +x near the +x arena edge
    vis = _PoseVision(None)
    cmd._vision = vis
    x0 = tight.arena_half_len - 300.0
    vis._s = PoseSample(x=x0, y=0.0, o=0.0, confidence=1.0, frame_number=1,
                        t_perf=time.perf_counter(), t_wall=time.time(),
                        t_capture=time.time(), t_sent=time.time())
    cmd.set_velocity(True, 1, vx=0.10, vy=0.0, w=0.0, frame="world", safe=True)
    cmd._send_one(t)                                   # establishes velocity ref
    # next frame ~12.7 mm later in 1/60 s -> ~760 mm/s toward the edge
    vis._s = PoseSample(x=x0 + 12.7, y=0.0, o=0.0, confidence=1.0, frame_number=2,
                        t_perf=t._vref_t + 1 / 60.0, t_wall=time.time(),
                        t_capture=time.time(), t_sent=time.time())
    cmd._send_one(t)
    st = cmd.stats(True, 1)
    assert st["meas_speed_mm_s"] > 300.0, st["meas_speed_mm_s"]
    assert st["zeroed_emergency"] >= 1, st
    assert st["last_sent"] == (0.0, 0.0, 0.0), st["last_sent"]


@_test("commander")
def emergency_allows_slow_motion_with_room():
    cmd, rec = _make_commander(_PoseVision(None))
    lim = safety.limits(boundary_inset=250.0, brake_zone=300.0,
                        half_len=2250.0, half_wid=1115.0)
    cmd.set_limits(lim)
    cmd.register(True, 1, "127.0.0.1", 50514)
    t = cmd._targets[(True, 1)]
    vis = _PoseVision(None); cmd._vision = vis
    # near centre, slow (~120 mm/s) -> plenty of stopping room -> allowed
    vis._s = PoseSample(x=0.0, y=0.0, o=0.0, confidence=1.0, frame_number=1,
                        t_perf=time.perf_counter(), t_wall=time.time(),
                        t_capture=time.time(), t_sent=time.time())
    cmd.set_velocity(True, 1, vx=0.10, vy=0.0, w=0.0, frame="world", safe=True)
    cmd._send_one(t)
    vis._s = PoseSample(x=2.0, y=0.0, o=0.0, confidence=1.0, frame_number=2,
                        t_perf=t._vref_t + 1 / 60.0, t_wall=time.time(),
                        t_capture=time.time(), t_sent=time.time())
    cmd._send_one(t)
    st = cmd.stats(True, 1)
    assert st["zeroed_emergency"] == 0, st
    assert math.hypot(st["last_sent"][0], st["last_sent"][1]) > 0.05, st["last_sent"]


@_test("commander")
def stop_distance_grows_with_speed():
    cmd, _ = _make_commander(_PoseVision(None))
    d_slow = cmd._stop_distance(100.0)
    d_fast = cmd._stop_distance(760.0)
    assert d_fast > d_slow > 0
    # at the measured ~760 mm/s the stopping distance is well over a metre
    assert d_fast > 1000.0, d_fast


@_test("commander")
def commander_grace_bridges_brief_dropout():
    # fresh pose first, then vision goes blank — within grace it keeps driving
    vision = _PoseVision(_fresh_pose())
    from .commander import Commander
    try:
        cmd = Commander(vision, sender_ip=None, send_hz=50.0,
                        pose_max_age=0.30, drive_grace=1.0)
    except (bridge.MissingDependency, bridge.TeamControlNotFound, ImportError) as e:
        raise SkipTest(str(e))
    cmd._sender = _RecordingSender()
    cmd.register(True, 1, "127.0.0.1", 50514)
    cmd.set_velocity(True, 1, vx=0.30, vy=0.0, w=0.0, frame="world", safe=True)
    cmd._send_one(cmd._targets[(True, 1)])             # caches a good pose
    vision._s = None                                   # vision drops out
    cmd._send_one(cmd._targets[(True, 1)])             # still within grace
    st = cmd.stats(True, 1)
    assert st["zeroed_no_pose"] == 0, "grace did not bridge the dropout"
    assert math.hypot(st["last_sent"][0], st["last_sent"][1]) > 0.1


# ══════════════════════════════════════════════════════════════════════════
# 8. simulated end-to-end sweep — real Diagnostics vs a fake robot model
# ══════════════════════════════════════════════════════════════════════════

_SIM_SETTINGS = dict(DEFAULTS)
_SIM_SETTINGS.update({
    "settle_s": 0.1, "accel_settle_s": 0.1, "max_run_s": 0.6,
    "health_window_s": 0.3, "latency_trials": 2, "stop_trials": 2,
    "angular_trials": 2, "speed_passes": 2, "poll_s": 0.003,
    "pose_max_age_s": 5.0, "vel_window_s": 0.15, "stop_buffer_mm": 200.0,
    "onset_disp_mm": 8.0, "onset_angle_rad": 0.04,
    "moving_mm_s": 80.0, "stopped_mm_s": 40.0,
    "test_speed_ms": 0.30, "test_w_rads": 0.25,
    "spin_trials": 2, "spin_w_rads": 0.5, "spin_seconds": 0.3,
})


class _SimRobot:
    """A first-order robot: command latency, accel/decel ramp, scale factors."""

    def __init__(self, speed_scale=1.0, w_scale=1.0, latency=0.04):
        self.x = self.y = self.o = 0.0
        self.vx = self.vy = self.w = 0.0          # actual mm/s, mm/s, rad/s
        self.cmd = (0.0, 0.0, 0.0)                # world m/s, m/s, rad/s
        self.cmd_t = time.perf_counter()
        self.last_update = self.cmd_t
        self.speed_scale = speed_scale
        self.w_scale = w_scale
        self.latency = latency
        self.lin_acc = 3000.0                     # mm/s^2
        self.ang_acc = 12.0                       # rad/s^2

    def set_cmd(self, vx, vy, w):
        self.cmd = (vx, vy, w)
        self.cmd_t = time.perf_counter()

    @staticmethod
    def _approach(cur, target, dv):
        if cur < target:
            return min(cur + dv, target)
        if cur > target:
            return max(cur - dv, target)
        return cur

    def update(self):
        now = time.perf_counter()
        dt = now - self.last_update
        if dt <= 0:
            return
        self.last_update = now
        active = (now - self.cmd_t) >= self.latency
        tvx = self.cmd[0] * 1000.0 * self.speed_scale if active else 0.0
        tvy = self.cmd[1] * 1000.0 * self.speed_scale if active else 0.0
        tw = self.cmd[2] * self.w_scale if active else 0.0
        self.vx = self._approach(self.vx, tvx, self.lin_acc * dt)
        self.vy = self._approach(self.vy, tvy, self.lin_acc * dt)
        self.w = self._approach(self.w, tw, self.ang_acc * dt)
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.o += self.w * dt


class _SimVision:
    def __init__(self, robot: _SimRobot, key, fps: float = 100.0):
        self.robot = robot
        self.key = key
        self.fps = fps
        self.t0 = time.perf_counter()

    def get_pose_sample(self, is_yellow, robot_id, max_age=None):
        if (bool(is_yellow), int(robot_id)) != self.key:
            return None
        self.robot.update()
        now = time.perf_counter()
        wall = time.time()
        return PoseSample(
            x=self.robot.x, y=self.robot.y, o=self.robot.o,
            confidence=1.0, frame_number=int((now - self.t0) * self.fps),
            t_perf=now, t_wall=wall,
            t_capture=wall - 0.008, t_sent=wall - 0.006,
        )

    def get_pose(self, is_yellow, robot_id, max_age=None):
        s = self.get_pose_sample(is_yellow, robot_id, max_age)
        return s.pose() if s else None

    def visible_robots(self):
        return [self.key]

    def status(self):
        return {
            "available": True, "error": None, "fps": self.fps,
            "frames_total": 1000, "drops": 0, "dupes": 0,
            "interval_ms": summarize([10.0] * 5, "ms"),
            "sent_latency_ms": summarize([6.0] * 5, "ms"),
            "capture_latency_ms": summarize([8.0] * 5, "ms"),
            "visible": 1, "age_s": 0.0,
        }


class _SimCommander:
    def __init__(self, robot: _SimRobot, key):
        self.robot = robot
        self.key = key
        self._last_sent = (0.0, 0.0, 0.0)

    def register(self, *a, **k):
        pass

    def enable(self, *a, **k):
        pass

    def set_velocity(self, is_yellow, robot_id, vx=0.0, vy=0.0, w=0.0,
                     frame="world", kick=0, dribble=0, safe=True):
        if (bool(is_yellow), int(robot_id)) != self.key:
            return
        self.robot.set_cmd(float(vx), float(vy), float(w))
        self._last_sent = (float(vx), float(vy), float(w))

    def stop_robot(self, is_yellow, robot_id):
        self.set_velocity(is_yellow, robot_id, 0.0, 0.0, 0.0)

    def stop_all(self):
        pass

    def last_sent(self, is_yellow, robot_id):
        return self._last_sent

    def stats(self, is_yellow, robot_id):
        return {"sends": 0, "send_errors": 0, "zeroed_no_pose": 0,
                "zeroed_safety": 0, "last_sent": self._last_sent}

    def reset_stats(self, is_yellow, robot_id):
        pass


class _SimTelemetry:
    def __init__(self, key):
        self.key = key

    def status(self):
        return {
            "available": True, "error": None, "packets": 6,
            "parse_errors": 0, "unattributed": 0, "rate_hz": 1.0,
            "interval_ms": summarize([1000.0] * 5, "ms"),
            "robots_seen": [f"{'Y' if self.key[0] else 'B'}{self.key[1]}"],
        }

    def robot_status(self, is_yellow, robot_id):
        if (bool(is_yellow), int(robot_id)) != self.key:
            return {"seen": False}
        return {
            "seen": True, "rate_hz": 1.0, "age_s": 0.2,
            "interval_ms": summarize([1000.0] * 5, "ms"),
            "voltage": 12.3, "ball": False, "robot_ts_ms": 123,
        }


def _run_sim(name: str, speed_scale=1.0, w_scale=1.0) -> dict:
    robot = _SimRobot(speed_scale=speed_scale, w_scale=w_scale)
    key = (True, 1)
    ctx = DiagContext(_SimVision(robot, key), _SimTelemetry(key),
                      _SimCommander(robot, key), _SIM_SETTINGS, _SYNTH_LIM)
    ref = RobotRef(key[0], key[1], "127.0.0.1", 50514)
    noop = lambda *a, **k: None                        # noqa: E731
    return BY_NAME[name](ctx).run(ref, noop, noop, lambda: False)


@_test("simulated sweep", tier="sim")
def sim_vision_health():
    res = _run_sim("vision_health")
    assert "error" not in res
    assert res["pose_noise"]["samples"] >= 1
    assert res["status"]["fps"] > 0


@_test("simulated sweep", tier="sim")
def sim_telemetry_health():
    res = _run_sim("telemetry_health")
    assert res["robot"]["seen"] is True
    assert res["global"]["rate_hz"] == 1.0


@_test("simulated sweep", tier="sim")
def sim_command_latency():
    res = _run_sim("command_latency")
    assert "error" not in res, res.get("error")
    assert res["latency_ms"]["n"] >= 1, "no motion detected in simulation"
    m = res["latency_ms"]["mean"]
    assert 5.0 <= m <= 1500.0, f"sim latency {m:.0f} ms out of plausible range"


@_test("simulated sweep", tier="sim")
def sim_stop_latency():
    res = _run_sim("stop_latency")
    assert "error" not in res, res.get("error")
    assert res["stop_latency_ms"]["n"] >= 1, "robot never reached/stopped"
    assert res["coast_distance_mm"]["n"] >= 1


@_test("simulated sweep", tier="sim")
def sim_speed_scale_recovers_unity():
    res = _run_sim("speed_scale", speed_scale=1.0)
    assert "error" not in res, res.get("error")
    assert res["speed_ratio"]["n"] >= 1, "no usable speed pass"
    ratio = res["speed_ratio"]["mean"]
    assert 0.5 <= ratio <= 1.5, f"sim speed ratio {ratio:.2f} unexpected"


@_test("simulated sweep", tier="sim")
def sim_angular_detects_rotation():
    res = _run_sim("angular")
    assert "error" not in res, res.get("error")
    assert res["no_motion_trials"] < res["trials"], "no rotation detected"
    assert res["w_scale"]["n"] >= 1


@_test("simulated sweep", tier="sim")
def sim_spin_calibration():
    res = _run_sim("spin")
    assert "error" not in res, res.get("error")
    assert res["no_motion_trials"] < res["trials"], "no spin detected"
    assert res["w_scale"]["n"] >= 1
    # a pure-spin command makes the sim robot rotate without translating, so the
    # measured centre drift must be ~0 (this is the calibration's headline metric)
    cd = res["center_drift_mm"]["mean"]
    assert cd is not None and cd < 50.0, f"unexpected spin centre drift {cd} mm"


# ══════════════════════════════════════════════════════════════════════════
# runner
# ══════════════════════════════════════════════════════════════════════════

def _run_one(fn):
    try:
        fn()
        return "PASS", ""
    except SkipTest as e:
        return "SKIP", str(e)
    except AssertionError as e:
        return "FAIL", (str(e) or "assertion failed")
    except Exception as e:                             # noqa: BLE001
        return "FAIL", f"{type(e).__name__}: {e}"


def run_selftest(log=None, include_sim: bool = True,
                 include_net: bool = True, stop=None) -> dict:
    """Run the whole suite. Returns a result dict; streams lines via `log`."""
    log = log or (lambda *a: None)
    stop = stop or (lambda: False)

    results = []
    counts = {"PASS": 0, "FAIL": 0, "SKIP": 0}
    section = None

    log("DiagTool self-test — offline, no robots required.")
    for sec, name, tier, fn in _TESTS:
        if stop():
            log("\n[aborted]")
            break
        if tier == "sim" and not include_sim:
            continue
        if tier == "net" and not include_net:
            continue
        if sec != section:
            log(f"\n-- {sec} " + "-" * max(0, 56 - len(sec)))
            section = sec
        status, detail = _run_one(fn)
        counts[status] += 1
        results.append((sec, name, status, detail))
        line = f"  [{status}] {name}"
        if detail:
            line += f"  -- {detail}"
        log(line)

    total = sum(counts.values())
    ok = counts["FAIL"] == 0
    summary = _summary_text(counts, ok, results)
    log("\n" + summary)
    return {
        "ok": ok, "total": total,
        "passed": counts["PASS"], "failed": counts["FAIL"],
        "skipped": counts["SKIP"], "results": results, "summary": summary,
    }


def _summary_text(counts, ok, results) -> str:
    lines = ["=" * 64,
             f"  SELF-TEST: {counts['PASS']} passed, {counts['FAIL']} failed, "
             f"{counts['SKIP']} skipped",
             "=" * 64]
    fails = [(s, n, d) for (s, n, st, d) in results if st == "FAIL"]
    if fails:
        lines.append("  Failures:")
        for s, n, d in fails:
            lines.append(f"    - {s} :: {n} -- {d}")
    skips = [(s, n, d) for (s, n, st, d) in results if st == "SKIP"]
    if skips:
        lines.append("  Skipped (optional dep / environment):")
        for s, n, d in skips:
            lines.append(f"    - {s} :: {n} -- {d}")
    lines.append("  RESULT: " + ("ALL GREEN" if ok else "FAILURES PRESENT"))
    return "\n".join(lines)


def main(argv=None) -> int:
    import argparse
    import sys

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass

    p = argparse.ArgumentParser(
        prog="diagtool-selftest",
        description="Offline self-test for DiagTool (no robots required).")
    p.add_argument("--no-sim", action="store_true",
                   help="skip the simulated end-to-end sweep (~8s)")
    p.add_argument("--no-net", action="store_true",
                   help="skip the socket/engine wiring checks")
    args = p.parse_args(argv)

    rep = run_selftest(log=print, include_sim=not args.no_sim,
                       include_net=not args.no_net)
    return 0 if rep["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
