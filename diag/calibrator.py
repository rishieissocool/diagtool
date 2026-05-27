"""
calibrator.py — turn raw diagnostic results into calibration values.

Takes the per-robot results produced by the diagnostics and derives the
correction factors the team cares about. The semantics deliberately match
TeamControl's existing calibration.json so the numbers are directly
comparable / transferable:

  speed_scale          actual / commanded linear speed   (ball_nav divides
                       commanded speed by this)
  w_scale              actual / commanded angular speed
  lateral_drift_per_m  mm of sideways drift per metre travelled
  stop_overshoot_mm    coast distance after a stop command
  command_latency_ms   command -> motion delay (informational)

DiagTool writes its own diag_calibration.json; it never overwrites
TeamControl's file. A ready-to-paste TeamControl-format block is produced so
a human can copy it across deliberately.
"""

from __future__ import annotations


def _mean(stat) -> float | None:
    if isinstance(stat, dict):
        return stat.get("mean")
    return None


def summarize_robot(label: str, results: dict) -> dict:
    """Collapse one robot's test results into calibration values."""
    ss = results.get("speed_scale", {})
    ang = results.get("angular", {})
    stop = results.get("stop_latency", {})
    lat = results.get("command_latency", {})

    out = {
        "robot": label,
        "speed_scale": _round(_mean(ss.get("speed_ratio")), 4),
        "actual_speed_ms": _round(_mean(ss.get("actual_speed_ms")), 4),
        "lateral_drift_per_m": _round(_mean(ss.get("lateral_drift_mm_per_m")), 2),
        "heading_drift_deg_per_m": _round(_mean(ss.get("heading_drift_deg_per_m")), 2),
        "w_scale": _round(_mean(ang.get("w_scale")), 4),
        "rotation_latency_ms": _round(_mean(ang.get("rotation_latency_ms")), 1),
        "stop_overshoot_mm": _round(_mean(stop.get("coast_distance_mm")), 1),
        "stop_latency_ms": _round(_mean(stop.get("stop_latency_ms")), 1),
        "command_latency_ms": _round(_mean(lat.get("latency_ms")), 1),
    }

    warnings = []
    if lat.get("no_motion_trials"):
        warnings.append(
            f"{lat['no_motion_trials']}/{lat.get('trials', '?')} latency trials "
            "saw NO motion — comms dead, robot off, or SAFE-mode limit freeze?")
    if ang.get("no_motion_trials"):
        warnings.append(
            f"{ang['no_motion_trials']}/{ang.get('trials', '?')} rotation trials "
            "saw NO rotation — check W limit / SAFE mode.")
    if out["speed_scale"] is not None and not (0.5 <= out["speed_scale"] <= 2.0):
        warnings.append(
            f"speed_scale={out['speed_scale']} is far from 1.0 — strong sign of a "
            "units/gearing mismatch in wheel velocity (see report suspects).")
    out["warnings"] = warnings
    return out


def teamcontrol_block(robot_summary: dict) -> dict:
    """A dict in TeamControl's calibration.json shape (for manual transfer)."""
    return {
        "speed_scale": robot_summary.get("speed_scale") or 1.0,
        "lateral_drift_per_m": robot_summary.get("lateral_drift_per_m") or 0.0,
        "stop_overshoot_mm": robot_summary.get("stop_overshoot_mm") or 0.0,
        "_diagtool_extras": {
            "w_scale": robot_summary.get("w_scale"),
            "command_latency_ms": robot_summary.get("command_latency_ms"),
            "rotation_latency_ms": robot_summary.get("rotation_latency_ms"),
        },
    }


def build_calibration(all_results: dict) -> dict:
    """all_results: {robot_label: {test_name: result_dict}} -> calibration doc."""
    robots = {}
    for label, res in all_results.items():
        robots[label] = summarize_robot(label, res)
    return {
        "robots": robots,
        "teamcontrol_format": {
            label: teamcontrol_block(rs) for label, rs in robots.items()
        },
    }


def _round(v, n):
    return None if v is None else round(float(v), n)
