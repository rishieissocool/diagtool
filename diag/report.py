"""
report.py — root-cause analysis + human/JSON report writing.

Two parts:

  1. SUSPECTS — a curated knowledge base of the delay sources found by reading
     2026-TeamControl and RobotFramework. Each suspect cites the exact file so
     the team can act, explains *why* it adds delay, and carries a `confirm()`
     rule that promotes it from SUSPECT to CONFIRMED when the live measurements
     back it up. (DiagTool only reads that code — it never edits it.)

  2. Rendering — build_report() assembles measurements + calibration +
     suspects into a structured doc; write_report() saves report.json and a
     readable report.txt under an output folder.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

from .metrics import fmt_stat


# ── Knowledge base of suspected delay sources ─────────────────────────────
# severity: 1 (highest) .. 5. Each `confirm` takes an `evidence` dict and
# returns "confirmed" | "suspect" | "cleared".

def _telemetry_confirm(ev):
    r = ev.get("telemetry_rate_hz")
    if r is None:
        return "suspect"
    return "confirmed" if r < 5.0 else "cleared"


def _latency_confirm(ev):
    m = ev.get("max_cmd_latency_ms")
    if m is None:
        return "suspect"
    return "confirmed" if m > 150.0 else "suspect"


def _speed_scale_confirm(ev):
    s = ev.get("speed_scale")
    if s is None:
        return "suspect"
    return "confirmed" if not (0.7 <= s <= 1.4) else "cleared"


def _angular_freeze_confirm(ev):
    return "confirmed" if ev.get("angular_no_motion") else "suspect"


SUSPECTS = [
    {
        "id": "telemetry_1hz",
        "severity": 1,
        "title": "Onboard telemetry is sent only once per second",
        "where": "RobotFramework/config/Main.yaml:5 (Sender_interval: 1000); "
                 "consumed in RobotFramework.cpp:359",
        "why": "The PC receives robot state (voltage/ball/state) at ~1 Hz, so "
               "any closed-loop or monitoring decision based on telemetry is up "
               "to ~1 s stale. It also makes true round-trip timing impossible.",
        "fix": "Lower Sender_interval to ~20–50 ms (20–50 Hz). Telemetry is a "
               "tiny UDP string; this is cheap.",
        "confirm": _telemetry_confirm,
    },
    {
        "id": "no_socket_flush",
        "severity": 1,
        "title": "Robot never flushes its UDP receive buffer (stale-command backlog)",
        "where": "RobotFramework/Networks/UDP.cpp:60 (clear_buffer exists) but "
                 "RobotFramework.cpp main loop never calls it; it reads ONE "
                 "datagram per 20 ms tick (RobotFramework.cpp:268).",
        "why": "If the server ever sends faster than the robot drains the "
               "socket, datagrams queue in the OS buffer and the robot keeps "
               "acting on OLD commands — control lag that grows over time and "
               "feels like 'huge delay'.",
        "fix": "Each receive tick, drain to the newest datagram (loop recvfrom "
               "with MSG_DONTWAIT, keep the last) or call clear_buffer() before "
               "the blocking receive. Act only on the freshest command.",
        "confirm": _latency_confirm,
    },
    {
        "id": "wheel_unit_scale",
        "severity": 2,
        "title": "Wheel velocity may be in rad/s while moteus expects rev/s",
        "where": "RobotFramework/Math/wheel_math.cpp:71-85 divides by "
                 "WHEEL_RADIUS (gives rad/s); Telemetry.cpp:43 feeds it to "
                 "moteus PositionMode.velocity, which is in revolutions/s.",
        "why": "rad/s vs rev/s differ by 2π (~6.28). A mismatch makes the robot "
               "move at the wrong speed, so PD controllers chase/overshoot and "
               "motion looks laggy/unstable. (A gearbox ratio can partly mask "
               "it.) The measured speed_scale quantifies the real factor.",
        "fix": "Divide linear wheel speed by wheel circumference (2*pi*r), not "
               "radius, and apply the gear ratio, so the moteus command is in "
               "rev/s. Then re-measure speed_scale (target ~1.0).",
        "confirm": _speed_scale_confirm,
    },
    {
        "id": "safe_mode_wlimit",
        "severity": 2,
        "title": "SAFE mode can silently freeze the robot on rotation",
        "where": "RobotFramework/Math/wheel_math.cpp:33-47 returns stop_vel if "
                 "any axis exceeds the limit; fallback W_LIMIT is 0.1 rad/s "
                 "(wheel_math.cpp:23) when Safety.yaml fails to load.",
        "why": "In SAFE mode (the default), a command slightly over a limit "
               "makes calculate() return all-zero wheel speeds — the robot just "
               "stops instead of clamping. With the 0.1 rad/s fallback, almost "
               "any turn freezes it.",
        "fix": "Use CAPPED mode for play, or raise/clamp instead of zeroing. "
               "Ensure Safety.yaml actually loads (wLimit 3.15).",
        "confirm": _angular_freeze_confirm,
    },
    {
        "id": "robot_read_cadence",
        "severity": 3,
        "title": "Robot reads commands every 20 ms with a 30 ms blocking timeout",
        "where": "RobotFramework/config/Main.yaml:4 (Reciver_interval: 20); "
                 "RobotFramework/Networks/UDP.cpp:34 (SO_RCVTIMEO 30 ms).",
        "why": "A fresh command can wait up to one read tick before it's seen, "
               "and the blocking recv adds jitter. Tens of ms of avoidable lag.",
        "fix": "Read more often (5–10 ms) and non-blocking, taking the newest "
               "datagram; keep the watchdog stop on real timeout.",
        "confirm": lambda ev: "suspect",
    },
    {
        "id": "dispatcher_rate_50ms",
        "severity": 3,
        "title": "Server sends to real robots at only 20 Hz (50 ms)",
        "where": "2026-TeamControl/src/TeamControl/dispatcher/dispatch.py:175 "
                 "(sends only if last + 0.05 < now).",
        "why": "Caps command freshness at 50 ms on the server side; combined "
               "with the robot's read cadence this stacks into the felt delay.",
        "fix": "Raise the per-robot send rate (e.g. 60–100 Hz) if the network "
               "and robots keep up; measure command_latency before/after.",
        "confirm": lambda ev: "suspect",
    },
    {
        "id": "no_timestamp_echo",
        "severity": 4,
        "title": "Command timestamp is sent but never echoed (no true RTT)",
        "where": "RobotCommand serialises time_set as the 7th field "
                 "(robot_command.py:66); Networks/decode.cpp:5 parses it into "
                 "`time` and then ignores it; telemetry never returns it.",
        "why": "Without the robot echoing the command timestamp back, the PC "
               "cannot measure a true closed-loop round-trip time; DiagTool "
               "infers latency from vision instead.",
        "fix": "Echo the last command's timestamp in the telemetry string "
               "(e.g. ',cmd_ts=<value>'); the PC can then compute exact RTT.",
        "confirm": lambda ev: "suspect",
    },
    {
        "id": "udp_send_target_bug",
        "severity": 4,
        "title": "Telemetry send sets the destination AFTER sending",
        "where": "RobotFramework/Networks/UDP.cpp:69-78 — sendto() runs before "
                 "target_addr.sin_port / sin_addr are assigned.",
        "why": "The first telemetry packet goes to an uninitialised address, "
               "and each packet uses the address captured on the *previous* "
               "receive. Combined with 1 Hz telemetry this loses early packets.",
        "fix": "Set target_addr (port + client IP) BEFORE calling sendto().",
        "confirm": lambda ev: "suspect",
    },
]


def build_evidence(all_results: dict, vision_status: dict,
                   telemetry_status: dict) -> dict:
    """Distil measurements into the scalar signals the confirm() rules use."""
    speed_scales, cmd_lats = [], []
    angular_no_motion = False
    for res in all_results.values():
        ss = res.get("speed_scale", {}).get("speed_ratio", {})
        if ss.get("mean") is not None:
            speed_scales.append(ss["mean"])
        cl = res.get("command_latency", {}).get("latency_ms", {})
        if cl.get("mean") is not None:
            cmd_lats.append(cl["mean"])
        if res.get("angular", {}).get("no_motion_trials"):
            angular_no_motion = True
    return {
        "telemetry_rate_hz": (telemetry_status or {}).get("rate_hz"),
        "vision_fps": (vision_status or {}).get("fps"),
        "max_cmd_latency_ms": max(cmd_lats) if cmd_lats else None,
        "speed_scale": (sum(speed_scales) / len(speed_scales)
                        if speed_scales else None),
        "angular_no_motion": angular_no_motion,
    }


def analyze_suspects(evidence: dict) -> list[dict]:
    out = []
    for s in SUSPECTS:
        status = s["confirm"](evidence)
        out.append({k: s[k] for k in ("id", "severity", "title", "where",
                                       "why", "fix")} | {"status": status})
    # confirmed first, then by severity
    rank = {"confirmed": 0, "suspect": 1, "cleared": 2}
    out.sort(key=lambda d: (rank.get(d["status"], 9), d["severity"]))
    return out


# ── Report assembly ───────────────────────────────────────────────────────

def build_report(all_results, calibration, vision_status, telemetry_status,
                 meta=None) -> dict:
    evidence = build_evidence(all_results, vision_status, telemetry_status)
    return {
        "meta": (meta or {}) | {"generated": time.strftime("%Y-%m-%d %H:%M:%S")},
        "environment": {
            "vision": vision_status,
            "telemetry": telemetry_status,
        },
        "results": all_results,
        "calibration": calibration,
        "evidence": evidence,
        "suspected_delay_sources": analyze_suspects(evidence),
    }


def render_text(report: dict) -> str:
    L = []
    add = L.append
    add("=" * 72)
    add("  DIAGTOOL REPORT — robot control / latency / calibration")
    add("=" * 72)
    add(f"generated : {report['meta'].get('generated')}")
    if report["meta"].get("robots"):
        add(f"robots    : {', '.join(report['meta']['robots'])}")
    add("")

    env = report["environment"]
    vs, ts = env.get("vision") or {}, env.get("telemetry") or {}
    add("-- Environment --------------------------------------------------------")
    if vs:
        add(f"  vision   : fps={vs.get('fps')} drops={vs.get('drops')} "
            f"dupes={vs.get('dupes')}  visible={vs.get('visible')}")
        sl = vs.get("sent_latency_ms") or {}
        if sl.get("n"):
            add(f"             vision latency (recv - t_sent): {fmt_stat(sl)}")
    else:
        add("  vision   : (no data)")
    if ts:
        add(f"  telemetry: rate={ts.get('rate_hz')} Hz packets={ts.get('packets')} "
            f"seen={ts.get('robots_seen')}")
    add("")

    add("-- Calibration per robot ---------------------------------------------")
    for label, rs in (report["calibration"].get("robots") or {}).items():
        add(f"  [{label}]")
        add(f"     speed_scale       : {rs.get('speed_scale')}   "
            f"(actual {rs.get('actual_speed_ms')} m/s)")
        add(f"     w_scale           : {rs.get('w_scale')}")
        add(f"     lateral drift     : {rs.get('lateral_drift_per_m')} mm/m")
        add(f"     heading drift     : {rs.get('heading_drift_deg_per_m')} deg/m")
        add(f"     stop overshoot    : {rs.get('stop_overshoot_mm')} mm")
        add(f"     command latency   : {rs.get('command_latency_ms')} ms")
        add(f"     rotation latency  : {rs.get('rotation_latency_ms')} ms")
        for w in rs.get("warnings", []):
            add(f"     [!] {w}")
    add("")

    add("-- Suspected delay sources (ranked) ----------------------------------")
    tag = {"confirmed": "CONFIRMED", "suspect": "suspect ", "cleared": "cleared  "}
    for s in report["suspected_delay_sources"]:
        add(f"  [{tag.get(s['status'], '?')}] (sev {s['severity']}) {s['title']}")
        add(f"      where: {s['where']}")
        add(f"      why  : {s['why']}")
        add(f"      fix  : {s['fix']}")
        add("")
    add("=" * 72)
    return "\n".join(L)


def write_report(out_dir, report: dict) -> dict:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    json_path = out / "report.json"
    txt_path = out / "report.txt"
    cal_path = out / "diag_calibration.json"

    json_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    txt_path.write_text(render_text(report), encoding="utf-8")
    cal_path.write_text(json.dumps(report.get("calibration", {}), indent=2),
                        encoding="utf-8")
    return {"json": str(json_path), "txt": str(txt_path), "calibration": str(cal_path)}
