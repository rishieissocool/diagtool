# DiagTool — robot control, diagnostics & calibration

A standalone tool for **finding and fixing the control-latency problem** with
the TurtleRabbit robots, and for **calibrating** them.

It is built *on top of* `2026-TeamControl` — it imports and reuses that
project's network code, vision decode, movement limits, field geometry and
wall-braking so it drives and measures the robots **exactly the way the real
program does**. It **never modifies** `2026-TeamControl` or `RobotFramework`;
it only reads them.

---

## What it measures (everything that contributes to "delay")

| Diagnostic | What it tells you |
|---|---|
| **Vision health** | frame rate, inter-frame jitter, dropped/duplicate frames, vision latency (`recv − t_sent`), and stationary pose noise (your measurement floor). |
| **Telemetry health** | onboard telemetry rate (**currently ~1 Hz**), jitter, age, voltage, parse errors. |
| **Command → motion latency** | the headline number: time from issuing a velocity command to the robot visibly starting to move — the *whole* felt delay. |
| **Stop latency & coast** | time to halt after a stop command, plus how far the robot coasts (overshoot). |
| **Linear speed scale & drift** | commanded vs actual speed (the calibration `speed_scale`), lateral drift (mm/m) and heading drift (deg/m). |
| **Rotation latency & w-scale** | turn-start latency and actual-vs-commanded angular speed. |

Each measurement reports mean / median / std / min / max / p05 / p95 across
several trials, per robot.

It then derives **calibration values** (in the same shape as
`2026-TeamControl/calibration.json`) and produces a ranked **root-cause
report**.

---

## Suspected delay sources it checks (found by reading the code)

These are baked into the report and get promoted to **CONFIRMED** when the
live measurements back them up:

1. **Telemetry is sent at 1 Hz** — `RobotFramework/config/Main.yaml` `Sender_interval: 1000`. PC sees robot state up to ~1 s stale.
2. **Robot never flushes its UDP buffer** — `RobotFramework/Networks/UDP.cpp` has `clear_buffer()` but `RobotFramework.cpp` never calls it and reads one datagram per 20 ms tick → stale-command backlog that grows into "huge delay".
3. **Wheel velocity units** — `RobotFramework/Math/wheel_math.cpp` divides by *wheel radius* (rad/s) while moteus `PositionMode.velocity` is in *rev/s* (factor 2π). Shows up as `speed_scale` far from 1.0.
4. **SAFE-mode freeze** — `wheel_math.cpp` returns all-zero wheel speeds if any axis exceeds a limit; the fallback `W_LIMIT` is `0.1 rad/s`, so almost any turn freezes the robot.
5. **Robot read cadence** — 20 ms read interval + 30 ms blocking recv timeout adds tens of ms.
6. **Server send rate** — `dispatcher/dispatch.py` caps real-robot sends at 20 Hz (50 ms).
7. **No timestamp echo** — the command timestamp is sent but the robot ignores it, so no true round-trip time is possible (DiagTool infers latency from vision instead).
8. **Telemetry send-target bug** — `UDP.cpp` sets the destination *after* `sendto()`.

Full detail (with file/line refs and suggested fixes) is in every generated
`report.txt`.

---

## Safety — it will not hit walls

Driving reuses TeamControl's own values:
* linear speed clamped to `MAX_SPEED`, angular to `MAX_W`,
* `ball_nav.wall_brake` slowdown near boundaries,
* a hard **outward-velocity guard** that removes any velocity pushing past the
  safety margin (`FIELD_MARGIN + ROBOT_RADIUS`),
* tests **stop before the margin** and **never drive a robot vision can't see**.

---

## Requirements

DiagTool runs in the **same Python environment as TeamControl**. It needs:

* Python ≥ 3.10
* `protobuf` (for vision decode) and `PyYAML`
* `PySide6` (only for the GUI — the CLI works without it)

```
pip install protobuf PyYAML PySide6
```

It locates `2026-TeamControl/src` automatically (sibling folder). Override with
`--teamcontrol-src <path>` or the `TEAMCONTROL_SRC` env var.

Robot IPs/ports and the vision config come from
`2026-TeamControl/src/TeamControl/utils/ipconfig.yaml` — the same file
TeamControl uses. (For real robots, ensure that file points at the real
`192.168.1.x` addresses, not `127.0.0.1`.)

---

## Running

### GUI (default)
```
python run_diag.py
```
Pick a robot, click a diagnostic (or **Run Full Sweep**), watch the field view,
log and live vision/telemetry health. **STOP** aborts immediately.

### CLI (headless / over SSH)
```
python run_diag.py --cli list                       # list robots
python run_diag.py --cli status                     # live vision/telemetry
python run_diag.py --cli health --robot Y0
python run_diag.py --cli test command_latency --robot Y0
python run_diag.py --cli sweep --real               # full battery, all real robots
python run_diag.py --cli sweep --robots Y0,Y1 --tests command_latency,speed_scale
```

Test names: `vision_health`, `telemetry_health`, `command_latency`,
`stop_latency`, `speed_scale`, `angular`.

Tune trial counts / speeds / thresholds in `diag_settings.yaml`.

---

## Output

Each sweep writes a timestamped folder under `diagtool/output/`:

* `report.txt` — readable summary: environment, per-robot calibration, ranked
  suspected delay sources with fixes.
* `report.json` — everything, machine-readable.
* `diag_calibration.json` — derived calibration. The report also includes a
  block in **TeamControl's `calibration.json` format** so you can copy values
  across *deliberately* — DiagTool never overwrites TeamControl's file.

---

## Layout

```
diagtool/
  run_diag.py            entry point (GUI default, --cli fallback)
  diag_settings.yaml     tunable thresholds
  diag/
    bridge.py            safe imports from 2026-TeamControl
    metrics.py           stats / rate / jitter helpers
    safety.py            wall-aware velocity limiting (TeamControl limits)
    sources.py           VisionSource + TelemetrySource (threaded)
    commander.py         continuous wall-safe command streamer
    diagnostics.py       the test battery
    calibrator.py        results -> calibration values
    report.py            root-cause analysis + report writing
    engine.py            lifecycle / orchestration
    cli.py               headless front-end
    ui/                  PySide6 dashboard
```
