# DiagTool — robot control, diagnostics & calibration

A standalone tool for **finding and fixing the control-latency problem** with
the TurtleRabbit robots, and for **calibrating** them.

It is built *on top of* `2026-TeamControl` — it reuses that project's network
code, vision decode, movement limits, field geometry and wall-braking so it
drives and measures the robots **exactly the way the real program does**. It
**never modifies** `2026-TeamControl` or `RobotFramework`; it only reads them.

DiagTool is **self-contained**: the specific TeamControl modules it needs are
vendored under [`vendor/TeamControl`](vendor/README.md), so it runs with no
external `2026-TeamControl` checkout. (Developers can still point it at a live
tree — see *Requirements* below.)

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

Every command is bounded to a conservative **drive arena** — the field minus
TeamControl's keep-off margin (`FIELD_MARGIN + ROBOT_RADIUS`) minus an extra
`boundary_inset_mm`. With the defaults the arena sits ~0.9 m inside every wall.

* linear speed clamped to `MAX_SPEED`, angular to `MAX_W`,
* **braking ramp**: the outward velocity is scaled down over `brake_zone_mm`
  before the arena edge, so a robot is already crawling when it reaches the
  line — then any outward component is **hard-zeroed** at the boundary (it can
  always still drive back toward centre),
* this guard runs on the commander for **every** send — including manual jog and
  direct mode — so a robot vision can see is *never* driven out of the arena,
* tests also **stop before the arena edge**, cap travel at `max_travel_mm`, and
  **never drive a robot vision can't see** (direct/jog without vision is capped
  to a slow `direct_blind_speed_ms`),
* the GUI field view draws the arena (orange dashed box) so you can see it.

Make it tighter or looser in `diag_settings.yaml` (`boundary_inset_mm`,
`brake_zone_mm`, `max_travel_mm`, `direct_blind_speed_ms`).

---

## Requirements

DiagTool ships the TeamControl code it needs (vendored), so you only install
Python packages — no separate TeamControl checkout required:

* Python ≥ 3.10
* `numpy` (vision decode), `protobuf` (vision + network decode), `PyYAML`
* `PySide6` (only for the GUI — the CLI works without it)

```
pip install -r requirements.txt
```

The vendored copy under `vendor/TeamControl` is used automatically. To develop
against a **live** TeamControl tree instead, point DiagTool at its `src` with
`--teamcontrol-src <path>` or the `TEAMCONTROL_SRC` env var (these take
precedence over the vendored copy).

Robot IPs/ports and the vision config come from DiagTool's own
[`ipconfig.yaml`](ipconfig.yaml) in this folder. (For real robots, ensure it
points at the real `192.168.1.x` addresses, not `127.0.0.1`.)

---

## Running

### GUI (default)
```
python run_diag.py
```
Pick a robot, then:
* **Manual jog** — Forward / Back / Left / Right / Turn buttons drive the
  selected robot in short 0.5 s pulses (direct send, body frame) so you can
  *see* it move and confirm the link before measuring anything. **■ STOP** halts
  the jog.
* **▶ Probe + Run All Tests** — one click: streams a heartbeat and checks the
  robot is reachable (probe verdict shown on top of the report), then runs the
  whole battery on that robot.
* Individual diagnostic buttons, **Run Full Sweep** (all real robots), and the
  red **STOP** which aborts immediately.

Duplicate-IP robots are flagged in the table (`⚠ DUP`) and the log.

### CLI (headless / over SSH)
```
python run_diag.py --cli list                       # list robots (+ IP-conflict warnings)
python run_diag.py --cli status                     # live vision/telemetry
python run_diag.py --cli probe --robot B1           # is the robot actually reachable?
python run_diag.py --cli health --robot Y0
python run_diag.py --cli test command_latency --robot Y0
python run_diag.py --cli sweep --real               # full battery, all real robots
python run_diag.py --cli sweep --robots Y0,Y1 --tests command_latency,speed_scale
```

**`probe`** is the first thing to run when a robot won't move. It streams a
harmless zero heartbeat straight to the robot's `ip:port` (add `--move 0.2` for a
gentle nudge) and listens, then gives a verdict:

* **SILENT** — PC sent commands with no errors but nothing replied → robot off,
  RobotFramework not running, or wrong IP/port in `ipconfig.yaml`.
* **ALIVE but did not move** — telemetry *is* coming back, so the robot receives
  commands; motion is frozen on the robot (SAFE-mode wheel-math / `W_LIMIT`, or
  wheel units). Fix RobotFramework, not DiagTool.
* **SEND FAILED** — the OS rejected the packets: the IP is wrong/unroutable.

> **One robot per IP.** If two labels share an IP (e.g. `B1` and `Y3` both on
> `192.168.1.4`), commands for one are acted on by whichever robot answers there
> — which looks exactly like "no motion / no telemetry". `list`, the GUI table,
> and every report flag this; give each robot a unique address.

Test names: `vision_health`, `telemetry_health`, `command_latency`,
`stop_latency`, `speed_scale`, `angular`.

`sweep --robot Y0` runs the **whole battery on one robot** (the GUI's
**▶ Run All Tests (selected robot)** button does the same).

Tune trial counts / speeds / thresholds in `diag_settings.yaml`.

### "NO MOTION within timeout" — how to read it

DiagTool streams commands the same way the real 2026 dispatcher does
(`RobotCommand` → `Sender.send(cmd, ip, port)`), but it also **instruments every
send** so a no-motion result tells you *where* the problem is. Each failed trial
now prints, e.g.:

```
trial 1: NO MOTION within timeout  [sent=78 no_pose=0 safety=0 send_err=0 last_vel=(0.30,0.00,0.00)]
```

* `sent>0, no_pose=0, send_err=0` → the PC **did** stream a real velocity and the
  robot didn't move → it's the **robot** (SAFE-mode `W_LIMIT` freeze, wheel
  units, comms on the robot side — see the suspects).
* `no_pose>0` → DiagTool zeroed the command because **vision** couldn't see the
  robot (wrong `shellID`↔pattern-ID mapping, low fps, occlusion). Fix vision /
  `ipconfig.yaml`, or raise `drive_grace_s`.
* `send_err>0` → the UDP send **failed** — almost always a wrong/unreachable
  robot IP in `ipconfig.yaml` (`last_error` shows the OS error).

To test the **command link itself** when vision is unreliable, set
`latency_direct_send: true` in `diag_settings.yaml`: the latency/rotation tests
then stream the command straight to the robot's `ip:port` with no vision gate or
wall guard — exactly like the real dispatcher — so you can watch the robot move.

### Self-test (no robots, no network)

Verify DiagTool itself is healthy from anywhere — even off the field network:

```
python run_diag.py --cli selftest          # full suite (~10s incl. sim sweep)
python run_diag.py --cli selftest --no-sim  # skip the simulated sweep (fast)
python -m diag.selftest                     # same, standalone
```

In the **GUI**, click **Self-Test (offline · no robots)**.

It runs unit checks of every module (stats, calibration, root-cause rules,
report writing, motion math, wall-safety limiting, config/inventory) **and** a
simulated end-to-end sweep: the real diagnostics are driven against a fake
vision + commander robot model with shrunk timings, so the whole measurement
pipeline runs with no hardware. Nothing is ever transmitted to a robot. Each
check is `PASS` / `FAIL` / `SKIP` (skip = an optional dependency such as
protobuf, or a busy UDP port — not a defect); the process exits non-zero only
on a real `FAIL`.

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
  requirements.txt       Python deps (numpy, protobuf, PyYAML, PySide6)
  diag_settings.yaml     tunable thresholds
  ipconfig.yaml          robot IPs/ports + vision config
  vendor/
    TeamControl/         vendored TeamControl modules DiagTool reuses
  diag/
    bridge.py            safe imports from vendored TeamControl
    metrics.py           stats / rate / jitter helpers
    safety.py            wall-aware velocity limiting (TeamControl limits)
    sources.py           VisionSource + TelemetrySource (threaded)
    commander.py         continuous wall-safe command streamer
    diagnostics.py       the test battery
    calibrator.py        results -> calibration values
    report.py            root-cause analysis + report writing
    engine.py            lifecycle / orchestration
    cli.py               headless front-end
    selftest.py          offline self-test (unit + simulated sweep)
    ui/                  PySide6 dashboard
```
