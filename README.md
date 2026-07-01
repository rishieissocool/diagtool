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

The **real field size comes from the SSL-Vision geometry packet** (`field_length`
/ `field_width`), not a hardcoded constant — so the arena matches the actual
arena. Until vision reports it, the `field_length_mm` / `field_width_mm` in
`diag_settings.yaml` are used (and act as a hard cap: the arena never exceeds the
smaller of vision vs. those). The resolved size is logged on start and shown in
the field view. *(A wrong hardcoded width is exactly what put a robot into a wall
before this was fixed.)*

Every command is bounded to a conservative **drive arena** — the field minus
TeamControl's keep-off margin (`FIELD_MARGIN + ROBOT_RADIUS`) minus an extra
`boundary_inset_mm`. With the defaults the arena sits ~0.6 m inside every wall.

* linear speed clamped to `MAX_SPEED`, angular to `MAX_W`,
* **braking ramp**: the outward velocity is scaled down over `brake_zone_mm`
  before the arena edge, so a robot is already crawling when it reaches the
  line — then any outward component is **hard-zeroed** at the boundary (it can
  always still drive back toward centre),
* **predictive emergency stop** — the commander measures the robot's *actual*
  velocity from vision (not the commanded one) and cuts power the instant its
  real stopping distance (a reaction-time + coast model) would carry it past the
  arena edge, and immediately if it is moving faster than `safety_max_speed_mm_s`.
  This is what protects a miscalibrated robot that moves several times faster
  than commanded, coasts over a metre, or drifts sideways,
* this guard runs on the commander for **every** send — including manual jog and
  direct mode — so a robot vision can see is *never* driven out of the arena,
* tests also **stop before the arena edge**, cap travel at `max_travel_mm`, and
  **never drive a robot vision can't see** (direct/jog without vision is capped
  to a slow `direct_blind_speed_ms`),
* the GUI field view draws the arena (orange dashed box) so you can see it.

Make it tighter or looser in `diag_settings.yaml` (`boundary_inset_mm`,
`brake_zone_mm`, `max_travel_mm`, `direct_blind_speed_ms`).

### Restrict testing to part of the field (drag a test zone)

At a competition you often only get **half a field** (or one corner). In the GUI
**Diagnostics** tab, **drag a box on the field view** to set a custom *test zone*:
driving, jog and the whole test battery are then kept inside that rectangle — not
just a smaller centred box, but wherever you draw it. The zone is the cyan box
(with its size in mm); a **Full field** button clears it. It's always clamped to
the keep-off-walls safe margin, so you can't accidentally drag a robot into a
wall, and it's **saved** (to `test_zone.yaml`) so it survives a restart. You can
also pin it from config via the `test_zone_*_mm` keys. Zone editing is locked
while a test is running.

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
The dashboard has three tabs:

* **Competition** — a live, glanceable wall of cards, one per robot, for use
  *during a match*: **battery gauge** (voltage → %, green/amber/red), vision and
  telemetry link (with telemetry rate), ball-in-dribbler, position/heading and
  robot state. The whole card tints by health so a robot that goes dark, loses
  its link, or runs low on battery jumps out without reading numbers. A summary
  header shows robots online, low batteries, vision fps, telemetry rate and the
  resolved field size. Click any card to jump to that robot's tests.
* **Robots** — a gallery with a **picture of every robot** plus its online state
  and battery. Drop real photos into `diag/ui/assets/robots/` named by label
  (`Y0.png`, `B1.jpg`, …) and they replace the drawn placeholders automatically
  (or point `robot_photo_dir` in `diag_settings.yaml` at another folder).
* **Drive** — move **many robots at once**. Pick a scope (all / a team / just the
  grSim robots / just the real ones), set the speed, and **hold** a direction
  button to drive every robot in scope; release or **STOP ALL** to halt. Each
  robot is driven through its own target (grSim packet or RobotCommand), so the
  same pad works in the simulator and on the field. Any robot vision can see is
  still arena-braked and emergency-stopped.
* **Auto-Calibrate** — **hands-off calibration of a whole team**, one robot at a
  time. Pick **our team colour** (Yellow/Blue) and **our side** of the field, tick
  **"restrict to our half"** (the common case — you only get one side) and the
  drive zone shrinks to that half (clamped to the keep-off-walls box). Fix any IPs
  inline (applies live), tick the robots, press **Start**: each robot is probed,
  then the full battery — including a dedicated **spin-in-place calibration** —
  runs on it while results stream into a table. **Every robot is different**, so
  each is measured and reported individually. A **per-robot report folder** plus a
  **combined report + `calibration_summary.csv`** are written under
  `output/autocal_<timestamp>/`. Unreachable robots are **skipped, not retried**,
  so one dead robot never stalls the batch. It drives **only** through the same
  wall-safe Commander every other test uses — one robot moves at a time, kept
  inside the zone, predictive-emergency-stopped — so it **cannot hit anything**.

  > **Spin calibration.** The spin test commands pure rotation (no translation)
  > and reports `w_scale`, spin latency, and the headline precision number
  > **centre-drift** — how far the robot's centre wanders while spinning. ~0 means
  > it spins cleanly in place; a large drift (or a boundary-guard trip) means the
  > wheel radii / encoder scales are mismatched and the robot walks while turning.

* **Setup** — edit each robot's **IP / port / target (real vs grSim) live**.
  Change a value and **Apply**: the running command stream switches to the new
  address immediately, no restart. **Reload from file** re-reads `ipconfig.yaml`;
  **Save to file** writes the table back to it.
* **Diagnostics** — the full test battery, manual jog, sweep and self-test:

  * **Manual jog** — Forward / Back / Left / Right / Turn buttons drive the
    selected robot in short 0.5 s pulses (direct send, body frame) so you can
    *see* it move and confirm the link before measuring anything. **■ STOP**
    halts the jog.
  * **▶ Probe + Run All Tests** — one click: streams a heartbeat and checks the
    robot is reachable (probe verdict shown on top of the report), then runs the
    whole battery on that robot.
  * Individual diagnostic buttons, **Run Full Sweep** (all real robots), and the
    red **STOP** which aborts immediately.

Duplicate-IP robots are flagged in the table (`⚠ DUP`), on the Competition cards
and in the log.

> **Battery gauge.** The robots report raw battery voltage in their telemetry;
> the gauge maps it to 0–100 % using `battery_full_v` / `battery_empty_v` in
> `diag_settings.yaml` (defaults are for a **6S LiPo**: 25.2 V full, 19.8 V
> empty). For a 4S pack use ~16.8 / 13.2. It is display-only and never affects
> driving.

> **grSim vs real robots.** Each robot has a *target*: a **real** robot is driven
> with a `RobotCommand` to its `ip:port` (exactly what RobotFramework parses); a
> **grSim** robot is driven with a grSim command packet sent to the grSim address
> (`grSim:` in `ipconfig.yaml`, default `127.0.0.1:20011`). A robot on a loopback
> IP (`127.0.0.1`) defaults to **grSim**, a LAN IP defaults to **real** — flip
> any robot in the **Setup** tab. So to test in grSim: start grSim, leave the sim
> robots on `127.0.0.1`, open the **Drive** tab, scope to *Sim (grSim)* and drive.

### CLI (headless / over SSH)
```
python run_diag.py --cli list                       # list robots (+ IP-conflict warnings)
python run_diag.py --cli status                     # live vision/telemetry
python run_diag.py --cli probe --robot B1           # is the robot actually reachable?
python run_diag.py --cli health --robot Y0
python run_diag.py --cli test command_latency --robot Y0
python run_diag.py --cli sweep --real               # full battery, all real robots
python run_diag.py --cli sweep --robots Y0,Y1 --tests command_latency,speed_scale
python run_diag.py --cli calibrate --real --half pos  # auto-calibrate every real
                                                    # robot on our +x half; writes
                                                    # per-robot + combined reports
```

**`calibrate`** is the headless twin of the **Auto-Calibrate** tab: it calibrates
each selected robot individually (full battery incl. the spin-in-place test) and
writes a per-robot report folder plus a combined report + `calibration_summary.csv`
under `output/autocal_<timestamp>/`. Pass `--half pos|neg` to keep robots on one
side of the field (recommended at a comp — robots are kept inside that half).

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
      main_window.py     tabbed window (Competition·Robots·Drive·Setup·Diagnostics)
      overview_tab.py    competition dashboard — live battery/link cards
      robots_tab.py      robot gallery — pictures + status per robot
      drive_tab.py       multi-robot movement (real + grSim)
      setup_tab.py       live IP/port/target editor
      widgets.py         battery gauge, robot pixmaps, shared cards
      field_view.py      top-down field canvas
      theme.py           dark stylesheet
      assets/robots/     drop <LABEL>.png robot photos here (optional)
```
