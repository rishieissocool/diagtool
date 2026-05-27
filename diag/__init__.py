"""
DiagTool — robot control / diagnostics / calibration tool for Team TurtleRabbits.

This package is a *separate* tool from 2026-TeamControl. It re-uses
2026-TeamControl's proven network, vision and movement code (read-only,
by importing it) but never modifies it, and never touches RobotFramework.

Its job: drive every robot through a battery of safe, wall-aware tests,
measure *everything* that contributes to control latency and mis-tracking
(vision rate/jitter, telemetry rate/age, command->motion latency, stop
latency, linear/angular speed scale, drift), then compute calibration
values and a root-cause report so the team can fix the "huge delays"
problem.

Layout:
    diag.bridge       — locate + import 2026-TeamControl modules safely
    diag.metrics      — online stats / rate / jitter helpers
    diag.safety       — wall-aware velocity clamping (uses TeamControl limits)
    diag.sources      — live vision + telemetry receivers (threaded)
    diag.commander    — continuous wall-safe command sender (threaded)
    diag.diagnostics  — the test battery
    diag.calibrator   — aggregate results -> calibration + recommendations
    diag.report       — write JSON + human-readable reports
    diag.engine       — lifecycle that ties it all together
    diag.cli          — headless command-line front-end
    diag.ui           — PySide6 dashboard (optional)
"""

__version__ = "0.1.0"
