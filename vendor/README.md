# vendor/

Vendored, read-only copy of the **specific** `2026-TeamControl` modules that
DiagTool reuses, so the tool is self-contained and needs no external checkout.

`diag/bridge.py` adds this directory to `sys.path`, so `import TeamControl…`
resolves here by default.

## What's here

Only the transitive import closure of the helpers `bridge.py` exposes:
network sending (`Sender`, `RobotCommand`), the SSL vision decode
(`Vision`, `Frame`, `Robot`), movement limits/geometry (`robot.constants`,
`robot.ball_nav`, `world.transform_cords`), onboard telemetry, and the
proto2 message definitions they depend on — **51 files**, not the whole
project.

## Provenance

- Source: https://github.com/WSU-TurtleRabbit/2026-TeamControl.git
- Commit: `9b512c48ee89e3760838a8dcfc716bbdf937d544` (2026-06-20)
- Path in source: `src/TeamControl/…`

The only local modification is an empty `TeamControl/utils/__init__.py`
(the source `utils/` is an implicit namespace package).

## Updating

Re-copy the same closure from an updated checkout. If TeamControl gains new
intra-package imports along these paths, re-run the closure analysis (see the
project README) so no module is missed.

These files are **not** edited by DiagTool — it reuses TeamControl's tested
code exactly as the real program runs it.
