"""
cli.py — headless front-end for DiagTool.

Runs the full engine without any GUI dependency, so it works over SSH on the
field machine. Examples:

    python run_diag.py --cli list
    python run_diag.py --cli status
    python run_diag.py --cli health --robot Y0
    python run_diag.py --cli test command_latency --robot Y0
    python run_diag.py --cli sweep --real
    python run_diag.py --cli sweep --robots Y0,Y1 --tests command_latency,speed_scale
"""

from __future__ import annotations

import argparse
import sys
import threading
import time

from . import bridge
from .diagnostics import ALL_DIAGNOSTICS, BY_NAME
from .report import render_text


def _log(msg=""):
    print(msg, flush=True)


def _make_stop() -> tuple[threading.Event, callable]:
    ev = threading.Event()
    return ev, ev.is_set


def _resolve_robots(engine, args):
    if getattr(args, "robots", None):
        wanted = [s.strip() for s in args.robots.split(",") if s.strip()]
        out = []
        for w in wanted:
            r = engine.find_robot(w)
            if r is None:
                _log(f"[!] unknown robot '{w}' (use 'list')")
            else:
                out.append(r)
        return out
    if getattr(args, "real", False):
        return engine.robots(real_only=True)
    if getattr(args, "robot", None):
        r = engine.find_robot(args.robot)
        return [r] if r else []
    return []


def _cmd_list(engine, args):
    _log(f"{'LABEL':6} {'TEAM':7} {'ID':3} {'IP':16} {'PORT':6} REAL")
    for r in engine.robots():
        _log(f"{r.label:6} {'yellow' if r.is_yellow else 'blue':7} "
             f"{r.robot_id:<3} {r.ip:16} {r.port:<6} {'yes' if r.is_real else 'no'}")
    _warn_conflicts(engine)
    return 0


def _warn_conflicts(engine, robots=None):
    conflicts = engine.ip_conflicts()
    if robots is not None:
        wanted = {r.ip for r in robots}
        conflicts = {ip: lbls for ip, lbls in conflicts.items() if ip in wanted}
    for ip, labels in conflicts.items():
        _log(f"[!] {ip} is shared by {', '.join(labels)} — commands for one go "
             "to whichever robot answers at that IP (or nothing). Give each robot "
             "a unique IP in ipconfig.yaml.")


def _cmd_probe(engine, args):
    r = engine.find_robot(args.robot) if args.robot else None
    if r is None:
        _log(f"probe needs a valid --robot LABEL (use 'list'). Got: {args.robot!r}")
        return 2
    move = float(args.move) if args.move else 0.0
    if move > 0:
        _log(f"[probe] direct nudge at {move:.2f} m/s — clear space around {r.label}.")
    res = engine.probe_robot(r, seconds=args.seconds, move_speed=move, log=_log)
    _log("")
    _log(f"probe {r.label} @ {res['ip']}:{res['port']} (id={res['robot_id']}): "
         f"telemetry={res['telemetry_packets']} pkts, sent={res['commands_sent']}, "
         f"send_err={res['send_errors']}, vision_motion={res['vision_motion_mm']}mm")
    return 0


def _cmd_status(engine, args):
    engine.start()
    secs = args.seconds
    _log(f"Sampling sources for {secs}s (Ctrl+C to stop early)...")
    end = time.time() + secs
    try:
        while time.time() < end:
            st = engine.source_status()
            v, t = st["vision"], st["telemetry"]
            line = (f"\rvision fps={v.get('fps')} visible={v.get('visible')} "
                    f"drops={v.get('drops')} | telemetry {t.get('rate_hz')}Hz "
                    f"pkts={t.get('packets')} seen={t.get('robots_seen')}   ")
            sys.stdout.write(line)
            sys.stdout.flush()
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    _log("")
    if st["vision"].get("error"):
        _log(f"[vision] {st['vision']['error']}")
    if st["telemetry"].get("error"):
        _log(f"[telemetry] {st['telemetry']['error']}")
    return 0


def _run_tests(engine, robots, test_names, args):
    if not robots:
        _log("No robots selected. Use --robot Y0, --robots Y0,Y1, or --real.")
        return 2
    _warn_conflicts(engine, robots)
    ev, stop = _make_stop()

    def progress(frac, text=""):
        sys.stdout.write(f"\r  [{frac * 100:5.1f}%] {text:50.50}")
        sys.stdout.flush()

    try:
        rep = engine.run_sweep(robots, test_names, log=_log,
                               progress=progress, stop=stop,
                               output_dir=args.output)
    except KeyboardInterrupt:
        ev.set()
        _log("\nInterrupted.")
        return 130
    _log("\n")
    _log(render_text(rep))
    return 0


def _cmd_health(engine, args):
    robots = _resolve_robots(engine, args) or engine.robots(real_only=True)
    return _run_tests(engine, robots, ["vision_health", "telemetry_health"], args)


def _cmd_test(engine, args):
    if args.name not in BY_NAME:
        _log(f"Unknown test '{args.name}'. Available: {', '.join(BY_NAME)}")
        return 2
    robots = _resolve_robots(engine, args)
    return _run_tests(engine, robots, [args.name], args)


def _cmd_sweep(engine, args):
    robots = _resolve_robots(engine, args) or engine.robots(real_only=True)
    tests = ([s.strip() for s in args.tests.split(",")] if args.tests
             else [d.name for d in ALL_DIAGNOSTICS])
    return _run_tests(engine, robots, tests, args)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="diagtool", description="Robot diagnostics & calibration (headless).")
    p.add_argument("--teamcontrol-src", help="path to 2026-TeamControl/src")
    p.add_argument("--output", help="output directory for reports")
    sub = p.add_subparsers(dest="command", required=True)

    sub.add_parser("list", help="list robots from ipconfig")

    sp = sub.add_parser("status", help="show live vision/telemetry status")
    sp.add_argument("--seconds", type=float, default=8.0)

    pp = sub.add_parser("probe",
                        help="is a robot actually reachable? (heartbeat + listen)")
    pp.add_argument("--robot", required=True, help="robot label, e.g. B1")
    pp.add_argument("--seconds", type=float, default=5.0)
    pp.add_argument("--move", type=float, default=0.0,
                    help="optional body-forward nudge speed m/s (default 0 = no motion)")

    for name in ("health",):
        hp = sub.add_parser(name, help="vision + telemetry health")
        _add_robot_args(hp)

    tp = sub.add_parser("test", help="run one diagnostic")
    tp.add_argument("name", choices=list(BY_NAME))
    _add_robot_args(tp)

    swp = sub.add_parser("sweep", help="run full battery + write report")
    _add_robot_args(swp)
    swp.add_argument("--tests", help="comma list of test names (default: all)")

    cp = sub.add_parser("calibrate", help="alias for sweep")
    _add_robot_args(cp)
    cp.add_argument("--tests", help="comma list of test names (default: all)")

    st = sub.add_parser("selftest",
                        help="offline self-test (no robots/network needed)")
    st.add_argument("--no-sim", action="store_true",
                    help="skip the simulated end-to-end sweep (~8s)")
    st.add_argument("--no-net", action="store_true",
                    help="skip the socket/engine wiring checks")
    return p


def _add_robot_args(sp):
    sp.add_argument("--robot", help="single robot label, e.g. Y0")
    sp.add_argument("--robots", help="comma list of labels, e.g. Y0,Y1")
    sp.add_argument("--real", action="store_true",
                    help="all robots whose ip is not loopback")


def main(argv=None) -> int:
    # Reports contain a few en/em dashes; make sure printing them to a legacy
    # Windows console can never crash the run.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass

    args = build_parser().parse_args(argv)
    if args.teamcontrol_src:
        import os
        os.environ["TEAMCONTROL_SRC"] = args.teamcontrol_src

    # selftest is self-contained: it gates its own dependencies and never
    # needs a live Engine, so run it before any TeamControl/engine setup.
    if args.command == "selftest":
        from .selftest import run_selftest
        rep = run_selftest(log=_log, include_sim=not args.no_sim,
                           include_net=not args.no_net)
        return 0 if rep["ok"] else 1

    try:
        bridge.ensure_on_path()
    except bridge.TeamControlNotFound as e:
        _log(str(e))
        return 2

    from .engine import Engine
    try:
        engine = Engine()
    except Exception as e:
        _log(f"Failed to start engine: {e}")
        return 2

    dispatch = {
        "list": _cmd_list,
        "status": _cmd_status,
        "probe": _cmd_probe,
        "health": _cmd_health,
        "test": _cmd_test,
        "sweep": _cmd_sweep,
        "calibrate": _cmd_sweep,
    }
    fn = dispatch.get(args.command)
    try:
        return fn(engine, args)
    finally:
        engine.stop()


if __name__ == "__main__":
    raise SystemExit(main())
