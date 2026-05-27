#!/usr/bin/env python
"""
DiagTool entry point.

Default: launch the PySide6 GUI dashboard.
Headless: pass --cli to use the command-line tool (works over SSH).

    python run_diag.py                 # GUI
    python run_diag.py --cli list      # list robots
    python run_diag.py --cli sweep --real

If PySide6 is not installed, this falls back to the CLI automatically.
"""

import os
import sys

# Make `import diag` work no matter where we're launched from.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from diag import bridge  # noqa: E402


def main() -> int:
    argv = sys.argv[1:]

    # Allow --teamcontrol-src before --cli too.
    if "--teamcontrol-src" in argv:
        i = argv.index("--teamcontrol-src")
        try:
            os.environ["TEAMCONTROL_SRC"] = argv[i + 1]
            del argv[i:i + 2]
        except IndexError:
            print("--teamcontrol-src needs a path argument")
            return 2

    if "--cli" in argv:
        argv.remove("--cli")
        from diag.cli import main as cli_main
        return cli_main(argv or ["status"])

    if not bridge.have_pyside6():
        print("PySide6 not installed — falling back to the CLI.")
        print("For the GUI:  pip install PySide6")
        print("Running 'status' (Ctrl+C to stop); use --cli for other commands.\n")
        from diag.cli import main as cli_main
        return cli_main(argv or ["status"])

    try:
        bridge.ensure_on_path()
    except bridge.TeamControlNotFound as e:
        print(str(e))
        return 2

    from diag.ui.main_window import launch
    return launch()


if __name__ == "__main__":
    raise SystemExit(main())
