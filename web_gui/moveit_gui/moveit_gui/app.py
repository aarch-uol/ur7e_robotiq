"""Entry point:  ros2 run moveit_gui moveit_gui [--profile ur7e_lab] [--fake] [--program file.yaml]"""
from __future__ import annotations

import argparse
import signal
import sys
from pathlib import Path


def data_dir() -> Path:
    """profiles/, programs/, policies/: from the installed share dir, else the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("moveit_gui"))
        if (share / "profiles").is_dir():
            return share
    except Exception:
        pass
    return Path(__file__).resolve().parents[1]


def resolve_profile(arg: str, profiles_dir: Path) -> Path:
    p = Path(arg).expanduser()
    if p.suffix in (".yaml", ".yml") and p.exists():
        return p.resolve()
    cand = profiles_dir / f"{arg}.yaml"
    if cand.exists():
        return cand
    sys.exit(f"profile '{arg}' not found (looked for {p} and {cand})")


def prepare_dirs(args, root: Path):
    """Programs and policies live in the user's home. Examples are copied there on first start, and an example
    added by a later version is copied on the next start; a file already there is never overwritten."""
    profiles_dir = Path(args.profiles_dir)
    programs_dir = Path(args.programs_dir).expanduser()
    policies_dir = Path(args.policies_dir).expanduser()
    for d, src in ((programs_dir, root / "programs"), (policies_dir, root / "policies")):
        first = not d.exists()
        d.mkdir(parents=True, exist_ok=True)
        for f in src.glob("*"):
            if f.is_file() and (first or f.name.startswith("example_")) and not (d / f.name).exists():
                try:
                    (d / f.name).write_bytes(f.read_bytes())
                except OSError:
                    pass                     # a read-only folder: the examples stay in the package
    return profiles_dir, programs_dir, policies_dir


def main(argv=None):
    root = data_dir()
    ap = argparse.ArgumentParser(description="MoveIt GUI for UR + ROS 2 Jazzy")
    ap.add_argument("--profile", default="ur7e_lab", help="profile name in profiles/ or path to a YAML")
    ap.add_argument("--profiles-dir", default=str(root / "profiles"))
    ap.add_argument("--programs-dir", default=str(Path.home() / "moveit_gui_programs"),
                    help="where Open/Save look first (examples are copied here on first start)")
    ap.add_argument("--policies-dir", default=str(Path.home() / "moveit_gui_policies"))
    ap.add_argument("--program", help="program YAML to open at start")
    ap.add_argument("--fake", action="store_true", help="no ROS: simulated robot for trying the GUI")
    args, qt_args = ap.parse_known_args(argv)

    profiles_dir, programs_dir, policies_dir = prepare_dirs(args, root)

    from PyQt5 import QtCore, QtWidgets
    from .gui.main_window import MainWindow
    from .profile import RobotProfile

    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    app = QtWidgets.QApplication([sys.argv[0]] + qt_args)
    app.setStyle("Fusion")
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    tick = QtCore.QTimer()          # lets Python handle Ctrl-C while Qt runs
    tick.start(200)
    tick.timeout.connect(lambda: None)

    state = {"bridge": None, "window": None}

    def open_profile(path: Path, program: str = None):
        profile = RobotProfile.load(path)
        old_bridge, old_win = state["bridge"], state["window"]
        if old_win is not None:
            old_win.core.recorder.stop_and_wait("profile changed")   # auto-record can run here too
            old_win.close()
        if old_bridge is not None:
            old_bridge.shutdown()
        if args.fake:
            from .fake_bridge import FakeBridge
            bridge = FakeBridge(profile)
        else:
            from .ros_bridge import RosBridge
            bridge = RosBridge(profile)
        win = MainWindow(bridge, profile, profiles_dir, programs_dir, policies_dir, fake=args.fake)
        win.profile_change_requested.connect(lambda p: open_profile(Path(p)))
        if program:
            win.pages["PROGRAMS"].load_file(Path(program).expanduser())
        win.show()
        state["bridge"], state["window"] = bridge, win

    open_profile(resolve_profile(args.profile, profiles_dir), args.program)
    rc = app.exec_()
    if state["window"] is not None:
        state["window"].core.recorder.stop_and_wait("GUI closed")
    if state["bridge"] is not None:
        state["bridge"].shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
