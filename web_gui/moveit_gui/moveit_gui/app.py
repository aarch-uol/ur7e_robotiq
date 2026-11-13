"""Entry point:  ros2 run moveit_gui moveit_gui [--profile ur7e_lab] [--fake] [--program file.yaml]"""
from __future__ import annotations

import argparse
import hashlib
import json
import signal
import sys
from pathlib import Path
from typing import List


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


EXAMPLES_SEEN = ".examples.json"     # example file name -> sha256 of the version last copied in


def _sha(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


def copy_examples(d: Path, src: Path, first: bool = False) -> List[str]:
    """Copy the examples (example_*, or every file on a first start) from src into d. An example already there is
    replaced by a newer shipped version only while it is the copy put there before (nobody edited it): a changed
    copy is someone's work and stays. Returns the names copied or updated."""
    seen_f = d / EXAMPLES_SEEN
    try:
        seen = json.loads(seen_f.read_text())
        seen = seen if isinstance(seen, dict) else {}
    except (OSError, ValueError):
        seen = {}
    done = []
    for f in sorted(src.glob("*")):
        if not f.is_file() or not (first or f.name.startswith("example_")):
            continue
        dst = d / f.name
        try:
            new = f.read_bytes()
            if dst.exists():
                old = dst.read_bytes()
                if old == new:
                    seen[f.name] = _sha(new)
                    continue
                if seen.get(f.name) != _sha(old):
                    continue                 # edited here (or copied before this was kept track of): keep it
            dst.write_bytes(new)
            seen[f.name] = _sha(new)
            done.append(f.name)
        except OSError:
            pass                             # a read-only folder: the examples stay in the package
    try:
        seen_f.write_text(json.dumps(seen, indent=1, sort_keys=True))
    except OSError:
        pass
    return done


def prepare_dirs(args, root: Path):
    """Programs and policies live in the user's home. Examples are copied there on first start, an example added
    by a later version on the next start, and a newer version of an example nobody edited replaces it
    (copy_examples)."""
    profiles_dir = Path(args.profiles_dir)
    programs_dir = Path(args.programs_dir).expanduser()
    policies_dir = Path(args.policies_dir).expanduser()
    for d, src in ((programs_dir, root / "programs"), (policies_dir, root / "policies")):
        first = not d.exists()
        d.mkdir(parents=True, exist_ok=True)
        copy_examples(d, src, first)
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
