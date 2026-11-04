"""Examples reach an existing programs folder: new ones are copied on start, the user's files are never touched."""
from argparse import Namespace

from moveit_gui.app import prepare_dirs


def test_new_examples_are_added_and_existing_files_kept(tmp_path):
    root = tmp_path / "pkg"
    (root / "programs").mkdir(parents=True)
    (root / "policies").mkdir()
    (root / "programs" / "example_old.yaml").write_text("old: package")
    (root / "programs" / "example_new.yaml").write_text("new: package")
    (root / "programs" / "notes.yaml").write_text("not an example")
    progs = tmp_path / "data" / "programs"
    progs.mkdir(parents=True)
    (progs / "example_old.yaml").write_text("old: edited by the user")
    args = Namespace(profiles_dir=str(tmp_path / "prof"), programs_dir=str(progs), policies_dir=str(tmp_path / "pol"))
    prepare_dirs(args, root)
    assert (progs / "example_old.yaml").read_text() == "old: edited by the user"
    assert (progs / "example_new.yaml").read_text() == "new: package"
    assert not (progs / "notes.yaml").exists()                  # only examples are added to an existing folder
