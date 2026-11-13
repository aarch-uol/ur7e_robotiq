"""Example programs in the user's folder follow the shipped version until someone edits them."""
from moveit_gui.app import EXAMPLES_SEEN, copy_examples


def test_an_unedited_example_is_updated_and_an_edited_one_is_kept(tmp_path):
    src, d = tmp_path / "src", tmp_path / "programs"
    src.mkdir()
    d.mkdir()
    (src / "example_a.yaml").write_text("v1")
    (src / "example_b.yaml").write_text("v1")
    (src / "mine.yaml").write_text("x")
    assert copy_examples(d, src, first=True) == ["example_a.yaml", "example_b.yaml", "mine.yaml"]
    (d / "example_b.yaml").write_text("my edit")
    (src / "example_a.yaml").write_text("v2")
    (src / "example_b.yaml").write_text("v2")
    (src / "mine.yaml").write_text("y")
    assert copy_examples(d, src) == ["example_a.yaml"]
    assert (d / "example_a.yaml").read_text() == "v2"
    assert (d / "example_b.yaml").read_text() == "my edit"          # someone's work stays
    assert (d / "mine.yaml").read_text() == "x"                     # not an example: never touched again
    assert (d / EXAMPLES_SEEN).is_file()


def test_a_copy_from_before_the_record_is_kept_until_it_matches(tmp_path):
    src, d = tmp_path / "src", tmp_path / "programs"
    src.mkdir()
    d.mkdir()
    (src / "example_a.yaml").write_text("v2")
    (d / "example_a.yaml").write_text("v1")                         # copied by an older version: unknown
    assert copy_examples(d, src) == [] and (d / "example_a.yaml").read_text() == "v1"
    (d / "example_a.yaml").write_text("v2")                         # put right by hand: followed from now on
    copy_examples(d, src)
    (src / "example_a.yaml").write_text("v3")
    assert copy_examples(d, src) == ["example_a.yaml"]
