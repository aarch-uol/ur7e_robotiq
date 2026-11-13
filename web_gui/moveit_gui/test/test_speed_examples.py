"""The two mm/s scraping examples run on the fake robot and their straight moves get the speeds they set."""
import os
import yaml

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from test_program_force import cmd, s, tcp_base, tick_until  # noqa: E402,F401
from moveit_gui import transforms as tf  # noqa: E402
from moveit_gui.calibration import FixtureDef  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..", "programs")


def _run(s, d, timeout):
    """Runs the program; returns the mm/s every straight move asked the bridge for (0 = no speed set)."""
    b, seen = s.core.bridge, []
    move = b.move_pose

    def spy(*a, **kw):
        seen.append(round(1000.0 * float(kw.get("cart_speed") or 0.0), 1))
        return move(*a, **kw)
    b.move_pose = spy
    cmd(s, "program_set_yaml", text=yaml.safe_dump(d, sort_keys=False))
    cmd(s, "program_run")
    assert tick_until(s, lambda: s.core.program_result is not None, timeout)
    assert s.core.program_result[0], s.core.program_result
    return seen


def test_the_vial_example_scrapes_at_5_and_returns_at_15_mm_s(s):
    p = tcp_base(s)[:3, 3] * 1000.0
    s.core.store.put_fixture(FixtureDef("vial", "base_link", tf.mm_deg_to_T([p[0], p[1], p[2] - 130], [0, 0, 0])),
                             replace="vial" in s.core.store.fixtures)
    d = yaml.safe_load(open(os.path.join(ROOT, "example_vial_scrape_speed.yaml")))
    next(x for x in d["steps"] if x["type"] == "loop")["times"] = 2
    v = _run(s, d, 240)
    assert v == [0.0, 15.0] + [5.0, 15.0] * 2 + [30.0], v      # approach (joint move), rim, 2 x (down, up), lift


def test_the_rbf_example_scrapes_stroke_one_down_at_5_mm_s(s):
    p = tcp_base(s)[:3, 3] * 1000.0
    s.core.store.put_fixture(FixtureDef("flask", "base_link", tf.mm_deg_to_T([p[0], p[1], p[2] - 140], [0, 0, 0])),
                             replace="flask" in s.core.store.fixtures)
    d = yaml.safe_load(open(os.path.join(ROOT, "example_rbf_scrape_speed.yaml")))
    cut = next(i for i, x in enumerate(d["steps"]) if x.get("target") == "stroke_01_exit_safe")
    d["steps"] = d["steps"][:cut + 1] + \
        [x for x in d["steps"][cut + 1:] if x.get("target") == "approach_neck"][-1:] + [d["steps"][-1]]
    v = _run(s, d, 240)
    # approach (joint move), into the neck, to the top of the stroke, 3 x down the wall, out, up
    assert v == [0.0, 15.0, 15.0, 5.0, 5.0, 5.0, 15.0, 15.0], v
