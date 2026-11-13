"""fixed_bands.py: the vision node's /scraping_vision/fixed_band_targets (no ROS, no camera)."""
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "scraping_vision"))
import fixed_bands as fb  # noqa: E402

BS_PATH = Path(os.environ.get("BAND_SORTING_PY", "/ws/scraping/src/mujoco_multiagent_rl/policy_runner/band_sorting.py"))
EDGES = [0.0003, 0.00547, 0.01064, 0.01581, 0.02098, 0.02615, 0.03132, 0.0365, 0.04167]


@pytest.fixture(scope="module")
def bs():
    if not BS_PATH.is_file():
        pytest.skip(f"runner band_sorting.py not at {BS_PATH} (set BAND_SORTING_PY)")
    return fb.load_band_sorting([BS_PATH])


def robot_config(tmp_path, **band):
    import yaml
    s = {"enabled": False, "vial_bottom_z": 0.0563, "scene_variant": "vial_default", "band_edges_m": EDGES}
    s.update(band)
    p = tmp_path / "robot_config.yaml"
    p.write_text(yaml.safe_dump({"topics": {"fixed_band_targets": "/scraping_vision/fixed_band_targets"},
                                 "servo_admittance": {"base_frame": "base_link", "ee_frame": "spatula_tip"},
                                 "band_sorting": s}))
    return p


def test_config_reads_the_runner_settings(bs, tmp_path):
    c = fb.load_config(robot_config(tmp_path), bs)
    assert c.vial_bottom_z == pytest.approx(0.0563)
    assert list(c.edges) == pytest.approx(EDGES)
    assert (c.base_frame, c.ee_frame, c.topic) == ("base_link", "spatula_tip", "/scraping_vision/fixed_band_targets")


def test_no_vial_bottom_is_refused(bs, tmp_path):
    with pytest.raises(fb.FixedBandError, match="vial_bottom_z"):
        fb.load_config(robot_config(tmp_path, vial_bottom_z=None), bs)


def test_position_biases_are_refused(bs, tmp_path):
    import yaml
    p = robot_config(tmp_path)
    d = yaml.safe_load(p.read_text())
    d["policy"] = {"obs_position_bias_xyz_ee": [0.0, 0.0, 0.003]}
    p.write_text(yaml.safe_dump(d))
    with pytest.raises(fb.FixedBandError, match="bias"):
        fb.load_config(p, bs)


def test_material_pixels_become_points_with_their_area():
    K = np.array([[400.0, 0, 50], [0, 400.0, 40], [0, 0, 1]])
    mat = np.zeros((80, 100), np.uint8)
    mat[40, 50] = 255
    mat[10, 10] = 255                                    # outside the crop
    depth = np.full((80, 100), 200.0)                    # mm
    pts, area = fb.material_points(mat, depth, K, rect=(30, 30, 40, 40), stride=1)
    assert len(pts) == 1 and pts[0].tolist() == pytest.approx([0.0, 0.0, 0.2])
    assert area[0] == pytest.approx(0.2 * 0.2 / (400 * 400))
    depth[40, 50] = 0                                    # no depth: dropped
    assert len(fb.material_points(mat, depth, K, rect=(30, 30, 40, 40), stride=1)[0]) == 0


def test_targets_match_the_sim_term(bs, tmp_path):
    """occupancy = band material / episode-start material; target = EE z - mean z of the band (sim terms.py)."""
    c = fb.load_config(robot_config(tmp_path), bs)
    bands = fb.FixedBands(c, bs, latch_frames=1)
    bottom = c.vial_bottom_z
    z = np.array([bottom + 0.002, bottom + 0.003, bottom + 0.020])      # bands 0, 0, 3
    pts = np.column_stack([np.zeros(3), np.zeros(3), z])
    w = np.array([1.0, 1.0, 2.0])
    out = bands.compute(pts, w, ee_z=bottom + 0.05)
    assert out.shape == (16,) and out.dtype == np.float32
    occ, tgt = out[:8], out[8:]
    assert occ.tolist() == pytest.approx([0.5, 0, 0, 0.5, 0, 0, 0, 0])
    assert tgt[0] == pytest.approx(0.05 - 0.0025, abs=1e-6)
    assert tgt[3] == pytest.approx(0.05 - 0.020, abs=1e-6)
    assert tgt[1] == 0.0
    out2 = bands.compute(pts[:1], w[:1], ee_z=bottom + 0.05)              # scraped: a quarter left
    assert out2[:8].sum() == pytest.approx(0.25)


def test_the_start_total_is_the_median_of_the_first_frames(bs, tmp_path):
    c = fb.load_config(robot_config(tmp_path), bs)
    bands = fb.FixedBands(c, bs, latch_frames=3)
    pt = np.array([[0.0, 0.0, c.vial_bottom_z + 0.002]])
    assert bands.compute(pt, [0.0], 0.1) is None                          # nothing seen: not latched
    assert bands.compute(pt, [1.0], 0.1) is None
    assert bands.compute(pt, [3.0], 0.1) is None
    out = bands.compute(pt, [2.0], 0.1)
    assert bands.initial_total == 2.0 and out[0] == pytest.approx(1.0)
    bands.reset()
    assert not bands.latched


def test_quaternion_and_frame_change():
    R = fb.quat_matrix(0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4))   # +90° about z
    assert fb.to_frame([[1.0, 0, 0]], R, [0, 0, 1]).tolist()[0] == pytest.approx([0, 1, 1])
