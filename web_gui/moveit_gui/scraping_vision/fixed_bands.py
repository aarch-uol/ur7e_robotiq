"""/scraping_vision/fixed_band_targets for fixed-band policies (Round 44+), from the vision node's material map.

The policy wants 16 floats: occupancy[8] (remaining material per height band / material at the episode start)
and target_z[8] (EE z - mean material z per band, m; 0 in an empty band). This module makes them from every
material pixel the vision node finds (the depth threshold map, inside the same crop the midpoints use), not
from the k-means midpoints: each pixel becomes a base_link point weighted by the area it covers.

The banding is the policy runner's own (band_sorting.py in /ws/scraping/src, loaded by path), with the
settings of the policy's robot_config: band_sorting.vial_bottom_z, band_sorting.band_edges_m (or the training
config's surface bank via scene_variant), servo_admittance.base_frame/ee_frame and topics.fixed_band_targets.
band_sorting.enabled stays false: that switch is the runner's own fallback from world_midpoints.

The episode start is the moment the runner node appears (or ~/reset_fixed_bands is called): the material total
is the median of the next few frames. No ROS here; ros_integration.py does the topics and TF.
"""
from __future__ import annotations

import importlib.util
import math
import sys
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

BAND_SORTING_PATHS = (
    Path("/ws/scraping/src/mujoco_multiagent_rl/policy_runner/band_sorting.py"),
    Path(__file__).resolve().parents[2] / "src/mujoco_multiagent_rl/policy_runner/band_sorting.py",
)
TOPIC = "/scraping_vision/fixed_band_targets"


class FixedBandError(ValueError):
    pass


def load_band_sorting(paths: Sequence[Path] = BAND_SORTING_PATHS):
    """The runner's band_sorting module, loaded from its file (its package imports torch)."""
    for p in paths:
        p = Path(p)
        if p.is_file():
            name = "_runner_band_sorting"
            if name in sys.modules:
                return sys.modules[name]
            spec = importlib.util.spec_from_file_location(name, str(p))
            mod = importlib.util.module_from_spec(spec)
            sys.modules[name] = mod                      # dataclasses look the module up while it loads
            spec.loader.exec_module(mod)
            return mod
    raise FixedBandError("the policy runner's band_sorting.py was not found (looked in "
                         + ", ".join(str(p) for p in paths) + ")")


def _biases(d, path=""):
    """(key, value) of every obs_position_bias* entry in a robot_config, at any depth."""
    out = []
    if isinstance(d, dict):
        for k, v in d.items():
            if str(k).startswith("obs_position_bias"):
                out.append((f"{path}{k}", v))
            else:
                out += _biases(v, f"{path}{k}.")
    return out


@dataclass
class BandConfig:
    edges: np.ndarray
    vial_bottom_z: float
    base_frame: str
    ee_frame: str
    topic: str
    min_weight: float
    edges_from: str


def load_config(robot_config: Path, bs=None) -> BandConfig:
    import yaml
    robot_config = Path(robot_config).expanduser()
    bs = bs or load_band_sorting()
    try:
        rc = yaml.safe_load(robot_config.read_text()) or {}
    except Exception as e:                               # noqa: BLE001
        raise FixedBandError(f"cannot read {robot_config}: {e}") from None
    s = rc.get("band_sorting") or {}
    try:
        bottom = float(s.get("vial_bottom_z"))
    except (TypeError, ValueError):
        bottom = float("nan")
    if not math.isfinite(bottom):
        raise FixedBandError(f"band_sorting.vial_bottom_z is not set in {robot_config} (base_link z of the vial "
                             "bottom, m)")
    n_bins = int((rc.get("observation") or {}).get("occupancy_bins", bs.DEFAULT_N_BINS))
    if s.get("band_edges_m") is not None:
        try:
            edges = bs.validate_edges(s["band_edges_m"], n_bins)
        except bs.BandSortingError as e:
            raise FixedBandError(str(e)) from None
        edges_from = "band_sorting.band_edges_m"
    else:
        cfg_path = robot_config.with_name("config.yaml")
        try:
            cfg = yaml.safe_load(cfg_path.read_text()) or {}
            info = bs.derive_band_edges(cfg, s.get("scene_variant"), bank_root=s.get("bank_root") and
                                        Path(s["bank_root"]), config_dir=cfg_path.parent, n_bins=n_bins)
        except FileNotFoundError:
            raise FixedBandError(f"no band_sorting.band_edges_m and no training config at {cfg_path}") from None
        except bs.BandSortingError as e:
            raise FixedBandError(f"band edges: {e}") from None
        edges, edges_from = info.edges_m, f"surface bank {Path(info.bank_file).name}"
    bias = [(k, v) for k, v in _biases(rc) if v is not None and np.any(np.asarray(v, dtype=float) != 0.0)]
    if bias:
        raise FixedBandError("the robot config sets observation position biases (" + ", ".join(k for k, _ in bias)
                             + "): the runner adds them to target_z, this publisher does not. Use "
                               "band_sorting in the runner instead")
    sa = rc.get("servo_admittance") or {}
    return BandConfig(edges=np.asarray(edges, dtype=np.float64), vial_bottom_z=bottom,
                      base_frame=str(sa.get("base_frame") or "base_link"),
                      ee_frame=str(sa.get("ee_frame") or "spatula_tip"),
                      topic=str((rc.get("topics") or {}).get("fixed_band_targets") or TOPIC),
                      min_weight=float(s.get("min_confidence") or 0.0), edges_from=edges_from)


def material_points(material: np.ndarray, depth_mm: np.ndarray, K: np.ndarray, rect=None, stride: int = 2):
    """Camera-frame points (N, 3) and the area each covers (m²) of the material pixels inside rect (x, y, w, h)."""
    m = np.asarray(material) > 0
    if rect is not None:
        x, y, w, h = (int(v) for v in rect)
        crop = np.zeros_like(m)
        crop[max(0, y):max(0, y + h), max(0, x):max(0, x + w)] = True
        m &= crop
    s = max(1, int(stride))
    vs, us = np.nonzero(m[::s, ::s])
    vs, us = vs * s, us * s
    z = np.asarray(depth_mm, dtype=np.float64)[vs, us] / 1000.0
    ok = np.isfinite(z) & (z > 0.0)
    us, vs, z = us[ok], vs[ok], z[ok]
    fx, fy, cx, cy = float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])
    pts = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], axis=1)
    area = (z * z) * (s * s) / (fx * fy)                  # one pixel covers (z/fx)·(z/fy) m² at depth z
    return pts, area


def to_frame(pts: np.ndarray, rotation: np.ndarray, translation: Sequence[float]) -> np.ndarray:
    return np.asarray(pts, dtype=np.float64) @ np.asarray(rotation, dtype=np.float64).T + np.asarray(translation)


def quat_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class FixedBands:
    """Base-frame material points -> the 16 floats, with the episode-start total latched over a few frames."""

    def __init__(self, cfg: BandConfig, bs=None, latch_frames: int = 5):
        self.cfg = cfg
        self.bs = bs or load_band_sorting()
        self.latch_frames = max(1, int(latch_frames))
        self._lock = threading.Lock()                     # reset comes from the service / timer threads
        self.reset()

    def reset(self):
        with self._lock:
            self.initial_total: Optional[float] = None
            self._first: list = []

    @property
    def latched(self) -> bool:
        return self.initial_total is not None

    def compute(self, pts_base: np.ndarray, weights: np.ndarray, ee_z: float) -> Optional[np.ndarray]:
        """float32[2 * bins], or None until the episode-start total is known."""
        rows = np.column_stack([np.asarray(pts_base, dtype=np.float64).reshape(-1, 3),
                                np.asarray(weights, dtype=np.float64).reshape(-1)])
        w, mean_z = self.bs.band_statistics(rows, self.cfg.vial_bottom_z, self.cfg.edges, self.cfg.min_weight)
        with self._lock:
            if self.initial_total is None:
                total = float(w.sum())
                if total <= 0.0:
                    return None                           # nothing seen yet: no episode-start amount
                self._first.append(total)
                if len(self._first) < self.latch_frames:
                    return None
                self.initial_total = float(np.median(self._first))
            initial = self.initial_total
        return self.bs.targets_from_statistics(w, mean_z, float(ee_z), initial)
