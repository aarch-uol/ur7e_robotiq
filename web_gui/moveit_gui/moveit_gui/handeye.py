"""Camera hand-eye calibration with a ChArUco board (the target MoveIt Calibration uses too).

Two mountings:
  eye_in_hand   camera on the tool, board fixed in the cell   -> X = T_flange_camera  (flange = profile planning link)
  eye_to_hand   camera fixed in the cell, board on the tool   -> X = T_base_camera

Each sample is the flange pose (from TF) and the board pose seen by the camera (ChArUco corners + solvePnP with the
camera's own CameraInfo intrinsics: RealSense and ZED come factory-calibrated). OpenCV's calibrateHandEye solves X
with five methods (Tsai, Park, Horaud, Andreff, Daniilidis); Park is the result, the spread of the others is shown.

The check that matters: with the solved X every sample must put the board in the same place (in the base for
eye-in-hand, on the flange for eye-to-hand). Its spread, in mm and degrees, is the calibration's real accuracy,
including errors of the robot and the board print.

OpenCV 4.6 (Ubuntu 24.04, the robot image) and 4.7+ have different ArUco APIs; both are handled here.
"""
from __future__ import annotations

import base64
import math
import re
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

from . import transforms as tf

MOUNTS = ("eye_in_hand", "eye_to_hand")
DICTS = ("DICT_4X4_50", "DICT_5X5_100", "DICT_6X6_250")
DEFAULT_BOARD = {"squares_x": 7, "squares_y": 5, "square_mm": 35.0, "marker_mm": 26.0, "dict": "DICT_5X5_100"}
MIN_CORNERS = 8
MIN_SAMPLES = 5
GOOD_SAMPLES = 12                    # recommended: more pictures, smaller error
GOOD_MM, GOOD_DEG, OK_MM, OK_DEG = 2.0, 0.3, 5.0, 1.0
NAME = re.compile(r"^[a-z][a-z0-9_]{0,39}$")


class HandEyeError(ValueError):
    pass


def _cv():
    try:
        import cv2
        cv2.aruco
    except (ImportError, AttributeError):
        raise HandEyeError("OpenCV with the ArUco module (python3-opencv) is needed for camera calibration") from None
    return cv2


# ===================================================================== board
def check_board(b: Optional[dict]) -> dict:
    b = {**DEFAULT_BOARD, **(b or {})}
    try:
        sx, sy = int(b["squares_x"]), int(b["squares_y"])
        sq, mk = float(b["square_mm"]), float(b["marker_mm"])
    except (TypeError, ValueError):
        raise HandEyeError("board: squares are whole numbers, sizes in mm") from None
    if not (3 <= sx <= 20 and 3 <= sy <= 20):
        raise HandEyeError("board: 3 to 20 squares each way")
    if sy % 2 == 0:
        raise HandEyeError("board: use an odd number of rows (squares_y): OpenCV 4.6 and 4.7+ draw even-row "
                           "ChArUco boards differently")
    if not (5 <= sq <= 200 and 2 <= mk < sq):
        raise HandEyeError("board: square 5-200 mm, marker smaller than the square")
    if b["dict"] not in DICTS:
        raise HandEyeError(f"board: dictionary is one of {', '.join(DICTS)}")
    if sx * sy // 2 > {"DICT_4X4_50": 50, "DICT_5X5_100": 100, "DICT_6X6_250": 250}[b["dict"]]:
        raise HandEyeError("board: more markers than the dictionary has")
    return {"squares_x": sx, "squares_y": sy, "square_mm": sq, "marker_mm": mk, "dict": b["dict"]}


def _board(b: dict):
    cv2 = _cv()
    a = cv2.aruco
    d = a.getPredefinedDictionary(getattr(a, b["dict"]))
    sq, mk = b["square_mm"] / 1000.0, b["marker_mm"] / 1000.0
    if hasattr(a, "CharucoBoard_create"):                       # OpenCV <= 4.6
        return a.CharucoBoard_create(b["squares_x"], b["squares_y"], sq, mk, d), d
    return a.CharucoBoard((b["squares_x"], b["squares_y"]), sq, mk, d), d


def board_image(b: dict, px_per_mm: float = 10.0) -> np.ndarray:
    board, _ = _board(b)
    size = (int(round(b["squares_x"] * b["square_mm"] * px_per_mm)), int(round(b["squares_y"] * b["square_mm"] * px_per_mm)))
    if hasattr(board, "generateImage"):
        return board.generateImage(size, marginSize=0, borderBits=1)
    return board.draw(size, marginSize=0, borderBits=1)


def board_svg(b: dict) -> str:
    """A printable page: the board at its true size (print at 100 %, no 'fit to page')."""
    cv2 = _cv()
    b = check_board(b)
    img = board_image(b)
    ok, png = cv2.imencode(".png", img)
    w, h = b["squares_x"] * b["square_mm"], b["squares_y"] * b["square_mm"]
    label = (f"ChArUco {b['squares_x']}x{b['squares_y']}, square {b['square_mm']:g} mm, marker {b['marker_mm']:g} mm, "
             f"{b['dict']}. Print at 100 % (no fit to page), measure a square and enter it in the GUI. "
             f"Glue it flat on something rigid.")
    W, H = w + 20, h + 30
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}mm" height="{H}mm" viewBox="0 0 {W} {H}">'
            f'<rect width="{W}" height="{H}" fill="white"/>'
            f'<image x="10" y="10" width="{w}" height="{h}" preserveAspectRatio="none" '
            f'href="data:image/png;base64,{base64.b64encode(png.tobytes()).decode()}"/>'
            f'<text x="10" y="{h + 20}" font-family="sans-serif" font-size="3.2">{label}</text></svg>')


def _object_corners(board) -> np.ndarray:
    c = board.getChessboardCorners() if hasattr(board, "getChessboardCorners") else board.chessboardCorners
    return np.asarray(c, dtype=np.float64).reshape(-1, 3)


@dataclass
class Detection:
    ok: bool
    corners: int = 0
    T_cam_board: Optional[np.ndarray] = None
    reproj_px: float = float("nan")
    why: str = ""
    points: List[List[float]] = field(default_factory=list)      # image corners, for the preview
    markers: int = 0                                                # ArUco markers found (before corners)
    quads: List[List[List[float]]] = field(default_factory=list)    # their outlines, for the preview


def detect(gray: np.ndarray, b: dict, K, D) -> Detection:
    """ChArUco corners in a grey image and the board pose T_cam_board (metres) from them."""
    cv2 = _cv()
    a = cv2.aruco
    board, d = _board(b)
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    D = np.asarray(D if D is not None else [], dtype=np.float64).reshape(-1)
    if hasattr(a, "CharucoDetector"):
        corners, ids, mc, mids = a.CharucoDetector(board).detectBoard(gray)
    else:
        mc, mids, _ = a.detectMarkers(gray, d, parameters=a.DetectorParameters_create())
        if mids is None or len(mids) == 0:
            return Detection(False, why="no ArUco markers seen")
        _, corners, ids = a.interpolateCornersCharuco(mc, mids, gray, board, cameraMatrix=K, distCoeffs=D)
    nm = 0 if mids is None else len(mids)
    quads = [[[round(float(x), 1), round(float(y), 1)] for x, y in np.asarray(q).reshape(-1, 2)] for q in (mc if mc is not None else [])]
    n = 0 if ids is None else len(ids)
    if nm == 0:
        return Detection(False, why="no ArUco markers seen (board out of view, too far, blurred or the wrong dictionary)")
    if n < MIN_CORNERS:
        return Detection(False, n, why=f"{nm} markers, {n} board corners seen (need {MIN_CORNERS}): show more of the board",
                         markers=nm, quads=quads)
    obj = _object_corners(board)[np.asarray(ids).reshape(-1)]
    img = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return Detection(False, n, why="board pose could not be solved")
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
    R, _ = cv2.Rodrigues(rvec)
    return Detection(True, n, tf.from_rotation(R, tvec.reshape(3)), err,
                     points=[[round(float(x), 1), round(float(y), 1)] for x, y in img], markers=nm, quads=quads)


def render_board(b: dict, K, T_cam_board: np.ndarray, size: Tuple[int, int], noise: float = 2.0,
                 seed: int = 0) -> np.ndarray:
    """A synthetic camera image of the board (fake camera and tests): warp the board picture with the homography
    of the board plane, grey background, a little noise."""
    cv2 = _cv()
    px_mm = 4.0
    img = board_image(b, px_mm)
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    R, t = T_cam_board[:3, :3], T_cam_board[:3, 3]
    # board image pixel (u, v) -> board plane metres (u / px_mm / 1000, v / px_mm / 1000), board z = 0
    S = np.diag([1 / px_mm / 1000.0, 1 / px_mm / 1000.0, 1.0])
    H = K @ np.column_stack([R[:, 0], R[:, 1], t]) @ S
    out = cv2.warpPerspective(img, H, size, borderMode=cv2.BORDER_CONSTANT, borderValue=110,
                              flags=cv2.INTER_AREA)
    rng = np.random.default_rng(seed)
    return np.clip(out.astype(np.float32) + rng.normal(0, noise, out.shape), 0, 255).astype(np.uint8)


# ===================================================================== solve
@dataclass
class Sample:
    T_base_flange: np.ndarray
    T_cam_board: np.ndarray
    corners: int
    reproj_px: float
    t: float = field(default_factory=time.time)

    def to_yaml(self) -> dict:
        def T(M):
            p, q = tf.to_pos_quat(M)
            return {"position": [round(v, 7) for v in p], "quat_xyzw": [round(v, 9) for v in q]}
        return {"T_base_flange": T(self.T_base_flange), "T_cam_board": T(self.T_cam_board),
                "corners": self.corners, "reproj_px": round(self.reproj_px, 3)}


def _rot_deg(R: np.ndarray) -> float:
    return math.degrees(tf.rotation_angle(R))


METHODS = ("park", "tsai", "horaud", "andreff", "daniilidis")


def solve(samples: List[Sample], mount: str, method: str = "park") -> dict:
    """X and its quality. eye_in_hand: X = T_flange_cam; eye_to_hand: X = T_base_cam. `method` is the OpenCV
    solver whose X is used; the others are shown as a cross-check."""
    cv2 = _cv()
    if method not in METHODS:
        raise HandEyeError(f"solver is one of {', '.join(METHODS)}")
    if mount not in MOUNTS:
        raise HandEyeError(f"mount is one of {', '.join(MOUNTS)}")
    if len(samples) < MIN_SAMPLES:
        raise HandEyeError(f"{len(samples)} sample(s): at least {MIN_SAMPLES} ({GOOD_SAMPLES} or more is better)")
    A = [s.T_base_flange if mount == "eye_in_hand" else tf.inv(s.T_base_flange) for s in samples]
    spread = max((_rot_deg(A[i][:3, :3].T @ A[j][:3, :3]) for i in range(len(A)) for j in range(i)), default=0.0)
    if spread < 10.0:
        raise HandEyeError(f"the tool orientations differ by only {spread:.1f} deg: tilt the tool more between "
                           f"samples (20-30 deg each way)")
    methods = {"park": cv2.CALIB_HAND_EYE_PARK, "tsai": cv2.CALIB_HAND_EYE_TSAI, "horaud": cv2.CALIB_HAND_EYE_HORAUD,
               "andreff": cv2.CALIB_HAND_EYE_ANDREFF, "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS}
    Rg = [a[:3, :3] for a in A]
    tg = [a[:3, 3].reshape(3, 1) for a in A]
    Rt = [s.T_cam_board[:3, :3] for s in samples]
    tt = [s.T_cam_board[:3, 3].reshape(3, 1) for s in samples]
    sols: Dict[str, np.ndarray] = {}
    for name, m in methods.items():
        try:
            R, t = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=m)
            X = tf.from_rotation(R, np.asarray(t).reshape(3))
            if np.isfinite(X).all() and abs(np.linalg.det(X[:3, :3]) - 1) < 1e-3:
                sols[name] = X
        except cv2.error:
            continue
    if method not in sols:
        raise HandEyeError(f"the {method} solver failed: collect samples with more varied orientations, or pick "
                           f"another solver")
    X = sols[method]
    agree = {n: {"mm": round(float(np.linalg.norm(S[:3, 3] - X[:3, 3]) * 1000), 2),
                 "deg": round(_rot_deg(S[:3, :3].T @ X[:3, :3]), 3)} for n, S in sols.items() if n != method}
    # where every sample puts the board (base frame for eye-in-hand, flange frame for eye-to-hand)
    boards = [a @ X @ s.T_cam_board for a, s in zip(A, samples)]
    Tm, _ = tf.average_transforms(boards)
    dt = [float(np.linalg.norm(B[:3, 3] - Tm[:3, 3]) * 1000) for B in boards]
    dr = [_rot_deg(B[:3, :3].T @ Tm[:3, :3]) for B in boards]
    rms_mm, rms_deg = float(np.sqrt(np.mean(np.square(dt)))), float(np.sqrt(np.mean(np.square(dr))))
    verdict = ("good" if rms_mm <= GOOD_MM and rms_deg <= GOOD_DEG else
               "usable" if rms_mm <= OK_MM and rms_deg <= OK_DEG else "poor")
    return {"X": X, "mount": mount, "method": method, "verdict": verdict, "rms_mm": round(rms_mm, 2), "rms_deg": round(rms_deg, 3),
            "per_sample": [{"mm": round(a, 2), "deg": round(b, 3)} for a, b in zip(dt, dr)],
            "methods": agree, "orientation_spread_deg": round(spread, 1), "n": len(samples),
            "T_board": Tm}


# ===================================================================== poses
def sample_poses(T_base_flange: np.ndarray, pivot_base: np.ndarray, tilt_deg: float = 20.0) -> List[np.ndarray]:
    """Flange poses for a calibration run: the tool turned about `pivot_base` (the board centre for eye-in-hand,
    the tool tip for eye-to-hand) so the board stays in view. Tilts about two axes square to the line flange ->
    pivot, rolls about that line, and combinations: 12 poses, starting with the current one."""
    p = np.asarray(pivot_base, float).reshape(3)
    view = p - T_base_flange[:3, 3]
    n = np.linalg.norm(view)
    view = view / n if n > 1e-6 else T_base_flange[:3, 2]
    u = np.cross(view, [0.0, 0.0, 1.0])
    if np.linalg.norm(u) < 1e-3:
        u = np.cross(view, [1.0, 0.0, 0.0])
    u /= np.linalg.norm(u)
    v = np.cross(view, u)
    a, r = tilt_deg, tilt_deg * 1.25
    combos = [(0, 0, 0), (a, 0, 0), (-a, 0, 0), (0, a, 0), (0, -a, 0), (a * .7, a * .7, r), (-a * .7, -a * .7, -r),
              (a * .7, -a * .7, 0), (-a * .7, a * .7, 0), (0, 0, r), (0, 0, -r), (a * .5, -a * .5, -r * .6)]
    out = []
    for tu, tv, tr in combos:
        R = _axis_rot(u, math.radians(tu)) @ _axis_rot(v, math.radians(tv)) @ _axis_rot(view, math.radians(tr))
        M = np.eye(4)
        M[:3, :3] = R
        M[:3, 3] = p - R @ p
        out.append(M @ T_base_flange)
    return out


def _axis_rot(axis, ang: float) -> np.ndarray:
    x, y, z = np.asarray(axis, float) / np.linalg.norm(axis)
    c, s, C = math.cos(ang), math.sin(ang), 1 - math.cos(ang)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


# ===================================================================== saved results
def _T(M: np.ndarray) -> dict:
    p, q = tf.to_pos_quat(M)
    return {"position": [round(v, 7) for v in p], "quat_xyzw": [round(v, 9) for v in q]}


def _T_in(d: dict) -> np.ndarray:
    return tf.make(d["position"], tf.check_unit_quat(d["quat_xyzw"]))


class CameraCalibrations:
    """config_dir()/cameras/<profile>.yaml: {cameras: {name: {...}}, board: {...}}."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self.cameras: Dict[str, dict] = {}
        self.board = dict(DEFAULT_BOARD)
        try:
            d = yaml.safe_load(self.path.read_text()) or {}
            self.cameras = {str(k): v for k, v in (d.get("cameras") or {}).items() if isinstance(v, dict)}
            self.board = check_board(d.get("board"))
        except FileNotFoundError:
            pass
        except Exception:
            self.cameras = {}

    def _write(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text("# MoveIt GUI camera hand-eye calibrations (SETUP > Camera calibration).\n" +
                       yaml.safe_dump({"version": 1, "board": self.board, "cameras": self.cameras}, sort_keys=False))
        tmp.replace(self.path)

    def set_board(self, b: dict) -> dict:
        self.board = check_board(b)
        self._write()
        return self.board

    def save(self, name: str, topic: str, info_topic: str, optical_frame: str, flange: str, base: str,
             result: dict, samples: List[Sample], K) -> dict:
        if not NAME.match(name):
            raise HandEyeError("a camera name is lower_snake_case (a-z, 0-9, _)")
        mount = result["mount"]
        e = {"topic": topic, "info_topic": info_topic, "optical_frame": optical_frame, "mount": mount,
             "parent": flange if mount == "eye_in_hand" else base, "T_parent_optical": _T(result["X"]),
             "verdict": result["verdict"], "rms_mm": result["rms_mm"], "rms_deg": result["rms_deg"],
             "methods": result["methods"], "method": result.get("method", "park"), "ee_link": flange,
             "base_link": base, "samples": len(samples), "board": dict(self.board),
             "edited_by_hand": result.get("edited"),
             "K": [round(float(v), 4) for v in np.asarray(K).reshape(-1)],
             "saved": time.strftime("%Y-%m-%d %H:%M"), "sample_data": [s.to_yaml() for s in samples]}
        self.cameras[name] = e
        self._write()
        return e

    def delete(self, name: str):
        if self.cameras.pop(name, None) is None:
            raise HandEyeError(f"no camera calibration '{name}'")
        self._write()

    def T_parent_optical(self, name: str) -> np.ndarray:
        return _T_in(self.cameras[name]["T_parent_optical"])

    def view(self) -> List[dict]:
        out = []
        for n, e in self.cameras.items():
            T = self.T_parent_optical(n)
            xyz, rpy = tf.T_to_mm_deg(T)
            out.append({"name": n, **{k: v for k, v in e.items() if k not in ("sample_data", "T_parent_optical")},
                        "xyz_mm": [round(v, 2) for v in xyz], "rpy_deg": [round(v, 3) for v in rpy]})
        return out
