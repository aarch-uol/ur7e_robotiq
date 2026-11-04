"""Rigid transforms as 4x4 numpy matrices.

Convention used everywhere in this package:

    T_a_b  = pose of frame b expressed in frame a  (maps b-coordinates to a-coordinates)
    p_a    = T_a_b @ p_b
    T_a_c  = T_a_b @ T_b_c
    T_b_a  = inv(T_a_b)

ROS TF lookup_transform(target=a, source=b) returns T_a_b.
Quaternions are always (x, y, z, w), like geometry_msgs.
"""
from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

import numpy as np


class TransformError(ValueError):
    pass


QUAT_NORM_TOL = 1e-3     # |q| must be within 1 +- this to count as a rotation (after YAML rounding)


def normalize_quat(q_xyzw: Sequence[float]) -> np.ndarray:
    q = np.asarray(q_xyzw, dtype=float).reshape(-1)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise TransformError(f"quaternion must be 4 finite numbers, got {list(q_xyzw)}")
    n = float(np.linalg.norm(q))
    if n < 1e-9:
        raise TransformError("zero quaternion")
    return q / n


def check_unit_quat(q_xyzw: Sequence[float], tol: float = QUAT_NORM_TOL) -> np.ndarray:
    """Like normalize_quat, but refuse quaternions that are clearly not rotations."""
    q = np.asarray(q_xyzw, dtype=float).reshape(-1)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise TransformError(f"quaternion must be 4 finite numbers, got {list(q_xyzw)}")
    n = float(np.linalg.norm(q))
    if abs(n - 1.0) > tol:
        raise TransformError(f"quaternion is not unit length (|q| = {n:.4f})")
    return q / n


def quat_to_matrix(q_xyzw: Sequence[float]) -> np.ndarray:
    x, y, z, w = normalize_quat(q_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> (x, y, z, w) with w >= 0."""
    R = np.asarray(R, dtype=float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    return -q if q[3] < 0 else q


def make(position: Sequence[float] = (0, 0, 0), quat_xyzw: Sequence[float] = (0, 0, 0, 1)) -> np.ndarray:
    p = np.asarray(position, dtype=float).reshape(-1)
    if p.shape != (3,) or not np.all(np.isfinite(p)):
        raise TransformError(f"position must be 3 finite numbers, got {list(position)}")
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(quat_xyzw)
    T[:3, 3] = p
    return T


def from_rotation(R: np.ndarray, position: Sequence[float] = (0, 0, 0)) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(position, dtype=float)
    return T


def to_pos_quat(T: np.ndarray) -> Tuple[List[float], List[float]]:
    return [float(v) for v in T[:3, 3]], [float(v) for v in matrix_to_quat(T[:3, :3])]


def inv(T: np.ndarray) -> np.ndarray:
    R, p = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ p
    return out


def compose(*Ts: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    for T in Ts:
        out = out @ T
    return out


def is_valid(T: np.ndarray, tol: float = 1e-6) -> bool:
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        return False
    R = T[:3, :3]
    return (np.allclose(T[3], [0, 0, 0, 1], atol=tol) and np.allclose(R.T @ R, np.eye(3), atol=tol)
            and abs(np.linalg.det(R) - 1.0) < tol)


def rotation_angle(R: np.ndarray) -> float:
    """Angle [rad] of a rotation matrix."""
    c = (np.trace(R) - 1.0) / 2.0
    return float(math.acos(max(-1.0, min(1.0, c))))


def angle_between(Ta: np.ndarray, Tb: np.ndarray) -> float:
    return rotation_angle(Ta[:3, :3].T @ Tb[:3, :3])


# ---------------------------------------------------------- RPY (UI only)
def rpy_to_quat(roll: float, pitch: float, yaw: float) -> List[float]:
    """Fixed-axis XYZ roll/pitch/yaw [rad] -> quaternion (x, y, z, w). Same as tf2 setRPY."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy]


def quat_to_rpy(q_xyzw: Sequence[float]) -> List[float]:
    x, y, z, w = normalize_quat(q_xyzw)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [roll, pitch, yaw]


def mm_deg_to_T(xyz_mm: Sequence[float], rpy_deg: Sequence[float]) -> np.ndarray:
    """UI units (mm, degrees) -> SI transform."""
    return make([v / 1000.0 for v in xyz_mm], rpy_to_quat(*[math.radians(a) for a in rpy_deg]))


def T_to_mm_deg(T: np.ndarray) -> Tuple[List[float], List[float]]:
    """SI transform -> UI units (mm, degrees)."""
    p, q = to_pos_quat(T)
    return [v * 1000.0 for v in p], [math.degrees(a) for a in quat_to_rpy(q)]


# ---------------------------------------------------------- averaging
def average_quaternions(quats_xyzw: Iterable[Sequence[float]]) -> np.ndarray:
    """Rotation average (Markley et al. 2007): principal eigenvector of sum(q q^T).

    Insensitive to the q / -q sign ambiguity, unlike an element-wise mean.
    """
    Q = np.array([normalize_quat(q) for q in quats_xyzw])
    if len(Q) == 0:
        raise TransformError("no quaternions to average")
    M = Q.T @ Q
    w, V = np.linalg.eigh(M)
    q = V[:, int(np.argmax(w))]
    return -q if q[3] < 0 else q


def average_transforms(Ts: Sequence[np.ndarray]) -> Tuple[np.ndarray, dict]:
    """Mean pose + spread statistics of noisy samples of the same transform."""
    if len(Ts) == 0:
        raise TransformError("no samples")
    P = np.array([T[:3, 3] for T in Ts])
    q = average_quaternions([matrix_to_quat(T[:3, :3]) for T in Ts])
    T_mean = make(P.mean(axis=0), q)
    ang = np.array([angle_between(T_mean, T) for T in Ts])
    dist = np.linalg.norm(P - P.mean(axis=0), axis=1)
    stats = {
        "sample_count": len(Ts),
        "position_std_m": [float(v) for v in P.std(axis=0)],
        "position_rms_m": float(math.sqrt(float(np.mean(dist ** 2)))),
        "angle_rms_deg": float(math.degrees(math.sqrt(float(np.mean(ang ** 2))))),
        "angle_max_deg": float(math.degrees(float(ang.max()))),
    }
    return T_mean, stats
