"""Small, bounded PointCloud2 decoding and binary transport helpers."""
from __future__ import annotations

import struct
from typing import Optional, Tuple

import numpy as np


MAX_POINTS = 30000
PACKET_MAGIC = b"PCD1"

_DATATYPES = {
    1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8",
}


def _field(fields, name):
    return next((f for f in fields if str(getattr(f, "name", "")).lower() == name), None)


def _field_values(points: np.ndarray, field, bigendian: bool) -> np.ndarray:
    code = _DATATYPES.get(int(field.datatype))
    if code is None or int(getattr(field, "count", 1)) != 1:
        raise ValueError(f"unsupported PointCloud2 field '{field.name}'")
    dtype = np.dtype(code)
    if dtype.itemsize > 1:
        dtype = dtype.newbyteorder(">" if bigendian else "<")
    offset, end = int(field.offset), int(field.offset) + dtype.itemsize
    if offset < 0 or end > points.shape[1]:
        raise ValueError(f"PointCloud2 field '{field.name}' is outside point_step")
    return points[:, offset:end].reshape(-1).view(dtype).reshape(len(points))


def height_colors(points: np.ndarray) -> np.ndarray:
    """Return a compact blue-to-red colour ramp based on point height."""
    p = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    if not len(p):
        return np.empty((0, 3), dtype=np.uint8)
    lo, hi = float(np.min(p[:, 2])), float(np.max(p[:, 2]))
    t = np.zeros(len(p), dtype=np.float32) if hi <= lo else (p[:, 2] - lo) / (hi - lo)
    out = np.empty((len(p), 3), dtype=np.uint8)
    out[:, 0] = np.clip(255 * t, 0, 255).astype(np.uint8)
    out[:, 1] = np.clip(90 + 110 * (1 - np.abs(2 * t - 1)), 0, 255).astype(np.uint8)
    out[:, 2] = np.clip(255 * (1 - t), 0, 255).astype(np.uint8)
    return out


def _colour_values(points: np.ndarray, fields, bigendian: bool) -> Optional[np.ndarray]:
    channels = [_field(fields, c) for c in ("r", "g", "b")]
    if all(channels):
        values = [_field_values(points, f, bigendian).astype(np.float32) for f in channels]
        return np.clip(np.rint(np.stack(values, axis=1)), 0, 255).astype(np.uint8)
    packed_name = "rgba" if _field(fields, "rgba") else "rgb"
    packed_field = _field(fields, packed_name)
    if packed_field is None:
        return None
    values = _field_values(points, packed_field, bigendian)
    if np.issubdtype(values.dtype, np.floating):
        byte_order = ">" if bigendian else "<"
        packed = values.view(np.dtype(byte_order + "u4")).astype(np.uint32)
    else:
        packed = values.astype(np.uint32)
    shift = 24 if packed_name == "rgba" else 16
    return np.stack([(packed >> shift) & 255, (packed >> (shift - 8)) & 255,
                     (packed >> (shift - 16)) & 255], axis=1).astype(np.uint8)


def _bounded(points: np.ndarray, colors: np.ndarray, max_points: int, voxel_size: float):
    if len(points) <= max_points:
        return points, colors
    if voxel_size > 0:
        keys = np.floor(points / float(voxel_size)).astype(np.int64)
        _, keep = np.unique(keys, axis=0, return_index=True)
        keep.sort()
        points, colors = points[keep], colors[keep]
    if len(points) > max_points:
        keep = np.linspace(0, len(points) - 1, max_points, dtype=np.int64)
        points, colors = points[keep], colors[keep]
    return points, colors


def decode_pointcloud2(msg, max_points: int = MAX_POINTS, voxel_size: float = 0.01) -> Tuple[np.ndarray, np.ndarray]:
    """Decode common PointCloud2 layouts into bounded float32 XYZ and uint8 RGB arrays."""
    max_points = min(MAX_POINTS, max(1, int(max_points)))
    fields = list(getattr(msg, "fields", []))
    xyz = [_field(fields, name) for name in ("x", "y", "z")]
    if not all(xyz):
        raise ValueError("PointCloud2 needs x, y and z fields")
    width, height = int(msg.width), int(msg.height)
    point_step, row_step = int(msg.point_step), int(msg.row_step)
    if width <= 0 or height <= 0 or point_step <= 0 or row_step < width * point_step:
        return np.empty((0, 3), np.float32), np.empty((0, 3), np.uint8)
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if raw.size < row_step * height:
        raise ValueError("PointCloud2 data is shorter than row_step * height")
    rows = raw[:row_step * height].reshape(height, row_step)
    packed = rows[:, :width * point_step].reshape(-1, point_step)
    step = len(packed) // (4 * max_points)           # a big cloud (ZED ~900k) is thinned before it is decoded
    if step > 1:
        packed = packed[::step]
    bigendian = bool(getattr(msg, "is_bigendian", False))
    points = np.stack([_field_values(packed, f, bigendian) for f in xyz], axis=1).astype(np.float32)
    valid = np.isfinite(points).all(axis=1)
    points = points[valid]
    raw_colors = _colour_values(packed, fields, bigendian)
    colors = raw_colors[valid] if raw_colors is not None else height_colors(points)
    return _bounded(points, colors, max_points, float(voxel_size))


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    p = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    T = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    if not len(p):
        return p.copy()
    return ((np.c_[p, np.ones(len(p))] @ T.T)[:, :3]).astype(np.float32)


def pack_point_cloud(points: np.ndarray, colors: Optional[np.ndarray] = None) -> bytes:
    p = np.ascontiguousarray(np.asarray(points, dtype=np.float32).reshape(-1, 3))
    if len(p) > MAX_POINTS:
        raise ValueError(f"point cloud exceeds {MAX_POINTS} points")
    c = height_colors(p) if colors is None else np.ascontiguousarray(np.asarray(colors, dtype=np.uint8).reshape(-1, 3))
    if len(c) != len(p):
        raise ValueError("point and colour counts differ")
    return struct.pack("<4sBBHI", PACKET_MAGIC, 1, 1, 0, len(p)) + p.tobytes() + c.tobytes()
