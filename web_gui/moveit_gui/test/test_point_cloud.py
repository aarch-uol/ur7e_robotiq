import struct
from types import SimpleNamespace

import numpy as np
import pytest

from moveit_gui.fake_bridge import FakeBridge, FAKE_CLOUD_TOPIC
from moveit_gui.profile import RobotProfile
from moveit_gui.point_cloud import (decode_pointcloud2, height_colors, pack_point_cloud,
                                    transform_points)

ROOT = __import__("pathlib").Path(__file__).resolve().parents[1]


def _field(name, offset, datatype):
    return SimpleNamespace(name=name, offset=offset, datatype=datatype, count=1)


def _cloud(rows, fields, point_step, endian=False, row_padding=0):
    height, width = len(rows), len(rows[0])
    order = ">" if endian else "<"
    data = bytearray()
    for row in rows:
        line = bytearray()
        for point in row:
            raw = bytearray(point_step)
            for name, value in point.items():
                field = next(f for f in fields if f.name == name)
                if field.datatype == 7:
                    struct.pack_into(order + "f", raw, field.offset, value)
                elif field.datatype == 6:
                    struct.pack_into(order + "I", raw, field.offset, value)
            line.extend(raw)
        line.extend(b"x" * row_padding)
        data.extend(line)
    return SimpleNamespace(height=height, width=width, fields=fields, point_step=point_step,
                           row_step=width * point_step + row_padding, is_bigendian=endian, data=bytes(data))


def test_decode_pointcloud2_reads_padded_big_endian_xyz_and_packed_rgb():
    fields = [_field("x", 0, 7), _field("y", 4, 7), _field("z", 8, 7), _field("rgba", 12, 6)]
    msg = _cloud([[{"x": 1.0, "y": 2.0, "z": 3.0, "rgba": 0x11223344},
                  {"x": 4.0, "y": 5.0, "z": 6.0, "rgba": 0xAABBCCDD}]], fields, 16, endian=True, row_padding=4)

    points, colors = decode_pointcloud2(msg, max_points=10, voxel_size=0)

    assert points.tolist() == [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
    assert colors.tolist() == [[0x11, 0x22, 0x33], [0xAA, 0xBB, 0xCC]]


def test_decode_pointcloud2_drops_invalid_points_and_falls_back_to_height_colour():
    fields = [_field("x", 0, 7), _field("y", 4, 7), _field("z", 8, 7)]
    msg = _cloud([[{"x": 0.0, "y": 0.0, "z": 0.1}, {"x": np.nan, "y": 0.0, "z": 0.2},
                  {"x": 0.0, "y": 0.0, "z": 0.3}]], fields, 12)

    points, colors = decode_pointcloud2(msg, max_points=10, voxel_size=0)

    assert points.shape == (2, 3)
    assert colors.shape == (2, 3)
    assert not np.array_equal(colors[0], colors[1])
    assert np.all(colors >= 0) and np.all(colors <= 255)


def test_decode_pointcloud2_keeps_points_and_colours_bounded():
    fields = [_field("x", 0, 7), _field("y", 4, 7), _field("z", 8, 7), _field("r", 12, 7),
              _field("g", 16, 7), _field("b", 20, 7)]
    rows = [[{"x": float(i) / 1000, "y": float(i % 100) / 100, "z": 0.2,
              "r": 10.0, "g": 20.0, "b": 30.0} for i in range(30005)]]
    msg = _cloud(rows, fields, 24)

    points, colors = decode_pointcloud2(msg, max_points=30000, voxel_size=0)

    assert len(points) == 30000
    assert len(colors) == len(points)


def test_transform_points_applies_translation():
    points = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
    T = np.eye(4)
    T[:3, 3] = [0.5, -1.0, 2.0]

    assert np.allclose(transform_points(points, T), [[1.5, 1.0, 5.0]])


def test_height_colors_is_bounded_and_pack_has_binary_header():
    points = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)
    colors = height_colors(points)
    packet = pack_point_cloud(points, colors)

    magic, version, flags, reserved, count = struct.unpack_from("<4sBBHI", packet)
    assert (magic, version, flags, reserved, count) == (b"PCD1", 1, 1, 0, 2)
    assert len(packet) == 12 + 2 * 12 + 2 * 3


def test_decode_rejects_missing_xyz_fields():
    fields = [_field("x", 0, 7), _field("y", 4, 7)]
    msg = _cloud([[{"x": 0.0, "y": 0.0}]], fields, 8)

    with pytest.raises(ValueError, match="x, y and z"):
        decode_pointcloud2(msg)


def test_fake_bridge_exposes_a_bounded_table_and_vial_cloud():
    bridge = FakeBridge(RobotProfile.load(ROOT / "profiles" / "ur7e_lab.yaml"))

    assert (FAKE_CLOUD_TOPIC, "sensor_msgs/msg/PointCloud2") in bridge.point_cloud_topics()
    points, colors, message = bridge.point_cloud_snapshot(FAKE_CLOUD_TOPIC)

    assert message == ""
    assert len(points) == len(colors) > 1000
    assert len(points) <= 30000
    assert np.min(points[:, 2]) <= 0.001 and np.max(points[:, 2]) >= 0.1


def test_fake_bridge_reports_missing_topic_stale_data_and_tf_failure():
    bridge = FakeBridge(RobotProfile.load(ROOT / "profiles" / "ur7e_lab.yaml"))

    assert bridge.point_cloud_snapshot("/missing")[2] == "no point cloud topic"
    bridge.fake_point_cloud_age = 3.0
    assert bridge.point_cloud_snapshot(FAKE_CLOUD_TOPIC)[2] == "point cloud is stale"
    bridge.fake_point_cloud_age = 0.0
    bridge.fake_point_cloud_frame = "missing_frame"
    assert "TF unavailable" in bridge.point_cloud_snapshot(FAKE_CLOUD_TOPIC)[2]
