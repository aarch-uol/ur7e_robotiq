"""Camera inspection: decoding, statistics, and SETUP > Cameras on the simulated robot."""
import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from moveit_gui.camera import ImageDecodeError, ImageWatch, TopicStats, decode_image  # noqa: E402
from moveit_gui.fake_bridge import fake_image  # noqa: E402

from test_gui_v23 import app, pump, win  # noqa: E402,F401  (fixtures)


def test_rgb_and_bgr_give_the_same_picture():
    rgb = np.zeros((4, 6, 3), np.uint8)
    rgb[..., 0] = 200                                     # red
    out = decode_image("rgb8", 6, 4, 18, rgb.tobytes())
    assert out.shape == (4, 6, 3) and (out[..., 0] == 200).all() and (out[..., 2] == 0).all()
    out = decode_image("bgr8", 6, 4, 18, rgb[..., ::-1].tobytes())
    assert (out[..., 0] == 200).all() and (out[..., 2] == 0).all()
    bgra = np.dstack([rgb[..., ::-1], np.full((4, 6), 255, np.uint8)])     # ZED colour is bgra8
    assert (decode_image("bgra8", 6, 4, 24, bgra.tobytes())[..., 0] == 200).all()


def test_row_padding_and_downsampling():
    a = np.arange(8 * 10, dtype=np.uint8).reshape(8, 10)
    padded = np.hstack([a, np.zeros((8, 6), np.uint8)])   # step 16 > width 10
    out = decode_image("mono8", 10, 8, 16, padded.tobytes())
    assert (out[..., 1] == a).all()
    small = decode_image("mono8", 10, 8, 16, padded.tobytes(), max_width=5)
    assert small.shape == (4, 5, 3) and (small[..., 0] == a[::2, ::2]).all()


def test_depth_near_is_bright_and_no_return_is_black():
    d = np.array([[400, 800], [0, 600]], "<u2")            # RealSense 16UC1, mm
    out = decode_image("16UC1", 2, 2, 4, d.tobytes())[..., 0]
    assert out[1, 0] == 0 and out[0, 0] > out[1, 1] > out[0, 1]
    z = np.array([[0.4, np.nan], [0.8, 0.6]], "<f4")        # ZED 32FC1, m
    out = decode_image("32FC1", 2, 2, 8, z.tobytes())[..., 0]
    assert out[0, 1] == 0 and out[0, 0] > out[1, 1] > out[1, 0]


def test_bad_images_are_reported_not_crashed():
    with pytest.raises(ImageDecodeError, match="does not match"):
        decode_image("rgb8", 10, 10, 30, b"\0" * 20)
    with pytest.raises(ImageDecodeError, match="not supported"):
        decode_image("bayer_rggb8", 2, 2, 2, b"\0" * 4)
    w = ImageWatch("/x", max_decode_hz=1000)
    w.on_image("yuv422", 2, 2, 4, b"\0" * 8)
    assert "not supported" in w.latest().error


def test_rate_and_age():
    s = TopicStats()
    for i in range(31):
        s.tick(100.0 + i / 30.0)
    assert s.rate(now=101.0) == pytest.approx(30.0, rel=0.01)
    assert s.age(now=103.0) == pytest.approx(2.0)
    assert s.rate(now=110.0) == 0.0


def test_fake_frames_decode():
    for enc in ("rgb8", "bgr8", "mono8", "16UC1"):
        data, step = fake_image(enc, 64, 48, 0)
        assert decode_image(enc, 64, 48, step, data).shape == (48, 64, 3)


def test_cameras_page_status_preview_and_no_traffic_when_hidden(win):
    win.show()                                  # the preview only runs while the page is on screen
    win.nav.setCurrentRow(list(win.pages).index("SETUP"))
    s = win.pages["SETUP"]
    s.sub.setCurrentRow(4)
    pump(1.0)
    page = s.cameras
    rows = {r.topic: r for r in page.rows}
    assert rows["/camera/camera/color/image_rect_raw"].state.text() == "publishing"
    assert rows["/zed/zed_node/rgb/color/rect/image"].state.text() == "no publisher"      # not set up yet
    assert win.bridge.watched_image().topic == "/camera/camera/color/image_rect_raw"
    assert page.view.pixmap() is not None and not page.view.pixmap().isNull()
    assert "1280×720 · rgb8" in page.info.text()
    page.watch("/zed/zed_node/depth/depth_registered")
    assert "no image publisher" in page.view.text()
    page.combo.setEditText("/camera/camera/infra1/image_rect_raw")       # any topic, typed or picked
    page.combo.lineEdit().returnPressed.emit()
    pump(0.6)
    assert "mono8" in page.info.text()
    s.sub.setCurrentRow(0)
    pump(0.2)
    assert win.bridge.watched_image() is None
