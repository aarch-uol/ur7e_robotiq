"""A fake robot with real cameras (moveit_gui_web --fake --real-cameras): real topics go to bridge.real."""
import os

from moveit_gui.fake_bridge import FakeBridge
from moveit_gui.profile import RobotProfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REAL = "/zed/zed_node/rgb/color/rect/image"


class StubReal:
    """Stands in for ros_bridge.RealCameras (no ROS here)."""

    def __init__(self):
        self.watching, self.down = None, False

    def topic_types(self):
        return [(REAL, "sensor_msgs/msg/Image")]

    def image_topics(self):
        return [(REAL, "sensor_msgs/msg/Image")]

    def topic_publishers(self, t):
        return 1 if t == REAL else 0

    def watch_image(self, t):
        self.watching = t
        return True, f"watching {t}" if t else "preview off"

    def watched_image(self):
        return ("watch", self.watching) if self.watching else None

    def camera_info(self, t, timeout_s=3.0):
        return {"K": [1.0] * 9, "frame_id": "zed_left_camera_optical_frame"}, "ok"

    def grab_photo(self, t, timeout_s=3.0):
        return b"jpeg", ""

    def tf_parent(self, frame):
        return "zed_camera_link" if frame == "zed_left_camera_optical_frame" else None

    def open_image(self, t):
        self.streams = getattr(self, "streams", []) + [t]
        return "real watch", "ok"

    def close_image(self, t):
        self.streams.remove(t)

    def open_images(self):
        return {t: "real watch" for t in getattr(self, "streams", [])}

    def shutdown(self):
        self.down = True


def test_a_fake_robot_reads_real_camera_topics():
    b = FakeBridge(RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml")))
    fake = next(iter(b.fake_cameras))
    assert b.topic_publishers(REAL) == 0                       # without --real-cameras: only the simulated ones
    b.real = StubReal()
    assert (REAL, "sensor_msgs/msg/Image") in b.image_topics() and (fake, "sensor_msgs/msg/Image") in b.image_topics()
    assert b.topic_publishers(REAL) == 1 and b.topic_publishers(fake) == 1
    assert b.watch_image(REAL)[0] and b.watched_image() == ("watch", REAL)
    assert b.watch_image(fake)[0] and b.real.watching is None    # a simulated one: the real preview is off
    assert b.camera_info(REAL)[0]["frame_id"] == "zed_left_camera_optical_frame"
    assert b.grab_photo(REAL) == (b"jpeg", "") and b.grab_photo(fake)[0][:2] == b"\xff\xd8"
    assert b.tf_parent("zed_left_camera_optical_frame") == "zed_camera_link"
    w, _ = b.open_image(REAL)                                    # browser windows: real and simulated side by side
    fw, _ = b.open_image(fake)
    assert w == "real watch" and fw.topic == fake and set(b.open_images()) == {REAL, fake}
    b.close_image(REAL)
    b.close_image(fake)
    assert b.open_images() == {}
    b.shutdown()
    assert b.real.down
