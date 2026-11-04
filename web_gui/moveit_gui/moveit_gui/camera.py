"""Camera inspection without ROS or Qt: decode sensor_msgs/Image payloads and keep per-topic statistics.

The bridges subscribe to ONE image topic at a time (the one being previewed) and hand the raw message fields
here. Frames are downsampled while decoding so a 1280x720 stream costs the GUI a 640-wide RGB array.
CompressedImage payloads (jpeg/png) are kept as bytes; the GUI decodes them.
"""
from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

IMAGE, COMPRESSED = "sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"
IMAGE_TYPES = (IMAGE, COMPRESSED)


class ImageDecodeError(ValueError):
    pass


def _channels(encoding: str):
    e = encoding.lower()
    table = {"rgb8": (3, np.uint8, "rgb"), "bgr8": (3, np.uint8, "bgr"), "rgba8": (4, np.uint8, "rgb"),
             "bgra8": (4, np.uint8, "bgr"), "mono8": (1, np.uint8, "mono"), "8uc1": (1, np.uint8, "mono"),
             "mono16": (1, np.uint16, "depth"), "16uc1": (1, np.uint16, "depth"), "32fc1": (1, np.float32, "depth"),
             "8uc3": (3, np.uint8, "bgr")}
    if e not in table:
        raise ImageDecodeError(f"encoding '{encoding}' is not supported in the preview")
    return table[e]


def decode_image(encoding: str, width: int, height: int, step: int, data, is_bigendian: bool = False,
                 max_width: int = 640) -> np.ndarray:
    """sensor_msgs/Image -> HxWx3 uint8 RGB, downsampled by an integer factor to at most max_width.

    Depth (16UC1 in mm, 32FC1 in m) is shown as grey, near = bright, scaled between the 2nd and 98th percentile
    of the valid pixels; zeros / NaN (no return) are black.
    """
    ch, dtype, kind = _channels(encoding)
    itemsize = np.dtype(dtype).itemsize
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    if width <= 0 or height <= 0 or step < width * ch * itemsize or buf.size < step * height:
        raise ImageDecodeError(f"{width}x{height} step {step} does not match {buf.size} bytes")
    rows = buf[:step * height].reshape(height, step)[:, :width * ch * itemsize]
    dt = np.dtype(dtype).newbyteorder(">" if is_bigendian else "<") if itemsize > 1 else np.dtype(dtype)
    img = rows.copy().view(dt).reshape(height, width, ch)
    k = max(1, int(np.ceil(width / float(max_width))))
    img = img[::k, ::k]
    if kind == "depth":
        d = img[..., 0].astype(np.float32)
        valid = np.isfinite(d) & (d > 0)
        out = np.zeros(d.shape, np.uint8)
        if valid.any():
            lo, hi = np.percentile(d[valid], (2, 98))
            span = max(hi - lo, 1e-6)
            out[valid] = (255 - np.clip((d[valid] - lo) / span, 0, 1) * 215).astype(np.uint8)
        return np.repeat(out[..., None], 3, axis=2)
    if kind == "mono":
        return np.repeat(img[..., :1], 3, axis=2).astype(np.uint8)
    rgb = img[..., :3]
    if kind == "bgr":
        rgb = rgb[..., ::-1]
    return np.ascontiguousarray(rgb, dtype=np.uint8)


def to_gray(encoding: str, width: int, height: int, step: int, data, is_bigendian: bool = False) -> np.ndarray:
    """sensor_msgs/Image (colour or mono, 8 bit) -> HxW uint8 grey at full resolution, for calibration."""
    ch, dtype, kind = _channels(encoding)
    if kind == "depth" or dtype != np.uint8:
        raise ImageDecodeError(f"{encoding} is a depth image: calibrate on the colour or infrared stream")
    img = decode_image(encoding, width, height, step, data, is_bigendian, max_width=max(1, int(width)))
    return np.ascontiguousarray(img[..., 0] if kind == "mono" else
                                (0.299 * img[..., 0] + 0.587 * img[..., 1] + 0.114 * img[..., 2]).astype(np.uint8))


@dataclass
class CameraFrame:
    topic: str
    rgb: Optional[np.ndarray] = None        # decoded Image
    compressed: Optional[bytes] = None      # CompressedImage payload (jpeg/png)
    width: int = 0                          # of the original image
    height: int = 0
    encoding: str = ""
    frame_id: str = ""
    stamp: float = 0.0                      # wall time it arrived
    error: str = ""


@dataclass
class TopicStats:
    """Arrival rate over the last few seconds, and what the stream looks like."""
    window_s: float = 3.0
    times: deque = field(default_factory=lambda: deque(maxlen=300))

    def tick(self, t: Optional[float] = None):
        self.times.append(time.monotonic() if t is None else t)

    def rate(self, now: Optional[float] = None) -> float:
        now = time.monotonic() if now is None else now
        recent = [t for t in self.times if now - t <= self.window_s]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / max(recent[-1] - recent[0], 1e-6)

    def age(self, now: Optional[float] = None) -> Optional[float]:
        if not self.times:
            return None
        return (time.monotonic() if now is None else now) - self.times[-1]


class ImageWatch:
    """The single watched image topic: latest frame + statistics, shared between the ROS and Qt threads.

    Decoding is throttled to `max_decode_hz`; every message still counts for the rate.
    """

    def __init__(self, topic: str, max_decode_hz: float = 15.0, max_width: int = 640):
        self.topic = topic
        self.stats = TopicStats()
        self.max_width = max_width
        self._period = 1.0 / max_decode_hz
        self._last_decode = 0.0
        self._lock = threading.Lock()
        self._frame: Optional[CameraFrame] = None

    def on_image(self, encoding, width, height, step, data, is_bigendian=False, frame_id=""):
        now = time.monotonic()
        self.stats.tick(now)
        if now - self._last_decode < self._period:
            return
        self._last_decode = now
        f = CameraFrame(self.topic, width=int(width), height=int(height), encoding=str(encoding),
                        frame_id=str(frame_id), stamp=now)
        try:
            f.rgb = decode_image(encoding, width, height, step, data, is_bigendian, self.max_width)
        except ImageDecodeError as e:
            f.error = str(e)
        with self._lock:
            self._frame = f

    def on_compressed(self, fmt, data, frame_id=""):
        now = time.monotonic()
        self.stats.tick(now)
        if now - self._last_decode < self._period:
            return
        self._last_decode = now
        with self._lock:
            self._frame = CameraFrame(self.topic, compressed=bytes(data), encoding=str(fmt), frame_id=str(frame_id),
                                      stamp=now)

    def latest(self) -> Optional[CameraFrame]:
        with self._lock:
            return self._frame
