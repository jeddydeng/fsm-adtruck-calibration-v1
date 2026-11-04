#!/usr/bin/env python3
"""Four-camera processing SDK for Pi 186's 10 Hz raw fisheye mode."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading
import time
from typing import Optional
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

import cv2
import numpy as np

from camera_stream_client import CameraStreamClient, ClientError


SOURCES = ("front", "rear", "left", "right")


@dataclass(frozen=True)
class RawFrame:
    source: str
    sequence: int
    received_unix_ns: int
    received_monotonic_ns: int
    fps: float
    bgr: np.ndarray

    @property
    def age_s(self) -> float:
        return max(0.0, (time.monotonic_ns() - self.received_monotonic_ns) / 1e9)


class FisheyeRawReceiver:
    """Receive four decoded BGR frames concurrently from the local IPC service.

    `start()` switches the Pi's current output to `raw-all`. `close()` restores the
    mode that was active before this object started, but only if no other caller
    changed the mode in the meantime.
    """

    def __init__(self, base_url: str = "http://127.0.0.1:8086",
                 fps: float = 10.0) -> None:
        if not 1.0 <= fps <= 10.0:
            raise ValueError("fps must be in [1, 10]")
        self.base_url = base_url.rstrip("/")
        self.requested_fps = float(fps)
        self.client = CameraStreamClient(self.base_url)
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._latest: dict[str, RawFrame] = {}
        self._sequences = {source: 0 for source in SOURCES}
        self._times = {source: deque(maxlen=40) for source in SOURCES}
        self._errors: dict[str, str] = {}
        self._previous_mode: Optional[str] = None
        self._started = False

    def start(self, timeout_s: float = 5.0) -> "FisheyeRawReceiver":
        if self._started:
            return self
        self._previous_mode = str(self.client.status().get("mode", "bev"))
        result = self.client.set_mode("raw-all")
        if result.get("output_mode") != "raw-all":
            raise ClientError("Pi did not enter raw-all mode")
        self._stop.clear()
        self._started = True
        for source in SOURCES:
            thread = threading.Thread(target=self._receive_loop, args=(source,),
                                      name=f"fisheye-{source}", daemon=True)
            self._threads.append(thread)
            thread.start()
        try:
            self.wait_ready(timeout_s)
        except BaseException:
            self.close()
            raise
        return self

    def wait_ready(self, timeout_s: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while len(self._latest) < len(SOURCES):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    missing = [source for source in SOURCES
                               if source not in self._latest]
                    raise TimeoutError("no fresh frame from: " + ", ".join(missing))
                self._condition.wait(min(0.2, remaining))

    def read(self, source: str, after_sequence: int = 0,
             timeout_s: float = 1.0, copy: bool = True) -> RawFrame:
        if source not in SOURCES:
            raise ValueError("source must be one of: " + ", ".join(SOURCES))
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while (source not in self._latest or
                   self._latest[source].sequence <= after_sequence):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"no new {source} frame")
                self._condition.wait(min(0.1, remaining))
            frame = self._latest[source]
            if not copy:
                return frame
            return RawFrame(frame.source, frame.sequence, frame.received_unix_ns,
                            frame.received_monotonic_ns, frame.fps,
                            frame.bgr.copy())

    def read_all(self, timeout_s: float = 1.0,
                 max_age_s: float = 0.5, copy: bool = True) -> dict[str, RawFrame]:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                frames = {source: self._latest.get(source) for source in SOURCES}
                if all(frame is not None and frame.age_s <= max_age_s
                       for frame in frames.values()):
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("four fresh fisheye frames are not available")
                self._condition.wait(min(0.1, remaining))
            result = {source: frame for source, frame in frames.items()
                      if frame is not None}
            if not copy:
                return result
            return {
                source: RawFrame(frame.source, frame.sequence,
                                 frame.received_unix_ns,
                                 frame.received_monotonic_ns, frame.fps,
                                 frame.bgr.copy())
                for source, frame in result.items()
            }

    def status(self) -> dict:
        with self._condition:
            return {
                "running": self._started and not self._stop.is_set(),
                "mode": self.client.status().get("mode"),
                "sources": {
                    source: {
                        "sequence": (self._latest[source].sequence
                                     if source in self._latest else 0),
                        "fps": (self._latest[source].fps
                                if source in self._latest else 0.0),
                        "age_s": (self._latest[source].age_s
                                  if source in self._latest else None),
                        "error": self._errors.get(source),
                    }
                    for source in SOURCES
                },
            }

    def close(self, restore_previous: bool = True) -> None:
        if not self._started:
            return
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        for thread in self._threads:
            thread.join(timeout=3.0)
        self._threads.clear()
        self._started = False
        if restore_previous and self._previous_mode:
            try:
                if self.client.status().get("mode") == "raw-all":
                    self.client.set_mode(self._previous_mode)
            except ClientError:
                pass

    def _receive_loop(self, source: str) -> None:
        query = urlencode({"source": source, "fps": self.requested_fps})
        url = f"{self.base_url}/api/stream/mjpeg?{query}"
        while not self._stop.is_set():
            try:
                response = urlopen(Request(
                    url, headers={"Cache-Control": "no-store"}), timeout=2.0)
            except (HTTPError, URLError, TimeoutError, OSError) as error:
                with self._condition:
                    self._errors[source] = f"cannot open MJPEG stream: {error}"
                self._stop.wait(0.5)
                continue
            with self._condition:
                self._errors.pop(source, None)
            try:
                with response:
                    while not self._stop.is_set():
                        jpeg = self._read_mjpeg_frame(response)
                        if jpeg is None:
                            break
                        image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8),
                                             cv2.IMREAD_COLOR)
                        if image is None:
                            continue
                        monotonic_ns = time.monotonic_ns()
                        unix_ns = time.time_ns()
                        with self._condition:
                            self._sequences[source] += 1
                            timestamps = self._times[source]
                            timestamps.append(monotonic_ns)
                            fps = 0.0
                            if (len(timestamps) >= 2 and
                                    timestamps[-1] > timestamps[0]):
                                fps = (len(timestamps) - 1) * 1e9 / (
                                    timestamps[-1] - timestamps[0])
                            self._latest[source] = RawFrame(
                                source, self._sequences[source], unix_ns,
                                monotonic_ns, fps, image)
                            self._condition.notify_all()
            except (HTTPError, URLError, TimeoutError, OSError, ValueError) as error:
                with self._condition:
                    if not self._stop.is_set():
                        self._errors[source] = f"stream interrupted: {error}"
            self._stop.wait(0.2)

    @staticmethod
    def _read_mjpeg_frame(response) -> Optional[bytes]:
        while True:
            line = response.readline(4096)
            if not line:
                return None
            if line.startswith(b"--"):
                break
        content_length = None
        while True:
            line = response.readline(4096)
            if not line:
                return None
            if line in (b"\r\n", b"\n"):
                break
            name, separator, value = line.partition(b":")
            if separator and name.strip().lower() == b"content-length":
                content_length = int(value.strip())
        if content_length is None or not 4 <= content_length <= 8 * 1024 * 1024:
            raise ValueError("invalid MJPEG Content-Length")
        payload = response.read(content_length)
        if len(payload) != content_length:
            return None
        if not (payload.startswith(b"\xff\xd8") and payload.endswith(b"\xff\xd9")):
            raise ValueError("invalid JPEG frame")
        response.readline(4)
        return payload

    def __enter__(self) -> "FisheyeRawReceiver":
        return self.start()

    def __exit__(self, _type, _value, _traceback) -> None:
        self.close()


def main() -> None:
    with FisheyeRawReceiver() as receiver:
        started = time.monotonic()
        while time.monotonic() - started < 10.0:
            frames = receiver.read_all()
            print(" ".join(
                f"{source}={frames[source].bgr.shape[1]}x"
                f"{frames[source].bgr.shape[0]}@{frames[source].fps:.2f}Hz"
                for source in SOURCES))
            time.sleep(1.0)


if __name__ == "__main__":
    main()
