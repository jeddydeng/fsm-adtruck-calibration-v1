"""Small local HTTP client for the traffic-camera ROS 2 bridge."""

from __future__ import annotations

import json
from typing import Optional
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class TrafficCameraClient:
    def __init__(self, base_url: str = "http://127.0.0.1:8090") -> None:
        self.base_url = base_url.rstrip("/")

    def status(self) -> dict:
        try:
            with urlopen(Request(self.base_url + "/status",
                                 headers={"Cache-Control": "no-store"}),
                         timeout=0.35) as response:
                value = json.loads(response.read(65536).decode("utf-8"))
            return value if isinstance(value, dict) else {
                "online": False, "error": "第五路状态格式无效"}
        except (HTTPError, URLError, TimeoutError, OSError,
                json.JSONDecodeError) as error:
            return {"online": False, "query_state": "bridge_unavailable",
                    "error": str(error)}

    def frame(self) -> tuple[Optional[bytes], int]:
        try:
            with urlopen(Request(self.base_url + "/image.jpg",
                                 headers={"Cache-Control": "no-store"}),
                         timeout=0.5) as response:
                frame = response.read(2 * 1024 * 1024)
            if not (frame.startswith(b"\xff\xd8") and frame.endswith(b"\xff\xd9")):
                return None, 0
            status = self.status()
            return frame, int(status.get("frame_id") or 0)
        except (HTTPError, URLError, TimeoutError, OSError):
            return None, 0
