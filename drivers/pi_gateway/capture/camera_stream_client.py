#!/usr/bin/env python3
"""Control, capture, and record the Pi 186 fisheye camera streams."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import json
import os
from pathlib import Path
import time
from typing import Any, Optional
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

import cv2


MODES = ("bev", "raw-all", "front", "rear", "left", "right")
RAW_SOURCES = ("front", "rear", "left", "right")
PROJECT_ROOT = Path(os.environ.get(
    "FSM_PI_MASTER_ROOT", Path(__file__).resolve().parents[1]))


class ClientError(RuntimeError):
    pass


class CameraStreamClient:
    def __init__(self, base_url: str) -> None:
        self.base_url = base_url.rstrip("/")

    def _json(self, path: str, value: Optional[Any] = None) -> dict[str, Any]:
        data = None
        headers = {}
        if value is not None:
            data = json.dumps(value, ensure_ascii=False).encode("utf-8")
            headers["Content-Type"] = "application/json"
        request = Request(self.base_url + path, data=data, headers=headers,
                          method="POST" if data is not None else "GET")
        try:
            with urlopen(request, timeout=4.0) as response:
                return json.loads(response.read().decode("utf-8"))
        except HTTPError as error:
            try:
                detail = json.loads(error.read().decode("utf-8")).get("error")
            except Exception:
                detail = str(error)
            raise ClientError(str(detail)) from error
        except (URLError, TimeoutError, json.JSONDecodeError) as error:
            raise ClientError(str(error)) from error

    def status(self) -> dict[str, Any]:
        return self._json("/api/stream/status")

    def set_mode(self, mode: str) -> dict[str, Any]:
        if mode not in MODES:
            raise ClientError("unknown mode: " + mode)
        return self._json("/api/stream/mode", {"mode": mode})

    def snapshot(self, source: str, destination: Path) -> Path:
        request = Request(
            f"{self.base_url}/api/stream/frame/{source}.jpg",
            headers={"Cache-Control": "no-cache"})
        try:
            with urlopen(request, timeout=4.0) as response:
                payload = response.read()
        except (HTTPError, URLError, TimeoutError) as error:
            raise ClientError(f"{source}: {error}") from error
        if len(payload) < 4 or not payload.startswith(b"\xff\xd8"):
            raise ClientError(f"{source}: invalid JPEG response")
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_suffix(destination.suffix + ".tmp")
        temporary.write_bytes(payload)
        temporary.replace(destination)
        return destination

    def record(self, source: str, destination: Path, duration_s: float,
               fps: float) -> dict[str, Any]:
        query = urlencode({"source": source, "fps": fps})
        capture = cv2.VideoCapture(
            f"{self.base_url}/api/stream/mjpeg?{query}", cv2.CAP_FFMPEG)
        if not capture.isOpened():
            capture.release()
            raise ClientError(f"{source}: cannot open MJPEG stream")
        writer = None
        frames = 0
        started = time.monotonic()
        deadline = started + duration_s
        destination.parent.mkdir(parents=True, exist_ok=True)
        try:
            while time.monotonic() < deadline:
                ok, frame = capture.read()
                if not ok or frame is None:
                    if time.monotonic() - started > 3.0 and frames == 0:
                        raise ClientError(f"{source}: no frames received")
                    continue
                if writer is None:
                    height, width = frame.shape[:2]
                    writer = cv2.VideoWriter(
                        str(destination), cv2.VideoWriter_fourcc(*"mp4v"),
                        fps, (width, height))
                    if not writer.isOpened():
                        raise ClientError(f"{source}: cannot create {destination}")
                writer.write(frame)
                frames += 1
        finally:
            capture.release()
            if writer is not None:
                writer.release()
        elapsed = max(0.001, time.monotonic() - started)
        if frames == 0:
            raise ClientError(f"{source}: recording contains no frames")
        return {
            "source": source,
            "file": str(destination),
            "frames": frames,
            "elapsed_s": round(elapsed, 3),
            "recorded_fps": round(frames / elapsed, 2),
        }


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--base-url", default="http://127.0.0.1:8086")
    sub = result.add_subparsers(dest="command", required=True)
    sub.add_parser("status")
    mode = sub.add_parser("mode")
    mode.add_argument("mode", choices=MODES)
    snap = sub.add_parser("snapshot")
    snap.add_argument("--source", choices=("all", "bev", *RAW_SOURCES),
                      default="all")
    snap.add_argument("--output-dir", type=Path,
                      default=PROJECT_ROOT / "runtime/capture-image")
    record = sub.add_parser("record")
    record.add_argument("--source", choices=("all", "bev", *RAW_SOURCES),
                        default="all")
    record.add_argument("--duration", type=float, default=60.0)
    record.add_argument("--fps", type=float, default=10.0)
    record.add_argument("--output-dir", type=Path,
                        default=PROJECT_ROOT / "runtime/record-video")
    record.add_argument("--keep-mode", action="store_true",
                        help="do not restore the previous output mode")
    return result


def main() -> int:
    args = parser().parse_args()
    client = CameraStreamClient(args.base_url)
    try:
        if args.command == "status":
            print(json.dumps(client.status(), ensure_ascii=False, indent=2))
            return 0
        if args.command == "mode":
            print(json.dumps(client.set_mode(args.mode), ensure_ascii=False,
                             indent=2))
            return 0

        previous_mode = str(client.status().get("mode", "bev"))
        sources = RAW_SOURCES if args.source == "all" else (args.source,)
        required_mode = "raw-all" if args.source == "all" else args.source
        if required_mode in MODES and required_mode != previous_mode:
            client.set_mode(required_mode)
            time.sleep(0.4)
        timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
        try:
            if args.command == "snapshot":
                saved = [client.snapshot(
                    source, args.output_dir / f"{timestamp}_{source}.jpg")
                    for source in sources]
                print(json.dumps({"files": [str(path) for path in saved]},
                                 ensure_ascii=False, indent=2))
            else:
                if not 0.5 <= args.duration <= 86400:
                    raise ClientError("duration must be in [0.5, 86400] seconds")
                if not 1 <= args.fps <= 30:
                    raise ClientError("fps must be in [1, 30]")
                with ThreadPoolExecutor(max_workers=len(sources)) as pool:
                    futures = [pool.submit(
                        client.record, source,
                        args.output_dir / f"{timestamp}_{source}.mp4",
                        args.duration, args.fps) for source in sources]
                    recordings = [future.result() for future in futures]
                print(json.dumps({"recordings": recordings}, ensure_ascii=False,
                                 indent=2))
        finally:
            if not getattr(args, "keep_mode", False) and previous_mode in MODES:
                client.set_mode(previous_mode)
        return 0
    except (ClientError, OSError) as error:
        print(f"camera stream error: {error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
