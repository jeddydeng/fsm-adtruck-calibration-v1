#!/usr/bin/env python3
"""Request one fresh JPEG from each Raspberry Pi fisheye camera."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import secrets
import socket
import sys
import time

from w102_capture_protocol import (
    CAMERA_JPEG,
    CONTROL_JSON,
    FLAG_JPEG,
    FLAG_VALID,
    FragmentReassembler,
    encode_capture_request,
)


CAMERA_ROLES = {
    0: "FRONT",
    1: "REAR",
    2: "LEFT",
    3: "RIGHT",
}
ALL_CAMERA_MASK = sum(1 << stream_id for stream_id in CAMERA_ROLES)


class CaptureIncomplete(RuntimeError):
    def __init__(self, missing_streams: set[int], last_ack: dict | None) -> None:
        self.missing_streams = missing_streams
        self.last_ack = last_ack
        names = ", ".join(CAMERA_ROLES[index] for index in sorted(missing_streams))
        detail = f"missing fresh camera images: {names}"
        if last_ack:
            if last_ack.get("error"):
                detail += f"; gateway: {last_ack['error']}"
            else:
                detail += (
                    f"; gateway captured_mask={last_ack.get('captured_mask')}"
                    f" missing_mask={last_ack.get('missing_mask')}"
                )
        super().__init__(detail)


def is_jpeg(payload: bytes) -> bool:
    return len(payload) >= 4 and payload[:2] == b"\xff\xd8" and payload[-2:] == b"\xff\xd9"


def capture_four_cameras(
    gateway_ip: str = "192.168.1.186",
    control_port: int = 5611,
    bind_ip: str = "0.0.0.0",
    timeout_s: float = 2.0,
    retries: int = 5,
    output_dir: Path | None = None,
) -> list[Path]:
    if not 1 <= control_port <= 65535:
        raise ValueError("control port must be in [1, 65535]")
    if timeout_s <= 0:
        raise ValueError("timeout must be positive")
    if retries < 1:
        raise ValueError("retries must be at least one")

    target_dir = output_dir or Path(__file__).resolve().parent / "capture-image"
    images: dict[int, bytes] = {}
    last_ack: dict | None = None
    reassembler = FragmentReassembler(timeout_s=max(3.0, timeout_s * 2.0))

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as capture_socket:
        capture_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        capture_socket.bind((bind_ip, 0))
        reply_port = capture_socket.getsockname()[1]

        for _attempt in range(1, retries + 1):
            missing = set(CAMERA_ROLES) - set(images)
            if not missing:
                break
            requested_mask = sum(1 << stream_id for stream_id in missing)
            nonce = secrets.randbits(64) or 1
            request = encode_capture_request(reply_port, requested_mask, nonce)
            capture_socket.sendto(request, (gateway_ip, control_port))
            deadline = time.monotonic() + timeout_s

            while missing and time.monotonic() < deadline:
                capture_socket.settimeout(max(0.01, deadline - time.monotonic()))
                try:
                    datagram, source = capture_socket.recvfrom(65535)
                except socket.timeout:
                    break
                if source[0] != gateway_ip:
                    continue
                try:
                    complete = reassembler.push(datagram)
                except ValueError:
                    continue
                if complete is None:
                    continue
                header, payload = complete
                if header.message_type == CAMERA_JPEG:
                    if (
                        header.frame_id == nonce
                        and header.stream_id in missing
                        and header.flags & FLAG_JPEG
                        and header.flags & FLAG_VALID
                        and is_jpeg(payload)
                    ):
                        images[header.stream_id] = payload
                        missing.remove(header.stream_id)
                elif header.message_type == CONTROL_JSON:
                    try:
                        acknowledgement = json.loads(payload.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        continue
                    if acknowledgement.get("nonce") == nonce:
                        last_ack = acknowledgement

    missing = set(CAMERA_ROLES) - set(images)
    if missing:
        raise CaptureIncomplete(missing, last_ack)

    target_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
    saved: list[Path] = []
    temporary_paths: list[Path] = []
    try:
        for stream_id, role in CAMERA_ROLES.items():
            destination = target_dir / f"{timestamp}_{role.lower()}.jpg"
            if destination.exists():
                raise FileExistsError(f"refusing to overwrite {destination}")
            temporary = target_dir / f".{timestamp}_{role}.{os.getpid()}.tmp"
            with temporary.open("xb") as output:
                output.write(images[stream_id])
                output.flush()
                os.fsync(output.fileno())
            temporary_paths.append(temporary)
            os.replace(temporary, destination)
            saved.append(destination)
    finally:
        for temporary in temporary_paths:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
    return saved


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture the latest FRONT/REAR/LEFT/RIGHT JPEGs from the Raspberry Pi"
    )
    parser.add_argument("--gateway-ip", default="192.168.1.186")
    parser.add_argument("--control-port", type=int, default=5611)
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--retries", type=int, default=5)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "capture-image",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        paths = capture_four_cameras(
            gateway_ip=args.gateway_ip,
            control_port=args.control_port,
            bind_ip=args.bind_ip,
            timeout_s=args.timeout,
            retries=args.retries,
            output_dir=args.output_dir,
        )
    except CaptureIncomplete as error:
        print(f"Capture failed: {error}", file=sys.stderr)
        print("Check the named camera cables and wait for the Pi gateway to report them online.", file=sys.stderr)
        return 2
    except (OSError, ValueError) as error:
        print(f"Capture failed: {error}", file=sys.stderr)
        return 1

    print("Capture complete:")
    for path in paths:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
