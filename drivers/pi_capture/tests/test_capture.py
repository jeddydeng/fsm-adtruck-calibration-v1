from __future__ import annotations

import socket
import sys
import tempfile
import threading
from pathlib import Path
import unittest


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from capture_cameras import CAMERA_ROLES, capture_four_cameras  # noqa: E402
from w102_capture_protocol import (  # noqa: E402
    CAMERA_JPEG,
    CAPTURE_RAW,
    FLAG_JPEG,
    FLAG_VALID,
    FragmentReassembler,
    decode_control_request,
    encode_capture_request,
    encode_test_frame,
)


class CaptureProtocolTest(unittest.TestCase):
    def test_capture_control_crc_and_fields(self) -> None:
        packet = encode_capture_request(43210, 0x000F, 0x123456789ABCDEF0)
        self.assertEqual(
            decode_control_request(packet),
            (CAPTURE_RAW, 0x000F, 43210, 0x123456789ABCDEF0),
        )
        corrupted = bytearray(packet)
        corrupted[12] ^= 1
        with self.assertRaises(ValueError):
            decode_control_request(bytes(corrupted))

    def test_out_of_order_fragment_reassembly(self) -> None:
        payload = b"\xff\xd8" + b"camera" * 900 + b"\xff\xd9"
        packets = encode_test_frame(
            payload, CAMERA_JPEG, FLAG_JPEG | FLAG_VALID, 2, 7, 99
        )
        reassembler = FragmentReassembler()
        complete = None
        for packet in reversed(packets):
            result = reassembler.push(packet)
            if result is not None:
                complete = result
        self.assertIsNotNone(complete)
        self.assertEqual(complete[1], payload)

    def test_mock_gateway_saves_atomic_four_image_set(self) -> None:
        ready = threading.Event()
        address: list[tuple[str, int]] = []

        def server() -> None:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.bind(("127.0.0.1", 0))
                address.append(sock.getsockname())
                ready.set()
                request, source = sock.recvfrom(2048)
                command, mask, reply_port, nonce = decode_control_request(request)
                self.assertEqual(command, CAPTURE_RAW)
                self.assertEqual(mask, 0x000F)
                destination = (source[0], reply_port)
                for stream_id in CAMERA_ROLES:
                    payload = (
                        b"\xff\xd8"
                        + f"stream-{stream_id}".encode("ascii") * 500
                        + b"\xff\xd9"
                    )
                    packets = encode_test_frame(
                        payload,
                        CAMERA_JPEG,
                        FLAG_JPEG | FLAG_VALID,
                        stream_id,
                        1234,
                        nonce,
                    )
                    for packet in reversed(packets):
                        sock.sendto(packet, destination)

        thread = threading.Thread(target=server, daemon=True)
        thread.start()
        self.assertTrue(ready.wait(2.0))
        with tempfile.TemporaryDirectory() as temporary:
            saved = capture_four_cameras(
                gateway_ip="127.0.0.1",
                control_port=address[0][1],
                timeout_s=1.0,
                retries=1,
                output_dir=Path(temporary),
            )
            self.assertEqual(len(saved), 4)
            self.assertEqual(
                {path.name.rsplit("_", 1)[1] for path in saved},
                {f"{role.lower()}.jpg" for role in CAMERA_ROLES.values()},
            )
            self.assertTrue(all(path.read_bytes().startswith(b"\xff\xd8") for path in saved))
            self.assertEqual(list(Path(temporary).glob("*.tmp")), [])
        thread.join(2.0)
        self.assertFalse(thread.is_alive())


if __name__ == "__main__":
    unittest.main()
