"""Minimal W102 UDP capture protocol codec with strict fragment validation."""

from __future__ import annotations

from dataclasses import dataclass
import struct
import time
import zlib


DATA_MAGIC = b"W1UP"
CONTROL_MAGIC = b"W1CT"
VERSION = 1
HEADER_SIZE = 56
CONTROL_SIZE = 24
FRAGMENT_PAYLOAD_SIZE = 1400
MAX_FRAME_SIZE = 16 * 1024 * 1024

CAMERA_JPEG = 1
CONTROL_JSON = 5
CAPTURE_RAW = 5

FLAG_JPEG = 1 << 0
FLAG_VALID = 1 << 2

_HEADER = struct.Struct(">4sBBHHHQQQIIHHHHI")
_CONTROL = struct.Struct(">4sBBHHHQI")


@dataclass(frozen=True)
class Header:
    message_type: int
    flags: int
    stream_id: int
    boot_id: int
    frame_id: int
    capture_mono_ns: int
    total_size: int
    frame_crc32: int
    fragment_index: int
    fragment_count: int
    payload_size: int


@dataclass
class _Assembly:
    header: Header
    created_at: float
    fragments: list[bytes | None]
    received_size: int = 0


def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def encode_capture_request(reply_port: int, stream_mask: int, nonce: int) -> bytes:
    if not 1 <= reply_port <= 65535:
        raise ValueError("reply port must be in [1, 65535]")
    if not 1 <= stream_mask <= 0xFFFF:
        raise ValueError("stream mask must be non-zero and fit uint16")
    if not 0 <= nonce <= 0xFFFFFFFFFFFFFFFF:
        raise ValueError("nonce must fit uint64")
    prefix = _CONTROL.pack(
        CONTROL_MAGIC, VERSION, CAPTURE_RAW, stream_mask, reply_port, 0, nonce, 0
    )
    return prefix[:20] + struct.pack(">I", crc32(prefix[:20]))


def decode_control_request(packet: bytes) -> tuple[int, int, int, int]:
    if len(packet) != CONTROL_SIZE:
        raise ValueError("bad control packet size")
    magic, version, command, mask, port, _reserved, nonce, packet_crc = (
        _CONTROL.unpack(packet)
    )
    if magic != CONTROL_MAGIC or version != VERSION:
        raise ValueError("bad control packet identity")
    if crc32(packet[:20]) != packet_crc:
        raise ValueError("bad control packet CRC")
    return command, mask, port, nonce


def decode_datagram(datagram: bytes) -> tuple[Header, bytes]:
    if len(datagram) < HEADER_SIZE:
        raise ValueError("short datagram")
    values = _HEADER.unpack_from(datagram)
    (
        magic,
        version,
        header_size,
        message_type,
        flags,
        stream_id,
        boot_id,
        frame_id,
        capture_ns,
        total_size,
        frame_crc,
        fragment_index,
        fragment_count,
        payload_size,
        _reserved,
        header_crc,
    ) = values
    if magic != DATA_MAGIC or version != VERSION or header_size != HEADER_SIZE:
        raise ValueError("bad data packet identity")
    if crc32(datagram[:52]) != header_crc:
        raise ValueError("bad header CRC")
    if total_size <= 0 or total_size > MAX_FRAME_SIZE:
        raise ValueError("bad total frame size")
    expected_count = (
        total_size + FRAGMENT_PAYLOAD_SIZE - 1
    ) // FRAGMENT_PAYLOAD_SIZE
    offset = fragment_index * FRAGMENT_PAYLOAD_SIZE
    expected_payload_size = min(FRAGMENT_PAYLOAD_SIZE, total_size - offset)
    if (
        fragment_count != expected_count
        or fragment_count <= 0
        or fragment_index >= fragment_count
        or offset >= total_size
        or payload_size != expected_payload_size
        or HEADER_SIZE + payload_size != len(datagram)
    ):
        raise ValueError("inconsistent fragmentation")
    header = Header(
        message_type,
        flags,
        stream_id,
        boot_id,
        frame_id,
        capture_ns,
        total_size,
        frame_crc,
        fragment_index,
        fragment_count,
        payload_size,
    )
    return header, datagram[HEADER_SIZE:]


class FragmentReassembler:
    """Bounded reassembly keyed by boot, type, stream and request frame ID."""

    def __init__(
        self,
        timeout_s: float = 3.0,
        max_assemblies: int = 32,
        max_buffered_bytes: int = 32 * 1024 * 1024,
    ) -> None:
        self.timeout_s = timeout_s
        self.max_assemblies = max_assemblies
        self.max_buffered_bytes = max_buffered_bytes
        self._assemblies: dict[tuple[int, int, int, int], _Assembly] = {}
        self._buffered_bytes = 0

    def push(self, datagram: bytes, now: float | None = None) -> tuple[Header, bytes] | None:
        current = time.monotonic() if now is None else now
        self.expire(current)
        header, chunk = decode_datagram(datagram)
        key = (
            header.boot_id,
            header.message_type,
            header.stream_id,
            header.frame_id,
        )
        assembly = self._assemblies.get(key)
        if assembly is None:
            if len(self._assemblies) >= self.max_assemblies:
                self._drop(next(iter(self._assemblies)))
            assembly = _Assembly(
                header, current, [None] * header.fragment_count
            )
            self._assemblies[key] = assembly
        elif (
            assembly.header.total_size != header.total_size
            or assembly.header.frame_crc32 != header.frame_crc32
            or assembly.header.fragment_count != header.fragment_count
            or assembly.header.flags != header.flags
            or assembly.header.capture_mono_ns != header.capture_mono_ns
        ):
            self._drop(key)
            raise ValueError("fragment metadata changed within a frame")
        if assembly.fragments[header.fragment_index] is not None:
            return None
        assembly.fragments[header.fragment_index] = chunk
        assembly.received_size += len(chunk)
        self._buffered_bytes += len(chunk)
        while self._buffered_bytes > self.max_buffered_bytes and self._assemblies:
            self._drop(next(iter(self._assemblies)))
        if key not in self._assemblies or assembly.received_size != header.total_size:
            return None
        if any(fragment is None for fragment in assembly.fragments):
            return None
        payload = b"".join(
            fragment for fragment in assembly.fragments if fragment is not None
        )
        completed_header = assembly.header
        self._drop(key)
        if len(payload) != completed_header.total_size:
            raise ValueError("reassembled frame length mismatch")
        if crc32(payload) != completed_header.frame_crc32:
            raise ValueError("bad full-frame CRC")
        return completed_header, payload

    def expire(self, now: float | None = None) -> None:
        current = time.monotonic() if now is None else now
        for key, assembly in list(self._assemblies.items()):
            if current - assembly.created_at > self.timeout_s:
                self._drop(key)

    def _drop(self, key: tuple[int, int, int, int]) -> None:
        assembly = self._assemblies.pop(key, None)
        if assembly is not None:
            self._buffered_bytes -= assembly.received_size


def encode_test_frame(
    payload: bytes,
    message_type: int,
    flags: int,
    stream_id: int,
    boot_id: int,
    frame_id: int,
    capture_mono_ns: int | None = None,
) -> list[bytes]:
    """Reference encoder used only by the standard-library test suite."""
    if not payload or len(payload) > MAX_FRAME_SIZE:
        raise ValueError("payload size out of range")
    capture_ns = time.monotonic_ns() if capture_mono_ns is None else capture_mono_ns
    frame_crc = crc32(payload)
    count = (len(payload) + FRAGMENT_PAYLOAD_SIZE - 1) // FRAGMENT_PAYLOAD_SIZE
    output: list[bytes] = []
    for index in range(count):
        chunk = payload[
            index * FRAGMENT_PAYLOAD_SIZE : (index + 1) * FRAGMENT_PAYLOAD_SIZE
        ]
        prefix = _HEADER.pack(
            DATA_MAGIC,
            VERSION,
            HEADER_SIZE,
            message_type,
            flags,
            stream_id,
            boot_id,
            frame_id,
            capture_ns,
            len(payload),
            frame_crc,
            index,
            count,
            len(chunk),
            0,
            0,
        )
        header = prefix[:52] + struct.pack(">I", crc32(prefix[:52]))
        output.append(header + chunk)
    return output

