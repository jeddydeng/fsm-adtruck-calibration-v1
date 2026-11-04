"""W102 UDP v1 codec and bounded, restart-aware fragment reassembly."""

from __future__ import annotations

from collections import OrderedDict, deque
from dataclasses import dataclass, field
import struct
import time
import zlib
from typing import Dict, Optional, Tuple


MAGIC = b"W1UP"
CONTROL_MAGIC = b"W1CT"
VERSION = 1
HEADER_SIZE = 56
CONTROL_SIZE = 24
FRAGMENT_PAYLOAD_SIZE = 1400
MAX_FRAME_SIZE = 16 * 1024 * 1024

CAMERA_JPEG = 1
BEV_JPEG = 2
HEALTH_JSON = 3
COVERAGE_PNG = 4
CONTROL_JSON = 5

CONTROL_HELLO = 1
CONTROL_SET_FEATURES = 2
CONTROL_SET_OUTPUT_FPS = 3
CONTROL_SET_JPEG_QUALITY = 4
CONTROL_CAPTURE_RAW = 5
CONTROL_SET_OUTPUT_MODE = 6

OUTPUT_FRONT = 1 << 0
OUTPUT_REAR = 1 << 1
OUTPUT_LEFT = 1 << 2
OUTPUT_RIGHT = 1 << 3
OUTPUT_RAW_ALL = OUTPUT_FRONT | OUTPUT_REAR | OUTPUT_LEFT | OUTPUT_RIGHT
OUTPUT_BEV = 1 << 15

OUTPUT_MODE_MASKS = {
    "bev": OUTPUT_BEV,
    "raw-all": OUTPUT_RAW_ALL,
    "front": OUTPUT_FRONT,
    "rear": OUTPUT_REAR,
    "left": OUTPUT_LEFT,
    "right": OUTPUT_RIGHT,
}

FLAG_JPEG = 1 << 0
FLAG_REPLAY = 1 << 1
FLAG_VALID = 1 << 2
FLAG_PREVIEW_ONLY = 1 << 3
FLAG_SYNTHETIC_FILL = 1 << 4
FLAG_REPEATED = 1 << 5
FLAG_PARTIAL_CAMERAS = 1 << 6
FLAG_ILLUMINATION = 1 << 7

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


@dataclass(frozen=True)
class CompleteFrame:
    header: Header
    payload: bytes
    received_at: float
    source: Tuple[str, int]


@dataclass
class ReassemblyStats:
    datagrams: int = 0
    invalid_datagrams: int = 0
    header_crc_errors: int = 0
    frame_crc_errors: int = 0
    duplicate_fragments: int = 0
    completed_frames: int = 0
    replay_frames_seen: int = 0
    expired_frames: int = 0
    evicted_frames: int = 0
    stale_frames: int = 0
    sender_restarts: int = 0
    frame_gaps: int = 0


@dataclass
class _Assembly:
    header: Header
    source: Tuple[str, int]
    created_at: float
    fragments: list[Optional[bytes]]
    byte_count: int = 0


def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def decode_datagram(datagram: bytes) -> tuple[Header, bytes]:
    if len(datagram) < HEADER_SIZE:
        raise ValueError("short datagram")
    values = _HEADER.unpack_from(datagram)
    (magic, version, header_size, message_type, flags, stream_id, boot_id,
     frame_id, capture_ns, total_size, frame_crc, fragment_index,
     fragment_count, payload_size, _reserved, header_crc) = values
    if magic != MAGIC or version != VERSION or header_size != HEADER_SIZE:
        raise ValueError("bad protocol identity")
    if crc32(datagram[:52]) != header_crc:
        raise ValueError("header CRC")
    if fragment_count == 0 or fragment_index >= fragment_count:
        raise ValueError("bad fragment index/count")
    if payload_size == 0 or HEADER_SIZE + payload_size != len(datagram):
        raise ValueError("bad payload size")
    if total_size == 0 or total_size > MAX_FRAME_SIZE:
        raise ValueError("bad total size")
    expected_count = (total_size + FRAGMENT_PAYLOAD_SIZE - 1) // FRAGMENT_PAYLOAD_SIZE
    offset = fragment_index * FRAGMENT_PAYLOAD_SIZE
    expected_size = min(FRAGMENT_PAYLOAD_SIZE, total_size - offset)
    if fragment_count != expected_count or offset >= total_size or payload_size != expected_size:
        raise ValueError("inconsistent fragmentation")
    header = Header(message_type, flags, stream_id, boot_id, frame_id,
                    capture_ns, total_size, frame_crc, fragment_index,
                    fragment_count, payload_size)
    return header, datagram[HEADER_SIZE:]


def encode_frame(payload: bytes, message_type: int, flags: int,
                 stream_id: int, boot_id: int, frame_id: int,
                 capture_mono_ns: Optional[int] = None) -> list[bytes]:
    """Reference encoder used by tests and the included mock sender."""
    if not payload or len(payload) > MAX_FRAME_SIZE:
        raise ValueError("payload size out of range")
    capture_ns = capture_mono_ns if capture_mono_ns is not None else time.monotonic_ns()
    frame_crc = crc32(payload)
    count = (len(payload) + FRAGMENT_PAYLOAD_SIZE - 1) // FRAGMENT_PAYLOAD_SIZE
    output = []
    for index in range(count):
        chunk = payload[index * FRAGMENT_PAYLOAD_SIZE:(index + 1) * FRAGMENT_PAYLOAD_SIZE]
        prefix = _HEADER.pack(MAGIC, VERSION, HEADER_SIZE, message_type, flags,
                              stream_id, boot_id, frame_id, capture_ns,
                              len(payload), frame_crc, index, count, len(chunk),
                              0, 0)
        header = prefix[:52] + struct.pack(">I", crc32(prefix[:52]))
        output.append(header + chunk)
    return output


def encode_control(command: int, reply_port: int, stream_mask: int,
                   nonce: Optional[int] = None) -> bytes:
    if not 1 <= reply_port <= 65535:
        raise ValueError("reply port out of range")
    if not 1 <= command <= 255:
        raise ValueError("control command out of range")
    if not 0 <= stream_mask <= 0xFFFF:
        raise ValueError("stream mask out of range")
    nonce_value = nonce if nonce is not None else time.time_ns() & 0xFFFFFFFFFFFFFFFF
    prefix = _CONTROL.pack(CONTROL_MAGIC, VERSION, command, stream_mask,
                           reply_port, 0, nonce_value, 0)
    return prefix[:20] + struct.pack(">I", crc32(prefix[:20]))


def encode_hello(reply_port: int, stream_mask: int = 0xFFFF,
                 nonce: Optional[int] = None) -> bytes:
    return encode_control(CONTROL_HELLO, reply_port, stream_mask, nonce)


def decode_control(packet: bytes) -> tuple[int, int, int, int]:
    """Return (command, stream_mask, reply_port, nonce)."""
    if len(packet) != CONTROL_SIZE:
        raise ValueError("bad control size")
    (magic, version, command, stream_mask, reply_port, _reserved,
     nonce, packet_crc) = _CONTROL.unpack(packet)
    if (magic != CONTROL_MAGIC or version != VERSION or
            crc32(packet[:20]) != packet_crc):
        raise ValueError("bad control packet")
    return command, stream_mask, reply_port, nonce


def source_is_gateway(source_ip: str, gateway_ip: str) -> bool:
    """Explicit source allow-list used before allocating reassembly memory."""
    return source_ip == gateway_ip


class Reassembler:
    """Reassembles UDP frames with strict bounds and Pi boot-ID rollover."""

    def __init__(self, timeout_s: float = 0.5, max_assemblies: int = 128,
                 max_buffered_bytes: int = 64 * 1024 * 1024) -> None:
        self.timeout_s = timeout_s
        self.max_assemblies = max_assemblies
        self.max_buffered_bytes = max_buffered_bytes
        self.stats = ReassemblyStats()
        self.active_boot_id: Optional[int] = None
        self._retired_boot_ids: deque[int] = deque(maxlen=16)
        self._assemblies: "OrderedDict[tuple, _Assembly]" = OrderedDict()
        self._buffered_bytes = 0
        self._last_completed: Dict[tuple[int, int], int] = {}

    @property
    def buffered_assemblies(self) -> int:
        return len(self._assemblies)

    def expire(self, now: Optional[float] = None) -> None:
        current = time.monotonic() if now is None else now
        expired = [key for key, item in self._assemblies.items()
                   if current - item.created_at > self.timeout_s]
        for key in expired:
            self._drop(key)
            self.stats.expired_frames += 1

    def push(self, datagram: bytes, source: Tuple[str, int],
             now: Optional[float] = None) -> Optional[CompleteFrame]:
        current = time.monotonic() if now is None else now
        self.stats.datagrams += 1
        try:
            header, chunk = decode_datagram(datagram)
        except ValueError as error:
            self.stats.invalid_datagrams += 1
            if "header CRC" in str(error):
                self.stats.header_crc_errors += 1
            return None
        self.expire(current)
        if header.boot_id in self._retired_boot_ids:
            self.stats.stale_frames += 1
            return None
        if self.active_boot_id is None:
            self.active_boot_id = header.boot_id
        elif header.boot_id != self.active_boot_id:
            self._retired_boot_ids.append(self.active_boot_id)
            self.active_boot_id = header.boot_id
            self.stats.sender_restarts += 1
            self._clear_assemblies()
            self._last_completed.clear()

        key = (source[0], header.boot_id, header.message_type,
               header.stream_id, header.frame_id)
        assembly = self._assemblies.get(key)
        if assembly is None:
            assembly = _Assembly(header, source, current,
                                 [None] * header.fragment_count)
            self._assemblies[key] = assembly
        elif (assembly.header.total_size != header.total_size or
              assembly.header.frame_crc32 != header.frame_crc32 or
              assembly.header.fragment_count != header.fragment_count or
              assembly.header.flags != header.flags or
              assembly.header.capture_mono_ns != header.capture_mono_ns):
            self._drop(key)
            self.stats.invalid_datagrams += 1
            return None

        if assembly.fragments[header.fragment_index] is not None:
            self.stats.duplicate_fragments += 1
            return None
        assembly.fragments[header.fragment_index] = chunk
        assembly.byte_count += len(chunk)
        self._buffered_bytes += len(chunk)
        self._enforce_bounds()
        if key not in self._assemblies or assembly.byte_count != header.total_size:
            return None
        if any(part is None for part in assembly.fragments):
            return None
        payload = b"".join(part for part in assembly.fragments if part is not None)
        completed_header = assembly.header
        self._drop(key)
        if len(payload) != completed_header.total_size or crc32(payload) != completed_header.frame_crc32:
            self.stats.frame_crc_errors += 1
            return None
        if completed_header.flags & FLAG_REPLAY:
            # A cached HELLO response may race with the next live publication.
            # Count the valid replay even if the same frame ID was completed
            # moments earlier and is subsequently suppressed as stale.
            self.stats.replay_frames_seen += 1
        stream_key = (completed_header.message_type, completed_header.stream_id)
        previous = self._last_completed.get(stream_key)
        if previous is not None and completed_header.frame_id <= previous:
            self.stats.stale_frames += 1
            return None
        if previous is not None and completed_header.frame_id > previous + 1:
            self.stats.frame_gaps += completed_header.frame_id - previous - 1
        self._last_completed[stream_key] = completed_header.frame_id
        self.stats.completed_frames += 1
        return CompleteFrame(completed_header, payload, current, source)

    def _drop(self, key: tuple) -> None:
        item = self._assemblies.pop(key, None)
        if item is not None:
            self._buffered_bytes -= item.byte_count

    def _clear_assemblies(self) -> None:
        self._assemblies.clear()
        self._buffered_bytes = 0

    def _enforce_bounds(self) -> None:
        while (len(self._assemblies) > self.max_assemblies or
               self._buffered_bytes > self.max_buffered_bytes):
            key = next(iter(self._assemblies))
            self._drop(key)
            self.stats.evicted_frames += 1
