"""Safe framing for the D1 camera stream.

Version 1 frames have a fixed-size, network-byte-order header followed by a
UTF-8 JSON object and the JPEG bytes::

    0               4  5  6       8              12              16
    +----------------+--+--+-------+---------------+---------------+
    | magic "D1SP"   |ver|fl|hdrlen| metadata_len  | jpeg_len      |
    +----------------+--+--+-------+---------------+---------------+
    | metadata JSON ...                                  | JPEG ... |
    +----------------------------------------------------+----------+

The module deliberately does not deserialize executable object formats.  A
``FrameReader`` retains partial input when a socket timeout is raised, so the
same reader can be called again after a transient timeout without losing
framing state.
"""

from __future__ import annotations

import json
import math
import struct
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, Protocol


MAGIC = b"D1SP"
PROTOCOL_VERSION = 1
FLAGS_NONE = 0

_HEADER_STRUCT = struct.Struct("!4sBBHII")
HEADER_SIZE = _HEADER_STRUCT.size
MAX_METADATA_BYTES = 256 * 1024
MAX_JPEG_BYTES = 16 * 1024 * 1024
MAX_FRAME_BYTES = HEADER_SIZE + MAX_METADATA_BYTES + MAX_JPEG_BYTES
DEFAULT_RECV_CHUNK_BYTES = 64 * 1024
_MIN_JPEG_BYTES = 4
_UINT32_MAX = (1 << 32) - 1


class StreamProtocolError(Exception):
    """Base class for stream framing and payload errors."""


class InvalidHeaderError(StreamProtocolError):
    """The fixed frame header is not valid for this protocol."""


class UnsupportedVersionError(InvalidHeaderError):
    """The peer uses a stream protocol version this module cannot decode."""

    def __init__(self, version: int):
        self.version = version
        super().__init__(
            f"unsupported stream protocol version {version}; "
            f"expected {PROTOCOL_VERSION}"
        )


class FrameSizeError(StreamProtocolError):
    """A declared or supplied frame component exceeds its configured limit."""

    def __init__(self, component: str, actual: int, limit: int):
        self.component = component
        self.actual = actual
        self.limit = limit
        super().__init__(
            f"{component} is {actual} bytes, exceeding the {limit}-byte limit"
        )


class TruncatedFrameError(StreamProtocolError):
    """The connection closed before a complete header or frame arrived."""

    def __init__(self, component: str, expected: int, received: int):
        self.component = component
        self.expected = expected
        self.received = received
        super().__init__(
            f"truncated {component}: expected {expected} bytes, received {received}"
        )


class MetadataEncodeError(StreamProtocolError):
    """Metadata cannot be represented as a strict JSON object."""


class MetadataDecodeError(StreamProtocolError):
    """Metadata bytes are not a valid strict JSON object."""


class InvalidJpegError(StreamProtocolError):
    """The image payload is not a complete JPEG byte sequence."""


class StreamReadError(StreamProtocolError):
    """A recv-like object violated the socket read contract."""


class _Receivable(Protocol):
    def recv(self, size: int) -> bytes:
        """Return up to ``size`` bytes, or an empty byte string at EOF."""


class _Sendable(Protocol):
    def sendall(self, data: bytes) -> None:
        """Send all bytes or raise an exception."""


@dataclass(frozen=True)
class StreamLimits:
    """Resource limits applied before reading or allocating frame payloads."""

    max_metadata_bytes: int = MAX_METADATA_BYTES
    max_jpeg_bytes: int = MAX_JPEG_BYTES
    max_frame_bytes: int = MAX_FRAME_BYTES
    recv_chunk_bytes: int = DEFAULT_RECV_CHUNK_BYTES

    def __post_init__(self) -> None:
        values = {
            "max_metadata_bytes": self.max_metadata_bytes,
            "max_jpeg_bytes": self.max_jpeg_bytes,
            "max_frame_bytes": self.max_frame_bytes,
            "recv_chunk_bytes": self.recv_chunk_bytes,
        }
        for name, value in values.items():
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{name} must be an integer")
            if value <= 0:
                raise ValueError(f"{name} must be greater than zero")
        if self.max_metadata_bytes > _UINT32_MAX:
            raise ValueError("max_metadata_bytes cannot exceed the wire-format limit")
        if self.max_jpeg_bytes > _UINT32_MAX:
            raise ValueError("max_jpeg_bytes cannot exceed the wire-format limit")
        if self.max_frame_bytes < HEADER_SIZE:
            raise ValueError(f"max_frame_bytes must be at least {HEADER_SIZE}")


DEFAULT_LIMITS = StreamLimits()


@dataclass(frozen=True)
class StreamFrame:
    """One decoded camera frame."""

    metadata: dict[str, Any]
    jpeg: bytes
    version: int = PROTOCOL_VERSION


def _require_limits(limits: StreamLimits) -> None:
    if not isinstance(limits, StreamLimits):
        raise TypeError("limits must be a StreamLimits instance")


def _raise_json_constant(value: str) -> None:
    raise ValueError(f"non-finite JSON number {value!r} is not allowed")


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON object key {key!r}")
        result[key] = value
    return result


def _check_finite_json_numbers(value: Any) -> None:
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("non-finite JSON numbers are not allowed")
        return
    if isinstance(value, dict):
        for child in value.values():
            _check_finite_json_numbers(child)
        return
    if isinstance(value, list):
        for child in value:
            _check_finite_json_numbers(child)


def _encode_metadata(metadata: Mapping[str, Any], limits: StreamLimits) -> bytes:
    if not isinstance(metadata, Mapping):
        raise MetadataEncodeError("metadata must be a mapping encoded as a JSON object")
    if any(not isinstance(key, str) for key in metadata):
        raise MetadataEncodeError("metadata object keys must be strings")
    try:
        text = json.dumps(
            dict(metadata),
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        )
    except (TypeError, ValueError, RecursionError) as exc:
        raise MetadataEncodeError(f"metadata is not JSON serializable: {exc}") from exc
    encoded = text.encode("utf-8")
    if len(encoded) > limits.max_metadata_bytes:
        raise FrameSizeError(
            "metadata", len(encoded), limits.max_metadata_bytes
        )
    return encoded


def _decode_metadata(data: bytes) -> dict[str, Any]:
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise MetadataDecodeError("metadata is not valid UTF-8") from exc
    try:
        metadata = json.loads(
            text,
            object_pairs_hook=_unique_object,
            parse_constant=_raise_json_constant,
        )
        _check_finite_json_numbers(metadata)
    except (ValueError, RecursionError) as exc:
        raise MetadataDecodeError(f"metadata is not valid strict JSON: {exc}") from exc
    if not isinstance(metadata, dict):
        raise MetadataDecodeError("metadata JSON must contain an object at the top level")
    return metadata


def _jpeg_bytes(jpeg: Any, limits: StreamLimits) -> bytes:
    try:
        view = memoryview(jpeg)
    except TypeError as exc:
        raise InvalidJpegError("JPEG payload must support the buffer protocol") from exc
    if view.nbytes > limits.max_jpeg_bytes:
        raise FrameSizeError("JPEG", view.nbytes, limits.max_jpeg_bytes)
    try:
        data = view.tobytes()
    finally:
        view.release()
    if len(data) < _MIN_JPEG_BYTES:
        raise InvalidJpegError("JPEG payload is too short")
    if not data.startswith(b"\xff\xd8") or not data.endswith(b"\xff\xd9"):
        raise InvalidJpegError("JPEG payload is missing its SOI or EOI marker")
    return data


def _validate_declared_sizes(
    metadata_size: int, jpeg_size: int, limits: StreamLimits
) -> int:
    if metadata_size <= 0:
        raise InvalidHeaderError("metadata length must be greater than zero")
    if jpeg_size < _MIN_JPEG_BYTES:
        raise InvalidHeaderError(
            f"JPEG length must be at least {_MIN_JPEG_BYTES} bytes"
        )
    if metadata_size > limits.max_metadata_bytes:
        raise FrameSizeError("metadata", metadata_size, limits.max_metadata_bytes)
    if jpeg_size > limits.max_jpeg_bytes:
        raise FrameSizeError("JPEG", jpeg_size, limits.max_jpeg_bytes)
    total_size = HEADER_SIZE + metadata_size + jpeg_size
    if total_size > limits.max_frame_bytes:
        raise FrameSizeError("frame", total_size, limits.max_frame_bytes)
    return total_size


def _parse_header(header: bytes, limits: StreamLimits) -> tuple[int, int, int, int]:
    if len(header) != HEADER_SIZE:
        raise TruncatedFrameError("header", HEADER_SIZE, len(header))
    magic, version, flags, header_size, metadata_size, jpeg_size = (
        _HEADER_STRUCT.unpack(header)
    )
    if magic != MAGIC:
        raise InvalidHeaderError(f"invalid stream magic {magic!r}")
    if version != PROTOCOL_VERSION:
        raise UnsupportedVersionError(version)
    if flags != FLAGS_NONE:
        raise InvalidHeaderError(f"unsupported frame flags 0x{flags:02x}")
    if header_size != HEADER_SIZE:
        raise InvalidHeaderError(
            f"invalid header length {header_size}; expected {HEADER_SIZE}"
        )
    total_size = _validate_declared_sizes(metadata_size, jpeg_size, limits)
    return version, metadata_size, jpeg_size, total_size


def _decode_complete_packet(packet: bytes, limits: StreamLimits) -> StreamFrame:
    version, metadata_size, jpeg_size, total_size = _parse_header(
        packet[:HEADER_SIZE], limits
    )
    if len(packet) < total_size:
        raise TruncatedFrameError("frame", total_size, len(packet))
    if len(packet) > total_size:
        raise InvalidHeaderError(
            f"frame has {len(packet) - total_size} unexpected trailing bytes"
        )
    metadata_end = HEADER_SIZE + metadata_size
    metadata = _decode_metadata(packet[HEADER_SIZE:metadata_end])
    jpeg = _jpeg_bytes(packet[metadata_end : metadata_end + jpeg_size], limits)
    return StreamFrame(metadata=metadata, jpeg=jpeg, version=version)


def encode_frame(
    metadata: Mapping[str, Any],
    jpeg: Any,
    *,
    limits: StreamLimits = DEFAULT_LIMITS,
) -> bytes:
    """Encode metadata and JPEG bytes as one version 1 wire frame."""

    _require_limits(limits)
    metadata_bytes = _encode_metadata(metadata, limits)
    jpeg_bytes = _jpeg_bytes(jpeg, limits)
    total_size = _validate_declared_sizes(
        len(metadata_bytes), len(jpeg_bytes), limits
    )
    header = _HEADER_STRUCT.pack(
        MAGIC,
        PROTOCOL_VERSION,
        FLAGS_NONE,
        HEADER_SIZE,
        len(metadata_bytes),
        len(jpeg_bytes),
    )
    packet = header + metadata_bytes + jpeg_bytes
    if len(packet) != total_size:  # Defensive assertion without relying on -O.
        raise StreamProtocolError("encoded frame length does not match its header")
    return packet


def send_frame(
    connection: _Sendable,
    metadata: Mapping[str, Any],
    jpeg: Any,
    *,
    limits: StreamLimits = DEFAULT_LIMITS,
) -> int:
    """Encode and send one complete frame, returning its wire byte count."""

    packet = encode_frame(metadata, jpeg, limits=limits)
    connection.sendall(packet)
    return len(packet)


def decode_frame(
    packet: Any, *, limits: StreamLimits = DEFAULT_LIMITS
) -> StreamFrame:
    """Decode exactly one in-memory wire frame and reject trailing bytes."""

    _require_limits(limits)
    try:
        view = memoryview(packet)
    except TypeError as exc:
        raise StreamProtocolError("frame packet must support the buffer protocol") from exc
    if view.nbytes > limits.max_frame_bytes:
        view.release()
        raise FrameSizeError("frame", view.nbytes, limits.max_frame_bytes)
    try:
        data = view.tobytes()
    finally:
        view.release()
    if len(data) < HEADER_SIZE:
        raise TruncatedFrameError("header", HEADER_SIZE, len(data))
    return _decode_complete_packet(data, limits)


def read_exact(
    connection: _Receivable,
    size: int,
    *,
    allow_eof: bool = False,
    max_bytes: int = MAX_FRAME_BYTES,
    chunk_bytes: int = DEFAULT_RECV_CHUNK_BYTES,
) -> bytes | None:
    """Read exactly ``size`` bytes from a recv-like object.

    Socket timeouts propagate unchanged.  ``allow_eof`` returns ``None`` only
    when EOF occurs before any bytes are read; partial EOF is always an error.
    Use :class:`FrameReader` when a read must be safely retried after a timeout.
    """

    for name, value in {
        "size": size,
        "max_bytes": max_bytes,
        "chunk_bytes": chunk_bytes,
    }.items():
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{name} must be an integer")
    if size < 0:
        raise ValueError("size cannot be negative")
    if max_bytes <= 0 or chunk_bytes <= 0:
        raise ValueError("max_bytes and chunk_bytes must be greater than zero")
    if size > max_bytes:
        raise FrameSizeError("read", size, max_bytes)
    if size == 0:
        return b""

    result = bytearray()
    while len(result) < size:
        requested = min(chunk_bytes, size - len(result))
        try:
            chunk = connection.recv(requested)
        except InterruptedError:
            continue
        if not isinstance(chunk, bytes):
            raise StreamReadError("recv() must return bytes")
        if len(chunk) > requested:
            raise StreamReadError(
                f"recv({requested}) returned {len(chunk)} bytes"
            )
        if not chunk:
            if allow_eof and not result:
                return None
            raise TruncatedFrameError("stream data", size, len(result))
        result.extend(chunk)
    return bytes(result)


class FrameReader:
    """Stateful, timeout-resumable frame reader for one stream connection."""

    def __init__(
        self,
        connection: _Receivable,
        *,
        limits: StreamLimits = DEFAULT_LIMITS,
    ):
        _require_limits(limits)
        self._connection = connection
        self._limits = limits
        self._buffer = bytearray()
        self._peer_closed = False

    @property
    def buffered_bytes(self) -> int:
        """Number of partial-frame bytes retained by this reader."""

        return len(self._buffer)

    def _fill(
        self, required: int, *, allow_clean_eof: bool, component: str
    ) -> bool:
        while len(self._buffer) < required:
            requested = min(
                self._limits.recv_chunk_bytes, required - len(self._buffer)
            )
            try:
                chunk = self._connection.recv(requested)
            except InterruptedError:
                continue
            if not isinstance(chunk, bytes):
                raise StreamReadError("recv() must return bytes")
            if len(chunk) > requested:
                raise StreamReadError(
                    f"recv({requested}) returned {len(chunk)} bytes"
                )
            if not chunk:
                self._peer_closed = True
                if allow_clean_eof and not self._buffer:
                    return False
                raise TruncatedFrameError(component, required, len(self._buffer))
            self._buffer.extend(chunk)
        return True

    def read_frame(self) -> StreamFrame | None:
        """Read one frame, or return ``None`` for clean EOF between frames.

        If the socket raises ``TimeoutError`` (including ``socket.timeout``),
        partial bytes remain buffered.  Call this method again on the same
        instance to resume the frame.
        """

        if self._peer_closed:
            return None
        if not self._fill(
            HEADER_SIZE, allow_clean_eof=True, component="header"
        ):
            return None
        _, _, _, total_size = _parse_header(
            bytes(self._buffer[:HEADER_SIZE]), self._limits
        )
        self._fill(total_size, allow_clean_eof=False, component="frame")
        packet = bytes(self._buffer[:total_size])
        frame = _decode_complete_packet(packet, self._limits)
        del self._buffer[:total_size]
        return frame


def read_frame(
    connection: _Receivable,
    *,
    limits: StreamLimits = DEFAULT_LIMITS,
) -> StreamFrame | None:
    """Read one frame from a connection.

    Retain a :class:`FrameReader` instead when callers intend to catch a socket
    timeout and retry the same partial frame.
    """

    return FrameReader(connection, limits=limits).read_frame()


__all__ = [
    "DEFAULT_LIMITS",
    "DEFAULT_RECV_CHUNK_BYTES",
    "FLAGS_NONE",
    "FrameReader",
    "FrameSizeError",
    "HEADER_SIZE",
    "InvalidHeaderError",
    "InvalidJpegError",
    "MAGIC",
    "MAX_FRAME_BYTES",
    "MAX_JPEG_BYTES",
    "MAX_METADATA_BYTES",
    "MetadataDecodeError",
    "MetadataEncodeError",
    "PROTOCOL_VERSION",
    "StreamFrame",
    "StreamLimits",
    "StreamProtocolError",
    "StreamReadError",
    "TruncatedFrameError",
    "UnsupportedVersionError",
    "decode_frame",
    "encode_frame",
    "read_exact",
    "read_frame",
    "send_frame",
]
