import socket
import struct

import pytest

import d1_stream_protocol as protocol


JPEG = b"\xff\xd8test-jpeg-payload\xff\xd9"
METADATA = {
    "protocol": "camera-v1",
    "sequence": 42,
    "tag_found": True,
    "position_m": [0.1, -0.2, 0.3],
}


class ScriptedReceiver:
    """Small recv-compatible source supporting fragmentation and errors."""

    def __init__(self, *events):
        self.events = list(events)
        self.recv_calls = 0

    def recv(self, size):
        self.recv_calls += 1
        if not self.events:
            return b""
        event = self.events.pop(0)
        if isinstance(event, BaseException):
            raise event
        event = bytes(event)
        if len(event) > size:
            self.events.insert(0, event[size:])
            return event[:size]
        return event


class RecordingSender:
    def __init__(self):
        self.data = None

    def sendall(self, data):
        self.data = data


def raw_frame(
    metadata,
    jpeg=JPEG,
    *,
    magic=protocol.MAGIC,
    version=protocol.PROTOCOL_VERSION,
    flags=protocol.FLAGS_NONE,
    header_size=protocol.HEADER_SIZE,
    metadata_size=None,
    jpeg_size=None,
):
    if metadata_size is None:
        metadata_size = len(metadata)
    if jpeg_size is None:
        jpeg_size = len(jpeg)
    header = struct.pack(
        "!4sBBHII",
        magic,
        version,
        flags,
        header_size,
        metadata_size,
        jpeg_size,
    )
    return header + metadata + jpeg


def test_round_trip_uses_versioned_network_order_header():
    packet = protocol.encode_frame(METADATA, memoryview(JPEG))

    magic, version, flags, header_size, metadata_size, jpeg_size = struct.unpack(
        "!4sBBHII", packet[: protocol.HEADER_SIZE]
    )
    assert magic == b"D1SP"
    assert version == 1
    assert flags == 0
    assert header_size == 16
    assert metadata_size > 0
    assert jpeg_size == len(JPEG)

    frame = protocol.decode_frame(packet)
    assert frame.version == protocol.PROTOCOL_VERSION
    assert frame.metadata == METADATA
    assert frame.jpeg == JPEG


def test_send_frame_sends_one_complete_decodable_packet():
    sender = RecordingSender()

    sent = protocol.send_frame(sender, METADATA, bytearray(JPEG))

    assert sent == len(sender.data)
    assert protocol.decode_frame(sender.data).metadata == METADATA


def test_reader_handles_fragmentation_and_multiple_frames():
    first = protocol.encode_frame({"sequence": 1}, JPEG)
    second = protocol.encode_frame({"sequence": 2}, JPEG)
    receiver = ScriptedReceiver(
        (first + second)[:3],
        (first + second)[3:19],
        (first + second)[19:],
    )
    reader = protocol.FrameReader(receiver)

    assert reader.read_frame().metadata == {"sequence": 1}
    assert reader.read_frame().metadata == {"sequence": 2}
    assert reader.read_frame() is None


def test_reader_preserves_partial_frame_across_socket_timeout():
    packet = protocol.encode_frame(METADATA, JPEG)
    receiver = ScriptedReceiver(
        packet[:7], socket.timeout("temporary timeout"), packet[7:]
    )
    reader = protocol.FrameReader(receiver)

    with pytest.raises(socket.timeout, match="temporary timeout"):
        reader.read_frame()
    assert reader.buffered_bytes == 7

    assert reader.read_frame().metadata == METADATA
    assert reader.buffered_bytes == 0


def test_read_exact_handles_fragments_and_clean_eof():
    assert protocol.read_exact(ScriptedReceiver(b"ab", b"c", b"def"), 6) == b"abcdef"
    assert protocol.read_exact(ScriptedReceiver(), 4, allow_eof=True) is None


def test_read_exact_rejects_partial_eof():
    with pytest.raises(protocol.TruncatedFrameError) as error:
        protocol.read_exact(ScriptedReceiver(b"abc"), 4, allow_eof=True)
    assert error.value.expected == 4
    assert error.value.received == 3


def test_clean_eof_between_frames_returns_none():
    assert protocol.read_frame(ScriptedReceiver()) is None


def test_truncated_header_is_rejected():
    packet = protocol.encode_frame(METADATA, JPEG)
    with pytest.raises(protocol.TruncatedFrameError) as error:
        protocol.read_frame(ScriptedReceiver(packet[: protocol.HEADER_SIZE - 1]))
    assert error.value.expected == protocol.HEADER_SIZE


def test_truncated_payload_is_rejected():
    packet = protocol.encode_frame(METADATA, JPEG)
    with pytest.raises(protocol.TruncatedFrameError) as error:
        protocol.read_frame(ScriptedReceiver(packet[:-3]))
    assert error.value.expected == len(packet)
    assert error.value.received == len(packet) - 3


@pytest.mark.parametrize(
    ("overrides", "error_type"),
    [
        ({"magic": b"NOPE"}, protocol.InvalidHeaderError),
        ({"version": 2}, protocol.UnsupportedVersionError),
        ({"flags": 1}, protocol.InvalidHeaderError),
        ({"header_size": protocol.HEADER_SIZE + 1}, protocol.InvalidHeaderError),
    ],
)
def test_malformed_header_is_rejected(overrides, error_type):
    packet = raw_frame(b"{}", **overrides)
    with pytest.raises(error_type):
        protocol.decode_frame(packet)


@pytest.mark.parametrize(
    "metadata",
    [
        b"\xff",
        b"{",
        b"[]",
        b'{"duplicate":1,"duplicate":2}',
        b'{"overflow":1e999}',
        b'{"constant":NaN}',
    ],
)
def test_malformed_metadata_is_rejected(metadata):
    with pytest.raises(protocol.MetadataDecodeError):
        protocol.decode_frame(raw_frame(metadata))


@pytest.mark.parametrize(
    "metadata",
    [
        {"not_json": object()},
        {"not_finite": float("nan")},
        {1: "non-string key"},
    ],
)
def test_unencodable_metadata_is_rejected(metadata):
    with pytest.raises(protocol.MetadataEncodeError):
        protocol.encode_frame(metadata, JPEG)


@pytest.mark.parametrize(
    "jpeg",
    [
        b"not-a-jpeg",
        b"\xff\xd8missing-end-marker",
        b"missing-start-marker\xff\xd9",
    ],
)
def test_invalid_jpeg_is_rejected(jpeg):
    with pytest.raises(protocol.InvalidJpegError):
        protocol.decode_frame(raw_frame(b"{}", jpeg))


def test_too_short_declared_jpeg_is_rejected_as_a_bad_header():
    with pytest.raises(protocol.InvalidHeaderError, match="JPEG length"):
        protocol.decode_frame(raw_frame(b"{}", b"abc"))


def test_encode_rejects_empty_jpeg():
    with pytest.raises(protocol.InvalidJpegError):
        protocol.encode_frame({}, b"")


def test_encode_rejects_oversized_metadata():
    limits = protocol.StreamLimits(
        max_metadata_bytes=8,
        max_jpeg_bytes=len(JPEG),
        max_frame_bytes=protocol.HEADER_SIZE + 8 + len(JPEG),
    )
    with pytest.raises(protocol.FrameSizeError) as error:
        protocol.encode_frame({"long": "metadata"}, JPEG, limits=limits)
    assert error.value.component == "metadata"


def test_encode_rejects_oversized_jpeg_before_building_packet():
    limits = protocol.StreamLimits(
        max_metadata_bytes=32,
        max_jpeg_bytes=len(JPEG) - 1,
        max_frame_bytes=protocol.HEADER_SIZE + 32 + len(JPEG),
    )
    with pytest.raises(protocol.FrameSizeError) as error:
        protocol.encode_frame({}, JPEG, limits=limits)
    assert error.value.component == "JPEG"


@pytest.mark.parametrize(
    ("metadata_size", "jpeg_size", "component"),
    [
        (9, 4, "metadata"),
        (2, 9, "JPEG"),
    ],
)
def test_announced_component_oversize_is_rejected_before_payload_read(
    metadata_size, jpeg_size, component
):
    limits = protocol.StreamLimits(
        max_metadata_bytes=8,
        max_jpeg_bytes=8,
        max_frame_bytes=protocol.HEADER_SIZE + 16,
    )
    header_only = raw_frame(
        b"",
        b"",
        metadata_size=metadata_size,
        jpeg_size=jpeg_size,
    )
    receiver = ScriptedReceiver(header_only)

    with pytest.raises(protocol.FrameSizeError) as error:
        protocol.FrameReader(receiver, limits=limits).read_frame()
    assert error.value.component == component
    assert receiver.recv_calls == 1


def test_announced_total_oversize_is_rejected_before_payload_read():
    limits = protocol.StreamLimits(
        max_metadata_bytes=8,
        max_jpeg_bytes=8,
        max_frame_bytes=protocol.HEADER_SIZE + 5,
    )
    header_only = raw_frame(b"", b"", metadata_size=2, jpeg_size=4)
    receiver = ScriptedReceiver(header_only)

    with pytest.raises(protocol.FrameSizeError) as error:
        protocol.FrameReader(receiver, limits=limits).read_frame()
    assert error.value.component == "frame"
    assert receiver.recv_calls == 1


def test_decode_rejects_trailing_bytes():
    packet = protocol.encode_frame(METADATA, JPEG)
    with pytest.raises(protocol.InvalidHeaderError, match="trailing"):
        protocol.decode_frame(packet + b"extra")


def test_read_exact_applies_a_size_cap_before_recv():
    receiver = ScriptedReceiver(b"data")
    with pytest.raises(protocol.FrameSizeError):
        protocol.read_exact(receiver, 5, max_bytes=4)
    assert receiver.recv_calls == 0
