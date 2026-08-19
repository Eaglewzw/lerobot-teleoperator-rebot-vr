from __future__ import annotations

import json
import struct

import numpy as np
import pytest

from lerobot_teleoperator_rebot_vr.tracking import (
    LatestSampleBuffer,
    TrackingSampleError,
    parse_controller_sample,
)
from lerobot_teleoperator_rebot_vr.xr_v1 import (
    CMD_FUNCTION,
    PacketParser,
    PacketStreamDecoder,
    TrackingDecoder,
    V1TrackingSource,
)


def _tracking(
    *,
    timestamp_ns: int = 100,
    pose: object = "1,2,3,0,0,0,1",
    grip: object = 0.9,
    trigger: object = 0.25,
    primary_button: object = True,
    secondary_button: object = False,
) -> dict[str, object]:
    return {
        "timeStampNs": timestamp_ns,
        "Controller": {
            "right": {
                "pose": pose,
                "grip": grip,
                "trigger": trigger,
                "primaryButton": primary_button,
                "secondaryButton": secondary_button,
                "status": 1,
            }
        },
    }


def _packet(
    tracking: dict[str, object],
    *,
    command: int = CMD_FUNCTION,
    function_name: str = "Tracking",
    nested_string: bool = True,
) -> bytes:
    value: object = json.dumps(tracking) if nested_string else tracking
    body = json.dumps({"functionName": function_name, "value": value})
    return PacketParser.pack(command, body)


def test_stream_decoder_handles_fragmentation_sticky_packets_and_resync() -> None:
    first = _packet(_tracking(timestamp_ns=1))
    second = bytearray(_packet(_tracking(timestamp_ns=2)))
    second[0] = 0x3F
    decoder = PacketStreamDecoder()
    packets: list[dict[str, object]] = []
    for byte in first:
        packets.extend(decoder.feed(bytes((byte,))))
    packets.extend(decoder.feed(b"bad-prefix" + bytes(second)))
    decoded = [TrackingDecoder.decode_tracking(packet) for packet in packets]
    assert [item["timeStampNs"] for item in decoded if item is not None] == [1, 2]
    assert decoder.buffered_bytes == 0


def test_stream_decoder_recovers_from_bad_tail_and_body_lengths() -> None:
    valid = _packet(_tracking(timestamp_ns=3))
    bad_tail = bytearray(_packet(_tracking(timestamp_ns=2)))
    bad_tail[-1] = 0
    negative_length = struct.pack("<BBi", 0xCF, CMD_FUNCTION, -1) + b"ignored-data"
    oversized_length = (
        struct.pack("<BBi", 0xCF, CMD_FUNCTION, 1024 * 1024 + 1)
        + b"ignored-data"
    )
    decoder = PacketStreamDecoder()
    packets = decoder.feed(
        negative_length + oversized_length + bytes(bad_tail) + valid
    )
    assert len(packets) == 1
    assert TrackingDecoder.decode_tracking(packets[0])["timeStampNs"] == 3


def test_tracking_decoder_accepts_string_or_object_and_ignores_other_messages() -> None:
    for nested_string in (True, False):
        packet = PacketStreamDecoder().feed(
            _packet(_tracking(), nested_string=nested_string)
        )[0]
        assert TrackingDecoder.decode_tracking(packet) == _tracking()

    source = V1TrackingSource()
    source.feed_bytes(_packet(_tracking(), command=0x23), received_monotonic_ns=1)
    source.feed_bytes(
        _packet(_tracking(), function_name="NotTracking"),
        received_monotonic_ns=2,
    )
    assert source.latest_sample() is None


def test_controller_sample_xyzw_normalization_button_validation_and_readonly() -> None:
    sample = parse_controller_sample(
        _tracking(
            pose=[1, 2, 3, 0, 0, 0, 2],
            grip=1.5,
            trigger=-1,
            primary_button="true",
            secondary_button="1",
        ),
        "RIGHT",
        received_monotonic_ns=123,
        stream_epoch=4,
    )
    assert sample.side == "right"
    assert sample.tracking_timestamp_ns == 100
    assert sample.position == pytest.approx([1, 2, 3])
    assert sample.quaternion_xyzw == pytest.approx([0, 0, 0, 1])
    assert sample.grip == 1.0
    assert sample.trigger == 0.0
    assert sample.primary_button is True
    assert sample.secondary_button is True
    assert not sample.position.flags.writeable
    assert not sample.quaternion_xyzw.flags.writeable

    assert parse_controller_sample(
        _tracking(primary_button=0, secondary_button=0), "right"
    ).primary_button is False
    assert parse_controller_sample(
        _tracking(secondary_button="false"), "right"
    ).secondary_button is False
    with pytest.raises(TrackingSampleError, match="primaryButton"):
        parse_controller_sample(_tracking(primary_button="pressed"), "right")
    with pytest.raises(TrackingSampleError, match="secondaryButton"):
        parse_controller_sample(_tracking(secondary_button="pressed"), "right")


@pytest.mark.parametrize(
    "pose",
    (
        "1,2,3",
        "1,2,3,0,0,bad,1",
        "1,2,3,0,0,0,0",
        "1,2,3,nan,0,0,1",
    ),
)
def test_invalid_numeric_frames_are_rejected(pose: object) -> None:
    with pytest.raises(TrackingSampleError):
        parse_controller_sample(_tracking(pose=pose), "right")


def test_fractional_tracking_timestamp_is_rejected() -> None:
    with pytest.raises(TrackingSampleError, match="timeStampNs must be an integer"):
        parse_controller_sample(_tracking(timestamp_ns=1.5), "right")


def test_invalid_frame_does_not_replace_latest_and_latest_slot_never_queues() -> None:
    source = V1TrackingSource()
    source.feed_bytes(_packet(_tracking(timestamp_ns=1)), received_monotonic_ns=10)
    valid = source.latest_sample()
    source.feed_bytes(
        _packet(_tracking(timestamp_ns=2, pose="invalid")),
        received_monotonic_ns=20,
    )
    assert source.latest_sample() is valid
    assert source.stats().invalid_frame_count == 1

    slot = LatestSampleBuffer()
    first = parse_controller_sample(_tracking(timestamp_ns=3), "right")
    second = parse_controller_sample(_tracking(timestamp_ns=4), "right")
    slot.publish(first)
    slot.publish(second)
    latest, sequence = slot.latest()
    assert latest is second
    assert sequence == 2


def test_clock_rollback_and_reconnect_clear_sample_and_increment_epoch() -> None:
    source = V1TrackingSource()
    source.feed_bytes(_packet(_tracking(timestamp_ns=200)), received_monotonic_ns=10)
    assert source.latest_sample().stream_epoch == 0
    source.feed_bytes(_packet(_tracking(timestamp_ns=100)), received_monotonic_ns=20)
    assert source.latest_sample().stream_epoch == 1

    source._on_connect(("127.0.0.1", 12345))
    assert source.latest_sample() is None
    assert source.stats().stream_epoch == 2
    source.feed_bytes(_packet(_tracking(timestamp_ns=300)), received_monotonic_ns=30)
    assert source.latest_sample().stream_epoch == 2
    source._on_disconnect()
    assert source.latest_sample() is None
