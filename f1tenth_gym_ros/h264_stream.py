"""H.264/H.265 helpers for the pilot page's low-latency video path (no ROS imports, tested).

The OAK-D encodes H.264 on its own chip (oakd_camera, h264_kbps > 0) and publishes one
access unit per message on /oakd/video. web_pilot forwards those bytes to the browser,
which decodes them with WebCodecs. Wire format of GET /vstream, repeated per frame:

    u32 big-endian payload length | u8 flags (bit 0 = keyframe) | payload (Annex-B access unit)

Latency comes first: a client that falls behind skips ahead to the newest keyframe instead
of receiving a backlog, because a frame that arrives late is no use to someone driving.
"""
import struct

HEADER = struct.Struct('>IB')


def is_keyframe(data, codec='h264'):
    """True if an Annex-B access unit is a point where a decoder can start: for H.264 an IDR
    slice (NAL type 5) or SPS (7); for H.265 an IDR/CRA picture (19-21) or VPS/SPS (32, 33)."""
    i, n = 0, len(data)
    while True:
        i = data.find(b'\x00\x00\x01', i)
        if i < 0 or i + 3 >= n:
            return False
        b = data[i + 3]
        if codec == 'h265':
            if (b >> 1) & 0x3F in (19, 20, 21, 32, 33):
                return True
        elif b & 0x1F in (5, 7):
            return True
        i += 3


def pack(key, data):
    """One frame on the wire."""
    return HEADER.pack(len(data), 1 if key else 0) + bytes(data)


def select(buf, last, max_lag):
    """Frames to send next to a client whose last sent sequence number is `last`.

    buf: frames as (seq, key, data), oldest first. A new client (last is None), one that
    fell off the end of the ring, or one more than `max_lag` frames behind restarts at the
    newest keyframe it has not been sent yet; until one exists it gets nothing, because
    delta frames without their keyframe cannot be decoded.
    """
    newer = [f for f in buf if last is None or f[0] > last]
    if not newer:
        return []
    behind = last is None or buf[0][0] > last + 1 or len(newer) > max_lag
    if not behind:
        return newer
    keys = [i for i, f in enumerate(newer) if f[1]]
    return newer[keys[-1]:] if keys else []
