"""Tests for the pilot page's H.264 forwarding helpers (no ROS needed)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'f1tenth_gym_ros'))
import h264_stream as H  # noqa: E402

SPS = b'\x00\x00\x00\x01\x67\x42\xe0\x1f'
PPS = b'\x00\x00\x00\x01\x68\xce\x3c\x80'
IDR = b'\x00\x00\x01\x65\x88\x84'
P = b'\x00\x00\x00\x01\x41\x9a\x02'


def frames(spec):
    """'KPPK' -> [(1, True, ...), (2, False, ...), ...]"""
    return [(i + 1, c == 'K', (SPS + PPS + IDR) if c == 'K' else P) for i, c in enumerate(spec)]


def test_keyframe_detection():
    assert H.is_keyframe(SPS + PPS + IDR)
    assert H.is_keyframe(IDR)
    assert not H.is_keyframe(P)
    assert not H.is_keyframe(b'')
    assert not H.is_keyframe(b'\x00\x00\x01')


def test_pack_header():
    b = H.pack(True, b'abc')
    assert b[:4] == b'\x00\x00\x00\x03' and b[4] == 1 and b[5:] == b'abc'
    assert H.pack(False, b'')[4] == 0


def test_new_client_starts_at_newest_keyframe():
    buf = frames('KPPKPP')
    assert [f[0] for f in H.select(buf, None, 6)] == [4, 5, 6]


def test_new_client_waits_without_keyframe():
    assert H.select(frames('PPP'), None, 6) == []


def test_in_step_client_gets_next_frames():
    buf = frames('KPPKPP')
    assert [f[0] for f in H.select(buf, 4, 6)] == [5, 6]
    assert H.select(buf, 6, 6) == []


def test_lagging_client_skips_to_newest_keyframe():
    buf = frames('KPPPPPPKPP')
    assert [f[0] for f in H.select(buf, 1, 3)] == [8, 9, 10]


def test_lagging_client_without_new_keyframe_waits():
    buf = frames('KPPPPPPPPP')
    assert H.select(buf, 1, 3) == []


def test_client_off_the_ring_restarts():
    buf = frames('KPPKPP')[3:]          # seq 4..6 left in the ring
    assert [f[0] for f in H.select(buf, 1, 6)] == [4, 5, 6]


def test_h265_keyframe_detection():
    vps = b'\x00\x00\x00\x01\x40\x01'      # NAL type 32 (VPS)
    idr = b'\x00\x00\x01\x26\x01'           # type 19 (IDR_W_RADL)
    trail = b'\x00\x00\x01\x02\x01'         # type 1 (TRAIL_R)
    assert H.is_keyframe(vps + idr, 'h265')
    assert H.is_keyframe(idr, 'h265')
    assert not H.is_keyframe(trail, 'h265')
