import math

import pytest

from pushpak_brain.telemetry_core import (PeerTable, SurvivorTracker, confidence_percent, millimetres,
                                          parse_heartbeat_key, quaternion_rpy_cdeg, timestamp_ms)


def test_parse_heartbeat_key():
    assert parse_heartbeat_key('pushpak/heartbeat/0') == 0
    assert parse_heartbeat_key('pushpak/heartbeat/12') == 12
    for bad in ('pushpak/heartbeat', 'pushpak/keyframe/1', 'pushpak/heartbeat/x', 'pushpak/heartbeat/-1',
                'other/heartbeat/1', 'pushpak/heartbeat/1/extra'):
        assert parse_heartbeat_key(bad) is None, bad


def test_timestamp_ms_wraps_uint32():
    assert timestamp_ms(12, 345_678_901) == 12345
    assert timestamp_ms(2 ** 32 // 1000 + 1, 0) < 2 ** 32


def test_millimetres():
    assert millimetres(1.234) == 1234
    assert millimetres(-2.2) == -2200
    with pytest.raises(ValueError):
        millimetres(float('nan'))
    with pytest.raises(ValueError):
        millimetres(3e6)


def test_quaternion_rpy_cdeg():
    assert quaternion_rpy_cdeg(0.0, 0.0, 0.0, 1.0) == (0, 0, 0)
    assert quaternion_rpy_cdeg(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)) == (0, 0, 9000)
    with pytest.raises(ValueError):
        quaternion_rpy_cdeg(0.0, 0.0, 0.0, 0.0)


def test_confidence_percent():
    assert confidence_percent(0.844) == 84
    for bad in (-0.1, 1.1, float('nan')):
        with pytest.raises(ValueError):
            confidence_percent(bad)


def test_survivor_tracker_dedups_within_radius():
    t = SurvivorTracker(1.5)
    assert t.observe(2.2, -1.6) == (1, 1)
    assert t.observe(2.9, -1.0) == (1, 2)
    assert t.observe(-1.5, 1.8) == (2, 1)
    assert t.observe(2.2, -1.6) == (1, 3)


def test_peer_table_excludes_self_and_expires():
    p = PeerTable(own_id=1, timeout_s=1.75)
    p.heard(1, 10.0)
    p.heard(0, 10.0)
    p.heard(3, 11.0)
    assert p.alive(11.5) == [0, 3]
    assert p.alive(11.8) == [3]
    assert p.alive(13.0) == []
