import json
from types import SimpleNamespace as NS

import zenoh_gateway as gw


def kf(**k):
    base = dict(timestamp_ms=12345, drone_id=1, pos_x_mm=1500, pos_y_mm=-250, pos_z_mm=1600,
                roll_cdeg=0, pitch_cdeg=0, yaw_cdeg=9000, status_flags=0b10010)
    base.update(k)
    return NS(**base)


def sv(**k):
    base = dict(timestamp_ms=20000, drone_id=1, survivor_id=1, pos_x_mm=2200, pos_y_mm=-1600,
                pos_z_mm=0, confidence_pct=84, hit_count=1)
    base.update(k)
    return NS(**base)


DECODERS = {'keyframe': lambda b: kf(), 'survivor': lambda b: sv(), 'heartbeat': lambda b: NS(drone_id=1)}


def test_parse_key():
    assert gw.parse_key('pushpak/keyframe/1') == ('keyframe', 1)
    for bad in ('pushpak/keyframe', 'pushpak/keyframe/1/x', 'other/keyframe/1', 'pushpak/nope/1',
                'pushpak/keyframe/x', 'pushpak/keyframe/-1'):
        assert gw.parse_key(bad) is None, bad


def test_validate_rejections():
    assert gw.validate('pushpak/keyframe/1', b'x' * 51, DECODERS) == (None, 'oversize')
    assert gw.validate('pushpak/bad/1', b'x', DECODERS) == (None, 'bad_key')
    assert gw.validate('pushpak/keyframe/2', b'x', DECODERS) == (None, 'id_mismatch')

    def boom(b):
        raise ValueError
    assert gw.validate('pushpak/heartbeat/1', b'x', dict(DECODERS, heartbeat=boom)) == (None, 'decode')
    hi = dict(DECODERS, survivor=lambda b: sv(confidence_pct=101))
    assert gw.validate('pushpak/survivor/1', b'x', hi) == (None, 'confidence')
    ok, reason = gw.validate('pushpak/keyframe/1', b'x' * 50, DECODERS)
    assert reason is None and ok[:2] == ('keyframe', 1)


def test_keyframe_translation_matches_section_9():
    out = dict(gw.keyframe_messages(kf()))
    assert out['pose'] == {'timestamp': 12.345, 'drone_id': 'drone_01', 'x': 1.5, 'y': -0.25, 'z': 1.6,
                           'yaw_deg': 90.0}
    assert out['altitude'] == {'timestamp': 12.345, 'drone_id': 'drone_01', 'z': 1.6}
    assert out['ekf_health'] == {'timestamp': 12.345, 'drone_id': 'drone_01', 'vio_active': False,
                                 'radar_active': True, 'backtracking': False, 'isolated': True}


def test_victim_table_ids_and_updates():
    t = gw.VictimTable()
    snap = t.update(sv())
    assert snap == [{'victim_id': 'v_001', 'first_seen': 20.0, 'image_path': '', 'acked': False,
                     'confidence': 0.84, 'world_x': 2.2, 'world_y': -1.6, 'last_seen': 20.0, 'hit_count': 1}]
    snap = t.update(sv(timestamp_ms=25000, hit_count=3, confidence_pct=90))
    assert len(snap) == 1 and snap[0]['first_seen'] == 20.0 and snap[0]['last_seen'] == 25.0
    assert snap[0]['hit_count'] == 3 and snap[0]['confidence'] == 0.9
    snap = t.update(sv(survivor_id=2))
    assert [v['victim_id'] for v in snap] == ['v_001', 'v_002']
    snap = t.update(sv(drone_id=2, survivor_id=1))
    assert [v['victim_id'] for v in snap] == ['v_001', 'v_002', 'v_003']


def test_link_status():
    s = gw.link_status(1, 100.0, 55.5, 101.5)
    assert s == {'timestamp': 55.5, 'drone_id': 'drone_01', 'state': 'ONLINE', 'cached_packets': 0,
                 'last_sync_sec': 1.5, 'rssi_dbm': None}
    assert gw.link_status(1, 100.0, 55.5, 102.5)['state'] == 'OFFLINE'
    never = gw.link_status(1, None, None, 5.0)
    assert never['state'] == 'OFFLINE' and never['last_sync_sec'] is None


def test_envelope_is_json_with_null_rssi():
    e = json.loads(gw.envelope('status', gw.link_status(1, None, None, 0.0)))
    assert e['type'] == 'status' and e['data']['rssi_dbm'] is None
