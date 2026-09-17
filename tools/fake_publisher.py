#!/usr/bin/env python3
"""Writes every PUSHPAK stream with plausible fake data.

No simulator, no ROS. Build the frontend against this.
    python3 tools/fake_publisher.py
    python3 tools/fake_publisher.py --drone-id drone_01 --host localhost
"""
import argparse
import json
import math
import random
import time

import redis

VICTIM_SITES = [(3.4, -1.1), (-2.2, 4.8), (5.9, 3.0)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--drone-id', default='drone_00')
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--port', type=int, default=6379)
    args = ap.parse_args()

    r = redis.Redis(host=args.host, port=args.port, decode_responses=True)
    r.ping()
    did = args.drone_id
    print(f'publishing fake telemetry for {did} -> {args.host}:{args.port}')
    print('ctrl-c to stop')

    t0 = time.time()
    tick = 0
    victims = {}
    cached = 0
    last_sync = 0.0
    link_state = 'ONLINE'

    while True:
        tick += 1
        t = round(time.time() - t0, 3)

        # lawnmower search pattern
        x = round(4.0 * math.sin(t * 0.12), 4)
        y = round(0.9 * t % 8.0 - 4.0, 4)
        z = round(2.0 + 0.03 * math.sin(t * 1.7), 4)
        yaw = round((t * 6.0) % 360.0 - 180.0, 4)

        r.xadd(f'telemetry:{did}:pose', {
            'timestamp': t, 'drone_id': did,
            'x': x, 'y': y, 'z': z, 'yaw_deg': yaw,
        }, maxlen=100, approximate=True)

        r.xadd(f'telemetry:{did}:altitude', {
            'timestamp': t, 'drone_id': did,
            'z': round(z + random.gauss(0, 0.01), 4),
        }, maxlen=100, approximate=True)

        feats = random.randint(40, 60)
        r.xadd(f'telemetry:{did}:velocity', {
            'timestamp': t, 'drone_id': did,
            'linear_x': round(random.gauss(0, 0.05), 4),
            'linear_y': round(random.gauss(0, 0.05), 4),
            'linear_z': 0.0, 'angular_z': 0.0,
            'is_valid': 'True' if feats > 20 else 'False',
            'features': feats,
        }, maxlen=100, approximate=True)

        r.xadd(f'telemetry:{did}:flow_debug', {
            'timestamp': t, 'drone_id': did,
            'u_raw': round(random.gauss(0, 0.1), 4),
            'v_raw': round(random.gauss(0, 0.1), 4),
            'u_med': round(random.gauss(0, 0.1), 4),
            'v_med': round(random.gauss(0, 0.1), 4),
            'u_std': round(abs(random.gauss(0.2, 0.05)), 4),
            'v_std': round(abs(random.gauss(0.2, 0.05)), 4),
            'frame_diff': round(abs(random.gauss(0.06, 0.02)), 4),
            'gyro_x': round(random.gauss(0, 0.002), 4),
            'gyro_y': round(random.gauss(0, 0.002), 4),
            'dt': 0.066, 'altitude': z, 'features': feats,
        }, maxlen=100, approximate=True)

        # link: 20 s online, 10 s offline, 5 s degraded, repeat
        phase = t % 35.0
        if phase < 20.0:
            new_state = 'ONLINE'
        elif phase < 30.0:
            new_state = 'OFFLINE'
        else:
            new_state = 'DEGRADED'
        if new_state == 'OFFLINE':
            cached += 1
        elif link_state == 'OFFLINE':
            cached = 0
            last_sync = t
        link_state = new_state

        r.xadd(f'link:{did}:status', {
            'timestamp': t, 'drone_id': did,
            'state': link_state,
            'cached_packets': cached,
            'last_sync_sec': round(last_sync, 3),
            'rssi_dbm': round(-55 - 25 * (link_state != 'ONLINE') + random.gauss(0, 3), 1),
        }, maxlen=100, approximate=True)

        # a detection every ~4 s, snapping to one of three sites
        if tick % 40 == 0:
            site = VICTIM_SITES[(tick // 40) % len(VICTIM_SITES)]
            det_id = f'det_{tick:05d}'
            conf = round(random.uniform(0.45, 0.95), 4)
            r.xadd(f'detections:{did}', {
                'timestamp': t, 'drone_id': did,
                'det_id': det_id, 'class_name': 'person', 'confidence': conf,
                'bbox_x': random.randint(40, 400), 'bbox_y': random.randint(40, 300),
                'bbox_w': random.randint(30, 90), 'bbox_h': random.randint(60, 150),
                'drone_x': x, 'drone_y': y, 'drone_z': z,
                'image_path': f'/workspace/detections/{det_id}.jpg',
            }, maxlen=100, approximate=True)

            vid = f'v_{VICTIM_SITES.index(site) + 1:03d}'
            v = victims.get(vid)
            if v is None:
                v = {'victim_id': vid, 'confidence': conf,
                     'world_x': site[0], 'world_y': site[1],
                     'first_seen': t, 'last_seen': t, 'hit_count': 1,
                     'image_path': f'/workspace/detections/{det_id}.jpg',
                     'acked': False}
            else:
                v['confidence'] = round(max(v['confidence'], conf), 4)
                v['last_seen'] = t
                v['hit_count'] += 1
            victims[vid] = v
            r.hset(f'victims:{did}', vid, json.dumps(v))

        time.sleep(0.1)


if __name__ == '__main__':
    main()