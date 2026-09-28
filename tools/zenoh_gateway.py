#!/usr/bin/env python3
import argparse
import asyncio
import json
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAX_PAYLOAD_B = 50
KINDS = ('keyframe', 'survivor', 'heartbeat')
OWN_ID = 0


def drone_name(drone_id):
    return f'drone_{drone_id:02d}'


def parse_key(key):
    parts = key.split('/')
    if len(parts) != 3 or parts[0] != 'pushpak' or parts[1] not in KINDS:
        return None
    try:
        drone_id = int(parts[2])
    except ValueError:
        return None
    if drone_id < 0:
        return None
    return parts[1], drone_id


def validate(key, payload, decoders):
    if len(payload) > MAX_PAYLOAD_B:
        return None, 'oversize'
    parsed = parse_key(key)
    if parsed is None:
        return None, 'bad_key'
    kind, key_id = parsed
    try:
        msg = decoders[kind](payload)
    except Exception:
        return None, 'decode'
    if msg.drone_id != key_id:
        return None, 'id_mismatch'
    if kind == 'survivor' and msg.confidence_pct > 100:
        return None, 'confidence'
    return (kind, key_id, msg), None


def health(msg, drone_id, timestamp_s):
    f = msg.status_flags
    return {'timestamp': timestamp_s, 'drone_id': drone_name(drone_id),
            'vio_active': bool(f & 1), 'radar_active': bool(f & 2),
            'backtracking': bool(f & 8), 'isolated': bool(f & 16)}


def keyframe_messages(msg):
    t = msg.timestamp_ms / 1000.0
    name = drone_name(msg.drone_id)
    x, y, z = msg.pos_x_mm / 1000.0, msg.pos_y_mm / 1000.0, msg.pos_z_mm / 1000.0
    return [
        ('pose', {'timestamp': t, 'drone_id': name, 'x': x, 'y': y, 'z': z, 'yaw_deg': msg.yaw_cdeg / 100.0}),
        ('altitude', {'timestamp': t, 'drone_id': name, 'z': z}),
        ('ekf_health', health(msg, msg.drone_id, t)),
    ]


class VictimTable:
    def __init__(self):
        self.by_key = {}

    def update(self, msg):
        key = (msg.drone_id, msg.survivor_id)
        t = msg.timestamp_ms / 1000.0
        v = self.by_key.get(key)
        if v is None:
            v = {'victim_id': f'v_{len(self.by_key) + 1:03d}', 'first_seen': t,
                 'image_path': '', 'acked': False}
            self.by_key[key] = v
        v.update({'confidence': msg.confidence_pct / 100.0, 'world_x': msg.pos_x_mm / 1000.0,
                  'world_y': msg.pos_y_mm / 1000.0, 'last_seen': t, 'hit_count': msg.hit_count})
        return self.snapshot()

    def snapshot(self):
        return sorted(self.by_key.values(), key=lambda v: v['victim_id'])


def link_status(drone_id, last_heard_mono, last_ts_s, now_mono, online_window_s=2.0):
    age = None if last_heard_mono is None else now_mono - last_heard_mono
    return {'timestamp': last_ts_s, 'drone_id': drone_name(drone_id),
            'state': 'ONLINE' if age is not None and age < online_window_s else 'OFFLINE',
            'cached_packets': 0, 'last_sync_sec': age, 'rssi_dbm': None}


def envelope(msg_type, data):
    return json.dumps({'type': msg_type, 'data': data}, separators=(',', ':'))


def load_pb():
    out = tempfile.mkdtemp(prefix='pushpak_pb_')
    subprocess.run(['protoc', f'-I{ROOT / "proto"}', f'--python_out={out}', 'pushpak.proto'], check=True)
    sys.path.insert(0, out)
    import pushpak_pb2
    return pushpak_pb2


class Gateway:
    def __init__(self, args, pb):
        self.args = args
        self.pb = pb
        self.decoders = {'keyframe': pb.SubMapKeyframe.FromString, 'survivor': pb.SurvivorEvent.FromString,
                         'heartbeat': pb.Heartbeat.FromString}
        self.clients = set()
        self.victims = VictimTable()
        self.last_heard = {}
        self.last_ts = {}
        self.counts = {'accepted': 0, 'oversize': 0, 'bad_key': 0, 'decode': 0, 'id_mismatch': 0,
                       'confidence': 0, 'other_drone_pose': 0}
        self.loop = None

    def on_sample(self, sample):
        self.loop.call_soon_threadsafe(self.handle, str(sample.key_expr), bytes(sample.payload))

    def broadcast(self, msg_type, data):
        import websockets
        websockets.broadcast(self.clients, envelope(msg_type, data))

    def handle(self, key, payload):
        result, reason = validate(key, payload, self.decoders)
        if reason:
            self.counts[reason] += 1
            return
        kind, drone_id, msg = result
        if drone_id == OWN_ID:
            return
        self.counts['accepted'] += 1
        if kind == 'heartbeat':
            self.last_heard[drone_id] = time.monotonic()
            self.last_ts[drone_id] = msg.timestamp_ms / 1000.0
        elif kind == 'keyframe':
            if drone_id != self.args.pose_drone_id:
                self.counts['other_drone_pose'] += 1
                return
            for msg_type, data in keyframe_messages(msg):
                self.broadcast(msg_type, data)
        else:
            self.broadcast('victims', self.victims.update(msg))

    async def ws_handler(self, ws):
        self.clients.add(ws)
        print(f'dashboard connected ({len(self.clients)} clients)', flush=True)
        try:
            await ws.send(envelope('victims', self.victims.snapshot()))
            await ws.wait_closed()
        finally:
            self.clients.discard(ws)

    async def status_loop(self, session):
        next_report = time.monotonic() + 10.0
        while True:
            await asyncio.sleep(0.5)
            hb = self.pb.Heartbeat(timestamp_ms=int(time.time() * 1000) & 0xFFFFFFFF, drone_id=OWN_ID,
                                   status_flags=0)
            session.put(f'pushpak/heartbeat/{OWN_ID}', hb.SerializeToString())
            now = time.monotonic()
            d = self.args.pose_drone_id
            self.broadcast('status', link_status(d, self.last_heard.get(d), self.last_ts.get(d), now))
            if now >= next_report:
                alive = sorted(i for i, t in self.last_heard.items() if now - t < 2.0)
                print(f'counts {self.counts} peers_alive {alive} clients {len(self.clients)}', flush=True)
                next_report = now + 10.0

    async def run(self):
        import websockets
        import zenoh
        self.loop = asyncio.get_running_loop()
        conf = zenoh.Config()
        conf.insert_json5('mode', '"peer"')
        if self.args.listen:
            conf.insert_json5('listen/endpoints', json.dumps(self.args.listen))
        if self.args.connect:
            conf.insert_json5('connect/endpoints', json.dumps(self.args.connect))
        session = zenoh.open(conf)
        self.subscriber = session.declare_subscriber('pushpak/**', self.on_sample)
        print(f'zenoh_gateway: peer {OWN_ID} (no zenohd), ws://0.0.0.0:{self.args.ws_port}, '
              f'pose from {drone_name(self.args.pose_drone_id)}', flush=True)
        try:
            async with websockets.serve(self.ws_handler, '0.0.0.0', self.args.ws_port):
                await self.status_loop(session)
        finally:
            session.close()


def main():
    p = argparse.ArgumentParser(description='Pushpak Zenoh peer -> dashboard WebSocket (README section 9)')
    p.add_argument('--ws-port', type=int, default=8765)
    p.add_argument('--pose-drone-id', type=int, required=True)
    p.add_argument('--listen', action='append', default=[])
    p.add_argument('--connect', action='append', default=[])
    args = p.parse_args()
    try:
        asyncio.run(Gateway(args, load_pb()).run())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
