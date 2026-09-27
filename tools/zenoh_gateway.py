#!/usr/bin/env python3
"""Ground peer 0: frozen protobuf telemetry -> dashboard WebSocket snapshots."""
import argparse
import asyncio
import json
from pathlib import Path
import threading
import time

from google.protobuf import descriptor_pb2, message_factory
from google.protobuf.message import DecodeError
import websockets
import zenoh

descriptor = descriptor_pb2.FileDescriptorSet.FromString(
    Path(__file__).with_name('pushpak.desc').read_bytes())
MESSAGES = message_factory.GetMessages(descriptor.file)
KINDS = {'heartbeat': 'Heartbeat', 'keyframe': 'SubMapKeyframe', 'survivor': 'SurvivorEvent'}


def decode(key, payload):
    parts = key.split('/')
    if len(parts) != 3 or parts[0] != 'pushpak' or parts[1] not in KINDS or len(payload) > 50:
        return None
    try:
        drone_id = int(parts[2])
        msg = MESSAGES['swarm.telemetry.' + KINDS[parts[1]]]().FromString(payload)
    except (ValueError, DecodeError):
        return None
    if msg.drone_id != drone_id or (parts[1] == 'survivor' and msg.confidence_pct > 100):
        return None
    return parts[1], msg


class GatewayState:
    """Bounded state shared by Zenoh callback threads and the asyncio server."""
    def __init__(self, drone_id, clock=time.monotonic):
        self.drone_id = drone_id
        self.clock = clock
        self.lock = threading.Lock()
        self.heartbeat = None
        self.pose = None
        self.pose_received = None
        self.victims = {}

    def receive(self, key, payload):
        decoded = decode(key, payload)
        if decoded is None:
            return
        kind, msg = decoded
        if msg.drone_id != self.drone_id:
            return
        with self.lock:
            now = self.clock()
            if kind == 'heartbeat':
                self.heartbeat = now
            elif kind == 'keyframe':
                self.pose_received = now
                self.pose = dict(timestamp=msg.timestamp_ms / 1000, drone_id=str(msg.drone_id),
                                 x=msg.pos_x_mm / 1000, y=msg.pos_y_mm / 1000,
                                 z=msg.pos_z_mm / 1000, yaw_deg=msg.yaw_cdeg / 100)
            else:
                identity = f'{msg.drone_id}:{msg.survivor_id}'
                previous = self.victims.get(identity)
                self.victims[identity] = dict(
                    victim_id=identity, confidence=msg.confidence_pct / 100,
                    world_x=msg.pos_x_mm / 1000, world_y=msg.pos_y_mm / 1000,
                    first_seen=previous['first_seen'] if previous else msg.timestamp_ms / 1000,
                    last_seen=msg.timestamp_ms / 1000, hit_count=msg.hit_count,
                    image_path='', acked=False)
                if len(self.victims) > 1000:
                    del self.victims[next(iter(self.victims))]

    def snapshot(self):
        with self.lock:
            now = self.clock()
            age = None if self.heartbeat is None else now - self.heartbeat
            pose_fresh = self.pose_received is not None and now - self.pose_received <= 0.75
            return {'type': 'snapshot', 'data': {
                'pose': self.pose if pose_fresh else None,
                'altitude': None, 'velocity': None, 'flowDebug': None,
                'linkStatus': dict(timestamp=time.time(), drone_id=str(self.drone_id),
                                   state='ONLINE' if age is not None and age < 1.75 else 'OFFLINE',
                                   last_sync_sec=age, rssi_dbm=None, cached_packets=None),
                'victims': list(self.victims.values()),
            }}


async def run(args):
    state = GatewayState(args.drone_id)
    config = zenoh.Config()
    config.insert_json5('mode', '"peer"')
    for name in ('connect', 'listen'):
        endpoints = getattr(args, name)
        if endpoints:
            config.insert_json5(name + '/endpoints', json.dumps(endpoints))
    session = zenoh.open(config)
    subscriber = session.declare_subscriber(
        'pushpak/**', lambda sample: state.receive(str(sample.key_expr), bytes(sample.payload)))

    async def handler(websocket):
        try:
            while True:
                await asyncio.wait_for(websocket.send(json.dumps(state.snapshot())), timeout=2)
                await asyncio.sleep(0.2)
        except (websockets.ConnectionClosed, asyncio.TimeoutError):
            return

    try:
        async with websockets.serve(handler, args.host, args.port):
            print(f'Zenoh ground peer 0 -> ws://{args.host}:{args.port}; displaying drone {args.drone_id}', flush=True)
            heartbeat = MESSAGES['swarm.telemetry.Heartbeat'](drone_id=0)
            while True:
                heartbeat.timestamp_ms = int(time.monotonic() * 1000) & 0xffffffff
                session.put('pushpak/heartbeat/0', heartbeat.SerializeToString())
                await asyncio.sleep(0.5)
    finally:
        subscriber.undeclare()
        session.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drone-id', type=int, choices=range(1, 256), default=1)
    parser.add_argument('--connect', action='append', default=[])
    parser.add_argument('--listen', action='append', default=[])
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
