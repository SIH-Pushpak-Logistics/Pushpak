"""Actual Rust protobuf -> Zenoh 1.0 -> Python gateway -> WebSocket acceptance."""
import asyncio
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import unittest

import websockets


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


class GatewayEndToEndTest(unittest.IsolatedAsyncioTestCase):
    async def test_rust_messages_and_loss_reach_websocket(self):
        root = Path(__file__).resolve().parents[1]
        binary = Path(os.environ.get('PUSHPAK_RIG_BINARY', root / 'hitl/zenoh_publisher/target/debug/pushpak_hitl_zenoh_publisher'))
        self.assertTrue(binary.is_file(), 'build Rust publisher and set PUSHPAK_RIG_BINARY if needed')
        endpoint = f'tcp/127.0.0.1:{free_port()}'
        port = free_port()
        gateway = subprocess.Popen([sys.executable, str(root / 'tools/zenoh_gateway.py'),
                                    '--listen', endpoint, '--port', str(port)],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        publisher = None
        rig = None
        try:
            for _ in range(100):
                try:
                    ws = await websockets.connect(f'ws://127.0.0.1:{port}')
                    break
                except OSError:
                    if gateway.poll() is not None:
                        self.fail(gateway.stderr.read())
                    await asyncio.sleep(.1)
            else:
                self.fail('gateway did not start')
            await ws.close()
            async with websockets.connect(f'ws://127.0.0.1:{port}') as ws:
                publisher = subprocess.Popen([str(binary), '--drone-id', '1', '--connect', endpoint],
                                             stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                                             stderr=subprocess.DEVNULL, text=True)
                rig = subprocess.Popen([str(binary), '--drone-id', '2', '--connect', endpoint],
                                       stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                                       stderr=subprocess.DEVNULL, text=True)
                messages = [
                    dict(kind='heartbeat', timestamp_ms=12345, status_flags=0),
                    dict(kind='keyframe', timestamp_ms=12345, pos_x_mm=1250, pos_y_mm=-500,
                         pos_z_mm=1500, roll_cdeg=0, pitch_cdeg=0, yaw_cdeg=9000, status_flags=0),
                    dict(kind='survivor', timestamp_ms=12345, survivor_id=7, pos_x_mm=2000,
                         pos_y_mm=3000, pos_z_mm=0, confidence_pct=87, hit_count=2),
                ]
                for _ in range(80):
                    for message in messages:
                        publisher.stdin.write(json.dumps(message) + '\n')
                    publisher.stdin.flush()
                    for message in messages:
                        rig_message = dict(message)
                        if rig_message['kind'] == 'keyframe':
                            rig_message['pos_x_mm'] = -2500
                        rig.stdin.write(json.dumps(rig_message) + '\n')
                    rig.stdin.flush()
                    fleet = json.loads(await asyncio.wait_for(ws.recv(), 2))['data']
                    data = fleet['1']
                    rig_data = fleet['2']
                    if (data['pose'] and data['victims'] and data['linkStatus']['state'] == 'ONLINE'
                            and rig_data['pose'] and rig_data['victims']
                            and rig_data['linkStatus']['state'] == 'ONLINE'):
                        break
                else:
                    self.fail('Rust telemetry did not reach WebSocket')
                self.assertEqual(data['pose']['x'], 1.25)
                self.assertEqual(data['pose']['yaw_deg'], 90)
                self.assertEqual(data['victims'][0]['victim_id'], '1:7')
                self.assertEqual(data['victims'][0]['confidence'], .87)
                self.assertEqual(rig_data['pose']['x'], -2.5)
                self.assertEqual(rig_data['victims'][0]['victim_id'], '2:7')
                publisher.terminate()
                publisher.wait(timeout=5)
                publisher.stdin.close()
                for _ in range(30):
                    rig.stdin.write(json.dumps(dict(kind='heartbeat', timestamp_ms=12345,
                                                   status_flags=0)) + '\n')
                    rig.stdin.flush()
                    fleet = json.loads(await asyncio.wait_for(ws.recv(), 2))['data']
                    data = fleet['1']
                    rig_data = fleet['2']
                    if data['linkStatus']['state'] == 'OFFLINE':
                        break
                self.assertEqual(data['linkStatus']['state'], 'OFFLINE')
                self.assertIsNone(data['pose'])
                self.assertEqual(rig_data['linkStatus']['state'], 'ONLINE')
                publisher = subprocess.Popen([str(binary), '--drone-id', '1', '--connect', endpoint],
                                             stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                                             stderr=subprocess.DEVNULL, text=True)
                for _ in range(50):
                    for message in messages:
                        publisher.stdin.write(json.dumps(message) + '\n')
                    publisher.stdin.flush()
                    rig.stdin.write(json.dumps(dict(kind='heartbeat', timestamp_ms=12345,
                                                   status_flags=0)) + '\n')
                    rig.stdin.flush()
                    fleet = json.loads(await asyncio.wait_for(ws.recv(), 2))['data']
                    data = fleet['1']
                    if data['linkStatus']['state'] == 'ONLINE' and data['pose']:
                        break
                self.assertEqual(data['linkStatus']['state'], 'ONLINE')
                self.assertEqual(data['pose']['x'], 1.25)
                self.assertEqual(fleet['2']['linkStatus']['state'], 'ONLINE')
        finally:
            for process in (publisher, rig, gateway):
                if process is not None:
                    if process.poll() is None:
                        process.terminate()
                    process.wait(timeout=5)
                    for stream in (process.stdin, process.stderr):
                        if stream:
                            stream.close()
