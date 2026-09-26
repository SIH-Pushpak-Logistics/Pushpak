#!/usr/bin/env python3
"""Check Rust Zenoh 1.0.0 Heartbeat delivery to a Python peer.

Run after building src/pushpak_peer. Requires eclipse-zenoh==1.0.0 and
protobuf (the Docker image pins protobuf==3.20.3). No external protoc is used.
"""

import argparse
import importlib.metadata
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import time

from google.protobuf import descriptor_pb2, descriptor_pool, message_factory
import zenoh


ROOT = Path(__file__).resolve().parents[1]
HEARTBEAT_FIELDS = {
    'timestamp_ms': 1,
    'drone_id': 2,
    'status_flags': 3,
}


def heartbeat_message_type():
    """Build the three-field Heartbeat decoder from the checked-in contract."""
    source = (ROOT / 'proto' / 'pushpak.proto').read_text(encoding='utf-8')
    match = re.search(r'message\s+Heartbeat\s*\{([^}]*)\}', source, re.S)
    if match is None:
        raise RuntimeError('Heartbeat is missing from proto/pushpak.proto')
    fields = {
        name: int(number)
        for name, number in re.findall(r'uint32\s+(\w+)\s*=\s*(\d+)\s*;', match.group(1))
    }
    if fields != HEARTBEAT_FIELDS:
        raise RuntimeError(f'Heartbeat fields changed: {fields}')

    file_proto = descriptor_pb2.FileDescriptorProto()
    file_proto.name = 'pushpak_heartbeat_check.proto'
    file_proto.package = 'swarm.telemetry'
    file_proto.syntax = 'proto3'
    message = file_proto.message_type.add()
    message.name = 'Heartbeat'
    for name, number in fields.items():
        field = message.field.add()
        field.name = name
        field.number = number
        field.label = descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL
        field.type = descriptor_pb2.FieldDescriptorProto.TYPE_UINT32
    pool = descriptor_pool.DescriptorPool()
    descriptor = pool.AddSerializedFile(file_proto.SerializeToString())
    heartbeat = descriptor.message_types_by_name['Heartbeat']
    if hasattr(message_factory, 'GetMessageClass'):
        return message_factory.GetMessageClass(heartbeat)
    return message_factory.MessageFactory(pool).GetPrototype(heartbeat)


def free_loopback_port():
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        return listener.getsockname()[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--peer-binary', type=Path,
                        default=ROOT / 'src' / 'pushpak_peer' / 'target' / 'debug' /
                        ('pushpak_peer.exe' if os.name == 'nt' else 'pushpak_peer'))
    parser.add_argument('--timeout', type=float, default=12.0)
    args = parser.parse_args()
    if importlib.metadata.version('eclipse-zenoh') != '1.0.0':
        raise RuntimeError('this check requires eclipse-zenoh==1.0.0')
    if not args.peer_binary.is_file():
        raise SystemExit(f'build the peer first: {args.peer_binary}')
    heartbeat_type = heartbeat_message_type()
    endpoint = f'tcp/127.0.0.1:{free_loopback_port()}'
    config = zenoh.Config()
    config.insert_json5('mode', '"peer"')
    config.insert_json5('listen/endpoints', json.dumps([endpoint]))
    config.insert_json5('scouting/multicast/enabled', 'false')
    process = None
    print(f'Python eclipse-zenoh==1.0.0 listening on {endpoint}', flush=True)
    with zenoh.open(config) as session:
        with session.declare_subscriber('pushpak/heartbeat/3') as subscriber:
            process = subprocess.Popen([
                str(args.peer_binary), '--drone-id', '3',
                '--keyframes', str(ROOT / 'src/pushpak_peer/testdata/keyframes.csv'),
                '--connect', endpoint,
            ], cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True)
            try:
                deadline = time.monotonic() + args.timeout
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        raise RuntimeError(f'Rust peer exited with {process.returncode}')
                    sample = subscriber.try_recv()
                    if sample is None:
                        time.sleep(0.05)
                        continue
                    payload = bytes(sample.payload)
                    heartbeat = heartbeat_type.FromString(payload)
                    print(f'key={sample.key_expr} payload_hex={payload.hex()} '
                          f'Heartbeat(timestamp_ms={heartbeat.timestamp_ms}, '
                          f'drone_id={heartbeat.drone_id}, '
                          f'status_flags={heartbeat.status_flags})', flush=True)
                    if str(sample.key_expr) != 'pushpak/heartbeat/3':
                        raise AssertionError('unexpected Zenoh key')
                    if heartbeat.drone_id != 3 or heartbeat.status_flags != 0:
                        raise AssertionError('decoded Heartbeat fields do not match Rust peer')
                    print('PASS: Rust Heartbeat decoded by Python subscriber', flush=True)
                    return
                raise TimeoutError('no Rust Heartbeat arrived at Python subscriber')
            finally:
                if process.poll() is None:
                    process.terminate()
                output, _ = process.communicate(timeout=5)
                print(output.rstrip(), flush=True)


if __name__ == '__main__':
    main()
