# Rust to Python Heartbeat check

This check runs a Python `eclipse-zenoh==1.0.0` subscriber in peer mode on a
loopback TCP endpoint, starts the Rust `pushpak_peer` with an explicit direct
connection, and decodes `pushpak/heartbeat/3` with protobuf. The decoder
verifies its field numbers against `proto/pushpak.proto`. It does not require a
system `protoc`; the Rust `build.rs` vendors it.

From the repository root, use a Python environment with
`eclipse-zenoh==1.0.0` and `protobuf==3.20.3` (the Docker pin), then run:

```bash
cargo build --locked --manifest-path src/pushpak_peer/Cargo.toml
python3 tools/check_rust_python_heartbeat.py
```

Raw local output from 26 Sep 2026 on Windows, with those exact Python package
versions:

```text
eclipse-zenoh: 1.0.0
protobuf: 3.20.3
Python eclipse-zenoh==1.0.0 listening on tcp/127.0.0.1:51616
key=pushpak/heartbeat/3 payload_hex=080e1003 Heartbeat(timestamp_ms=14, drone_id=3, status_flags=0)
PASS: Rust Heartbeat decoded by Python subscriber
pushpak_peer 3 ready (Zenoh peer mode; no zenohd)
```

The port and `timestamp_ms` vary on each run. This proves the wire format and
key are compatible across Rust and Python on one host. The separately reported
Ubuntu/Mac two-laptop test checks the physical network path; its raw logs are
pending from the tester.
