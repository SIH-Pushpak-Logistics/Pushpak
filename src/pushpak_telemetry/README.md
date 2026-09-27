# Pushpak telemetry implementation notes

The workspace-level [Zenoh + Protobuf contract](../../README.md#8-zenoh--protobuf-contract--protopushpakproto)
defines accepted wire messages. See the [two-laptop acceptance test](../../docs/ZENOH_TWO_LAPTOP_TEST.md)
for build and run commands and the [Rust-to-Python heartbeat check](../../docs/RUST_PYTHON_HEARTBEAT_CHECK.md)
for the interoperability command.

Use `--locked` for reproducible builds and tests. The internal Zenoh crates are
also pinned to 1.0.0 because that release's caret dependencies otherwise select
incompatible later internals.

`peers_alive(timeout)` tracks valid heartbeats immediately after opening a peer,
even without `subscribe_all`. Callbacks run on Zenoh threads and should return
quickly.

Replay uses CSV timestamps to select the most recent pose at 5 Hz, loops with a
200 ms final hold, and stamps both heartbeat and keyframe from the same elapsed
replay clock, wrapping at `uint32` milliseconds. Real ROS producers must supply
their own simulation or ROS timestamps through the library API.

The frozen proto permits a 54 B keyframe at unrestricted 32-bit extremes. The
operational-angle boundary test fits within the 50 B acceptance limit, while a
separate test records the 54 B schema limit.
