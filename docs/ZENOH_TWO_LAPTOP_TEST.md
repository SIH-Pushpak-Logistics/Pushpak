# Zenoh Two-Laptop Acceptance Test

## Objective

Prove that two Pushpak peers exchange telemetry directly over Zenoh without a
central server. Passing this test proves the routerless communication path; it
does not yet test the ROS survivor-detection integration.

Zenoh still needs an IP network (Ethernet or Wi-Fi); it does not provide a radio
link itself. The previously shared terminal-style image was a rendering of
selected local test log lines, not a screenshot of a two-machine test.

The peer expires heartbeats after 1.75 seconds and checks every 100 ms, leaving
margin within the two-second target. Measure the actual delay on the test
machines; operating-system scheduling is not a real-time guarantee.

## Required setup

- Two laptops connected to the same phone hotspot or travel router.
- Rust installed on both laptops (`rustc --version` and `cargo --version`).
- TCP port `7447` allowed through the host firewall.
- Repository branch `feat/zenoh-telemetry` checked out on both laptops.
- No `zenohd` process running on either laptop.

Use the phone hotspot or travel router rather than campus Wi-Fi. Campus Wi-Fi
may block peer discovery or communication between connected devices.

## 1. Prepare both laptops

Run these commands on **Laptop A and Laptop B** from the repository root:

```bash
git fetch origin
git checkout feat/zenoh-telemetry
git pull --ff-only origin feat/zenoh-telemetry

rustc --version
cargo --version

cargo test --manifest-path src/pushpak_telemetry/Cargo.toml
cargo test --manifest-path src/pushpak_peer/Cargo.toml
cargo build --release --manifest-path src/pushpak_peer/Cargo.toml
```

Expected test result:

```text
test result: ok
```

The first Zenoh build can take 10–15 minutes.

## 2. Confirm that no Zenoh router is running

Run on **both laptops**:

```bash
pgrep -a zenohd
```

The command must print nothing. If it returns a process, stop that process
before continuing. Do not start `zenohd` during this test.

On Windows PowerShell, use:

```powershell
Get-Process zenohd -ErrorAction SilentlyContinue
```

This must also print nothing.

## 3. Find Laptop A's hotspot IP address

On Laptop A:

```bash
hostname -I
```

Choose the address belonging to the phone hotspot or travel-router interface.
Do not use `127.0.0.1`, a Docker address, or a VPN address. In the examples
below, Laptop A's address is shown as `192.168.1.20`; replace it with the real
address.

Windows PowerShell alternative:

```powershell
Get-NetIPAddress -AddressFamily IPv4 |
  Where-Object IPAddress -NotLike '127.*'
```

## 4. Start Laptop A as a listening peer

From the repository root on Laptop A:

```bash
cargo run --release --manifest-path src/pushpak_peer/Cargo.toml -- \
  --drone-id 3 \
  --keyframes src/pushpak_peer/testdata/keyframes.csv \
  --listen tcp/0.0.0.0:7447
```

Expected startup line:

```text
pushpak_peer 3 ready (Zenoh peer mode; no zenohd)
```

Leave this terminal running.

## 5. Connect Laptop B directly to Laptop A

From the repository root on Laptop B, replacing `192.168.1.20` with Laptop A's
real hotspot IP:

```bash
cargo run --release --manifest-path src/pushpak_peer/Cargo.toml -- \
  --drone-id 4 \
  --keyframes src/pushpak_peer/testdata/keyframes.csv \
  --connect tcp/192.168.1.20:7447
```

Within a few seconds, Laptop A must show messages such as:

```text
HEARTBEAT peer=4 t_ms=... flags=0
PEER ALIVE 4
KEYFRAME peer=4 t_ms=... pos=(...,...,...)
```

Laptop B must show messages such as:

```text
HEARTBEAT peer=3 t_ms=... flags=0
PEER ALIVE 3
KEYFRAME peer=3 t_ms=... pos=(...,...,...)
```

This is bidirectional communication. Laptop A is a Zenoh **peer**, not a
central router; there is still no `zenohd` process.

## 6. Test peer-loss detection

Stop Laptop B's peer with `Ctrl+C`, or disconnect Laptop B from the hotspot.
Keep Laptop A running.

Laptop A must print the following within two seconds of the last heartbeat:

```text
PEER LOST 4
```

Laptop A must continue running and publishing its own telemetry. The loss of
Laptop B must not stop or crash Laptop A.

## 7. Evidence to save

Save screenshots or terminal logs showing all of the following:

- `pgrep -a zenohd` produced no output on both laptops.
- Laptop A received `HEARTBEAT peer=4` and printed `PEER ALIVE 4`.
- Laptop B received `HEARTBEAT peer=3` and printed `PEER ALIVE 3`.
- Each laptop received the other laptop's `KEYFRAME` messages.
- Laptop A printed `PEER LOST 4` within two seconds after Laptop B stopped.
- Laptop A remained operational after Laptop B stopped.

Record the two laptop IP addresses, operating systems, test date, and the Git
commit printed by:

```bash
git rev-parse HEAD
```

## Pass criteria

The test passes only when:

1. Both peers exchange heartbeats and keyframes in both directions.
2. No `zenohd` process runs on either laptop.
3. Killing or disconnecting one peer does not stop the other peer.
4. The surviving peer reports peer loss within two seconds.

## Troubleshooting

### No messages appear on either laptop

1. Confirm both laptops are on the same hotspot/router subnet.
2. Recheck Laptop A's IP address.
3. Confirm Laptop A is still running with `--listen tcp/0.0.0.0:7447`.
4. Check that port `7447` is listening on Laptop A:

   ```bash
   ss -ltnp | grep 7447
   ```

5. Test the port from Laptop B:

   ```bash
   nc -vz 192.168.1.20 7447
   ```

6. Allow inbound TCP port `7447` in Laptop A's firewall, then retry.
7. Disable VPN software temporarily if it changes routing between the laptops.

Do not solve a failed test by starting `zenohd`; that would invalidate the
decentralized acceptance test.

### Multicast discovery does not work

That is acceptable on restrictive networks. The explicit `--listen` and
`--connect` commands above bypass multicast discovery while preserving direct
peer-to-peer communication.

### Build fails on Windows with `link.exe not found`

Install Visual Studio Build Tools 2022 with the **Desktop development with C++**
workload, restart the terminal, and run the Cargo command again.

## After this test passes

Also run three peers (IDs 0, 3 and 4), with a direct connection between 3 and 4.
Stop peer 0, representing the ground station, and confirm that 3 and 4 continue
receiving each other's heartbeats and keyframes. This is the ground-station-loss
demo; a two-peer test alone does not establish it.

Send the screenshots/logs and recorded commit to the team. The HITL Zenoh
publisher is on `hitl/jetson-rig`; its local integration rehearsal and Jetson
instructions are in [the HITL README](../hitl/zenoh_publisher/README.md).
