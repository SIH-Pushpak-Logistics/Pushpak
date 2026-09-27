# Pushpak v2 dashboard

The browser consumes snapshots from `tools/zenoh_gateway.py`. Redis is not used.
The gateway is ground peer **0**, with an explicit selected vehicle (default **1**).

From the repository root:

```sh
python -m pip install -r tools/requirements-gateway.txt
python tools/zenoh_gateway.py --drone-id 1 --connect tcp/SCOUT_IP:7447
```

In another terminal:

```sh
cd dashboard
npm ci
npm run dev
```

`npm run server` also starts the gateway using Python on PATH. Pass gateway flags
with `npm run server -- --drone-id 2 --connect tcp/RIG_IP:7447`.
For a browser on another machine, bind the gateway with `--host 0.0.0.0` and set
`VITE_WS_URL=ws://GROUND_IP:8765` when starting/building Vite. HTTPS needs a WSS proxy.

The selected drone ID is displayed in the header. A browser WebSocket connection
is separate from the vehicle heartbeat status. Heartbeats expire after 1.75 s;
poses disappear after 0.75 s without a keyframe. Detections remain as observations.
Coordinates are metres in odom, yaw is degrees, confidence is a fraction.
Height is odom Z, **not** a ToF AGL measurement. RSSI, cache count, velocity,
flow diagnostics, images and acknowledgements are unavailable in the frozen wire
schema and are not fabricated. One vehicle is selected per gateway instance;
multiple vehicles require separate instances/ports, not mixed tracks.

`tools/pushpak.desc` is generated from the unchanged frozen schema:

```sh
protoc --proto_path=proto --descriptor_set_out=tools/pushpak.desc proto/pushpak.proto
```

The gateway caches at most 1,000 survivor IDs for the selected vehicle and sends
bounded snapshots at 5 Hz. IDs are scoped by vehicle; distinct vehicle odom origins
are not assumed to be aligned. Raw peer tests still use `src/pushpak_peer`.
