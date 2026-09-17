# PUSHPAK — Redis Stream Contracts v1

Frozen interface between navigation, perception and UI.
Change a key name here → tell everyone. Do not change one silently.

## Rules

- Transport: Redis at `localhost:6379`, port `8765` exposed for the WebSocket bridge.
- All values are STRINGS. Floats are formatted to 4 decimal places by
  `RedisTelemetryPublisher.send_payload()`. Booleans are `"True"` / `"False"`.
- Every payload carries `timestamp` (SIM seconds, not wall clock) and `drone_id`.
- Streams are capped at `maxlen=100, approximate=True`. They are latest-value
  registers, NOT durable logs. Never treat a stream as history.
- `{id}` = drone id, currently always `drone_00`.

## Helper API

```python
from swarm_utils.redis_bridge import RedisTelemetryPublisher, RedisTelemetrySubscriber

pub = RedisTelemetryPublisher(stream_name='...', logger=self.get_logger())
pub.send_payload(drone_id, timestamp_sec, key=value, ...)

sub = RedisTelemetrySubscriber(streams=['...'], logger=self.get_logger())
sub.get_latest('stream', max_age_sec=0.5)   # None if older than max_age_sec
sub.get_age('stream')                        # seconds, or None
```

---

## EXISTING — produced today, read-only for UI

### `telemetry:{id}:altitude`      producer: altimeter_node · ~30 Hz
`timestamp, drone_id, z`
AGL metres from the downward rangefinder.

### `telemetry:{id}:velocity`      producer: vision_nav_node · ~15 Hz
`timestamp, drone_id, linear_x, linear_y, linear_z, angular_z, is_valid, features`
Body-frame m/s from optical flow. `is_valid="False"` means DO NOT TRUST.

### `telemetry:{id}:flow_debug`    producer: vision_nav_node · ~15 Hz
`timestamp, drone_id, u_raw, v_raw, u_med, v_med, u_std, v_std,
 frame_diff, gyro_x, gyro_y, dt, altitude, features`
Diagnostics. UI may show `features` as a perception-health indicator.

---

## NEW — to be built

### `telemetry:{id}:pose`          producer: pose_publisher_node · 10 Hz
| key | type | notes |
|---|---|---|
| `timestamp` | float | sim seconds |
| `drone_id`  | str   | |
| `x`,`y`,`z` | float | metres, ENU, local origin |
| `yaw_deg`   | float | ENU degrees, 0 = East, +90 = North |

UI: live position readout + map track.

### `detections:{id}`              producer: yolo_node · on detection, ≤5 Hz
| key | type | notes |
|---|---|---|
| `timestamp` | float | |
| `drone_id`  | str   | |
| `det_id`    | str   | unique per detection |
| `class_name`| str   | `"person"` |
| `confidence`| float | 0.0–1.0 |
| `bbox_x`,`bbox_y`,`bbox_w`,`bbox_h` | float | pixels |
| `drone_x`,`drone_y`,`drone_z` | float | drone pose at capture |
| `image_path`| str   | `/workspace/detections/<det_id>.jpg` |

Raw per-frame hits, NOT deduplicated. UI does not read this — read `victims:`.

### `victims:{id}`                 producer: commander_node · REDIS HASH
**Not a stream.** `HSET victims:{id} <victim_id> <json>`. Streams cap at 100
entries; a victim list that silently drops the oldest is useless.
`commander_node` uses a direct `redis.Redis()` client for this — the one
documented exception to the helper API.

JSON value:
```json
{
  "victim_id": "v_001",
  "confidence": 0.87,
  "world_x": 3.42, "world_y": -1.08,
  "first_seen": 71.2, "last_seen": 84.6,
  "hit_count": 14,
  "image_path": "/workspace/detections/v_001.jpg",
  "acked": false
}
```
UI: `HGETALL victims:drone_00` → alert list + map markers.
Ack: `HSET` the same field with `acked: true`.

### `link:{id}:status`             producer: link_monitor_node · 2 Hz
| key | type | notes |
|---|---|---|
| `timestamp` | float | |
| `drone_id`  | str   | |
| `state`     | str   | `ONLINE` \| `DEGRADED` \| `OFFLINE` |
| `cached_packets` | int | queued while offline |
| `last_sync_sec`  | float | sim time of last successful sync |
| `rssi_dbm`       | float | simulated |

UI: link panel. This drives the link-loss demo.

---

## Panel → stream map

| Panel | Reads |
|---|---|
| Live telemetry | `:pose`, `:altitude`, `:velocity` |
| 2D map + track | `:pose`, `victims:` |
| Victim alerts  | `victims:` |
| Link status    | `link:{id}:status` |

## Fake-data generator

`tools/fake_publisher.py` writes all five streams with plausible values so the
frontend can be built with no simulator running. Run it, build against it.