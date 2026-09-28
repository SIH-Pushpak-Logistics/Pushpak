import math

MAX_PAYLOAD_B = 50
INT32_MIN, INT32_MAX = -(2 ** 31), 2 ** 31 - 1


def parse_heartbeat_key(key):
    parts = key.split('/')
    if len(parts) != 3 or parts[0] != 'pushpak' or parts[1] != 'heartbeat':
        return None
    try:
        drone_id = int(parts[2])
    except ValueError:
        return None
    return drone_id if drone_id >= 0 else None


def timestamp_ms(sec, nanosec):
    return (sec * 1000 + nanosec // 1_000_000) & 0xFFFFFFFF


def millimetres(metres):
    if not math.isfinite(metres):
        raise ValueError('position must be finite')
    value = int(round(metres * 1000.0))
    if not INT32_MIN <= value <= INT32_MAX:
        raise ValueError('position exceeds int32 millimetres')
    return value


def quaternion_rpy_cdeg(x, y, z, w):
    if not all(math.isfinite(v) for v in (x, y, z, w)):
        raise ValueError('quaternion must be finite')
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        raise ValueError('quaternion has zero length')
    x, y, z, w = x / n, y / n, z / n, w / n
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return tuple(int(round(math.degrees(a) * 100.0)) for a in (roll, pitch, yaw))


def confidence_percent(confidence):
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise ValueError('confidence must be in [0, 1]')
    return int(round(confidence * 100.0))


class SurvivorTracker:
    def __init__(self, radius_m):
        self.radius_sq = radius_m * radius_m
        self.entries = []

    def observe(self, x, y):
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError('survivor position must be finite')
        for i, (px, py, hits) in enumerate(self.entries):
            if (x - px) ** 2 + (y - py) ** 2 <= self.radius_sq:
                self.entries[i] = (px, py, hits + 1)
                return i + 1, hits + 1
        self.entries.append((x, y, 1))
        return len(self.entries), 1


class PeerTable:
    def __init__(self, own_id, timeout_s):
        self.own_id = own_id
        self.timeout_s = timeout_s
        self.last = {}

    def heard(self, drone_id, now_s):
        if drone_id != self.own_id:
            self.last[drone_id] = now_s

    def alive(self, now_s):
        return sorted(i for i, t in self.last.items() if now_s - t < self.timeout_s)
