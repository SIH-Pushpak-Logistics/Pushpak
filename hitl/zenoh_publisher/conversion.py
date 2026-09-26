"""ROS value conversion shared by the rig adapter and its local tests."""

import math


def millimetres(metres):
    if not math.isfinite(metres):
        raise ValueError("position must be finite")
    value = round(metres * 1000)
    if not -(2**31) <= value < 2**31:
        raise ValueError("position exceeds sint32 millimetre range")
    return value


def centidegrees(radians):
    if not math.isfinite(radians):
        raise ValueError("orientation must be finite")
    return round(math.degrees(radians) * 100)


def quaternion_rpy_cdeg(q):
    x, y, z, w = q
    if not all(map(math.isfinite, q)):
        raise ValueError("quaternion must be finite")
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-9:
        raise ValueError("quaternion has zero length")
    x, y, z, w = (v / norm for v in q)
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return tuple(map(centidegrees, (roll, pitch, yaw)))


def confidence_percent(confidence):
    if not math.isfinite(confidence) or not 0 <= confidence <= 1:
        raise ValueError("confidence must be a finite fraction in [0, 1]")
    return round(confidence * 100)


class SurvivorTracker:
    """Give nearby detections one stable ID and count repeated hits."""

    def __init__(self, radius_m=1.5):
        self.radius_squared = radius_m * radius_m
        self.entries = []

    def observe(self, position):
        if not all(map(math.isfinite, position)):
            raise ValueError("survivor position must be finite")
        for index, (previous, hits) in enumerate(self.entries):
            if sum((a - b)**2 for a, b in zip(position, previous)) <= self.radius_squared:
                self.entries[index] = (position, hits + 1)
                return index + 1, hits + 1
        self.entries.append((position, 1))
        return len(self.entries), 1
