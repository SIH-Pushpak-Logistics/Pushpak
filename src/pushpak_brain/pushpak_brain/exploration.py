import math


def validate_gains(kp, v_max_mps):
    for name, value in (('kp', kp), ('v_max_mps', v_max_mps)):
        if not (isinstance(value, (int, float)) and math.isfinite(value) and value > 0.0):
            raise ValueError(f'{name} must be a finite positive number, got {value!r}')


def lawnmower(x_min, x_max, y_min, y_max, lane_spacing_m):
    if not (x_max > x_min and y_max >= y_min and lane_spacing_m > 0.0):
        raise ValueError('need x_max > x_min, y_max >= y_min, lane_spacing_m > 0')
    n = int(math.floor((y_max - y_min) / lane_spacing_m + 1e-9)) + 1
    ys = [y_min + i * lane_spacing_m for i in range(n)]
    if y_max - ys[-1] > 1e-9:
        ys.append(y_max)
    waypoints = []
    for i, y in enumerate(ys):
        x_start, x_end = (x_min, x_max) if i % 2 == 0 else (x_max, x_min)
        waypoints.append((x_start, y))
        waypoints.append((x_end, y))
    return waypoints


def path_length(start, waypoints):
    total = 0.0
    prev = start
    for wp in waypoints:
        total += math.hypot(wp[0] - prev[0], wp[1] - prev[1])
        prev = wp
    return total


def p_velocity(pos, target, kp, v_max_mps):
    ex = target[0] - pos[0]
    ey = target[1] - pos[1]
    dist = math.hypot(ex, ey)
    if dist == 0.0:
        return 0.0, 0.0
    gain = min(kp, v_max_mps / dist)
    return ex * gain, ey * gain


def advance(waypoints, index, pos, accept_radius_m):
    while index < len(waypoints):
        wx, wy = waypoints[index]
        if math.hypot(wx - pos[0], wy - pos[1]) > accept_radius_m:
            break
        index += 1
    return index
