#!/usr/bin/env python3
import csv
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOLERANCE_M = 0.8
WINDOW_S = 60.0
WANTED = ('/pushpak/airborne', '/odometry/filtered', '/sim/ground_truth/odom',
          '/mavros/state', '/drone/tof_range')


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_bag(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in WANTED}
    out = {name: [] for name in WANTED}
    while reader.has_next():
        topic, data, t_bag = reader.read_next()
        if topic in types:
            out[topic].append((t_bag * 1e-9, deserialize_message(data, types[topic])))
    for name in WANTED:
        if not out[name]:
            raise SystemExit(f'{uri}: no messages on {name}')
    return out


def pose_series(entries):
    arr = np.array([(stamp(m), m.pose.pose.position.x, m.pose.pose.position.y,
                     m.pose.pose.position.z, yaw(m.pose.pose.orientation)) for _, m in entries])
    arr = arr[np.argsort(arr[:, 0])]
    arr[:, 4] = np.unwrap(arr[:, 4])
    return arr


def evaluate(uri, bias):
    bag = read_bag(uri)
    odom = bag['/odometry/filtered']
    t_air_bag = bag['/pushpak/airborne'][0][0]
    t0 = stamp(min(odom, key=lambda e: abs(e[0] - t_air_bag))[1])

    est = pose_series(odom)
    gt = pose_series(bag['/sim/ground_truth/odom'])
    e = est[(est[:, 0] >= t0) & (est[:, 0] <= t0 + WINDOW_S)]
    tt = e[:, 0] - t0
    g = np.column_stack([np.interp(e[:, 0], gt[:, 0], gt[:, k]) for k in range(1, 5)])

    dpsi = e[0, 4] - g[0, 3]
    c, s = math.cos(dpsi), math.sin(dpsi)
    dgx, dgy = g[:, 0] - g[0, 0], g[:, 1] - g[0, 1]
    rgx, rgy = c * dgx - s * dgy, s * dgx + c * dgy
    dex, dey = e[:, 1] - e[0, 1], e[:, 2] - e[0, 2]
    d = np.hypot(dex - rgx, dey - rgy)
    slope = np.polyfit(tt, d, 1)[0] * 60.0
    span = float(tt[-1])
    predicted = bias * span

    states = [m for _, m in bag['/mavros/state'] if t0 <= stamp(m) <= t0 + WINDOW_S]
    modes = sorted({m.mode for m in states})
    armed = bool(states) and all(m.armed for m in states)
    tof = np.array([m.range for _, m in bag['/drone/tof_range']
                    if t0 <= stamp(m) <= t0 + WINDOW_S and math.isfinite(m.range)])
    trace = np.array([m.twist.covariance[0] + m.twist.covariance[7] + m.twist.covariance[14]
                      for _, m in odom if t0 <= stamp(m) <= t0 + WINDOW_S])

    ok_span = span >= WINDOW_S - 0.5
    ok_mode = modes == ['GUIDED'] and armed
    ok_drift = abs(d[-1] - predicted) <= TOLERANCE_M
    passed = ok_span and ok_mode and ok_drift

    csv_path = uri.rstrip('/') + '_drift.csv'
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'd_m', 'est_dx_m', 'est_dy_m', 'gt_dx_aligned_m', 'gt_dy_aligned_m', 'gt_z_m'])
        for row in zip(tt, d, dex, dey, rgx, rgy, g[:, 2]):
            w.writerow([f'{v:.4f}' for v in row])

    print(f'=== {uri}  (bias_initial_x = {bias:.2f} m/s)')
    print(f'  window {span:.1f} s sim from airborne | yaw offset {math.degrees(dpsi):+.2f} deg')
    print(f'  d(end) {d[-1]:.3f} m | predicted b*t {predicted:.3f} m | error {d[-1] - predicted:+.3f} m '
          f'(tolerance {TOLERANCE_M} m) | slope {slope:.3f} m/min')
    print(f'  estimate moved {math.hypot(dex[-1], dey[-1]):.3f} m, truth moved {math.hypot(rgx[-1], rgy[-1]):.3f} m')
    print(f'  modes {modes} armed throughout {armed}')
    if tof.size:
        print(f'  ToF {tof.min():.3f}..{tof.max():.3f} m (std {tof.std():.3f}) | true z {g[:, 2].min():.3f}..{g[:, 2].max():.3f} m')
    if trace.size:
        print(f'  Tr(Sigma_v) median {np.median(trace):.5f} max {trace.max():.5f}')
    print(f'  span {"ok" if ok_span else "SHORT"} | mode {"ok" if ok_mode else "FAIL"} | '
          f'drift {"ok" if ok_drift else "FAIL"} | {"PASS" if passed else "FAIL"} | csv {csv_path}')
    return passed


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: eval_hover_gate.py BAG:BIAS [BAG:BIAS ...]')
    results = []
    for arg in sys.argv[1:]:
        uri, bias = arg.rsplit(':', 1)
        results.append(evaluate(uri, float(bias)))
    print('GATE', 'PASS' if all(results) else 'FAIL')
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
