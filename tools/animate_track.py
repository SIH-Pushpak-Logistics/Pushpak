#!/usr/bin/env python3
import argparse
import importlib.util
import json
import math
import os
import sys

import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from matplotlib.patches import Polygon, Rectangle

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PARAMS = os.path.join(REPO, 'src', 'drone_bringup', 'config', 'pushpak_params.yaml')
EXPLORATION = os.path.join(REPO, 'src', 'pushpak_brain', 'pushpak_brain', 'exploration.py')
COLUMNS = ('t_s', 'est_x', 'est_y', 'true_x', 'true_y', 'true_z', 'd_m')
HFOV_RAD = 1.047
IMG_W, IMG_H = 320, 240
TAN_HALF_H = math.tan(HFOV_RAD / 2.0)
TAN_HALF_V = TAN_HALF_H * IMG_H / IMG_W
FPS = 30
END_CARD_S = 3.0
C_TRUTH, C_EST, C_ERR, C_FOOT, C_GREY = '#000000', '#E69F00', '#D55E00', '#0072B2', '#999999'


def load_track(path):
    data = np.genfromtxt(path, delimiter=',', names=True)
    missing = [c for c in COLUMNS if c not in data.dtype.names]
    if missing:
        sys.exit(f'CSV is missing columns {missing}; header has {list(data.dtype.names)}')
    track = {c: np.asarray(data[c], dtype=float) for c in COLUMNS}
    if not np.all(np.diff(track['t_s']) > 0):
        sys.exit('t_s is not strictly increasing')
    return track


def load_waypoints():
    spec = importlib.util.spec_from_file_location('pushpak_exploration', EXPLORATION)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with open(PARAMS) as f:
        p = yaml.safe_load(f)['pushpak_brain']['ros__parameters']
    return module.lawnmower(p['explore_x_min_m'], p['explore_x_max_m'], p['explore_y_min_m'],
                            p['explore_y_max_m'], p['explore_lane_spacing_m'])


def footprint_half(z):
    return z * TAN_HALF_V, z * TAN_HALF_H


def footprint_corners(x, y, z, yaw):
    hx, hy = footprint_half(z)
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * bx - s * by, y + s * bx + c * by) for bx, by in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))]


def covered(vx, vy, x, y, z, yaw):
    hx, hy = footprint_half(z)
    dx, dy = vx - x, vy - y
    c, s = math.cos(yaw), math.sin(yaw)
    bx, by = c * dx + s * dy, -s * dx + c * dy
    return abs(bx) <= hx and abs(by) <= hy


def first_covered(track, victims, yaw):
    out = {}
    for name, (vx, vy) in victims.items():
        hit = None
        for i in range(len(track['t_s'])):
            if covered(vx, vy, track['true_x'][i], track['true_y'][i], track['true_z'][i], yaw):
                hit = track['t_s'][i]
                break
        dist = np.hypot(track['true_x'] - vx, track['true_y'] - vy)
        out[name] = (hit, float(dist.min()), float(track['t_s'][int(dist.argmin())]))
    return out


def stats(track):
    d = track['d_m']
    i = int(np.argmax(d))
    return float(d.mean()), float(d[i]), float(track['t_s'][i]), float(d[-1])


def parse_expect_victims(text):
    out = {}
    for item in text.split(','):
        name, value = item.split('=')
        out[name.strip()] = float(value)
    return out


def run_checks(track, victims, waypoints, yaw, expect_stats, expect_victims, tol_s):
    t = track['t_s']
    t0 = t[0]
    gaps = np.diff(t)
    ok = True
    print(f'rows: {len(t)}  t_s first {t[0]:.3f} last {t[-1]:.3f}  span {t[-1] - t0:.3f} s  '
          f'gap min {gaps.min():.4f} max {gaps.max():.4f} s')
    gap = np.hypot(track['est_x'] - track['true_x'], track['est_y'] - track['true_y'])
    print(f'absolute gap hypot(est-true): min {gap.min():.3f} m  mean {gap.mean():.3f} m  max {gap.max():.3f} m')
    mean, dmax, tmax, final = stats(track)
    line = (f'error mean {mean:.3f} m  max {dmax:.3f} m at t_s {tmax:.2f} (+{tmax - t0:.2f} s)  '
            f'final {final:.3f} m')
    if expect_stats:
        exp = [float(v) for v in expect_stats.split(',')]
        s_ok = all(abs(a - b) <= 0.0005 for a, b in zip((mean, dmax, final), exp))
        ok &= s_ok
        line += f'  expected {exp[0]:.3f}/{exp[1]:.3f}/{exp[2]:.3f}  {"PASS" if s_ok else "FAIL"}'
    print(line)
    print(f'waypoints ({len(waypoints)}) from {os.path.relpath(PARAMS, REPO)} via lawnmower(): '
          + ' '.join(f'({x:g},{y:g})' for x, y in waypoints))
    print(f'camera footprint: half-size x = z*{TAN_HALF_V:.4f}, y = z*{TAN_HALF_H:.4f}, yaw {math.degrees(yaw):g} deg')
    for name, (hit, dmin, tdmin) in first_covered(track, victims, yaw).items():
        hit_txt = 'never' if hit is None else f't_s {hit:.2f} (+{hit - t0:.2f} s)'
        line = (f'{name} at ({victims[name][0]:g}, {victims[name][1]:g}): first under footprint {hit_txt}; '
                f'closest {dmin:.2f} m at +{tdmin - t0:.2f} s')
        if expect_victims and name in expect_victims:
            e = expect_victims[name]
            v_ok = hit is not None and (abs(hit - e) <= tol_s or abs(hit - t0 - e) <= tol_s)
            ok &= v_ok
            line += f'  expected {e:.1f} s  {"PASS" if v_ok else "FAIL"}'
        print(line)
    print('ALL CHECKS PASS' if ok else 'SOME CHECKS FAILED: send this output, do not render')
    return ok


def render(track, victims, waypoints, yaw, speed, out_path, png_path):
    t = track['t_s'] - track['t_s'][0]
    span = float(t[-1])
    n_move = int(math.floor(span * FPS / speed)) + 1
    n_end = int(END_CARD_S * FPS)
    frame_t = np.minimum(np.arange(n_move) * speed / FPS, span)
    if frame_t[-1] < span:
        frame_t = np.append(frame_t, span)
        n_move += 1
    mean, dmax, _, final = stats(track)
    hits = {name: (None if h is None else h - track['t_s'][0])
            for name, (h, _, _) in first_covered(track, victims, yaw).items()}

    xs = np.concatenate([track['est_x'], track['true_x'], [p[0] for p in waypoints], [v[0] for v in victims.values()]])
    ys = np.concatenate([track['est_y'], track['true_y'], [p[1] for p in waypoints], [v[1] for v in victims.values()]])
    pad = 0.75
    x_lo, x_hi, y_lo, y_hi = xs.min() - pad, xs.max() + pad, ys.min() - pad, ys.max() + pad

    fig = plt.figure(figsize=(19.2, 10.8), dpi=100, facecolor='white')
    fig.text(0.5, 0.955, "GPS-denied search in simulation: the drone's own estimate vs ground truth",
             ha='center', va='center', fontsize=26, weight='bold')
    fig.text(0.5, 0.022, 'Gazebo + ArduPilot SITL, 28 Sep 2026. The drone navigates only on its own estimate '
             '(IMU + radar velocity + ToF height). Ground truth is used only for scoring. '
             'Bag phase4_lawnmower_2026-09-28.', ha='center', va='center', fontsize=13, color='#333333')

    ax = fig.add_axes([0.03, 0.115, 0.56, 0.79])
    ax.set_xlim(x_lo, x_hi)
    ax.set_ylim(y_lo, y_hi)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xticks(np.arange(math.ceil(x_lo), math.floor(x_hi) + 1, 1.0))
    ax.set_yticks(np.arange(math.ceil(y_lo), math.floor(y_hi) + 1, 1.0))
    ax.grid(True, color='#dddddd', linewidth=0.8)
    ax.tick_params(labelsize=14)
    ax.set_xlabel('x (m)', fontsize=18)
    ax.set_ylabel('y (m)', fontsize=18)
    wx = [p[0] for p in waypoints]
    wy = [p[1] for p in waypoints]
    ax.add_patch(Rectangle((min(wx), min(wy)), max(wx) - min(wx), max(wy) - min(wy),
                           fill=False, edgecolor='#bbbbbb', linewidth=1.2))
    ax.plot(wx, wy, 'o', color='#bbbbbb', markersize=7, zorder=2, label='Planned waypoints')
    for k, (px, py) in enumerate(waypoints, start=1):
        ax.annotate(str(k), (px, py), textcoords='offset points', xytext=(6, 6), fontsize=11, color='#888888')
    foot = Polygon(footprint_corners(0, 0, 1, yaw), closed=True, facecolor=C_FOOT, alpha=0.15,
                   edgecolor=C_FOOT, linewidth=1.5, zorder=3, label='Camera footprint')
    ax.add_patch(foot)
    truth_line, = ax.plot([], [], '-', color=C_TRUTH, linewidth=2.5, zorder=4, label='Ground truth (scoring only)')
    est_line, = ax.plot([], [], '--', color=C_EST, linewidth=2.5, zorder=5, label="Drone's own estimate (no GPS)")
    err_link, = ax.plot([], [], '-', color=C_ERR, linewidth=1.5, zorder=6)
    truth_dot, = ax.plot([], [], 'o', color=C_TRUTH, markersize=10, zorder=7)
    est_dot, = ax.plot([], [], 'o', color=C_EST, markersize=10, zorder=8)
    vic_marks = {}
    for name, (vx, vy) in victims.items():
        m, = ax.plot([vx], [vy], '*', color=C_GREY, markersize=24, zorder=9)
        ax.annotate(name, (vx, vy), textcoords='offset points', xytext=(10, -18), fontsize=13, color='#555555')
        vic_marks[name] = m
    ax.plot([], [], '*', color=C_GREY, markersize=16, label='Victim (red once under the camera)')
    ax.legend(loc='upper left', bbox_to_anchor=(1.01, 1.0), fontsize=14, frameon=False)

    gap = np.hypot(track['est_x'] - track['true_x'], track['est_y'] - track['true_y'])
    track['gap'] = gap
    gap_max = float(gap.max())
    ex = fig.add_axes([0.66, 0.36, 0.31, 0.30])
    ex.set_xlim(0, span)
    ex.set_ylim(0, 1.15 * gap_max if gap_max > 0 else 1.0)
    ex.set_xlabel('time (s)', fontsize=18)
    ex.set_ylabel('hypot(est − true) (m)', fontsize=18)
    ex.tick_params(labelsize=14)
    ex.grid(True, color='#eeeeee')
    err_curve, = ex.plot([], [], '-', color=C_ERR, linewidth=2.2)
    err_dot, = ex.plot([], [], 'o', color=C_ERR, markersize=8)
    mean_line = ex.axhline(0, color='#555555', linestyle='--', linewidth=1.2)

    live = fig.text(0.66, 0.24, '', fontsize=22, va='top', family='DejaVu Sans Mono')
    card = fig.text(0.31, 0.5, '', fontsize=21, ha='center', va='center', weight='bold', zorder=20,
                    bbox=dict(boxstyle='round,pad=0.8', facecolor='white', edgecolor='#333333', linewidth=2))
    card.set_visible(False)

    cols = {c: np.interp(frame_t, t, track[c]) for c in ('est_x', 'est_y', 'true_x', 'true_y', 'true_z', 'd_m', 'gap')}

    def update(k):
        i = min(k, n_move - 1)
        now = frame_t[i]
        j = int(np.searchsorted(t, now, side='right'))
        tx = np.append(track['true_x'][:j], cols['true_x'][i])
        ty = np.append(track['true_y'][:j], cols['true_y'][i])
        exs = np.append(track['est_x'][:j], cols['est_x'][i])
        eys = np.append(track['est_y'][:j], cols['est_y'][i])
        truth_line.set_data(tx, ty)
        est_line.set_data(exs, eys)
        truth_dot.set_data([cols['true_x'][i]], [cols['true_y'][i]])
        est_dot.set_data([cols['est_x'][i]], [cols['est_y'][i]])
        err_link.set_data([cols['true_x'][i], cols['est_x'][i]], [cols['true_y'][i], cols['est_y'][i]])
        foot.set_xy(footprint_corners(cols['true_x'][i], cols['true_y'][i], cols['true_z'][i], yaw))
        for name, mark in vic_marks.items():
            mark.set_color(C_ERR if hits[name] is not None and hits[name] <= now else C_GREY)
        dt_seen = np.append(track['gap'][:j], cols['gap'][i])
        err_curve.set_data(np.append(t[:j], now), dt_seen)
        err_dot.set_data([now], [cols['gap'][i]])
        mean_so_far = float(dt_seen.mean())
        mean_line.set_ydata([mean_so_far, mean_so_far])
        live.set_text(f't                     {now:6.1f} s\nhypot(est − true) now {cols["gap"][i]:6.2f} m\n'
                      f'mean so far           {mean_so_far:6.2f} m\nmax so far            {float(dt_seen.max()):6.2f} m')
        if k >= n_move:
            card.set_text(f'{span:.0f} s  ·  mean error {mean:.2f} m  ·  max {dmax:.2f} m  ·  final {final:.2f} m\n'
                          '10/10 waypoints (flight log)')
            card.set_visible(True)
        return []

    anim = FuncAnimation(fig, update, frames=n_move + n_end, blit=False)
    writer = FFMpegWriter(fps=FPS, codec='libx264', extra_args=['-pix_fmt', 'yuv420p', '-crf', '18'])
    anim.save(out_path, writer=writer, dpi=100)
    update(n_move + n_end - 1)
    fig.savefig(png_path, dpi=100, facecolor='white')
    plt.close(fig)
    print(f'wrote {out_path} ({n_move + n_end} frames) and {png_path}')


def main():
    ap = argparse.ArgumentParser(description='Animate onboard estimate vs ground truth from a track CSV.')
    ap.add_argument('--csv', required=True)
    ap.add_argument('--victims', required=True)
    ap.add_argument('--speed', type=float, default=4.0)
    ap.add_argument('--out', default='drift_lawnmower_4x.mp4')
    ap.add_argument('--png', default='drift_lawnmower_final.png')
    ap.add_argument('--yaw-deg', type=float, default=0.0)
    ap.add_argument('--expect-stats', default='')
    ap.add_argument('--expect-victims', default='')
    ap.add_argument('--victim-tol-s', type=float, default=0.3)
    ap.add_argument('--check-only', action='store_true')
    args = ap.parse_args()
    if args.speed <= 0:
        sys.exit('--speed must be positive')
    track = load_track(args.csv)
    with open(args.victims) as f:
        victims = {k: (float(v[0]), float(v[1])) for k, v in json.load(f).items()}
    waypoints = load_waypoints()
    yaw = math.radians(args.yaw_deg)
    expect_victims = parse_expect_victims(args.expect_victims) if args.expect_victims else {}
    ok = run_checks(track, victims, waypoints, yaw, args.expect_stats, expect_victims, args.victim_tol_s)
    if args.check_only:
        sys.exit(0 if ok else 1)
    render(track, victims, waypoints, yaw, args.speed, args.out, args.png)


if __name__ == '__main__':
    main()
