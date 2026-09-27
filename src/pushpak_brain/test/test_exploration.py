import math

import pytest

from pushpak_brain.exploration import advance, lawnmower, p_velocity, path_length, validate_gains


def test_lawnmower_demo_area():
    wps = lawnmower(-3.0, 3.0, -3.0, 3.0, 1.5)
    assert wps == [(-3.0, -3.0), (3.0, -3.0), (3.0, -1.5), (-3.0, -1.5), (-3.0, 0.0), (3.0, 0.0),
                   (3.0, 1.5), (-3.0, 1.5), (-3.0, 3.0), (3.0, 3.0)]


def test_lawnmower_adds_final_lane_at_y_max():
    wps = lawnmower(0.0, 1.0, 0.0, 1.0, 0.4)
    assert sorted({y for _, y in wps}) == pytest.approx([0.0, 0.4, 0.8, 1.0])


@pytest.mark.parametrize('args', [(1.0, 1.0, 0.0, 1.0, 0.5), (0.0, 1.0, 1.0, 0.0, 0.5), (0.0, 1.0, 0.0, 1.0, 0.0)])
def test_lawnmower_rejects_bad_area(args):
    with pytest.raises(ValueError):
        lawnmower(*args)


def test_path_length_demo_area():
    wps = lawnmower(-3.0, 3.0, -3.0, 3.0, 1.5)
    assert path_length((0.0, 0.0), wps) == pytest.approx(3.0 * math.sqrt(2.0) + 36.0)


@pytest.mark.parametrize('kp,v', [(float('nan'), 0.7), (0.8, float('nan')), (0.0, 0.7), (0.8, -1.0), (None, 0.7)])
def test_validate_gains_rejects(kp, v):
    with pytest.raises(ValueError):
        validate_gains(kp, v)


def test_p_velocity_saturates_far_away():
    vx, vy = p_velocity((0.0, 0.0), (3.0, 4.0), 0.8, 0.7)
    assert math.hypot(vx, vy) == pytest.approx(0.7)
    assert (vx, vy) == pytest.approx((0.42, 0.56))


def test_p_velocity_linear_near_target():
    assert p_velocity((0.0, 0.0), (0.5, 0.0), 0.8, 0.7) == pytest.approx((0.4, 0.0))


def test_p_velocity_zero_at_target():
    assert p_velocity((1.0, 2.0), (1.0, 2.0), 0.8, 0.7) == (0.0, 0.0)


def test_advance_skips_all_reached_waypoints():
    wps = [(0.0, 0.0), (0.1, 0.0), (5.0, 0.0)]
    assert advance(wps, 0, (0.05, 0.0), 0.25) == 2
    assert advance(wps, 2, (0.05, 0.0), 0.25) == 2
    assert advance(wps, 3, (0.0, 0.0), 0.25) == 3


def test_closed_loop_covers_demo_area_within_budget():
    kp, v_max, accept, dt = 0.8, 0.7, 0.25, 0.05
    wps = lawnmower(-3.0, 3.0, -3.0, 3.0, 1.5)
    pos, idx, t, peak = (0.0, 0.0), 0, 0.0, 0.0
    while idx < len(wps) and t < 120.0:
        idx = advance(wps, idx, pos, accept)
        if idx == len(wps):
            break
        vx, vy = p_velocity(pos, wps[idx], kp, v_max)
        peak = max(peak, math.hypot(vx, vy))
        pos = (pos[0] + vx * dt, pos[1] + vy * dt)
        t += dt
    assert idx == len(wps)
    assert peak <= v_max + 1e-9
    assert t < 80.0
    print(f'closed-loop coverage time {t:.1f} s')
