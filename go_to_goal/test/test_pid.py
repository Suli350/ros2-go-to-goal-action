import math

import pytest

from go_to_goal.pid import PID, clamp, normalize_angle


def test_proportional_only():
    pid = PID(kp=2.0)
    assert pid.update(1.5, 0.1) == pytest.approx(3.0)


def test_output_is_limited():
    pid = PID(kp=100.0, output_limit=2.0)
    assert pid.update(1.0, 0.1) == pytest.approx(2.0)
    assert pid.update(-1.0, 0.1) == pytest.approx(-2.0)


def test_integral_accumulates_and_clamps():
    pid = PID(kp=0.0, ki=1.0, integral_limit=0.25)
    for _ in range(10):
        out = pid.update(1.0, 0.1)
    assert out == pytest.approx(0.25)


def test_derivative_and_reset():
    pid = PID(kp=0.0, kd=1.0)
    assert pid.update(0.0, 0.1) == pytest.approx(0.0)
    assert pid.update(1.0, 0.1) == pytest.approx(10.0)
    pid.reset()
    assert pid.update(5.0, 0.1) == pytest.approx(0.0)


def test_closed_loop_converges():
    # Simulate a first-order plant x' = u and drive x to 1.0
    pid = PID(kp=2.0, ki=0.5, output_limit=5.0)
    x, dt = 0.0, 0.01
    for _ in range(2000):
        x += pid.update(1.0 - x, dt) * dt
    assert x == pytest.approx(1.0, abs=1e-3)


def test_helpers():
    assert clamp(5, 0, 1) == 1
    assert normalize_angle(2.5 * math.pi) == pytest.approx(0.5 * math.pi)


def test_bad_dt():
    with pytest.raises(ValueError):
        PID(1.0).update(1.0, 0.0)
