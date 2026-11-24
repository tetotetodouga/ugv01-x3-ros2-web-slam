"""Pure encoder math; these tests never open a serial port or publish ROS data."""
import math

import pytest

from ugv01_room_explore.wheel_odometry import WheelOdometry


def test_cumulative_distance_survives_a_telemetry_pause():
    state = WheelOdometry()
    state.update(100, 100, 0.0)
    assert state.update(105, 105, 0.7)
    assert state.x == pytest.approx(0.05)
    assert state.y == 0


def test_small_steps_are_not_discarded_after_advancing_baseline():
    state = WheelOdometry()
    state.update(0, 0, 0.0)
    for i in range(1, 101):
        state.update(i * 0.01, i * 0.01, i * 0.01)
    assert state.x == pytest.approx(0.01)


def test_turn_in_place_preserves_center():
    state = WheelOdometry()
    state.update(0, 0, 0.0)
    state.update(-1, 1, 0.1)
    assert state.x == 0
    assert state.y == 0
    assert state.yaw == pytest.approx(0.02 / 0.19602)


def test_quantized_velocity_uses_a_time_window():
    state = WheelOdometry()
    state.update(0, 0, 0.0)
    for i in range(1, 31):
        state.update(i // 10, i // 10, i * 0.01)
    assert state.vx == pytest.approx(0.03 / 0.25, abs=0.02)
    assert state.vx < 0.2  # one 1 cm tick at 100 Hz must not report 1 m/s
    assert state.x == pytest.approx(0.03)
    for i in range(31, 61):
        state.update(3, 3, i * 0.01)
    assert state.vx == 0


def test_encoder_reset_does_not_teleport_robot():
    state = WheelOdometry()
    state.update(1000, 1000, 0.0)
    assert not state.update(0, 0, 0.02)
    assert state.x == 0
    assert state.update(1, 1, 0.12)
    assert state.x == pytest.approx(0.01)


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_bad_encoder_samples_do_not_poison_state(bad):
    state = WheelOdometry()
    state.update(0, 0, 0.0)
    assert not state.update(bad, 1, 0.1)
    assert state.update(1, 1, 0.2)
    assert state.x == pytest.approx(0.01)


def test_out_of_order_sample_is_ignored():
    state = WheelOdometry()
    state.update(0, 0, 1.0)
    assert not state.update(1, 1, 0.9)
    assert state.update(1, 1, 1.1)
    assert state.x == pytest.approx(0.01)
