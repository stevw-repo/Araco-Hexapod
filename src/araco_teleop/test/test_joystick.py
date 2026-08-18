# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

import math

from araco_teleop.joystick import JoystickState
import pytest


MAPPING = {
    'axis_count': 6,
    'button_count': 12,
    'publication_rate_hz': 50,
    'deadzone': 0.08,
    'heartbeat_timeout_s': 0.12,
    'control_response': {
        'kind': 'legacy_p_only_time_invariant',
        'reference_period_s': 0.005,
        'normal_error_fraction': 0.02,
        'height_error_fraction': 0.01,
    },
    'roll_left_button': 2,
    'roll_right_button': 3,
    'roll_button_scale_rad': 0.15,
    'axes': {
        'forward': {'index': 1, 'invert': False, 'scale': 0.24},
        'lateral': {'index': 0, 'invert': False, 'scale': 0.24},
        'walking_yaw': {'index': 3, 'invert': False, 'scale': 1.2},
        'body_height': {
            'index': 2, 'positive_end_is_zero': True, 'range_m': 0.03},
        'body_pitch': {'index': 5, 'invert': True, 'scale': 0.15},
        'posture_yaw': {'index': 4, 'invert': False, 'scale': 0.2},
        'gimbal_yaw': {
            'index': 4, 'invert': False, 'scale': math.pi / 10.0},
    },
}

NORMAL_ALPHA_20MS = 1.0 - (1.0 - 0.02) ** 4
HEIGHT_ALPHA_20MS = 1.0 - (1.0 - 0.01) ** 4


def report(trigger=False, roll_left=False, roll_right=False):
    buttons = [0] * 12
    buttons[0] = int(trigger)
    buttons[2] = int(roll_left)
    buttons[3] = int(roll_right)
    return [0.0, 0.0, 1.0, 0.0, 0.0, 0.0], buttons


def test_starts_inactive_then_fresh_neutral_report_is_active_without_trigger():
    state = JoystickState(MAPPING)
    assert not state.sample(now=1.0).active
    axes, buttons = report()
    state.accept(axes, buttons, now=1.0)
    intent = state.sample(now=1.01)
    assert intent.active
    assert intent == type(intent)(active=True)


def test_legacy_axes_and_polarities_map_to_bounded_motion():
    state = JoystickState(MAPPING)
    axes = [1.0, -1.0, -1.0, 1.0, 1.0, -1.0]
    buttons = [0] * 12
    state.accept(axes, buttons, now=1.0)
    intent = state.sample(now=1.01)
    assert math.isclose(
        intent.vx, -0.24 / math.sqrt(2.0) * NORMAL_ALPHA_20MS)
    assert math.isclose(
        intent.vy, 0.24 / math.sqrt(2.0) * NORMAL_ALPHA_20MS)
    assert math.isclose(intent.wz, 1.2 * NORMAL_ALPHA_20MS)
    assert math.isclose(intent.body_z, -0.03 * HEIGHT_ALPHA_20MS)
    assert intent.body_roll == 0.0
    assert math.isclose(intent.body_pitch, 0.15 * NORMAL_ALPHA_20MS)
    assert math.isclose(intent.body_yaw, 0.2 * NORMAL_ALPHA_20MS)
    assert math.isclose(
        intent.gimbal_yaw, math.pi / 10.0 * NORMAL_ALPHA_20MS)
    assert math.isclose(
        intent.body_yaw / 0.2,
        intent.gimbal_yaw / (math.pi / 10.0))


def test_translation_is_radially_normalized_like_legacy_input():
    state = JoystickState(MAPPING)
    axes, buttons = report()
    axes[0] = 1.0
    axes[1] = 1.0
    state.accept(axes, buttons, now=1.0)
    intent = state.sample(now=1.01)
    assert math.isclose(
        math.hypot(intent.vx, intent.vy), 0.24 * NORMAL_ALPHA_20MS)
    assert math.isclose(
        intent.vx, 0.24 / math.sqrt(2.0) * NORMAL_ALPHA_20MS)
    assert math.isclose(
        intent.vy, 0.24 / math.sqrt(2.0) * NORMAL_ALPHA_20MS)


def test_height_axis_maps_negative_end_to_full_lowering():
    state = JoystickState(MAPPING)
    axes, buttons = report()
    axes[2] = -1.0
    state.accept(axes, buttons, now=1.0)
    intent = state.sample(now=1.01)
    assert math.isclose(intent.body_z, -0.03 * HEIGHT_ALPHA_20MS)
    assert intent.controls_centered()


def test_hat_vertical_maps_directly_and_buttons_one_and_two_are_unused():
    state = JoystickState(MAPPING)
    axes, buttons = report(trigger=True)
    axes[5] = 1.0
    buttons[1] = 1
    state.accept(axes, buttons, now=1.0)
    intent = state.sample(now=1.01)
    assert intent.body_roll == 0.0
    assert math.isclose(intent.body_pitch, -0.15 * NORMAL_ALPHA_20MS)


def test_dedicated_roll_buttons_map_left_right_and_cancel_together():
    state = JoystickState(MAPPING)
    axes, buttons = report(roll_left=True)
    state.accept(axes, buttons, now=1.0)
    first = state.sample(now=1.01).body_roll
    assert math.isclose(first, -0.15 * NORMAL_ALPHA_20MS)
    buttons[2] = 0
    buttons[3] = 1
    state.accept(axes, buttons, now=1.02)
    second = state.sample(now=1.03).body_roll
    assert math.isclose(
        second, first + NORMAL_ALPHA_20MS * (0.15 - first))
    buttons[2] = 1
    state.accept(axes, buttons, now=1.04)
    third = state.sample(now=1.05).body_roll
    assert math.isclose(second * (1.0 - NORMAL_ALPHA_20MS), third)


def test_roll_buttons_must_be_distinct_and_in_range():
    mapping = dict(MAPPING)
    mapping['roll_left_button'] = mapping['roll_right_button']
    with pytest.raises(ValueError, match='must be distinct'):
        JoystickState(mapping)
    mapping = dict(MAPPING)
    mapping['roll_right_button'] = 12
    with pytest.raises(ValueError, match='out of range'):
        JoystickState(mapping)


def test_deadzone_rescales_and_timeout_or_invalidation_release():
    state = JoystickState(MAPPING)
    axes, buttons = report()
    axes[1] = 0.04
    state.accept(axes, buttons, now=1.0)
    assert state.sample(now=1.01).vx == 0.0
    axes[1] = 0.54
    state.accept(axes, buttons, now=1.02)
    assert math.isclose(
        state.sample(now=1.03).vx, 0.12 * NORMAL_ALPHA_20MS)
    assert not state.sample(now=1.141).active
    state.accept(axes, buttons, now=2.0)
    state.invalidate()
    assert not state.sample(now=2.0).active


def test_legacy_response_is_time_step_independent():
    coarse = JoystickState(MAPPING)
    fine = JoystickState(MAPPING)
    neutral_axes, buttons = report()
    coarse.accept(neutral_axes, buttons, now=1.0)
    fine.accept(neutral_axes, buttons, now=1.0)
    coarse.sample(now=1.0)
    fine.sample(now=1.0)

    step_axes = neutral_axes.copy()
    step_axes[1] = 1.0
    coarse.accept(step_axes, buttons, now=1.0)
    fine.accept(step_axes, buttons, now=1.0)
    coarse_output = coarse.sample(now=1.02).vx
    for stamp in (1.005, 1.010, 1.015, 1.020):
        fine_output = fine.sample(now=stamp).vx
    assert math.isclose(coarse_output, fine_output, rel_tol=1.0e-12)


def test_every_control_uses_response_filter_and_axis4_is_shared():
    state = JoystickState(MAPPING)
    axes, buttons = report(roll_right=True)
    axes[:] = [1.0, 1.0, -1.0, 1.0, 1.0, -1.0]
    state.accept(axes, buttons, now=1.0)
    intent = state.sample(now=1.02)
    normal_targets = (
        0.24 / math.sqrt(2.0), 0.24 / math.sqrt(2.0), 1.2,
        0.15, 0.15, 0.2, math.pi / 10.0)
    actual = (
        intent.vx, intent.vy, intent.wz, intent.body_roll,
        intent.body_pitch, intent.body_yaw, intent.gimbal_yaw)
    for value, target in zip(actual, normal_targets):
        assert math.isclose(value, target * NORMAL_ALPHA_20MS)
    assert math.isclose(intent.body_z, -0.03 * HEIGHT_ALPHA_20MS)


def test_release_decays_smoothly_and_centered_recovery_clears_residual_motion():
    state = JoystickState(MAPPING)
    axes, buttons = report()
    state.accept(axes, buttons, now=1.0)
    state.sample(now=1.0)
    axes[1] = 1.0
    state.accept(axes, buttons, now=1.01)
    moving = state.sample(now=1.02)
    axes[1] = 0.0
    state.accept(axes, buttons, now=1.02)
    assert state.raw_controls_centered()
    decaying = state.sample(now=1.04)
    assert math.isclose(
        decaying.vx, moving.vx * (1.0 - NORMAL_ALPHA_20MS))
    assert not decaying.controls_centered()
    state.reset_motion_response()
    assert state.sample(now=1.06).controls_centered()


def test_axis4_outputs_must_share_physical_axis_and_polarity():
    mapping = dict(MAPPING)
    mapping['axes'] = dict(MAPPING['axes'])
    mapping['axes']['gimbal_yaw'] = {
        'index': 3, 'invert': False, 'scale': math.pi / 10.0}
    with pytest.raises(ValueError, match='must share one physical axis'):
        JoystickState(mapping)


def test_malformed_reports_fail_closed():
    state = JoystickState(MAPPING)
    axes, buttons = report()
    with pytest.raises(ValueError, match='dimensions'):
        state.accept(axes[:-1], buttons, now=1.0)
    with pytest.raises(ValueError, match='invalid axis'):
        state.accept([math.nan] + axes[1:], buttons, now=1.0)
    invalid_buttons = buttons.copy()
    invalid_buttons[4] = 2
    with pytest.raises(ValueError, match='invalid button'):
        state.accept(axes, invalid_buttons, now=1.0)
    assert not state.sample(now=1.0).active
