# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

import math

from araco_teleop.joystick import JoystickState
import pytest


MAPPING = {
    'axis_count': 6,
    'button_count': 12,
    'deadzone': 0.08,
    'heartbeat_timeout_s': 0.12,
    'roll_left_button': 2,
    'roll_right_button': 3,
    'roll_button_scale_rad': 0.15,
    'axes': {
        'forward': {'index': 1, 'invert': False, 'scale': 0.2},
        'lateral': {'index': 0, 'invert': False, 'scale': 0.2},
        'walking_yaw': {'index': 3, 'invert': False, 'scale': 1.2},
        'body_height': {
            'index': 2, 'positive_end_is_zero': True, 'range_m': 0.03},
        'body_pitch': {'index': 5, 'invert': True, 'scale': 0.15},
        'posture_yaw': {'index': 4, 'invert': False, 'scale': 0.2},
    },
}


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
    assert intent.vx == -0.2
    assert intent.vy == 0.2
    assert intent.wz == 1.2
    assert intent.body_z == -0.03
    assert intent.body_roll == 0.0
    assert intent.body_pitch == 0.15
    assert intent.body_yaw == 0.2


def test_height_axis_maps_negative_end_to_full_lowering():
    state = JoystickState(MAPPING)
    axes, buttons = report()
    axes[2] = -1.0
    state.accept(axes, buttons, now=1.0)
    assert state.sample(now=1.01).body_z == -0.03


def test_hat_vertical_maps_directly_and_buttons_one_and_two_are_unused():
    state = JoystickState(MAPPING)
    axes, buttons = report(trigger=True)
    axes[5] = 1.0
    buttons[1] = 1
    state.accept(axes, buttons, now=1.0)
    intent = state.sample(now=1.01)
    assert intent.body_roll == 0.0
    assert intent.body_pitch == -0.15


def test_dedicated_roll_buttons_map_left_right_and_cancel_together():
    state = JoystickState(MAPPING)
    axes, buttons = report(roll_left=True)
    state.accept(axes, buttons, now=1.0)
    assert state.sample(now=1.01).body_roll == -0.15
    buttons[2] = 0
    buttons[3] = 1
    state.accept(axes, buttons, now=1.02)
    assert state.sample(now=1.03).body_roll == 0.15
    buttons[2] = 1
    state.accept(axes, buttons, now=1.04)
    assert state.sample(now=1.05).body_roll == 0.0


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
    assert math.isclose(state.sample(now=1.03).vx, 0.1)
    assert not state.sample(now=1.141).active
    state.accept(axes, buttons, now=2.0)
    state.invalidate()
    assert not state.sample(now=2.0).active


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
