# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

from araco_teleop.keyboard import KeyboardState


KEYS = {
    'w': {'field': 'vx', 'value': 0.05},
    'a': {'field': 'vy', 'value': 0.05},
    'q': {'field': 'wz', 'value': 0.3},
}


def test_adapter_starts_released_and_requires_deadman():
    state = KeyboardState(KEYS)
    assert not state.sample(now=0.0).active
    state.accept('w', now=1.0)
    assert not state.sample(now=1.01).active


def test_deadman_and_motion_key_are_short_lived():
    state = KeyboardState(KEYS, timeout_s=0.12)
    state.accept(' ', now=1.0)
    state.accept('w', now=1.01)
    assert state.sample(now=1.02).vx == 0.05
    assert not state.sample(now=1.14).active


def test_unknown_key_releases():
    state = KeyboardState(KEYS)
    state.accept(' ', now=1.0)
    state.accept('q', now=1.01)
    assert state.sample(now=1.02).wz == 0.3
    state.accept('x', now=1.03)
    assert not state.sample(now=1.04).active
