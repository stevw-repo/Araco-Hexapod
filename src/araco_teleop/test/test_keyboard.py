# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

import json

from araco_teleop.keyboard import decode_snapshot
from araco_teleop.keyboard import encode_snapshot
from araco_teleop.keyboard import KeyboardState
import pytest


KEYS = {
    'w': {'field': 'vx', 'value': 0.05},
    's': {'field': 'vx', 'value': -0.05},
    'a': {'field': 'vy', 'value': 0.05},
    'd': {'field': 'vy', 'value': -0.05},
    'q': {'field': 'wz', 'value': 0.3},
    'e': {'field': 'wz', 'value': -0.3},
}


def snapshot(sequence, focused=True, pressed=frozenset()):
    return encode_snapshot(sequence, focused, set(pressed))


def test_adapter_starts_released_and_requires_deadman():
    state = KeyboardState(KEYS)
    assert not state.sample(now=0.0).active
    state.accept_snapshot(snapshot(1, pressed={'w'}), now=1.0)
    assert not state.sample(now=1.01).active


def test_focused_deadman_state_is_short_lived():
    state = KeyboardState(KEYS, timeout_s=0.12)
    state.accept_snapshot(snapshot(1, pressed={'space', 'w'}), now=1.0)
    assert state.sample(now=1.11).vx == 0.05
    assert not state.sample(now=1.121).active


def test_multiple_held_keys_combine_and_neutral_retains_deadman_authority():
    state = KeyboardState(KEYS)
    state.accept_snapshot(
        snapshot(1, pressed={'space', 'w', 'a', 'q'}), now=1.0)
    intent = state.sample(now=1.01)
    assert (intent.vx, intent.vy, intent.wz) == (0.05, 0.05, 0.3)

    state.accept_snapshot(
        snapshot(2, pressed={'space', 'w', 's'}), now=1.02)
    neutral = state.sample(now=1.03)
    assert neutral.active
    assert (neutral.vx, neutral.vy, neutral.wz) == (0.0, 0.0, 0.0)


def test_releasing_directions_does_not_release_source_until_deadman_releases():
    state = KeyboardState(KEYS)
    state.accept_snapshot(
        snapshot(1, pressed={'space', 'w', 'a'}), now=1.0)
    assert state.sample(now=1.01).active

    state.accept_snapshot(snapshot(2, pressed={'space', 'a'}), now=1.02)
    lateral = state.sample(now=1.03)
    assert lateral.active
    assert (lateral.vx, lateral.vy) == (0.0, 0.05)

    state.accept_snapshot(snapshot(3, pressed={'space'}), now=1.04)
    neutral = state.sample(now=1.05)
    assert neutral.active
    assert (neutral.vx, neutral.vy, neutral.wz) == (0.0, 0.0, 0.0)

    state.accept_snapshot(snapshot(4, pressed=set()), now=1.06)
    assert not state.sample(now=1.07).active


def test_focus_loss_and_explicit_invalidation_release_immediately():
    state = KeyboardState(KEYS)
    state.accept_snapshot(
        snapshot(1, focused=False, pressed={'space', 'q'}), now=1.0)
    assert not state.sample(now=1.01).active

    state.accept_snapshot(snapshot(2, pressed={'space', 'q'}), now=1.02)
    assert state.sample(now=1.03).active
    state.invalidate()
    assert not state.sample(now=1.03).active


def test_unregistered_key_and_malformed_payload_fail_closed():
    state = KeyboardState(KEYS)
    with pytest.raises(ValueError, match='unregistered'):
        state.accept_snapshot(snapshot(1, pressed={'space', 'x'}), now=1.0)
    with pytest.raises(ValueError, match='valid JSON'):
        state.accept_snapshot('not-json', now=1.0)


def test_protocol_codec_is_deterministic_and_strict():
    payload = snapshot(7, pressed={'w', 'space'})
    assert payload == (
        '{"focused":true,"pressed":["space","w"],'
        '"protocol":"araco.keyboard-state.v1","sequence":7}'
    )
    assert decode_snapshot(payload) == {
        'sequence': 7,
        'focused': True,
        'pressed': {'space', 'w'},
    }
    invalid = json.loads(payload)
    invalid['extra'] = True
    with pytest.raises(ValueError, match='unexpected fields'):
        decode_snapshot(json.dumps(invalid))
    with pytest.raises(ValueError, match='unsigned 64-bit'):
        encode_snapshot(2**64, True, set())
