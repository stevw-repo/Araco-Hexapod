# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

"""Pure key-to-candidate mapping for the simulator keyboard adapter."""

from __future__ import annotations

from dataclasses import dataclass
import json
import time


PROTOCOL = 'araco.keyboard-state.v1'


@dataclass(frozen=True)
class KeyIntent:
    """One validated planar key intent."""

    active: bool = False
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


class KeyboardState:
    """Deadman-gated full key state with a fail-closed heartbeat."""

    def __init__(
        self,
        keys: dict[str, dict[str, float]],
        deadman_key: str = 'space',
        timeout_s: float = 0.12,
    ):
        self._keys = keys
        self._deadman_key = deadman_key
        self._timeout_s = timeout_s
        self._fresh_until = 0.0
        self._pressed = frozenset()
        self._focused = False

    def accept_snapshot(self, payload: str, now: float | None = None) -> None:
        """Accept one validated full-state heartbeat from the focused UI."""
        stamp = time.monotonic() if now is None else now
        snapshot = decode_snapshot(payload)
        allowed = set(self._keys) | {self._deadman_key}
        if not snapshot['pressed'] <= allowed:
            raise ValueError('keyboard snapshot contains an unregistered key')
        self._pressed = frozenset(snapshot['pressed'])
        self._focused = snapshot['focused']
        self._fresh_until = stamp + self._timeout_s

    def invalidate(self) -> None:
        """Release every key immediately."""
        self._fresh_until = 0.0
        self._pressed = frozenset()
        self._focused = False

    def sample(self, now: float | None = None) -> KeyIntent:
        """Return a combined intent only for a fresh focused deadman state."""
        stamp = time.monotonic() if now is None else now
        if (
            stamp > self._fresh_until
            or not self._focused
            or self._deadman_key not in self._pressed
        ):
            return KeyIntent()
        values = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}
        for key in self._pressed:
            mapping = self._keys.get(key)
            if mapping is not None:
                values[mapping['field']] += float(mapping['value'])
        # Holding the deadman retains source ownership even when the operator
        # has released every direction key. The zero command is an active
        # stand, allowing direction changes without a new safety enable cycle.
        return KeyIntent(True, **values)


def encode_snapshot(sequence: int, focused: bool, pressed: set[str]) -> str:
    """Encode a deterministic keyboard-state heartbeat."""
    if (
        not isinstance(sequence, int)
        or isinstance(sequence, bool)
        or sequence < 0
        or sequence > 0xffffffffffffffff
    ):
        raise ValueError('keyboard sequence must be an unsigned 64-bit integer')
    return json.dumps({
        'protocol': PROTOCOL,
        'sequence': sequence,
        'focused': focused,
        'pressed': sorted(pressed),
    }, sort_keys=True, separators=(',', ':'))


def decode_snapshot(payload: str) -> dict[str, object]:
    """Decode a strict keyboard-state heartbeat."""
    try:
        value = json.loads(payload)
    except json.JSONDecodeError as error:
        raise ValueError('keyboard snapshot is not valid JSON') from error
    if not isinstance(value, dict) or set(value) != {
        'protocol', 'sequence', 'focused', 'pressed'
    }:
        raise ValueError('keyboard snapshot has unexpected fields')
    if value['protocol'] != PROTOCOL:
        raise ValueError('keyboard snapshot protocol mismatch')
    if (
        not isinstance(value['sequence'], int)
        or isinstance(value['sequence'], bool)
        or value['sequence'] < 0
        or value['sequence'] > 0xffffffffffffffff
    ):
        raise ValueError('keyboard snapshot sequence is invalid')
    if not isinstance(value['focused'], bool):
        raise ValueError('keyboard snapshot focus flag is invalid')
    pressed = value['pressed']
    if (
        not isinstance(pressed, list)
        or any(not isinstance(key, str) or not key for key in pressed)
        or len(pressed) != len(set(pressed))
    ):
        raise ValueError('keyboard snapshot pressed-key set is invalid')
    return {
        'sequence': value['sequence'],
        'focused': value['focused'],
        'pressed': set(pressed),
    }
