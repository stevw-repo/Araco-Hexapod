# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

"""Pure key-to-candidate mapping for the simulator keyboard adapter."""

from __future__ import annotations

from dataclasses import dataclass
import time


@dataclass(frozen=True)
class KeyIntent:
    """One validated planar key intent."""

    active: bool = False
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


class KeyboardState:
    """Deadman-gated, automatically releasing keyboard state."""

    def __init__(self, keys: dict[str, dict[str, float]], timeout_s: float = 0.12):
        self._keys = keys
        self._timeout_s = timeout_s
        self._deadman_until = 0.0
        self._intent_until = 0.0
        self._intent = KeyIntent()

    def accept(self, key: str, now: float | None = None) -> None:
        """Accept one key event; space arms the short-lived deadman window."""
        stamp = time.monotonic() if now is None else now
        if key == ' ':
            self._deadman_until = stamp + self._timeout_s
            return
        mapping = self._keys.get(key)
        if mapping is None:
            self._intent = KeyIntent()
            self._intent_until = stamp
            return
        values = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}
        values[mapping['field']] = float(mapping['value'])
        self._intent = KeyIntent(True, **values)
        self._intent_until = stamp + self._timeout_s

    def sample(self, now: float | None = None) -> KeyIntent:
        """Return active intent only while both deadman and motion key are fresh."""
        stamp = time.monotonic() if now is None else now
        if stamp <= self._deadman_until and stamp <= self._intent_until:
            return self._intent
        return KeyIntent()
