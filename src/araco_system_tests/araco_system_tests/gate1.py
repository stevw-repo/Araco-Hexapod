# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

"""Pure numerical helpers for Gate 1 stable-hold scoring."""

from __future__ import annotations

import math


def vector_norm(x, y, z):
    """Return the finite Euclidean norm of a three-vector."""
    return math.sqrt(x * x + y * y + z * z)


def quaternion_roll_pitch(x, y, z, w):
    """Return roll and pitch from a finite normalized quaternion."""
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('non-finite quaternion')
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        raise ValueError('zero quaternion')
    x, y, z, w = (value / norm for value in values)
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return roll, math.asin(sin_pitch)


def rms(values):
    """Return root-mean-square and reject an empty or non-finite sequence."""
    if not values or not all(math.isfinite(value) for value in values):
        raise ValueError('RMS requires non-empty finite input')
    return math.sqrt(sum(value * value for value in values) / len(values))


def state_path(states):
    """Collapse adjacent duplicate safety states."""
    result = []
    for state in states:
        if not result or result[-1] != state:
            result.append(state)
    return result
