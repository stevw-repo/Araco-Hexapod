# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

import math

from araco_system_tests.gate1 import quaternion_roll_pitch
from araco_system_tests.gate1 import rms
from araco_system_tests.gate1 import state_path
from araco_system_tests.gate1 import vector_norm
import pytest


def test_vector_norm_and_rms():
    assert vector_norm(3.0, 4.0, 0.0) == 5.0
    assert rms([3.0, 4.0]) == math.sqrt(12.5)
    with pytest.raises(ValueError):
        rms([])


def test_quaternion_roll_pitch():
    roll, pitch = quaternion_roll_pitch(0.0, 0.0, 0.0, 1.0)
    assert roll == 0.0
    assert pitch == 0.0
    with pytest.raises(ValueError):
        quaternion_roll_pitch(0.0, 0.0, 0.0, 0.0)


def test_state_path_collapses_only_adjacent_duplicates():
    assert state_path([0, 0, 1, 1, 2, 2]) == [0, 1, 2]
    assert state_path([0, 1, 0]) == [0, 1, 0]
