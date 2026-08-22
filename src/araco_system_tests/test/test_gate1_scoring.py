# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

import math

from araco_system_tests.gate1 import classify_launch_log
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


GZ_SEGFAULT = (
    '[ERROR] [gazebo-1]: process has died [pid 1, exit code 139, '
    "cmd 'ruby /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz sim -r -s'].")
NODE_CRASH = (
    '[ERROR] [locomotion_node-5]: process has died [pid 2, exit code 1, '
    "cmd 'locomotion_node'].")
NODE_SIGINT = (
    '[ERROR] [locomotion_node-5]: process has died [pid 2, exit code -2, '
    "cmd 'locomotion_node'].")


def test_gz_shutdown_crash_is_classified_not_counted_as_failure():
    result = classify_launch_log(GZ_SEGFAULT, True, True, False)
    assert result['unclassified'] == []
    assert len(result['shutdown_defect']) == 1


def test_shutdown_defect_requires_completed_scoring_and_stop_request():
    # A crash before scoring finished, or with no stop requested, must block.
    assert classify_launch_log(GZ_SEGFAULT, False, True, False)['unclassified']
    assert classify_launch_log(GZ_SEGFAULT, True, False, False)['unclassified']


def test_real_node_failures_still_block():
    result = classify_launch_log(NODE_CRASH, True, True, True)
    assert len(result['unclassified']) == 1
    assert result['shutdown_defect'] == []


def test_group_signal_deaths_only_excused_after_escalation():
    # Without escalation a signalled death is unexplained and must block.
    assert classify_launch_log(NODE_SIGINT, True, True, False)['unclassified']
    assert not classify_launch_log(NODE_SIGINT, True, True, True)['unclassified']


def test_traceback_is_never_classified_as_the_shutdown_defect():
    text = 'Traceback (most recent call last):'
    assert len(classify_launch_log(text, True, True, True)['unclassified']) == 1
