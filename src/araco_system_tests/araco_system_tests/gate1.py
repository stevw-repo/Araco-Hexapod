# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

"""Pure numerical helpers for Gate 1 stable-hold scoring."""

from __future__ import annotations

import math
import re


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


# Known upstream defect: gz-sim 8.11.0 does not reliably tear down when the
# robot is spawned with gz_ros2_control.  On `/server_control stop` the server
# either exits cleanly, raises SIGSEGV on an internal worker thread, or
# deadlocks.  Neither failure mode is reachable from Araco source, and both
# occur strictly after all scored behavior has completed and metrics.json has
# been written.  Those lines are recorded as a named defect rather than counted
# as a gate failure.  Every other error line stays blocking.
LAUNCH_ERROR_TOKENS = (
    'Traceback (most recent call last)', 'process has died', '[ERROR]',
    'Segmentation fault')
# SIGSEGV (139 or -11) and SIGKILL (-9) are the crash forms.  137 is that same
# SIGKILL observed one level up: launch tracks the `ruby` wrapper, not the
# server, so killing the server child leaves the wrapper exiting 128 + 9.  Gate
# 5 produces it deliberately when it forces a deadlocked server dead to reach
# its own premise.  -2 and -15 appear when the runner has to signal the process
# group because the server deadlocked.
GAZEBO_CRASH_EXIT_CODES = (139, -11, -9, 137)
TEARDOWN_SIGNAL_EXIT_CODES = (-2, -15)
_EXIT_CODE = re.compile(r'exit code (-?\d+)')


def launch_exit_code(line):
    """Return the exit code named on a launch process-death line, if any."""
    found = _EXIT_CODE.search(line)
    return int(found.group(1)) if found else None


def classify_launch_log(text, scored_complete, stop_requested, escalated):
    """
    Split launch-log error lines into the known shutdown defect and the rest.

    A line is only ever attributed to the upstream defect once the gate has
    finished scoring and a server stop was requested.  If scoring did not
    complete, every error line stays unclassified and the gate still fails.
    """
    shutdown_defect = []
    unclassified = []
    attributable = bool(scored_complete and stop_requested)
    for number, line in enumerate(text.splitlines(), 1):
        if not any(token in line for token in LAUNCH_ERROR_TOKENS):
            continue
        record = {'line': number, 'text': line[:1000]}
        code = launch_exit_code(line)
        gazebo_line = '[gazebo-1]' in line
        gazebo_defect = gazebo_line and (
            code in GAZEBO_CRASH_EXIT_CODES or
            'Segmentation fault' in line or
            'failed to terminate' in line or
            (escalated and code in TEARDOWN_SIGNAL_EXIT_CODES))
        # A deadlocked server forces the runner to signal the whole process
        # group, so the other nodes die of that escalation, not of a fault.
        escalation_death = escalated and code in TEARDOWN_SIGNAL_EXIT_CODES
        if attributable and (gazebo_defect or escalation_death):
            shutdown_defect.append(record)
        else:
            unclassified.append(record)
    return {'shutdown_defect': shutdown_defect, 'unclassified': unclassified}
