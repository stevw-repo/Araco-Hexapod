# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

"""Pure evidence comparison and log-classification helpers for Gate 6."""

from __future__ import annotations

import json
import math
from pathlib import Path
import re
import statistics


from araco_system_tests.gate1 import GAZEBO_CRASH_EXIT_CODES
from araco_system_tests.gate1 import launch_exit_code
from araco_system_tests.gate1 import TEARDOWN_SIGNAL_EXIT_CODES

ANSI = re.compile(r'\x1b\[[0-9;]*m')
ERROR_TOKENS = ('[ERROR]', '[FATAL]', 'process has died', 'Segmentation fault')
SANITIZER_TOKENS = (
    'AddressSanitizer:', 'UndefinedBehaviorSanitizer:',
    'runtime error:', 'LeakSanitizer:')
ALLOWED_WARNING_TOKENS = (
    'root link base_link has an inertia',
    'Desired controller update period',
    'does not have read or write statistics initialized',
    'Executor is not available during hardware component initialization',
    'Waiting RM to load and initialize hardware',
    "Waiting for data on 'robot_description'",
)


def load_json(path):
    """Load one JSON file."""
    return json.loads(Path(path).read_text(encoding='utf-8'))


def is_gz_shutdown_defect(line):
    """
    Return True for the known gz-sim 8.11.0 teardown crash or deadlock.

    Only the simulator process may be excused for a crash signal, and only for
    the teardown signatures.  A signal-terminated process of any name is
    accepted because the runner signals the whole process group when the server
    deadlocks.  A crash in any other process stays an error.  The authoritative
    protection against masking a mid-run crash is the preflight sub-gate result,
    which fails independently of this classification.
    """
    code = launch_exit_code(line)
    if code in TEARDOWN_SIGNAL_EXIT_CODES:
        return True
    return '[gazebo-1]' in line and (
        code in GAZEBO_CRASH_EXIT_CODES or
        'Segmentation fault' in line or
        'failed to terminate' in line)


# Gate 6 runs up to 24 sub-gates in one attempt (six preflight plus six per
# repetition).  Each needs its own DDS domain.  `araco_gate1_evidence` already
# derives one from its own pid when none is given, but `100 + pid % 100`
# collides whenever two sub-gate pids differ by exactly 100, which is reachable
# across a suite that spawns dozens of processes per gate.  Assigning the ids
# explicitly removes that residual risk.  Domain ids stay in the 100-199 band
# the runner already uses.
DOMAIN_BAND_START = 100
DOMAIN_BAND_SIZE = 100


def sub_gate_domain_id(index, base):
    """
    Return the ROS domain id for one Gate 6 sub-gate.

    `index` is the sub-gate's position within the whole attempt, so ids stay
    distinct for any attempt of fewer than DOMAIN_BAND_SIZE sub-gates.  `base`
    separates concurrent attempts on one machine.
    """
    if index < 0:
        raise ValueError('sub-gate index must not be negative')
    return DOMAIN_BAND_START + (base + index) % DOMAIN_BAND_SIZE


def orphan_server_pids(pgrep_output, exclude=()):
    """
    Parse `pgrep -f` output into simulator pids to reap.

    The upstream gz-sim teardown defect leaves the real server alive after its
    gate has finished: it is a child of the `/bin/sh` wrapper, so signalling the
    launch process group misses it, and being deadlocked it ignores SIGTERM.
    Match on the argument list, never on the process name -- the server's name
    is `ruby`, so `pgrep -x "gz sim"` silently matches nothing.
    """
    excluded = set(exclude)
    pids = []
    for line in (pgrep_output or '').splitlines():
        entry = line.strip()
        if not entry:
            continue
        try:
            pid = int(entry)
        except ValueError:
            continue
        if pid > 0 and pid not in excluded and pid not in pids:
            pids.append(pid)
    return pids


def classify_logs(root):
    """Classify relevant error and warning lines without suppressing unknowns."""
    classified_warnings = []
    unclassified_warnings = []
    errors = []
    shutdown_defect = []
    sanitizer_failures = []
    for path in sorted(Path(root).rglob('*.log')):
        for number, raw in enumerate(
                path.read_text(encoding='utf-8', errors='replace').splitlines(), 1):
            line = ANSI.sub('', raw)
            record = {'path': str(path), 'line': number, 'text': line[:1000]}
            if any(token in line for token in SANITIZER_TOKENS):
                sanitizer_failures.append(record)
            if any(token in line for token in ERROR_TOKENS):
                target = shutdown_defect if is_gz_shutdown_defect(line) else errors
                target.append(record)
            elif '[WARN]' in line or '[Wrn]' in line:
                target = (classified_warnings if any(
                    token in line for token in ALLOWED_WARNING_TOKENS)
                    else unclassified_warnings)
                target.append(record)
    return {
        'classified_warnings': classified_warnings,
        'unclassified_warnings': unclassified_warnings,
        'errors': errors,
        'shutdown_defect': shutdown_defect,
        'sanitizer_failures': sanitizer_failures,
    }


def _range(values):
    finite = [value for value in values if isinstance(value, (int, float)) and
              math.isfinite(value)]
    return max(finite) - min(finite) if len(finite) == len(values) and finite else math.inf


def _gate4_cases(repetition):
    metrics = load_json(Path(repetition) / 'gate4/metrics.json')['metrics']
    return {item['name']: item for item in metrics['cases']}, metrics['timing']


def _terminal_epoch_path(path):
    """Keep the terminal reason/fault observation for each state epoch."""
    result = []
    for item in path:
        if result and result[-1][0] == item[0]:
            result[-1] = item
        else:
            result.append(item)
    return result


def _discrete_safety_path(path):
    """
    Return exact terminal epoch/state/reason outcomes for repeatability.

    Gate 5 owns the fault-mask matrix and reset assertions.  A recovery
    controller-manager operation can add a corroborating fault bit after the
    initiating fault is already latched without changing the safety epoch,
    state, or terminal reason.  That scheduler-dependent accumulation is not a
    separate discrete transition and must not make otherwise identical Gate 6
    state paths differ.
    """
    return [item[:3] for item in _terminal_epoch_path(path)]


def _discrete_validation_outcome(validation):
    """Normalize gate validation reports without assuming one report schema."""
    checks = validation.get('checks')
    if isinstance(checks, dict):
        return checks
    return {
        key: value for key, value in validation.items()
        if key not in ('errors', 'fidelity_limitations')
    }


def compare_repetitions(repetitions, thresholds):
    """Compare three complete suite repetitions against Gate 6 tolerances."""
    failures = []
    checks = {}
    gate4 = [_gate4_cases(path) for path in repetitions]
    case_names = [set(cases) for cases, _ in gate4]
    checks['identical_case_sets'] = len(case_names) == 3 and all(
        names == case_names[0] for names in case_names[1:])
    comparisons = {}
    if checks['identical_case_sets']:
        for name in sorted(case_names[0]):
            cases = [items[name] for items, _ in gate4]
            values = {
                'final_displacement_difference_m': _range([
                    case['final_displacement_m'] for case in cases]),
                'final_yaw_difference_rad': _range([
                    case['final_yaw_rad'] for case in cases]),
                'controlled_stop_distance_difference_m': _range([
                    case['stop']['drift_m'] for case in cases]),
                'roll_pitch_rms_difference_rad': _range([
                    case['roll_pitch_rms_rad'] for case in cases]),
            }
            comparisons[name] = values
            checks[f'{name}_physical_repeatability'] = (
                values['final_displacement_difference_m'] <=
                thresholds['maximum_final_displacement_difference_m'] and
                values['final_yaw_difference_rad'] <=
                thresholds['maximum_final_yaw_difference_rad'] and
                values['controlled_stop_distance_difference_m'] <=
                thresholds['maximum_controlled_stop_distance_difference_m'] and
                values['roll_pitch_rms_difference_rad'] <=
                thresholds['maximum_roll_pitch_rms_difference_rad'])

    real_time_factors = [timing['real_time_factor'] for _, timing in gate4]
    median_rtf = statistics.median(real_time_factors)
    checks['median_real_time_factor'] = (
        median_rtf >= thresholds['minimum_median_real_time_factor'])

    discrete_signatures = []
    fingerprints = []
    for repetition in repetitions:
        signature = {}
        for gate in range(6):
            gate_path = Path(repetition) / f'gate{gate}'
            validation = load_json(gate_path / 'validation_report.json')
            signature[str(gate)] = _discrete_validation_outcome(validation)
            if gate == 5:
                metrics = load_json(gate_path / 'metrics.json')['metrics']
                signature['5_safety_path'] = _discrete_safety_path(
                    metrics['safety_path'])
                signature['5_selection_path'] = metrics['selection_path']
        discrete_signatures.append(signature)
        fingerprints.append(load_json(
            Path(repetition) / 'gate0/gate_result.json')['behavior_fingerprint'])
    checks['exact_discrete_outcomes'] = all(
        value == discrete_signatures[0] for value in discrete_signatures[1:])
    checks['identical_behavior_fingerprints'] = len(set(fingerprints)) == 1

    for name, passed in checks.items():
        if not passed:
            failures.append(name)
    return checks, {
        'case_comparisons': comparisons,
        'real_time_factors': real_time_factors,
        'median_real_time_factor': median_rtf,
        'behavior_fingerprints': fingerprints,
    }, failures
