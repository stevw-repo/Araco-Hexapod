# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

"""Unit tests for Gate 6 comparison and log classification."""

from pathlib import Path

from araco_system_tests.gate6 import _discrete_safety_path
from araco_system_tests.gate6 import _discrete_validation_outcome
from araco_system_tests.gate6 import classify_logs
from araco_system_tests.gate6 import orphan_server_pids
from araco_system_tests.gate6 import sub_gate_domain_id
import pytest


def test_log_classifier_separates_known_warning_and_error(tmp_path: Path):
    log = tmp_path / 'sample.log'
    log.write_text(
        '[WARN] root link base_link has an inertia specified\n'
        '[WARN] new project warning\n'
        '[ERROR] process has died\n', encoding='utf-8')
    result = classify_logs(tmp_path)
    assert len(result['classified_warnings']) == 1
    assert len(result['unclassified_warnings']) == 1
    assert len(result['errors']) == 1


def test_log_classifier_detects_sanitizer(tmp_path: Path):
    (tmp_path / 'asan.log').write_text(
        'AddressSanitizer: heap-use-after-free\n', encoding='utf-8')
    result = classify_logs(tmp_path)
    assert len(result['sanitizer_failures']) == 1


def test_discrete_validation_outcome_uses_check_map_when_present():
    report = {
        'status': 'PASS',
        'checks': {'launch_exit': True, 'gate_score': True},
        'errors': [],
    }
    assert _discrete_validation_outcome(report) == report['checks']


def test_discrete_validation_outcome_supports_gate0_report_shape():
    report = {
        'gate': 0,
        'status': 'PASS',
        'behavior_fingerprint': 'abc',
        'behavior_fingerprints_equal': True,
        'profiles': {'development': 'abc', 'ci': 'abc'},
        'fidelity_limitations': ['not hardware evidence'],
    }
    assert _discrete_validation_outcome(report) == {
        'gate': 0,
        'status': 'PASS',
        'behavior_fingerprint': 'abc',
        'behavior_fingerprints_equal': True,
        'profiles': {'development': 'abc', 'ci': 'abc'},
    }


def test_discrete_safety_path_uses_terminal_state_reason_not_late_fault_union():
    path_a = [
        [9, 5, 18, 8],
        [10, 6, 18, 8],
        [10, 6, 26, 8],
        [11, 2, 3, 0],
    ]
    path_b = [
        [9, 5, 18, 8],
        [10, 6, 18, 8],
        [10, 6, 18, 12],
        [10, 6, 26, 12],
        [11, 2, 3, 0],
    ]
    assert _discrete_safety_path(path_a) == _discrete_safety_path(path_b)
    assert _discrete_safety_path(path_a) == [
        [9, 5, 18], [10, 6, 26], [11, 2, 3]]


def test_sub_gate_domain_ids_are_distinct_across_a_whole_attempt():
    # Six preflight gates plus three repetitions of six.
    ids = [sub_gate_domain_id(index, 7) for index in range(24)]
    assert len(set(ids)) == 24
    assert all(100 <= value <= 199 for value in ids)


def test_sub_gate_domain_id_is_deterministic_and_rejects_bad_index():
    assert sub_gate_domain_id(3, 7) == sub_gate_domain_id(3, 7)
    assert sub_gate_domain_id(0, 99) != sub_gate_domain_id(1, 99)
    with pytest.raises(ValueError):
        sub_gate_domain_id(-1, 0)


def test_orphan_server_pids_parses_and_excludes():
    assert orphan_server_pids('120\n121\n') == [120, 121]
    assert orphan_server_pids('120\n121\n', exclude=(121,)) == [120]


def test_orphan_server_pids_tolerates_empty_and_junk_output():
    assert orphan_server_pids('') == []
    assert orphan_server_pids(None) == []
    assert orphan_server_pids('\n  \nnot-a-pid\n130\n130\n') == [130]
