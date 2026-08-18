# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

import json
import math
from pathlib import Path

from araco_system_tests.slam import closure_error
from araco_system_tests.slam import FinalPoseDwell
from araco_system_tests.slam import is_catastrophic_cloud_drop
from araco_system_tests.slam import OrderedRoute
from araco_system_tests.slam import Pose2D
from araco_system_tests.slam import PoseStabilityWindow
from araco_system_tests.slam import relative_pose
from araco_system_tests.slam import TrackingLossMonitor
import jsonschema


ROOT = Path(__file__).resolve().parents[1]


def _contract():
    return json.loads((
        ROOT / 'config/perception/slam_acceptance_v0.yaml'
    ).read_text(encoding='utf-8'))


def test_slam_acceptance_contract_is_valid_and_ground_truth_is_observer_only():
    artifact = _contract()
    schema = json.loads((
        ROOT / 'schema/slam_acceptance_v1.schema.json'
    ).read_text(encoding='utf-8'))
    jsonschema.Draft202012Validator.check_schema(schema)
    jsonschema.Draft202012Validator(schema).validate(artifact['data'])

    data = artifact['data']
    assert artifact['artifact_version'] == '0.4.0'
    ground_truth = '/araco/simulation/ground_truth/odom'
    assert data['estimator_inputs_forbidden'] == [ground_truth]
    assert data['observer_topics']['ground_truth_odom'] == ground_truth
    assert data['observer_topics']['raw_odom'] == '/araco/perception/odom'
    assert data['observer_topics']['odom_info'] == '/araco/perception/odom_info'
    assert data['estimated_frames'] == {
        'map': 'map', 'odom': 'odom', 'base': 'base_link'}


def test_ordered_route_requires_every_waypoint_in_sequence():
    route = OrderedRoute([[1.0, 0.0], [0.0, 1.0], [0.0, 0.0]], 0.2)

    assert not route.update(Pose2D(0.0, 1.0, 0.0))
    assert route.next_index == 0
    assert route.update(Pose2D(0.9, 0.0, 0.0))
    assert route.update(Pose2D(0.0, 1.1, 0.0))
    assert route.update(Pose2D(0.1, 0.0, 0.0))
    assert route.complete


def test_final_pose_requires_position_heading_and_continuous_dwell():
    gate = FinalPoseDwell([0.0, 0.0], 0.2, 0.3, 2.0)

    assert not gate.update(Pose2D(0.1, 0.0, 0.5), 1.0)
    assert not gate.update(Pose2D(0.1, 0.0, 0.2), 2.0)
    assert not gate.update(Pose2D(0.3, 0.0, 0.2), 3.0)
    assert not gate.update(Pose2D(0.1, 0.0, 0.2), 4.0)
    assert gate.update(Pose2D(0.1, 0.0, 0.2), 6.0)


def test_pose_stability_uses_simulator_time_and_rejects_graph_jump():
    window = PoseStabilityWindow(3.0, 0.05, 0.04)

    assert not window.update(Pose2D(0.0, 0.0, 0.0), 10.0).stable
    # Repeated wall-time ticks at the same simulator stamp cannot finish it.
    assert not window.update(Pose2D(0.0, 0.0, 0.0), 10.0).stable
    assert not window.update(Pose2D(0.02, 0.0, 0.01), 12.0).stable
    assert window.update(Pose2D(0.01, 0.0, 0.02), 13.0).stable
    jump = window.update(Pose2D(0.20, 0.0, 0.02), 13.5)
    assert not jump.stable
    assert jump.translation_span_m > 0.15


def test_relative_pose_uses_the_route_start_axes():
    origin = Pose2D(2.0, 3.0, math.pi / 2.0)
    relative = relative_pose(Pose2D(2.0, 4.0, math.pi), origin)

    assert math.isclose(relative.x, 1.0, abs_tol=1e-12)
    assert math.isclose(relative.y, 0.0, abs_tol=1e-12)
    assert math.isclose(relative.yaw, math.pi / 2.0, abs_tol=1e-12)


def test_closure_error_compares_relative_motion_not_global_origins():
    translation, yaw = closure_error(
        Pose2D(10.0, -4.0, 0.3), Pose2D(10.1, -4.0, 0.4),
        Pose2D(-2.0, 7.0, -0.2), Pose2D(-2.0, 7.0, -0.2),
    )

    assert math.isclose(translation, 0.1, abs_tol=1e-12)
    assert math.isclose(yaw, 0.1, abs_tol=1e-12)


def test_corrected_slam_closure_is_distinct_from_raw_odometry_drift():
    truth_start = Pose2D(0.0, 0.0, 0.0)
    truth_end = Pose2D(0.05, -0.02, 0.01)
    corrected = closure_error(
        Pose2D(0.0, 0.0, 0.0), Pose2D(0.05, -0.02, 0.01),
        truth_start, truth_end)
    raw = closure_error(
        Pose2D(0.0, 0.0, 0.0), Pose2D(1.6, 0.2, 0.4),
        truth_start, truth_end)

    assert corrected == (0.0, 0.0)
    assert raw[0] > 1.5
    assert raw[1] > 0.35


def test_tracking_loss_monitor_reports_recovery_and_active_final_loss():
    monitor = TrackingLossMonitor()
    monitor.update(False, 0.0)
    monitor.update(True, 1.0)
    monitor.update(True, 2.5)
    monitor.update(False, 3.0)
    monitor.update(True, 4.0)
    monitor.update(True, 5.5)
    monitor.update(False, 5.0)  # Out-of-order data must not rewind duration.

    summary = monitor.summary()
    assert summary.events == 2
    assert summary.recoveries == 1
    assert math.isclose(summary.total_duration_s, 3.5, abs_tol=1e-12)
    assert math.isclose(summary.maximum_duration_s, 2.0, abs_tol=1e-12)
    assert summary.lost_at_finish


def test_cloud_drop_detection_ignores_small_clouds_and_flags_replacement():
    assert not is_catastrophic_cloud_drop(1000, 1, 5000, 0.75)
    assert not is_catastrophic_cloud_drop(10000, 4000, 5000, 0.75)
    assert is_catastrophic_cloud_drop(10000, 2000, 5000, 0.75)


def test_scorer_subscribes_to_outputs_without_feeding_truth_to_navigation():
    scorer = (ROOT / 'scripts/araco_slam_score').read_text(encoding='utf-8')
    navigation = (
        ROOT.parent / 'araco_navigation/launch/rtabmap_rgbd.launch.py'
    ).read_text(encoding='utf-8')

    assert "Info, topics['rtabmap_info']" in scorer
    assert "OdomInfo, topics['odom_info']" in scorer
    assert "PointCloud2, topics['cloud_map']" in scorer
    assert "topics['cloud_map'], self._on_cloud, map_qos" in scorer
    assert "self.frames['map'], self.frames['base'], Time()" in scorer
    assert "'raw_odom_translation_drift_m'" in scorer
    assert "'final_ground_truth_yaw_error_rad'" in scorer
    assert "'tracking_available_at_finish'" in scorer
    assert "'corrected_pose_converged'" in scorer
    assert 'route_complete_and_converged' in scorer
    assert 'allow_nan=False' in scorer
    assert '/ground_truth/' not in navigation


def test_diagnostic_records_synchronized_truth_odom_imu_and_gimbal():
    source = (ROOT / 'scripts/araco_slam_diagnose').read_text(encoding='utf-8')
    assert "'/araco/simulation/ground_truth/odom'" in source
    assert "'/araco/perception/odom'" in source
    assert "'/araco/camera/imu/data'" in source
    assert "'gimbal_yaw_joint'" in source
    assert 'Time.from_msg(message.header.stamp)' in source
    assert "'imu_timestamp_tf_failures'" in source
    assert 'publish_until_stopped' in source
    assert 'threading.Thread' in source
    assert "'safety_source_stale_events'" in source
    assert "'maximum_truth_translation_m'" in source
    assert "'samples.csv'" in source
    assert 'fixed_gimbal_imu forbids the gimbal_yaw scenario' in source
