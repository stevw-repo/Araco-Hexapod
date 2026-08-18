# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

import json
from pathlib import Path

import jsonschema


ROOT = Path(__file__).resolve().parents[1]


def _artifact():
    return json.loads((
        ROOT / 'config/slam/rtabmap_rgbd_sim_v0.yaml'
    ).read_text(encoding='utf-8'))


def _variant(name):
    return json.loads((ROOT / f'config/slam/{name}').read_text(encoding='utf-8'))


def test_rtabmap_contract_is_six_dof_registered_and_never_uses_ground_truth():
    artifact = _artifact()
    schema = json.loads((
        ROOT / 'schema/config_v1.schema.json'
    ).read_text(encoding='utf-8'))
    jsonschema.Draft202012Validator.check_schema(schema)
    jsonschema.Draft202012Validator(schema).validate(artifact['data'])

    data = artifact['data']
    assert artifact['artifact_version'] == '0.4.0'
    assert data['mode'] == 'six_dof_rgbd_visual'
    assert data['odometry']['imu_enabled'] is False
    assert data['odometry']['wait_for_imu'] is False
    assert data['odometry']['always_check_imu_tf'] is True
    assert data['odometry']['gimbal_policy'] == 'not_applicable'
    assert data['mapping']['gravity_sigma'] == 0.0
    assert data['synchronization']['registered_depth_required'] is True
    assert data['mapping']['six_dof'] is True
    assert data['mapping']['grid']['three_d'] is True
    assert data['odometry']['reset_countdown'] == 0
    assert data['ground_truth_input'] is False
    assert '/ground_truth/' not in json.dumps(data)


def test_rtabmap_contract_owns_the_complete_map_odom_base_transform_chain():
    frames = _artifact()['data']['frames']
    assert frames == {
        'map': 'map',
        'odom': 'odom',
        'base': 'base_link',
        'imu': 'camera_link',
    }


def test_launch_wires_only_standard_sensor_topics_and_project_outputs():
    source = (ROOT / 'launch/rtabmap_rgbd.launch.py').read_text(encoding='utf-8')
    assert "package='rtabmap_sync', executable='rgbd_sync'" in source
    assert "package='rtabmap_odom', executable='rgbd_odometry'" in source
    assert "package='rtabmap_slam', executable='rtabmap'" in source
    assert "'subscribe_imu': imu_enabled" in source
    assert 'if imu_enabled else []' in source
    assert 'ground_truth' not in source


def test_tracking_loss_cannot_automatically_start_a_new_map_segment():
    data = _artifact()['data']
    source = (ROOT / 'launch/rtabmap_rgbd.launch.py').read_text(encoding='utf-8')

    assert data['odometry']['reset_countdown'] == 0
    assert "'Odom/ResetCountdown': str(odometry['reset_countdown'])" in source


def test_diagnostic_variants_isolate_imu_without_changing_rgbd_geometry():
    schema = json.loads((ROOT / 'schema/config_v1.schema.json').read_text())
    operational = _artifact()['data']
    dynamic = _variant('rtabmap_rgbd_dynamic_gimbal_imu_sim_v0.yaml')['data']
    visual = _variant('rtabmap_rgbd_visual_only_sim_v0.yaml')['data']
    fixed = _variant('rtabmap_rgbd_fixed_gimbal_imu_sim_v0.yaml')['data']
    for data in (dynamic, visual, fixed):
        jsonschema.Draft202012Validator(schema).validate(data)
        assert data['inputs'] == operational['inputs']
        assert data['synchronization'] == operational['synchronization']
        assert data['ground_truth_input'] is False
    assert visual['odometry']['imu_enabled'] is False
    assert visual['odometry']['gimbal_policy'] == 'not_applicable'
    assert dynamic['odometry']['imu_enabled'] is True
    assert dynamic['odometry']['always_check_imu_tf'] is True
    assert dynamic['odometry']['gimbal_policy'] == 'dynamic'
    assert fixed['odometry']['imu_enabled'] is True
    assert fixed['odometry']['always_check_imu_tf'] is False
    assert fixed['odometry']['gimbal_policy'] == 'locked_center'
