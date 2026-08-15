# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

import json
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from araco_bringup.composer import compose_profile
from araco_bringup.composer import CompositionError
from araco_bringup.composer import load_artifact
import jsonschema
import pytest
import yaml


ARTIFACTS = {
    'araco_description': [
        'config/model/canonical_model_v1.yaml',
        'config/limits/provisional_sim_v0.yaml',
        'config/poses/nominal_standing_reference_v0.yaml',
        'config/dynamics/rough_estimate_v0.yaml',
        'config/resources/robot_description_v1.yaml',
    ],
    'araco_kinematics': ['config/solver/ik_v0.yaml'],
    'araco_locomotion': [
        'config/gait/tripod_slow_sim_v0.yaml',
        'config/policy/operational_sim_v0.yaml',
    ],
    'araco_supervision': [
        'config/sources/simulator_v0.yaml',
        'config/policy/simulator_v0.yaml',
        'config/qos/control_v0.yaml',
    ],
    'araco_teleop': ['config/mappings/keyboard_sim_v0.yaml'],
    'araco_gazebo': [
        'config/world/flat_ground_v0.yaml',
        'config/backend/gz_ros2_control_v0.yaml',
        'config/bridge/simulator_v0.yaml',
    ],
    'araco_bringup': [
        'config/wiring/single_robot_v0.yaml',
        'config/controllers/simulator_v0.yaml',
        'config/profiles/gazebo_dev_v0.yaml',
        'config/profiles/gazebo_ci_v0.yaml',
    ],
    'araco_system_tests': ['config/thresholds/gazebo_baseline_v0.yaml'],
}


def test_all_twenty_owner_artifacts_validate_from_installed_space():
    loaded = [
        load_artifact(package, relative)
        for package, paths in ARTIFACTS.items()
        for relative in paths
    ]
    assert len(loaded) == 20
    assert len({artifact.artifact_id for artifact in loaded}) == 20
    for artifact in loaded:
        package_share = Path(get_package_share_directory(artifact.package)).resolve()
        assert artifact.installed_path.is_relative_to(package_share)
        assert '/src/' not in str(artifact.installed_path)


def test_owner_schema_rejects_unknown_data_field():
    artifact = load_artifact(
        'araco_kinematics', 'config/solver/ik_v0.yaml'
    )
    corrupted = dict(artifact.document['data'])
    corrupted['unknown_motion_knob'] = 1
    share = Path(get_package_share_directory('araco_kinematics'))
    schema = json.loads((share / 'schema/config_v1.schema.json').read_text())
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.Draft202012Validator(schema).validate(corrupted)


def test_profiles_compose_deterministically_with_equal_behavior(tmp_path):
    dev_directory = tmp_path / 'dev'
    dev_repeat_directory = tmp_path / 'dev_repeat'
    ci_directory = tmp_path / 'ci'
    dev = compose_profile('gazebo_dev_v0', dev_directory)
    dev_repeat = compose_profile('gazebo_dev_v0', dev_repeat_directory)
    ci = compose_profile('gazebo_ci_v0', ci_directory)

    assert dev['behavior_fingerprint'] == ci['behavior_fingerprint']
    assert dev['input_selection_fingerprint'] != ci['input_selection_fingerprint']
    assert dev['run_fingerprint'] == dev_repeat['run_fingerprint']
    assert dev['generated_file_sha256'] == dev_repeat['generated_file_sha256']
    assert (dev_directory / 'description/robot.urdf').read_bytes() == (
        dev_repeat_directory / 'description/robot.urdf'
    ).read_bytes()

    root = ET.parse(dev_directory / 'description/robot.urdf').getroot()
    assert len(root.findall('link')) == 29
    assert len(root.findall('joint')) == 28
    revolute = [
        joint for joint in root.findall('joint')
        if joint.attrib['type'] == 'revolute'
    ]
    assert len(revolute) == 25
    visual_names = {
        visual.attrib['name']
        for link in root.findall('link')
        for visual in link.findall('visual')
    }
    primary_visuals = {name for name in visual_names if name.endswith('_primary_visual')}
    servo_cases = {name for name in visual_names if name.endswith('_servo_case_visual')}
    servo_horns = {name for name in visual_names if name.endswith('_servo_horn_visual')}
    assert len(primary_visuals) == 26
    assert len(servo_cases) == 13
    assert len(servo_horns) == 7
    assert len(visual_names) == 49
    assert {
        'camera_link_camera_body_visual',
        'camera_link_camera_hardware_visual',
        'camera_link_camera_optics_visual',
    } <= visual_names
    for leg in ('left_front', 'left_middle', 'left_rear',
                'right_front', 'right_middle', 'right_rear'):
        tibia = root.find(f"./link[@name='{leg}_tibia_link']")
        assert tibia is not None
        tibia_names = {visual.attrib['name'] for visual in tibia.findall('visual')}
        assert f'{leg}_tibia_link_primary_visual' in tibia_names
        assert f'{leg}_tibia_link_servo_case_visual' in tibia_names
        assert f'{leg}_tibia_link_servo_horn_visual' in tibia_names
    assert (
        dev['controller_partitions']['leg_joints']
        == ci['controller_partitions']['leg_joints']
    )
    assert len(dev['controller_partitions']['leg_joints']) == 24
    assert dev['controller_partitions']['gimbal_joints'] == ['gimbal_yaw_joint']
    assert len(dev['controller_partitions']['state_joints']) == 25

    leg_parameters = yaml.safe_load(
        (dev_directory / 'ros2_control/leg_trajectory_controller.yaml').read_text()
    )['leg_trajectory_controller']['ros__parameters']
    assert leg_parameters['joints'] == dev['controller_partitions']['leg_joints']
    assert leg_parameters['cmd_timeout'] == 0.1
    assert (dev_directory / 'validation_report.json').is_file()
    report = json.loads(
        (dev_directory / 'validation_report.json').read_text()
    )
    assert report['status'] == 'PASS'


def test_invalid_request_fails_before_bundle_emission(tmp_path):
    with pytest.raises(CompositionError):
        compose_profile('uninstalled_profile', tmp_path / 'forbidden')
    assert not (tmp_path / 'forbidden').exists()

    occupied = tmp_path / 'occupied'
    occupied.mkdir()
    with pytest.raises(CompositionError):
        compose_profile('gazebo_dev_v0', occupied)
    assert list(occupied.iterdir()) == []
