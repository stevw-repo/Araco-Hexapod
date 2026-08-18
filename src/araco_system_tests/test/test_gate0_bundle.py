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
    'araco_teleop': ['config/mappings/keyboard_sim_v1.yaml'],
    'araco_gazebo': [
        'config/world/flat_ground_v0.yaml',
        'config/world/rgbd_validation_v0.yaml',
        'config/backend/gz_ros2_control_v2.yaml',
        'config/bridge/simulator_v0.yaml',
    ],
    'araco_perception': [
        'config/sensors/gemini_335_sim_v0.yaml',
        'config/sensors/gemini_335_registered_sim_v0.yaml',
    ],
    'araco_navigation': [
        'config/slam/rtabmap_rgbd_sim_v0.yaml',
        'config/slam/rtabmap_rgbd_dynamic_gimbal_imu_sim_v0.yaml',
        'config/slam/rtabmap_rgbd_visual_only_sim_v0.yaml',
        'config/slam/rtabmap_rgbd_fixed_gimbal_imu_sim_v0.yaml',
    ],
    'araco_bringup': [
        'config/wiring/single_robot_v1.yaml',
        'config/controllers/simulator_v0.yaml',
        'config/profiles/gazebo_dev_v0.yaml',
        'config/profiles/gazebo_ci_v0.yaml',
        'config/profiles/gazebo_gate3_v0.yaml',
        'config/profiles/gazebo_gate4_v0.yaml',
        'config/profiles/gazebo_perception_v0.yaml',
        'config/profiles/gazebo_perception_diagnostic_visual_v0.yaml',
        'config/profiles/gazebo_perception_diagnostic_dynamic_imu_v0.yaml',
        'config/profiles/gazebo_perception_diagnostic_fixed_imu_v0.yaml',
    ],
    'araco_system_tests': [
        'config/thresholds/gazebo_baseline_v0.yaml',
        'config/scenarios/gate6_v0.yaml',
        'config/perception/slam_acceptance_v0.yaml',
    ],
}


def test_all_thirty_five_owner_artifacts_validate_from_installed_space():
    loaded = [
        load_artifact(package, relative)
        for package, paths in ARTIFACTS.items()
        for relative in paths
    ]
    assert len(loaded) == 35
    assert len({artifact.artifact_id for artifact in loaded}) == 35
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


def test_interactive_profile_uses_headed_simulator_timing_policy(tmp_path):
    policy = load_artifact(
        'araco_supervision', 'config/policy/headed_simulator_v0.yaml'
    )
    assert policy.version == '0.5.0'
    assert policy.document['deployment_scope'] == 'simulator_only'
    assert policy.document['data']['watchdogs_s'] == {
        'selected_command': 0.5,
        'safe_command': 0.5,
        'joint_state': 0.5,
        'locomotion_status': 0.5,
        'controller_state': 0.5,
        'provenance': 1.5,
        'clock_progress': 0.5,
    }
    source_registry = load_artifact(
        'araco_supervision', 'config/sources/headed_simulator_v0.yaml'
    )
    mapping = load_artifact(
        'araco_teleop', 'config/mappings/pxn_2113_pro_v0.yaml'
    )
    assert source_registry.document['data']['sources'][0][
        'freshness_timeout_s'] == 0.5
    assert mapping.document['data']['heartbeat_timeout_s'] == 0.5

    joystick_directory = tmp_path / 'joystick_headed_policy'
    dev_directory = tmp_path / 'dev_strict_policy'
    compose_profile('gazebo_joystick_v0', joystick_directory)
    compose_profile('gazebo_dev_v0', dev_directory)
    joystick_parameters = yaml.safe_load(
        (joystick_directory / 'node_params/safety_supervisor.yaml').read_text()
    )['/araco/safety_supervisor']['ros__parameters']
    dev_parameters = yaml.safe_load(
        (dev_directory / 'node_params/safety_supervisor.yaml').read_text()
    )['/araco/safety_supervisor']['ros__parameters']
    joystick_arbiter_parameters = yaml.safe_load(
        (joystick_directory / 'node_params/command_arbiter.yaml').read_text()
    )['/araco/command_arbiter']['ros__parameters']
    assert joystick_parameters['locomotion_status_timeout_s'] == 0.5
    assert joystick_parameters['clock_progress_timeout_s'] == 0.5
    assert dev_parameters['locomotion_status_timeout_s'] == 0.1
    assert dev_parameters['clock_progress_timeout_s'] == 0.25
    assert joystick_arbiter_parameters['teleop_timeout_s'] == 0.5


def test_perception_profile_composes_the_rgbd_scene_without_mutating_baselines(
        tmp_path):
    perception_directory = tmp_path / 'perception'
    joystick_directory = tmp_path / 'joystick'
    perception = compose_profile('gazebo_perception_v0', perception_directory)
    joystick = compose_profile('gazebo_joystick_v0', joystick_directory)

    assert perception['profile_id'] == 'gazebo_perception_v0'
    assert perception['accepted_overrides']['gui'] is True
    assert perception['accepted_overrides']['rviz'] is True
    perception_world = ET.parse(
        perception_directory / 'gazebo/resolved_world.sdf').getroot().find('world')
    joystick_world = ET.parse(
        joystick_directory / 'gazebo/resolved_world.sdf').getroot().find('world')
    assert perception_world.attrib['name'] == 'araco_rgbd_validation'
    assert joystick_world.attrib['name'] == 'araco_flat_ground'
    assert perception['behavior_fingerprint'] != joystick['behavior_fingerprint']
    assert perception['input_selection_fingerprint'] == (
        joystick['input_selection_fingerprint'])


def test_perception_diagnostic_profiles_compose_exact_estimator_variants(tmp_path):
    expected = {
        'gazebo_perception_diagnostic_dynamic_imu_v0': (
            'araco.navigation.rtabmap-rgbd-dynamic-gimbal-imu-sim', True,
            'dynamic'),
        'gazebo_perception_diagnostic_visual_v0': (
            'araco.navigation.rtabmap-rgbd-visual-only-sim', False,
            'not_applicable'),
        'gazebo_perception_diagnostic_fixed_imu_v0': (
            'araco.navigation.rtabmap-rgbd-fixed-gimbal-imu-sim', True,
            'locked_center'),
    }
    for profile, (artifact_id, imu_enabled, gimbal_policy) in expected.items():
        directory = tmp_path / profile
        manifest = compose_profile(profile, directory)
        selected = next(
            artifact for artifact in manifest['artifacts']
            if artifact['package'] == 'araco_navigation')
        assert selected['artifact_id'] == artifact_id
        normalized = json.loads((
            directory / 'normalized_artifacts' /
            f"{artifact_id.replace('.', '_')}.json"
        ).read_text())['data']
        assert normalized['odometry']['imu_enabled'] is imu_enabled
        assert normalized['odometry']['gimbal_policy'] == gimbal_policy


def test_profiles_compose_deterministically_with_equal_behavior(tmp_path):
    dev_directory = tmp_path / 'dev'
    dev_repeat_directory = tmp_path / 'dev_repeat'
    ci_directory = tmp_path / 'ci'
    gate3_directory = tmp_path / 'gate3'
    gate4_directory = tmp_path / 'gate4'
    dev = compose_profile('gazebo_dev_v0', dev_directory)
    dev_repeat = compose_profile('gazebo_dev_v0', dev_repeat_directory)
    ci = compose_profile('gazebo_ci_v0', ci_directory)
    gate3 = compose_profile('gazebo_gate3_v0', gate3_directory)
    gate4 = compose_profile('gazebo_gate4_v0', gate4_directory)

    assert dev['behavior_fingerprint'] == ci['behavior_fingerprint'] == (
        gate3['behavior_fingerprint']) == gate4['behavior_fingerprint']
    assert dev['input_selection_fingerprint'] != ci['input_selection_fingerprint']
    assert gate3['input_selection_fingerprint'] not in {
        dev['input_selection_fingerprint'], ci['input_selection_fingerprint']}
    assert gate4['input_selection_fingerprint'] == (
        gate3['input_selection_fingerprint'])
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
    camera_sensors = root.findall("./gazebo[@reference='camera_link']/sensor")
    assert {sensor.attrib['name'] for sensor in camera_sensors} == {
        'gemini_color', 'gemini_rgbd', 'gemini_imu'}
    assert root.find(
        './gazebo[@reference="camera_link"]/sensor[@name="gemini_color"]'
        '/camera/optical_frame_id'
    ).text == 'camera_color_optical_frame'
    assert root.find(
        './gazebo[@reference="camera_link"]/sensor[@name="gemini_rgbd"]'
        '/camera/optical_frame_id'
    ).text == 'camera_depth_optical_frame'
    for leg in ('left_front', 'left_middle', 'left_rear',
                'right_front', 'right_middle', 'right_rear'):
        tibia = root.find(f"./link[@name='{leg}_tibia_link']")
        assert tibia is not None
        tibia_names = {visual.attrib['name'] for visual in tibia.findall('visual')}
        assert f'{leg}_tibia_link_primary_visual' in tibia_names
        assert f'{leg}_tibia_link_servo_case_visual' in tibia_names
        assert f'{leg}_tibia_link_servo_horn_visual' in tibia_names

        foot = root.find(f"./link[@name='{leg}_foot_link']")
        assert foot is not None
        collision = foot.find('collision')
        assert collision is not None
        assert collision.find('geometry/sphere').attrib['radius'] == '0.004'
        collision_xyz = tuple(
            float(value) for value in collision.find('origin').attrib['xyz'].split())
        assert collision_xyz == pytest.approx((0.05, 0.0, 0.0))
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
    bridge_parameters = yaml.safe_load(
        (dev_directory / 'gazebo/bridge.yaml').read_text()
    )
    point_cloud_bridge = next(
        endpoint for endpoint in bridge_parameters
        if endpoint['ros_topic_name'] == '/araco/camera/depth/points'
    )
    assert point_cloud_bridge['frame_id'] == 'camera_link'
    safety_parameters = yaml.safe_load(
        (dev_directory / 'node_params/safety_supervisor.yaml').read_text()
    )['/araco/safety_supervisor']['ros__parameters']
    assert safety_parameters['startup_readiness_stable_s'] == 1.0
    assert (dev_directory / 'validation_report.json').is_file()
    report = json.loads(
        (dev_directory / 'validation_report.json').read_text()
    )
    assert report['status'] == 'PASS'


def test_joystick_profile_owns_operator_smoothing_without_double_shaping(tmp_path):
    joystick_directory = tmp_path / 'joystick'
    dev_directory = tmp_path / 'dev'

    compose_profile('gazebo_joystick_v0', joystick_directory)
    compose_profile('gazebo_dev_v0', dev_directory)

    joystick_parameters = yaml.safe_load(
        (joystick_directory / 'node_params/locomotion.yaml').read_text()
    )['/araco/locomotion']['ros__parameters']
    dev_parameters = yaml.safe_load(
        (dev_directory / 'node_params/locomotion.yaml').read_text()
    )['/araco/locomotion']['ros__parameters']

    assert joystick_parameters['operator_input_pre_filtered'] is True
    assert joystick_parameters['gait_preferred_maximum_stride_scale'] == 0.6
    assert joystick_parameters['gait_maximum_stride_m'] == 0.12
    assert joystick_parameters['gait_planar_command_scale_m_s'] == 0.24
    assert dev_parameters['operator_input_pre_filtered'] is False


def test_invalid_request_fails_before_bundle_emission(tmp_path):
    with pytest.raises(CompositionError):
        compose_profile('uninstalled_profile', tmp_path / 'forbidden')
    assert not (tmp_path / 'forbidden').exists()

    occupied = tmp_path / 'occupied'
    occupied.mkdir()
    with pytest.raises(CompositionError):
        compose_profile('gazebo_dev_v0', occupied)
    assert list(occupied.iterdir()) == []
