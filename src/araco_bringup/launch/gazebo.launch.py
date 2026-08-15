# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

"""Strict ordered Gazebo bringup for the development and CI profiles."""

from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from araco_bringup.composer import compose_profile
from araco_bringup.preflight import preflight_bundle
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _continue_on_success(target, following, label):
    def handler(event, _context):
        if event.returncode == 0:
            return following
        return [EmitEvent(event=Shutdown(reason=f'{label} failed with {event.returncode}'))]
    return OnProcessExit(target_action=target, on_exit=handler)


def _lifecycle(node_name, transition):
    return Node(
        package='araco_bringup', executable='lifecycle_transition',
        output='screen',
        arguments=['--node', node_name, '--transition', transition, '--timeout', '10'],
    )


def _runtime_actions(context):
    profile = LaunchConfiguration('profile').perform(context)
    requested_bundle = LaunchConfiguration('runtime_bundle').perform(context)
    if requested_bundle:
        bundle = Path(requested_bundle).expanduser().resolve()
    else:
        parent = Path(tempfile.mkdtemp(prefix='araco_gazebo_launch_'))
        bundle = parent / 'effective_config'
    manifest = compose_profile(profile, bundle)
    preflight_bundle(bundle)
    description = (bundle / 'description/robot.urdf').read_text(encoding='utf-8')
    pose_document = json.loads((
        bundle / 'normalized_artifacts/araco_description_nominal-standing-reference.json'
    ).read_text(encoding='utf-8'))
    base_pose = pose_document['data']['base_pose']
    x, y, z = (str(value) for value in base_pose['position_xyz_m'])
    gui = bool(manifest['accepted_overrides']['gui'])

    description_share_parent = str(
        Path(get_package_share_directory('araco_description')).parent
    )
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_entries = [description_share_parent]
    if existing_resource_path:
        resource_entries.append(existing_resource_path)
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(resource_entries),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f"-r {'-s ' if not gui else ''}-v 3 "
                       f'"{bundle / "gazebo/resolved_world.sdf"}"',
            'on_exit_shutdown': 'true',
        }.items(),
    )
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': description, 'use_sim_time': True}],
    )
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='simulation_bridge',
        namespace='araco', output='screen',
        parameters=[{'config_file': str(bundle / 'gazebo/bridge.yaml'), 'use_sim_time': True}],
    )
    contact_filter = Node(
        package='araco_gazebo', executable='contact_filter_node', output='screen',
        parameters=[{'use_sim_time': True}],
    )
    locomotion = Node(
        package='araco_locomotion', executable='locomotion_node', output='screen',
        parameters=[str(bundle / 'node_params/locomotion.yaml')],
    )
    safety = Node(
        package='araco_supervision', executable='safety_supervisor_node', output='screen',
        parameters=[str(bundle / 'node_params/safety_supervisor.yaml')],
    )
    arbiter = Node(
        package='araco_supervision', executable='command_arbiter_node', output='screen',
        parameters=[str(bundle / 'node_params/command_arbiter.yaml')],
    )
    teleop = Node(
        package='araco_teleop', executable='teleop_adapter_node', output='screen',
        parameters=[str(bundle / 'node_params/teleop_adapter.yaml')],
    )
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', 'araco_flat_ground', '-topic', 'robot_description',
                   '-name', 'araco', '-allow_renaming', 'false',
                   '-x', x, '-y', y, '-z', z],
    )

    def spawner(name):
        return Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=[name, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '30', '--switch-timeout', '10',
                       '--param-file', str(bundle / f'ros2_control/{name}.yaml')],
        )

    state_spawner = spawner('joint_state_broadcaster')
    leg_spawner = spawner('leg_trajectory_controller')
    gimbal_spawner = spawner('gimbal_trajectory_controller')
    configure_locomotion = _lifecycle('/araco/locomotion', 'configure')
    activate_locomotion = _lifecycle('/araco/locomotion', 'activate')
    configure_safety = _lifecycle('/araco/safety_supervisor', 'configure')
    activate_safety = _lifecycle('/araco/safety_supervisor', 'activate')
    holding_waiter = Node(
        package='araco_bringup', executable='wait_for_holding', output='screen',
        arguments=['--timeout', '30'], parameters=[{'use_sim_time': True}],
    )
    configure_arbiter = _lifecycle('/araco/command_arbiter', 'configure')
    activate_arbiter = _lifecycle('/araco/command_arbiter', 'activate')
    configure_teleop = _lifecycle('/araco/teleop_adapter', 'configure')
    activate_teleop = _lifecycle('/araco/teleop_adapter', 'activate')

    chain = [
        _continue_on_success(spawn, [state_spawner], 'robot spawn'),
        _continue_on_success(state_spawner, [leg_spawner], 'state broadcaster activation'),
        _continue_on_success(leg_spawner, [gimbal_spawner], 'leg controller activation'),
        _continue_on_success(
            gimbal_spawner, [configure_locomotion], 'gimbal controller activation'),
        _continue_on_success(configure_locomotion, [activate_locomotion], 'locomotion configure'),
        _continue_on_success(activate_locomotion, [configure_safety], 'locomotion activate'),
        _continue_on_success(configure_safety, [activate_safety], 'safety configure'),
        _continue_on_success(activate_safety, [holding_waiter], 'safety activate'),
        _continue_on_success(holding_waiter, [configure_arbiter], 'HOLDING readiness'),
        _continue_on_success(configure_arbiter, [activate_arbiter], 'arbiter configure'),
    ]
    if profile == 'gazebo_dev_v0':
        chain.extend([
            _continue_on_success(activate_arbiter, [configure_teleop], 'arbiter activate'),
            _continue_on_success(configure_teleop, [activate_teleop], 'teleop configure'),
        ])

    optional_nodes = [teleop] if profile == 'gazebo_dev_v0' else []
    return [
        gazebo_resource_path, gazebo, bridge, robot_state_publisher, contact_filter,
        locomotion, safety, arbiter, *optional_nodes, spawn,
        *[RegisterEventHandler(item) for item in chain],
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile', default_value='gazebo_dev_v0',
            choices=['gazebo_dev_v0', 'gazebo_ci_v0']),
        DeclareLaunchArgument(
            'runtime_bundle', default_value='',
            description='Fresh absent path for the immutable effective configuration.'),
        OpaqueFunction(function=_runtime_actions),
    ])
