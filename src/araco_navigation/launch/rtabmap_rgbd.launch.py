# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

"""Launch the simulator-only registered RGB-D RTAB-Map baseline."""

from __future__ import annotations

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _contract(path: Path):
    return json.loads(path.read_text(encoding='utf-8'))['data']


def _runtime_actions(context):
    configured_path = LaunchConfiguration('config_path').perform(context)
    path = (
        Path(configured_path).expanduser().resolve()
        if configured_path else
        Path(get_package_share_directory('araco_navigation'))
        / 'config/slam/rtabmap_rgbd_sim_v0.yaml'
    )
    contract = _contract(path)
    frames = contract['frames']
    inputs = contract['inputs']
    outputs = contract['outputs']
    synchronization = contract['synchronization']
    odometry = contract['odometry']
    imu_enabled = odometry['imu_enabled']
    mapping = contract['mapping']
    grid = mapping['grid']
    sensor_qos = 2 if synchronization['sensor_qos'] == 'best_effort' else 1

    rgbd_sync = Node(
        package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync',
        namespace='araco/perception', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': True,
            'approx_sync': synchronization['approximate'],
            'approx_sync_max_interval': synchronization['maximum_interval_s'],
            'topic_queue_size': synchronization['queue_size'],
            'sync_queue_size': synchronization['queue_size'],
            'qos': sensor_qos,
            'qos_camera_info': sensor_qos,
            'depth_scale': 1.0,
        }],
        remappings=[
            ('rgb/image', inputs['rgb']),
            ('depth/image', inputs['depth']),
            ('rgb/camera_info', inputs['camera_info']),
            ('rgbd_image', outputs['rgbd']),
        ],
    )

    rgbd_odometry = Node(
        package='rtabmap_odom', executable='rgbd_odometry',
        name='rgbd_odometry', namespace='araco/perception',
        output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': True,
            'frame_id': frames['base'],
            'odom_frame_id': frames['odom'],
            'publish_tf': odometry['publish_tf'],
            'wait_for_transform': 0.2,
            'wait_imu_to_init': odometry['wait_for_imu'],
            'always_check_imu_tf': odometry['always_check_imu_tf'],
            'subscribe_rgbd': True,
            'qos': sensor_qos,
            'qos_imu': sensor_qos,
            'Odom/Strategy': '0',
            'Odom/GuessMotion': 'true',
            'Odom/ResetCountdown': str(odometry['reset_countdown']),
            'OdomF2M/MaxSize': '1000',
            'Vis/MinInliers': str(odometry['minimum_inliers']),
            'Vis/MaxFeatures': str(odometry['maximum_features']),
        }],
        remappings=[
            ('rgbd_image', outputs['rgbd']),
            ('odom', outputs['odom']),
            ('odom_info', outputs['odom_info']),
        ] + ([('imu', inputs['imu'])] if imu_enabled else []),
    )

    rtabmap = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        namespace='araco/perception', output='screen', emulate_tty=True,
        parameters=[{
            'use_sim_time': True,
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_rgbd': True,
            'subscribe_odom_info': True,
            'subscribe_imu': imu_enabled,
            'frame_id': frames['base'],
            'map_frame_id': frames['map'],
            'odom_frame_id': '',
            'publish_tf': True,
            'database_path': LaunchConfiguration('database_path'),
            'wait_for_transform': 0.2,
            'qos_image': sensor_qos,
            'qos_odom': sensor_qos,
            'qos_imu': sensor_qos,
            'Mem/IncrementalMemory': str(mapping['incremental_memory']).lower(),
            'Reg/Force3DoF': 'false',
            'Optimizer/GravitySigma': str(mapping['gravity_sigma']),
            'Rtabmap/DetectionRate': str(mapping['detection_rate_hz']),
            'RGBD/CreateOccupancyGrid': 'true',
            'RGBD/ProximityBySpace': 'true',
            'Grid/Sensor': '1',
            'Grid/3D': str(grid['three_d']).lower(),
            'Grid/CellSize': str(grid['cell_size_m']),
            'Grid/DepthDecimation': str(grid['depth_decimation']),
            'Grid/RangeMin': str(grid['minimum_range_m']),
            'Grid/RangeMax': str(grid['maximum_range_m']),
            'Grid/FootprintLength': str(grid['footprint_length_m']),
            'Grid/FootprintWidth': str(grid['footprint_width_m']),
            'Grid/FootprintHeight': str(grid['footprint_height_m']),
        }],
        remappings=[
            ('rgbd_image', outputs['rgbd']),
            ('odom', outputs['odom']),
            ('odom_info', outputs['odom_info']),
            ('map', outputs['map']),
            ('cloud_map', outputs['cloud_map']),
            ('cloud_obstacles', outputs['cloud_obstacles']),
            ('cloud_ground', outputs['cloud_ground']),
        ] + ([('imu', inputs['imu'])] if imu_enabled else []),
    )

    return [
        rgbd_sync,
        rgbd_odometry,
        rtabmap,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'database_path',
            default_value=str(Path.home() / '.ros/araco_rgbd_map.db'),
            description='Persistent RTAB-Map database; existing data is reused.'),
        DeclareLaunchArgument(
            'config_path', default_value='',
            description='Exact composed RGB-D SLAM contract to launch.'),
        OpaqueFunction(function=_runtime_actions),
    ])
