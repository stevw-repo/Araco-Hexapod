# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

import json
from pathlib import Path

import jsonschema
import yaml


ROOT = Path(__file__).resolve().parents[1]


def _sensor_artifact(name):
    return json.loads((ROOT / f'config/sensors/{name}').read_text(encoding='utf-8'))


def test_gemini_simulator_contracts_validate_and_use_canonical_frames():
    schema = json.loads((ROOT / 'schema/config_v1.schema.json').read_text(encoding='utf-8'))
    jsonschema.Draft202012Validator.check_schema(schema)
    for name in ('gemini_335_sim_v0.yaml', 'gemini_335_registered_sim_v0.yaml'):
        data = _sensor_artifact(name)['data']
        jsonschema.Draft202012Validator(schema).validate(data)
        assert data['color']['optical_frame'] == 'camera_color_optical_frame'
        assert data['depth']['optical_frame'] == 'camera_depth_optical_frame'
        assert data['depth']['point_cloud_frame'] == 'camera_link'
        assert data['imu']['frame'] == 'camera_link'
        assert data['depth']['near_clip_m'] < data['depth']['far_clip_m']
        assert data['color']['near_clip_m'] < data['color']['far_clip_m']


def test_slam_sensor_variant_has_registered_rgb_and_depth_intrinsics():
    data = _sensor_artifact('gemini_335_registered_sim_v0.yaml')['data']
    for key in ('width', 'height', 'horizontal_fov_rad', 'update_rate_hz'):
        assert data['color'][key] == data['depth'][key]


def test_rviz_layout_covers_robot_tf_rgb_depth_and_points():
    rviz = yaml.safe_load((ROOT / 'rviz/gemini_rgbd_v0.rviz').read_text(encoding='utf-8'))
    displays = rviz['Visualization Manager']['Displays']
    classes = [display['Class'] for display in displays]
    assert 'rviz_default_plugins/RobotModel' in classes
    assert 'rviz_default_plugins/TF' in classes
    assert classes.count('rviz_default_plugins/Image') == 2
    assert 'rviz_default_plugins/PointCloud2' in classes
    topics = {display.get('Topic') for display in displays}
    assert '/araco/camera/color/image_raw' in topics
    assert '/araco/camera/depth/image_raw' in topics
    assert '/araco/camera/depth/points' in topics


def test_rtabmap_rviz_layout_covers_2d_and_colored_3d_maps():
    rviz = yaml.safe_load((
        ROOT / 'rviz/rtabmap_rgbd_v0.rviz'
    ).read_text(encoding='utf-8'))
    manager = rviz['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'map'
    displays = manager['Displays']
    assert any(
        display['Class'] == 'rviz_default_plugins/Map'
        and display['Topic']['Value'] == '/araco/perception/map'
        for display in displays
    )
    assert any(
        display['Class'] == 'rviz_default_plugins/PointCloud2'
        and display['Topic']['Value'] == '/araco/perception/cloud_map'
        and display['Color Transformer'] == 'RGB8'
        for display in displays
    )
