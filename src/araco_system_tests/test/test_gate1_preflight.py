# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

import json
import shutil
import xml.etree.ElementTree as ET

from araco_bringup.composer import compose_profile
from araco_bringup.preflight import preflight_bundle
from araco_bringup.preflight import PreflightError
import pytest
import yaml


@pytest.fixture
def composed(tmp_path):
    bundle = tmp_path / 'original'
    compose_profile('gazebo_ci_v0', bundle)
    assert preflight_bundle(bundle)['status'] == 'PASS'
    return bundle


def _copy(composed, tmp_path):
    target = tmp_path / 'corrupted'
    shutil.copytree(composed, target)
    return target


def test_missing_model_fails_before_launch(composed, tmp_path):
    bundle = _copy(composed, tmp_path)
    (bundle / 'description/robot.urdf').unlink()
    with pytest.raises(PreflightError, match='robot description'):
        preflight_bundle(bundle)


def test_missing_controller_file_fails_before_launch(composed, tmp_path):
    bundle = _copy(composed, tmp_path)
    (bundle / 'ros2_control/leg_trajectory_controller.yaml').unlink()
    with pytest.raises(PreflightError, match='runtime file'):
        preflight_bundle(bundle)


def test_incorrect_partition_fails_before_launch(composed, tmp_path):
    bundle = _copy(composed, tmp_path)
    path = bundle / 'manifest.json'
    document = json.loads(path.read_text(encoding='utf-8'))
    document['controller_partitions']['leg_joints'].pop()
    path.write_text(json.dumps(document), encoding='utf-8')
    with pytest.raises(PreflightError, match=r'24\+1'):
        preflight_bundle(bundle)


def test_unresolved_mesh_fails_before_launch(composed, tmp_path):
    bundle = _copy(composed, tmp_path)
    path = bundle / 'description/robot.urdf'
    tree = ET.parse(path)
    tree.getroot().find('.//mesh').set(
        'filename', 'package://araco_description/meshes/generated/missing.stl')
    tree.write(path, encoding='unicode')
    with pytest.raises(PreflightError, match='unresolved mesh'):
        preflight_bundle(bundle)


def test_absent_clock_bridge_fails_before_launch(composed, tmp_path):
    bundle = _copy(composed, tmp_path)
    path = bundle / 'gazebo/bridge.yaml'
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    document = [item for item in document if item['ros_topic_name'] != '/clock']
    path.write_text(yaml.safe_dump(document), encoding='utf-8')
    with pytest.raises(PreflightError, match='/clock'):
        preflight_bundle(bundle)


def test_invalid_initial_state_fails_before_launch(composed, tmp_path):
    bundle = _copy(composed, tmp_path)
    path = bundle / 'description/robot.urdf'
    tree = ET.parse(path)
    initial = tree.getroot().find(
        './/ros2_control/joint/state_interface/param[@name="initial_value"]')
    initial.text = 'nan'
    tree.write(path, encoding='unicode')
    with pytest.raises(PreflightError, match='invalid initial state'):
        preflight_bundle(bundle)
