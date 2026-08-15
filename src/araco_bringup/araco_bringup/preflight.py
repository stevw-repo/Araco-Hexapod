# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

"""Fail-closed validation of a composed runtime bundle before process start."""

from __future__ import annotations

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
import yaml


class PreflightError(RuntimeError):
    """A runtime bundle cannot safely be passed to launch."""


def _document(path: Path):
    try:
        text = path.read_text(encoding='utf-8')
        return yaml.safe_load(text) if path.suffix == '.yaml' else json.loads(text)
    except (FileNotFoundError, json.JSONDecodeError, yaml.YAMLError) as error:
        raise PreflightError(f'missing or invalid runtime file: {path.name}') from error


def _package_uri(uri: str) -> Path:
    if not uri.startswith('package://'):
        raise PreflightError(f'unresolved mesh URI: {uri}')
    package, separator, relative = uri.removeprefix('package://').partition('/')
    if not package or not separator or not relative:
        raise PreflightError(f'unresolved mesh URI: {uri}')
    try:
        path = Path(get_package_share_directory(package)) / relative
    except PackageNotFoundError as error:
        raise PreflightError(f'unresolved mesh package: {uri}') from error
    if not path.is_file():
        raise PreflightError(f'unresolved mesh resource: {uri}')
    return path


def preflight_bundle(bundle: Path) -> dict:
    """Validate exact Gate 1 files, partitions, resources, and initial state."""
    bundle = Path(bundle).resolve()
    manifest = _document(bundle / 'manifest.json')
    partitions = manifest.get('controller_partitions', {})
    legs = partitions.get('leg_joints', [])
    gimbal = partitions.get('gimbal_joints', [])
    states = partitions.get('state_joints', [])
    if (len(legs) != 24 or len(set(legs)) != 24 or
            gimbal != ['gimbal_yaw_joint'] or len(states) != 25 or
            len(set(states)) != 25 or set(states) != set(legs) | set(gimbal)):
        raise PreflightError('incorrect 24+1 controller partition')

    urdf_path = bundle / 'description/robot.urdf'
    try:
        root = ET.parse(urdf_path).getroot()
    except (FileNotFoundError, ET.ParseError) as error:
        raise PreflightError('missing or invalid expanded robot description') from error
    for mesh in root.findall('.//mesh'):
        _package_uri(mesh.attrib.get('filename', ''))
    systems = root.findall('ros2_control')
    if len(systems) != 1:
        raise PreflightError('expanded description lacks one ros2_control system')
    control_joints = systems[0].findall('joint')
    if {item.attrib.get('name') for item in control_joints} != set(states):
        raise PreflightError('ros2_control joint partition does not match manifest')
    for joint in control_joints:
        state = next(
            (item for item in joint.findall('state_interface')
             if item.attrib.get('name') == 'position'), None)
        initial = None if state is None else next(
            (item.text for item in state.findall('param')
             if item.attrib.get('name') == 'initial_value'), None)
        try:
            valid = initial is not None and math.isfinite(float(initial))
        except ValueError:
            valid = False
        if not valid:
            raise PreflightError(
                f'invalid initial state for {joint.attrib.get("name", "unknown")}')

    controller_files = {
        'joint_state_broadcaster': states,
        'leg_trajectory_controller': legs,
        'gimbal_trajectory_controller': gimbal,
    }
    for name, expected in controller_files.items():
        document = _document(bundle / f'ros2_control/{name}.yaml')
        parameters = document.get(name, {}).get('ros__parameters', {})
        if parameters.get('joints') != expected:
            raise PreflightError(f'incorrect controller file partition: {name}')
    bridge = _document(bundle / 'gazebo/bridge.yaml')
    if not any(
            item.get('ros_topic_name') == '/clock' and
            item.get('direction') == 'GZ_TO_ROS' for item in bridge):
        raise PreflightError('Gazebo bridge lacks required /clock input')
    try:
        ET.parse(bundle / 'gazebo/resolved_world.sdf')
    except (FileNotFoundError, ET.ParseError) as error:
        raise PreflightError('missing or invalid Gazebo world') from error
    return {
        'status': 'PASS',
        'state_joint_count': len(states),
        'leg_joint_count': len(legs),
        'gimbal_joint_count': len(gimbal),
        'mesh_count': len(root.findall('.//mesh')),
        'clock_bridge': True,
    }
