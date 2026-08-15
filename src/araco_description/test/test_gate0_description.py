# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def _artifact(relative_path):
    return json.loads((ROOT / relative_path).read_text(encoding='utf-8'))['data']


def _rotation_x(angle):
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array([[1, 0, 0], [0, cosine, -sine], [0, sine, cosine]])


def _rotation_y(angle):
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array([[cosine, 0, sine], [0, 1, 0], [-sine, 0, cosine]])


def _rotation_z(angle):
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array([[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]])


def _rpy(values):
    return _rotation_z(values[2]) @ _rotation_y(values[1]) @ _rotation_x(values[0])


def _nominal_boxes(model, pose):
    transforms = {'base_link': (np.eye(3), np.zeros(3))}
    for joint in model['joints']:
        parent_rotation, parent_translation = transforms[joint['parent']]
        joint_rotation = parent_rotation @ _rpy(joint['origin_rpy_rad'])
        joint_translation = (
            parent_translation + parent_rotation @ np.array(joint['origin_xyz_m'])
        )
        angle = pose['joint_positions_rad'][joint['name']]
        motion = _rotation_z(angle) if joint['axis'][2] else _rotation_y(angle)
        transforms[joint['child']] = (joint_rotation @ motion, joint_translation)
    boxes = {}
    for link in model['links']:
        geometry = model['geometry_classes'][link['geometry_class']]
        rotation, translation = transforms[link['name']]
        boxes[link['name']] = (
            translation + rotation @ np.array(geometry['collision_origin_xyz_m']),
            rotation,
            np.array(geometry['collision_size_m']) / 2.0,
        )
    return boxes


def _overlap(first, second):
    center_a, rotation_a, half_a = first
    center_b, rotation_b, half_b = second
    relative = rotation_a.T @ rotation_b
    absolute = np.abs(relative) + 1e-12
    translation = rotation_a.T @ (center_b - center_a)
    for axis in range(3):
        if abs(translation[axis]) > half_a[axis] + half_b @ absolute[axis, :]:
            return False
    for axis in range(3):
        if abs(translation @ relative[:, axis]) > half_b[axis] + half_a @ absolute[:, axis]:
            return False
    for axis_a in range(3):
        for axis_b in range(3):
            radius_a = (
                half_a[(axis_a + 1) % 3] * absolute[(axis_a + 2) % 3, axis_b]
                + half_a[(axis_a + 2) % 3] * absolute[(axis_a + 1) % 3, axis_b]
            )
            radius_b = (
                half_b[(axis_b + 1) % 3] * absolute[axis_a, (axis_b + 2) % 3]
                + half_b[(axis_b + 2) % 3] * absolute[axis_a, (axis_b + 1) % 3]
            )
            distance = abs(
                translation[(axis_a + 2) % 3] * relative[(axis_a + 1) % 3, axis_b]
                - translation[(axis_a + 1) % 3] * relative[(axis_a + 2) % 3, axis_b]
            )
            if distance > radius_a + radius_b:
                return False
    return True


def test_canonical_tree_roles_axes_and_pose_margin():
    model = _artifact('config/model/canonical_model_v1.yaml')
    limits = _artifact('config/limits/provisional_sim_v0.yaml')
    pose = _artifact('config/poses/nominal_standing_reference_v0.yaml')
    assert len(model['links']) == 26
    assert len(model['joints']) == 25
    assert len([joint for joint in model['joints'] if 'leg_command' in joint['roles']]) == 24
    assert [joint['name'] for joint in model['joints'] if 'gimbal_command' in joint['roles']] == [
        'gimbal_yaw_joint'
    ]
    assert all(abs(np.linalg.norm(joint['axis']) - 1.0) <= 1e-9 for joint in model['joints'])
    for joint in model['joints']:
        target = pose['joint_positions_rad'][joint['name']]
        selected = limits['classes'][limits['assignments'][joint['name']]]
        assert selected['lower_rad'] < target < selected['upper_rad']
    assert abs(np.linalg.norm(pose['base_pose']['orientation_xyzw']) - 1.0) <= 1e-9


def test_proxy_inclusive_mass_and_inertia_are_valid():
    dynamics = _artifact('config/dynamics/rough_estimate_v0.yaml')
    total_mass = sum(link['mass_kg'] for link in dynamics['links'].values())
    assert abs(total_mass - 3.924392774795984) <= 0.001
    assert {proxy['id'] for proxy in dynamics['base_proxies']} == {
        'pisugar_3_plus', 'main_servo_battery', 'servo_controller'
    }
    for link in dynamics['links'].values():
        inertia = link['inertia_kg_m2']
        matrix = np.array([
            [inertia['ixx'], inertia['ixy'], inertia['ixz']],
            [inertia['ixy'], inertia['iyy'], inertia['iyz']],
            [inertia['ixz'], inertia['iyz'], inertia['izz']],
        ])
        eigenvalues = np.linalg.eigvalsh(matrix)
        assert np.min(eigenvalues) > 1e-9
        assert eigenvalues[0] + eigenvalues[1] >= eigenvalues[2] - 1e-9


def test_resources_are_redistributable_hashed_and_reproducible(tmp_path):
    resources = _artifact('config/resources/robot_description_v1.yaml')['resources']
    assert all(
        item['redistribution'] == 'allowed' and item['license'] != 'UNKNOWN'
        for item in resources
    )
    for item in resources:
        relative = item['uri'].removeprefix('package://araco_description/')
        assert hashlib.sha256((ROOT / relative).read_bytes()).hexdigest() == item['sha256']
    subprocess.run([
        sys.executable,
        str(ROOT / 'meshes/source/generate_primitives.py'),
        str(ROOT / 'config/model/canonical_model_v1.yaml'),
        str(tmp_path),
    ], check=True)
    for generated in sorted((ROOT / 'meshes/generated').glob('*.stl')):
        assert generated.read_bytes() == (tmp_path / generated.name).read_bytes()

    exact_output = tmp_path / 'presentation_exact'
    subprocess.run([
        sys.executable,
        str(ROOT / 'meshes/source/normalize_fusion_exact_visuals.py'),
        str(ROOT / 'meshes/source/fusion_v2_exact_camera/source_manifest.json'),
        str(ROOT / 'config/model/canonical_model_v1.yaml'),
        str(ROOT / 'config/poses/nominal_standing_reference_v0.yaml'),
        str(exact_output),
    ], check=True)
    for generated in sorted((ROOT / 'meshes/presentation_exact').iterdir()):
        assert generated.read_bytes() == (exact_output / generated.name).read_bytes()


def test_fusion_exact_visuals_are_complete_link_local_and_auditable():
    model = _artifact('config/model/canonical_model_v1.yaml')
    resources = _artifact('config/resources/robot_description_v1.yaml')['resources']
    source = json.loads(
        (ROOT / 'meshes/source/fusion_v2_exact_camera/source_manifest.json').read_text(
            encoding='utf-8'
        )
    )
    normalization = json.loads(
        (ROOT / 'meshes/presentation_exact/normalization_manifest.json').read_text(
            encoding='utf-8'
        )
    )
    tibia_links = {
        link['name'] for link in model['links'] if link['geometry_class'] == 'tibia'
    }
    primary_links = {
        link['name'] for link in model['links'] if 'visual_mesh_uri' in link
    }
    assert primary_links == set(model['primary_link_order'])
    assert len(tibia_links) == 6
    assert source['unique_raw_mesh_count'] == 34
    assert source['export_count'] == 59
    assert source['reviewed_body_count'] == 77
    assert source['retained_proxy_links'] == []
    assert source['raw_mesh_encoding']['coordinate_space'] == 'source_component_local'
    assert normalization['visual_only'] is True
    assert normalization['collision_changed'] is False
    assert normalization['dynamics_changed'] is False
    assert normalization['joint_contract_changed'] is False
    assert normalization['alignment_policy'] == (
        'preserve recorded Fusion occurrence; no forced tibia alignment'
    )
    assert normalization['retained_proxy_links'] == []
    assert normalization['output_count'] == 49
    assert normalization['output_role_counts'] == {
        'primary': 26,
        'servo_case': 13,
        'servo_horn': 7,
        'camera_body': 1,
        'camera_hardware': 1,
        'camera_optics': 1,
    }
    assert sum(entry['triangle_count'] for entry in normalization['outputs']) == 2066740
    assert sum(entry['degenerate_triangle_count'] for entry in normalization['outputs']) == 0
    assert sum(
        entry['removed_source_degenerate_triangle_count']
        for entry in normalization['outputs']
    ) == 36
    assert {
        entry['canonical_link'] for entry in normalization['outputs']
        if entry['visual_role'] == 'primary'
    } == primary_links
    assert {
        resource['uri'] for resource in resources
        if resource['id'].endswith('_fusion_exact_visual')
    } == {
        'package://araco_description/meshes/presentation_exact/' + entry['path']
        for entry in normalization['outputs']
    }
    for raw_mesh in (ROOT / 'meshes/source/fusion_v2_exact_camera/raw_by_sha256').glob('*.stl'):
        assert hashlib.sha256(raw_mesh.read_bytes()).hexdigest() == raw_mesh.stem
    for output in normalization['outputs']:
        bounds = np.array([output['bounds_min_m'], output['bounds_max_m']])
        assert np.all(np.isfinite(bounds))
        assert np.max(np.abs(bounds)) < 0.5
        assert np.all(bounds[1] > bounds[0])


def test_exact_visual_policy_represents_tibias_and_known_servo_inventory():
    model = _artifact('config/model/canonical_model_v1.yaml')
    policy = model['visual_policy']
    assert policy['fidelity'] == 'fusion_exact_presentation'
    inventory = policy['exact_mesh_inventory']
    assert inventory['reviewed_source_bodies'] == 77
    assert inventory['servo_models'] == {'DS3235': 19, 'DS5160': 6}
    assert inventory['tibia_components'] == 6
    assert inventory['gemini_335_exterior'] == {
        'housing_and_bracket_bodies': 5,
        'hardware_bodies': 6,
        'optical_bodies': 4,
        'internal_bodies_included': 0,
    }
    assert inventory['retained_visual_proxies'] == 0
    assert inventory['alignment_policy'] == 'preserve_fusion_occurrence_no_force_alignment'
    auxiliary = [item for link in model['links'] for item in link.get('auxiliary_visuals', [])]
    assert sum(item['role'] == 'servo_case' for item in auxiliary) == 13
    assert sum(item['role'] == 'servo_horn' for item in auxiliary) == 7
    camera_frame = next(
        frame for frame in model['fixed_frames'] if frame['child'] == 'camera_link'
    )
    assert {item['role'] for item in camera_frame['visuals']} == {
        'camera_body', 'camera_hardware', 'camera_optics'
    }
    for link in model['links']:
        if link['geometry_class'] == 'tibia':
            assert {item['role'] for item in link['auxiliary_visuals']} == {
                'servo_case', 'servo_horn'
            }


def test_nominal_proxy_collision_has_no_unexpected_pairs():
    model = _artifact('config/model/canonical_model_v1.yaml')
    pose = _artifact('config/poses/nominal_standing_reference_v0.yaml')
    boxes = _nominal_boxes(model, pose)
    adjacent = {
        frozenset((joint['parent'], joint['child']))
        for joint in model['joints']
    }
    unexpected = []
    names = list(boxes)
    for index, first_name in enumerate(names):
        for second_name in names[index + 1:]:
            if frozenset((first_name, second_name)) in adjacent:
                continue
            if _overlap(boxes[first_name], boxes[second_name]):
                unexpected.append((first_name, second_name))
    assert unexpected == []
