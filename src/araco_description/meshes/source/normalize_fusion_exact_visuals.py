#!/usr/bin/env python3
# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT
"""Generate exact, material-classified link-local presentation meshes."""

from __future__ import annotations

import argparse
from pathlib import Path

from normalize_fusion_visuals import (
    _canonical_link_transforms,
    _load_json,
    _occurrence_transform,
    _read_binary_stl,
    _safe_source_path,
    _sha256,
    _write_binary_stl,
    _write_json,
)
import numpy as np


EXPECTED_SOURCE_SCHEMA = {'name': 'araco_fusion_visual_source', 'version': 1}
EXPECTED_ROLE_COUNTS = {
    'primary': 25,
    'servo_case': 13,
    'tibia_component_auto': 6,
    'camera_body': 5,
    'camera_hardware': 6,
    'camera_optics': 4,
}
EXPECTED_TIBIA_COMPONENT_TRIANGLES = [3864, 3864, 6448, 48436, 48436]
EXPECTED_OUTPUT_ROLE_COUNTS = {
    'primary': 26,
    'servo_case': 13,
    'servo_horn': 7,
    'camera_body': 1,
    'camera_hardware': 1,
    'camera_optics': 1,
}


def _connected_triangle_components(triangles: np.ndarray) -> list[np.ndarray]:
    """Return triangle-index arrays connected by shared float32 STL vertices."""
    triangle_count = len(triangles)
    flattened = triangles.reshape(-1, 3)
    _, inverse = np.unique(flattened, axis=0, return_inverse=True)
    parent = np.arange(triangle_count, dtype=np.int64)
    sizes = np.ones(triangle_count, dtype=np.int64)

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = int(parent[index])
        return index

    def union(first: int, second: int) -> None:
        first_root = find(first)
        second_root = find(second)
        if first_root == second_root:
            return
        if sizes[first_root] < sizes[second_root]:
            first_root, second_root = second_root, first_root
        parent[second_root] = first_root
        sizes[first_root] += sizes[second_root]

    first_triangle_by_vertex: dict[int, int] = {}
    for triangle_index in range(triangle_count):
        for vertex_index in inverse[triangle_index * 3:triangle_index * 3 + 3]:
            key = int(vertex_index)
            previous = first_triangle_by_vertex.setdefault(key, triangle_index)
            union(triangle_index, previous)

    roots = np.fromiter(
        (find(index) for index in range(triangle_count)),
        dtype=np.int64,
        count=triangle_count,
    )
    return [
        np.flatnonzero(roots == root)
        for root in np.unique(roots)
    ]


def _classify_entry(entry: dict, triangles: np.ndarray) -> dict[str, np.ndarray]:
    role = entry['visual_role']
    if role == 'primary':
        return {'primary': triangles}
    if role in {'camera_body', 'camera_hardware', 'camera_optics'}:
        return {role: triangles}

    components = _connected_triangle_components(triangles)
    components.sort(key=len)
    counts = [len(component) for component in components]
    if role == 'tibia_component_auto':
        if counts != EXPECTED_TIBIA_COMPONENT_TRIANGLES:
            raise ValueError(
                'tibia connected-solid topology changed for {}: {}'.format(
                    entry['asset_id'], counts
                )
            )
        return {
            'servo_horn': triangles[np.concatenate(components[:2])],
            'primary': triangles[components[2]],
            'servo_case': triangles[np.concatenate(components[3:])],
        }

    if role != 'servo_case':
        raise ValueError('unsupported exact visual role: {}'.format(role))
    if len(components) == 1:
        # The DS5160 case and horn form one connected vendor solid.
        return {'servo_case': triangles}
    if len(components) == 2 and counts[0] in {4004, 4116} and counts[1] in {51422, 52584}:
        # The DS3235 exports preserve separate case and horn shells.
        return {
            'servo_horn': triangles[components[0]],
            'servo_case': triangles[components[1]],
        }
    raise ValueError(
        'servo connected-solid topology changed for {}: {}'.format(
            entry['asset_id'], counts
        )
    )


def normalize(source_manifest_path: Path, model_path: Path, pose_path: Path, output: Path):
    source = _load_json(source_manifest_path)
    model = _load_json(model_path)['data']
    pose = _load_json(pose_path)['data']
    if source.get('schema') != EXPECTED_SOURCE_SCHEMA:
        raise ValueError('unsupported Fusion visual source schema')
    if source.get('raw_mesh_encoding') != {
        'format': 'binary_stl',
        'length_unit': 'millimeter',
        'coordinate_space': 'source_component_local',
    }:
        raise ValueError('unsupported raw mesh encoding or coordinate space')
    if source.get('export_count') != 59 or source.get('reviewed_body_count') != 77:
        raise ValueError('exact source manifest must contain 59 exports and 77 bodies')
    if source.get('retained_proxy_links') != []:
        raise ValueError('exact source manifest unexpectedly retains visual proxies')
    if source.get('rights_boundary', {}).get('license') != 'mixed-open-source':
        raise ValueError('exact source manifest lacks the mixed-open-source boundary')
    actual_role_counts = {
        role: sum(entry['visual_role'] == role for entry in source['exports'])
        for role in EXPECTED_ROLE_COUNTS
    }
    if actual_role_counts != EXPECTED_ROLE_COUNTS:
        raise ValueError('exact source role inventory changed: {}'.format(actual_role_counts))

    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise ValueError('refusing to overwrite non-empty mesh output: {}'.format(output))

    canonical_transforms = _canonical_link_transforms(model, pose)
    fusion_to_ros = np.asarray(model['frame_conversion']['fusion_to_ros_rotation'], dtype=float)
    base_datum = np.asarray(model['frame_conversion']['base_datum_fusion_m'], dtype=float)
    if not np.allclose(fusion_to_ros.T @ fusion_to_ros, np.eye(3), atol=1e-12):
        raise ValueError('Fusion-to-ROS rotation is not orthonormal')

    source_root = source_manifest_path.parent
    grouped: dict[tuple[str, str], list[np.ndarray]] = {}
    grouped_sources: dict[tuple[str, str], list[dict]] = {}
    raw_hashes_verified: set[str] = set()
    classification_records = []
    for entry in source['exports']:
        raw_path = _safe_source_path(source_root, entry['raw_mesh_path'])
        if raw_path.stat().st_size != entry['raw_mesh_size_bytes']:
            raise ValueError('raw mesh size mismatch: {}'.format(raw_path))
        if entry['raw_mesh_sha256'] not in raw_hashes_verified:
            if _sha256(raw_path) != entry['raw_mesh_sha256']:
                raise ValueError('raw mesh hash mismatch: {}'.format(raw_path))
            raw_hashes_verified.add(entry['raw_mesh_sha256'])
        triangles_mm = _read_binary_stl(raw_path)
        if len(triangles_mm) != entry['raw_triangle_count']:
            raise ValueError('raw triangle count mismatch: {}'.format(raw_path))

        occurrence_rotation, occurrence_translation = _occurrence_transform(
            entry['source_occurrence_transform_root_fusion']
        )
        fusion_root = np.einsum(
            'ij,tkj->tki', occurrence_rotation, triangles_mm * 0.001
        )
        fusion_root += occurrence_translation
        ros_base = np.einsum('ij,tkj->tki', fusion_to_ros, fusion_root - base_datum)

        link_name = entry['canonical_link']
        if link_name not in canonical_transforms:
            raise ValueError('unknown canonical link: {}'.format(link_name))
        link_rotation, link_translation = canonical_transforms[link_name]
        link_local = np.einsum('ij,tkj->tki', link_rotation.T, ros_base - link_translation)
        classified = _classify_entry(entry, link_local)
        classification_records.append({
            'asset_id': entry['asset_id'],
            'input_triangle_count': len(link_local),
            'output_roles': {
                role: len(role_triangles)
                for role, role_triangles in sorted(classified.items())
            },
        })
        source_record = {
            'asset_id': entry['asset_id'],
            'creator': entry['creator'],
            'license': entry['license'],
        }
        for output_role, role_triangles in classified.items():
            key = (link_name, output_role)
            grouped.setdefault(key, []).append(role_triangles)
            grouped_sources.setdefault(key, []).append(source_record)

    output_role_counts = {
        role: sum(output_role == role for _, output_role in grouped)
        for role in EXPECTED_OUTPUT_ROLE_COUNTS
    }
    if output_role_counts != EXPECTED_OUTPUT_ROLE_COUNTS:
        raise ValueError('exact output role coverage changed: {}'.format(output_role_counts))

    outputs = []
    output_link_order = model['primary_link_order'] + [
        frame['child']
        for frame in model.get('fixed_frames', [])
        if any(key[0] == frame['child'] for key in grouped)
    ]
    for link_name in output_link_order:
        for output_role in (
            'primary',
            'servo_case',
            'servo_horn',
            'camera_body',
            'camera_hardware',
            'camera_optics',
        ):
            key = (link_name, output_role)
            if key not in grouped:
                continue
            triangles = np.concatenate(grouped[key], axis=0)
            if not np.all(np.isfinite(triangles)):
                raise ValueError('normalized mesh contains non-finite vertices: {}'.format(key))
            normal_lengths = np.linalg.norm(
                np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]),
                axis=1,
            )
            removed_degenerate_count = int(np.count_nonzero(normal_lengths <= 1e-15))
            triangles = triangles[normal_lengths > 1e-15]
            if not len(triangles):
                raise ValueError(
                    'normalized mesh contains only degenerate triangles: {}'.format(key)
                )
            stem = link_name.removesuffix('_link')
            filename = (
                '{}.stl'.format(stem)
                if output_role == 'primary'
                else '{}__{}.stl'.format(stem, output_role)
            )
            path = output / filename
            degenerate_count = _write_binary_stl(
                path, '{} {}'.format(link_name, output_role), triangles
            )
            minimum = np.min(triangles.reshape(-1, 3), axis=0)
            maximum = np.max(triangles.reshape(-1, 3), axis=0)
            sources = grouped_sources[key]
            outputs.append({
                'canonical_link': link_name,
                'visual_role': output_role,
                'path': filename,
                'sha256': _sha256(path),
                'size_bytes': path.stat().st_size,
                'triangle_count': len(triangles),
                'degenerate_triangle_count': degenerate_count,
                'removed_source_degenerate_triangle_count': removed_degenerate_count,
                'bounds_min_m': minimum.tolist(),
                'bounds_max_m': maximum.tolist(),
                'source_asset_ids': [item['asset_id'] for item in sources],
                'creators': sorted({item['creator'] for item in sources}),
                'licenses': sorted({item['license'] for item in sources}),
            })

    normalization_manifest = {
        'schema': {'name': 'araco_fusion_exact_visual_normalization', 'version': 1},
        'generator': {
            'name': 'normalize_fusion_exact_visuals.py',
            'coordinate_math': 'float64',
            'output_encoding': 'deterministic_binary_stl_float32',
            'connected_solid_classification': 'exact_float32_shared_vertex_topology',
        },
        'inputs': {
            'source_manifest_sha256': _sha256(source_manifest_path),
            'canonical_model_sha256': _sha256(model_path),
            'nominal_pose_sha256': _sha256(pose_path),
        },
        'transform_pipeline': [
            'millimeter source-component-local STL to meters',
            'recorded Fusion occurrence transform to Fusion root',
            'accepted Fusion-to-ROS rotation and base datum to base_link',
            'inverse canonical nominal link transform to ROS link-local coordinates',
        ],
        'alignment_policy': 'preserve recorded Fusion occurrence; no forced tibia alignment',
        'visual_only': True,
        'collision_changed': False,
        'dynamics_changed': False,
        'joint_contract_changed': False,
        'retained_proxy_links': [],
        'classification_records': classification_records,
        'output_count': len(outputs),
        'output_role_counts': output_role_counts,
        'outputs': outputs,
    }
    _write_json(output / 'normalization_manifest.json', normalization_manifest)
    print('Generated {} exact link-local presentation meshes in {}'.format(len(outputs), output))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source_manifest', type=Path)
    parser.add_argument('canonical_model', type=Path)
    parser.add_argument('nominal_pose', type=Path)
    parser.add_argument('output', type=Path)
    arguments = parser.parse_args()
    normalize(
        arguments.source_manifest.resolve(),
        arguments.canonical_model.resolve(),
        arguments.nominal_pose.resolve(),
        arguments.output.resolve(),
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
