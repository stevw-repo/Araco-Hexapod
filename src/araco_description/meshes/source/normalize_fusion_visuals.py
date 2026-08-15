#!/usr/bin/env python3
# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT
"""Generate deterministic meter-scale ROS link-local meshes from Fusion source."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct

import numpy as np


EXPECTED_SOURCE_SCHEMA = {'name': 'araco_fusion_visual_source', 'version': 1}
EXPECTED_TIBIA_LINKS = {
    'left_front_tibia_link',
    'left_middle_tibia_link',
    'left_rear_tibia_link',
    'right_front_tibia_link',
    'right_middle_tibia_link',
    'right_rear_tibia_link',
}


def _load_json(path: Path):
    return json.loads(path.read_text(encoding='utf-8'))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _write_json(path: Path, value) -> None:
    path.write_text(
        json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n',
        encoding='utf-8',
    )


def _rotation_x(angle: float) -> np.ndarray:
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array([[1, 0, 0], [0, cosine, -sine], [0, sine, cosine]], dtype=float)


def _rotation_y(angle: float) -> np.ndarray:
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array([[cosine, 0, sine], [0, 1, 0], [-sine, 0, cosine]], dtype=float)


def _rotation_z(angle: float) -> np.ndarray:
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array([[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]], dtype=float)


def _rpy(values) -> np.ndarray:
    return _rotation_z(values[2]) @ _rotation_y(values[1]) @ _rotation_x(values[0])


def _axis_angle(axis, angle: float) -> np.ndarray:
    vector = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(vector)
    if not math.isfinite(norm) or abs(norm - 1.0) > 1e-9:
        raise ValueError('joint axis is not finite and unit length: {}'.format(axis))
    x_axis, y_axis, z_axis = vector / norm
    cross = np.array(
        [[0.0, -z_axis, y_axis], [z_axis, 0.0, -x_axis], [-y_axis, x_axis, 0.0]]
    )
    return np.eye(3) + math.sin(angle) * cross + (1.0 - math.cos(angle)) * (cross @ cross)


def _canonical_link_transforms(model, pose):
    transforms = {'base_link': (np.eye(3), np.zeros(3))}
    positions = pose['joint_positions_rad']
    for joint in model['joints']:
        if joint['parent'] not in transforms:
            raise ValueError('canonical joints are not in parent-before-child order')
        parent_rotation, parent_translation = transforms[joint['parent']]
        origin_rotation = _rpy(joint['origin_rpy_rad'])
        joint_rotation = parent_rotation @ origin_rotation
        joint_translation = (
            parent_translation
            + parent_rotation @ np.asarray(joint['origin_xyz_m'], dtype=float)
        )
        transforms[joint['child']] = (
            joint_rotation @ _axis_angle(joint['axis'], positions[joint['name']]),
            joint_translation,
        )
    for frame in model.get('fixed_frames', []):
        if frame['parent'] not in transforms:
            raise ValueError('canonical fixed frames are not in parent-before-child order')
        parent_rotation, parent_translation = transforms[frame['parent']]
        transforms[frame['child']] = (
            parent_rotation @ _rpy(frame['origin_rpy_rad']),
            parent_translation
            + parent_rotation @ np.asarray(frame['origin_xyz_m'], dtype=float),
        )
    return transforms


def _occurrence_transform(record):
    rotation = np.column_stack(
        [
            [record[field][axis] for axis in ('x', 'y', 'z')]
            for field in ('x_axis', 'y_axis', 'z_axis')
        ]
    )
    translation = np.array(
        [record['origin_m'][axis] for axis in ('x', 'y', 'z')], dtype=float
    )
    if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
        raise ValueError('occurrence transform is non-finite')
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9):
        raise ValueError('occurrence rotation is not orthonormal')
    if np.linalg.det(rotation) < 0.999999 or np.linalg.det(rotation) > 1.000001:
        raise ValueError('occurrence rotation is not proper')
    return rotation, translation


def _read_binary_stl(path: Path):
    payload = path.read_bytes()
    if len(payload) < 84:
        raise ValueError('binary STL is shorter than its header: {}'.format(path))
    triangle_count = struct.unpack_from('<I', payload, 80)[0]
    if len(payload) != 84 + triangle_count * 50:
        raise ValueError('binary STL length/count mismatch: {}'.format(path))
    dtype = np.dtype(
        [('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')]
    )
    records = np.frombuffer(payload, dtype=dtype, offset=84, count=triangle_count)
    vertices = records['vertices'].astype(np.float64)
    if not np.all(np.isfinite(vertices)):
        raise ValueError('binary STL contains non-finite vertices: {}'.format(path))
    return vertices


def _write_binary_stl(path: Path, link_name: str, triangles: np.ndarray):
    header_text = 'Araco MIT link-local meters {}'.format(link_name).encode('ascii')
    header = header_text[:80].ljust(80, b'\0')
    first_edges = triangles[:, 1] - triangles[:, 0]
    second_edges = triangles[:, 2] - triangles[:, 0]
    normals = np.cross(first_edges, second_edges)
    lengths = np.linalg.norm(normals, axis=1)
    degenerate = lengths <= 1e-15
    normals[~degenerate] /= lengths[~degenerate, np.newaxis]
    normals[degenerate] = 0.0
    with path.open('wb') as output:
        output.write(header)
        output.write(struct.pack('<I', len(triangles)))
        for normal, vertices in zip(normals, triangles):
            values = normal.tolist() + vertices.reshape(-1).tolist()
            output.write(struct.pack('<12fH', *values, 0))
    return int(np.count_nonzero(degenerate))


def _safe_source_path(source_root: Path, relative: str) -> Path:
    path = (source_root / relative).resolve()
    try:
        path.relative_to(source_root.resolve())
    except ValueError as exception:
        raise ValueError('raw mesh path escapes source root') from exception
    return path


def normalize(source_manifest_path: Path, model_path: Path, pose_path: Path, output: Path):
    source = _load_json(source_manifest_path)
    model_document = _load_json(model_path)
    pose_document = _load_json(pose_path)
    model = model_document['data']
    pose = pose_document['data']
    if source.get('schema') != EXPECTED_SOURCE_SCHEMA:
        raise ValueError('unsupported Fusion visual source schema')
    if source.get('raw_mesh_encoding') != {
        'format': 'binary_stl',
        'length_unit': 'millimeter',
        'coordinate_space': 'source_component_local',
    }:
        raise ValueError('unsupported raw mesh encoding or coordinate space')
    if source.get('export_count') != 25 or len(source.get('exports', [])) != 25:
        raise ValueError('source manifest must contain exactly 25 exports')
    if {
        entry['canonical_link'] for entry in source.get('retained_proxy_links', [])
    } != EXPECTED_TIBIA_LINKS:
        raise ValueError('source manifest must retain exactly the six tibia proxies')
    if source.get('rights_boundary', {}).get('license') != 'MIT':
        raise ValueError('source manifest is not approved for MIT redistribution')

    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise ValueError('refusing to overwrite non-empty mesh output: {}'.format(output))

    canonical_transforms = _canonical_link_transforms(model, pose)
    fusion_to_ros = np.asarray(model['frame_conversion']['fusion_to_ros_rotation'], dtype=float)
    base_datum = np.asarray(model['frame_conversion']['base_datum_fusion_m'], dtype=float)
    if not np.allclose(fusion_to_ros.T @ fusion_to_ros, np.eye(3), atol=1e-12):
        raise ValueError('Fusion-to-ROS rotation is not orthonormal')

    source_root = source_manifest_path.parent
    link_triangles = {}
    link_sources = {}
    raw_hashes_verified = set()
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
        component_m = triangles_mm * 0.001
        fusion_root = np.einsum('ij,tkj->tki', occurrence_rotation, component_m)
        fusion_root += occurrence_translation
        ros_base = np.einsum(
            'ij,tkj->tki', fusion_to_ros, fusion_root - base_datum
        )

        link_name = entry['canonical_link']
        if link_name not in canonical_transforms:
            raise ValueError('unknown canonical link: {}'.format(link_name))
        link_rotation, link_translation = canonical_transforms[link_name]
        link_local = np.einsum(
            'ij,tkj->tki', link_rotation.T, ros_base - link_translation
        )
        link_triangles.setdefault(link_name, []).append(link_local)
        link_sources.setdefault(link_name, []).append(entry['asset_id'])

    expected_detailed_links = {
        link['name']
        for link in model['links']
        if link['name'] not in EXPECTED_TIBIA_LINKS
    }
    if set(link_triangles) != expected_detailed_links:
        raise ValueError('normalized link coverage differs from canonical non-tibia links')

    outputs = []
    for link_name in model['primary_link_order']:
        if link_name not in link_triangles:
            continue
        triangles = np.concatenate(link_triangles[link_name], axis=0)
        if not np.all(np.isfinite(triangles)):
            raise ValueError('normalized mesh contains non-finite vertices: {}'.format(link_name))
        filename = '{}.stl'.format(link_name.removesuffix('_link'))
        path = output / filename
        degenerate_count = _write_binary_stl(path, link_name, triangles)
        minimum = np.min(triangles.reshape(-1, 3), axis=0)
        maximum = np.max(triangles.reshape(-1, 3), axis=0)
        outputs.append(
            {
                'canonical_link': link_name,
                'path': filename,
                'sha256': _sha256(path),
                'size_bytes': path.stat().st_size,
                'triangle_count': len(triangles),
                'degenerate_triangle_count': degenerate_count,
                'bounds_min_m': minimum.tolist(),
                'bounds_max_m': maximum.tolist(),
                'source_asset_ids': link_sources[link_name],
            }
        )

    normalization_manifest = {
        'schema': {'name': 'araco_fusion_visual_normalization', 'version': 1},
        'generator': {
            'name': 'normalize_fusion_visuals.py',
            'coordinate_math': 'float64',
            'output_encoding': 'deterministic_binary_stl_float32',
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
        'visual_only': True,
        'collision_changed': False,
        'dynamics_changed': False,
        'joint_contract_changed': False,
        'retained_proxy_links': sorted(EXPECTED_TIBIA_LINKS),
        'output_count': len(outputs),
        'outputs': outputs,
    }
    _write_json(output / 'normalization_manifest.json', normalization_manifest)
    print('Generated {} link-local detailed meshes in {}'.format(len(outputs), output))


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
