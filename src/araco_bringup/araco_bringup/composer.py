# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

"""Fail-closed Gate 0 artifact validation and runtime-bundle composition."""

from __future__ import annotations

import copy
from dataclasses import dataclass
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
from typing import Any
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import jsonschema
import xacro
import yaml

from .strict_yaml import canonical_json_bytes
from .strict_yaml import content_sha256
from .strict_yaml import load_strict


class CompositionError(RuntimeError):
    """Raised when preflight cannot produce a valid immutable bundle."""


PROFILE_PATHS = {
    'gazebo_dev_v0': 'config/profiles/gazebo_dev_v0.yaml',
    'gazebo_joystick_v0': 'config/profiles/gazebo_joystick_v0.yaml',
    'gazebo_perception_v0': 'config/profiles/gazebo_perception_v0.yaml',
    'gazebo_perception_diagnostic_visual_v0': (
        'config/profiles/gazebo_perception_diagnostic_visual_v0.yaml'),
    'gazebo_perception_diagnostic_dynamic_imu_v0': (
        'config/profiles/gazebo_perception_diagnostic_dynamic_imu_v0.yaml'),
    'gazebo_perception_diagnostic_fixed_imu_v0': (
        'config/profiles/gazebo_perception_diagnostic_fixed_imu_v0.yaml'),
    'gazebo_ci_v0': 'config/profiles/gazebo_ci_v0.yaml',
    'gazebo_gate3_v0': 'config/profiles/gazebo_gate3_v0.yaml',
    'gazebo_gate4_v0': 'config/profiles/gazebo_gate4_v0.yaml',
    'gazebo_gate5_v0': 'config/profiles/gazebo_gate5_v0.yaml',
}

_ENVELOPE_REQUIRED = {
    'schema_id', 'schema_version', 'artifact_id', 'artifact_version',
    'owner_package', 'deployment_scope', 'evidence', 'dependencies', 'data',
}
_ENVELOPE_ALLOWED = _ENVELOPE_REQUIRED | {'generated_from'}


@dataclass(frozen=True)
class Artifact:
    """One validated, normalized installed artifact."""

    package: str
    relative_path: str
    installed_path: Path
    document: dict[str, Any]
    sha256: str

    @property
    def artifact_id(self) -> str:
        return str(self.document['artifact_id'])

    @property
    def version(self) -> str:
        return str(self.document['artifact_version'])


def _package_share(package: str) -> Path:
    try:
        return Path(get_package_share_directory(package)).resolve()
    except Exception as error:
        raise CompositionError(f'cannot resolve installed package {package!r}: {error}') from error


def _installed_resource(package: str, relative_path: str) -> Path:
    relative = Path(relative_path)
    if relative.is_absolute() or '..' in relative.parts or not relative.parts:
        raise CompositionError(f'invalid installed relative path {relative_path!r}')
    share = _package_share(package)
    # Keep the lexical installed path here.  With ``--symlink-install`` the
    # final resource component intentionally resolves into the source tree;
    # resolving it before this containment check would reject every valid
    # development install.  Absolute paths and traversal components have
    # already been rejected above.
    path = share / relative
    if path != share and share not in path.parents:
        raise CompositionError(f'resource escapes package share: {relative_path!r}')
    if not path.is_file():
        raise CompositionError(f'unresolved installed resource {package}:{relative_path}')
    return path


def _validate_envelope(document: Any, package: str, path: Path) -> None:
    if not isinstance(document, dict):
        raise CompositionError(f'{path}: artifact root must be a mapping')
    keys = set(document)
    if keys != _ENVELOPE_REQUIRED and keys != _ENVELOPE_ALLOWED:
        raise CompositionError(
            f'{path}: envelope fields differ; missing={sorted(_ENVELOPE_REQUIRED - keys)}, '
            f'unknown={sorted(keys - _ENVELOPE_ALLOWED)}'
        )
    if document['schema_version'] != 1:
        raise CompositionError(f'{path}: unsupported schema_version')
    if document['owner_package'] != package:
        raise CompositionError(f'{path}: owner_package does not match containing package')
    if document['deployment_scope'] not in {
        'simulator_only', 'test_only', 'deployment_eligible'
    }:
        raise CompositionError(f'{path}: invalid deployment_scope')
    evidence = document['evidence']
    if not isinstance(evidence, dict) or set(evidence) != {'class', 'sources'}:
        raise CompositionError(f'{path}: evidence must contain only class and sources')
    if evidence['class'] not in {
        'design_fact', 'simulator_estimate', 'operational_policy', 'test_contract'
    }:
        raise CompositionError(f'{path}: invalid evidence class')
    if not isinstance(evidence['sources'], list):
        raise CompositionError(f'{path}: evidence sources must be a list')
    if not isinstance(document['dependencies'], list):
        raise CompositionError(f'{path}: dependencies must be a list')
    for dependency in document['dependencies']:
        if not isinstance(dependency, dict) or set(dependency) != {
            'artifact_id', 'artifact_version'
        }:
            raise CompositionError(f'{path}: invalid exact dependency')


def _schema_path(document: dict[str, Any], package: str) -> Path:
    prefix = 'araco://schemas/'
    schema_id = document['schema_id']
    if not isinstance(schema_id, str) or not schema_id.startswith(prefix):
        raise CompositionError(f'invalid local schema identity {schema_id!r}')
    parts = schema_id[len(prefix):].split('/')
    expected_owner = package.removeprefix('araco_')
    if len(parts) != 3 or parts[0] != expected_owner or parts[2] != 'v1':
        raise CompositionError(f'schema identity does not match owner: {schema_id!r}')
    return _installed_resource(package, f'schema/{parts[1]}_v1.schema.json')


def load_artifact(package: str, relative_path: str) -> Artifact:
    """Resolve and validate one exact installed artifact."""
    path = _installed_resource(package, relative_path)
    try:
        document = load_strict(path)
        _validate_envelope(document, package, path)
        schema_path = _schema_path(document, package)
        schema = json.loads(schema_path.read_text(encoding='utf-8'))
        jsonschema.Draft202012Validator.check_schema(schema)
        jsonschema.Draft202012Validator(schema).validate(document['data'])
    except (ValueError, json.JSONDecodeError, jsonschema.ValidationError,
            jsonschema.SchemaError) as error:
        raise CompositionError(f'{path}: validation failed: {error}') from error
    artifact = Artifact(
        package=package,
        relative_path=relative_path,
        installed_path=path,
        document=document,
        sha256=content_sha256(document),
    )
    _semantic_validate(artifact)
    return artifact


def _unique(values: list[str], label: str) -> None:
    if len(values) != len(set(values)):
        raise CompositionError(f'duplicate {label}')


def _vector(values: Any, size: int, label: str) -> list[float]:
    if not isinstance(values, list) or len(values) != size:
        raise CompositionError(f'{label} must contain {size} values')
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise CompositionError(f'{label} contains a non-finite value')
    return result


def _semantic_validate(artifact: Artifact) -> None:
    kind = artifact.document['data']['kind']
    validator = {
        'canonical_model': _validate_model,
        'joint_limits': _validate_limits,
        'nominal_pose': _validate_pose,
        'dynamics': _validate_dynamics,
        'resources': _validate_resources,
        'source_registry': _validate_sources,
        'qos': _validate_qos,
        'profile': _validate_profile,
        'simulated_rgbd_imu': _validate_simulated_rgbd_imu,
        'rgbd_slam': _validate_rgbd_slam,
    }.get(kind)
    if validator:
        validator(artifact)


def _validate_simulated_rgbd_imu(artifact: Artifact) -> None:
    data = artifact.document['data']
    for camera_name in ('color', 'depth'):
        camera = data[camera_name]
        if float(camera['near_clip_m']) >= float(camera['far_clip_m']):
            raise CompositionError(f'{camera_name} camera clip interval is empty')
    ros_topics = [
        data['color']['ros_image_topic'],
        data['color']['ros_camera_info_topic'],
        data['depth']['ros_depth_image_topic'],
        data['depth']['ros_camera_info_topic'],
        data['depth']['ros_points_topic'],
        data['imu']['ros_topic'],
    ]
    _unique(ros_topics, 'simulated sensor ROS topic')


def _validate_rgbd_slam(artifact: Artifact) -> None:
    data = artifact.document['data']
    _unique(list(data['outputs'].values()), 'RGB-D SLAM output topic')
    if data['ground_truth_input']:
        raise CompositionError('ground truth cannot be an RGB-D SLAM input')
    odometry = data['odometry']
    policy = odometry['gimbal_policy']
    if policy == 'not_applicable' and (
            odometry['imu_enabled'] or odometry['wait_for_imu']
            or data['mode'] != 'six_dof_rgbd_visual'
            or data['mapping']['gravity_sigma'] != 0.0):
        raise CompositionError(
            'visual-only SLAM must disable IMU initialization and gravity')
    if policy == 'dynamic' and (
            not odometry['imu_enabled'] or not odometry['wait_for_imu']
            or not odometry['always_check_imu_tf']
            or data['mode'] != 'six_dof_rgbd_inertial'):
        raise CompositionError(
            'dynamic-gimbal IMU must use timestamped transform checks')
    if policy == 'locked_center' and (
            not odometry['imu_enabled'] or not odometry['wait_for_imu']
            or odometry['always_check_imu_tf']
            or data['mode'] != 'six_dof_rgbd_inertial'):
        raise CompositionError(
            'latest IMU transform is allowed only by the locked-gimbal contract')


def _validate_model(artifact: Artifact) -> None:
    data = artifact.document['data']
    links = data['links']
    joints = data['joints']
    link_names = [link['name'] for link in links]
    joint_names = [joint['name'] for joint in joints]
    _unique(link_names, 'model link name')
    _unique(joint_names, 'model joint name')
    if len(links) != 26 or data['primary_link_order'] != link_names:
        raise CompositionError('canonical model must contain exactly 26 ordered primary links')
    if len(joints) != 25 or any(joint['type'] != 'revolute' for joint in joints):
        raise CompositionError('canonical model must contain exactly 25 revolute joints')
    kinematics = data['kinematics']
    geometry = kinematics['leg_geometry_m']
    if (
        set(geometry) != {'coxa', 'femur', 'tibia', 'foot'}
        or not all(math.isfinite(float(value)) and float(value) > 0.0
                   for value in geometry.values())
        or kinematics['standing_branch'] not in {'knee_down', 'knee_up'}
        or not math.isfinite(float(kinematics['standing_foot_pitch_rad']))
    ):
        raise CompositionError('canonical leg kinematics are invalid')
    if data['root_link'] != 'base_link' or link_names[0] != 'base_link':
        raise CompositionError('base_link must be the canonical root')
    children = []
    for joint in joints:
        if joint['parent'] not in link_names or joint['child'] not in link_names:
            raise CompositionError(f"unknown joint endpoint for {joint['name']}")
        axis = _vector(joint['axis'], 3, f"{joint['name']} axis")
        if abs(math.sqrt(sum(value * value for value in axis)) - 1.0) > 1e-9:
            raise CompositionError(f"{joint['name']} axis is not normalized")
        _vector(joint['origin_xyz_m'], 3, f"{joint['name']} origin")
        _vector(joint['origin_rpy_rad'], 3, f"{joint['name']} rotation")
        children.append(joint['child'])
    _unique(children, 'joint child')
    if set(children) != set(link_names) - {'base_link'}:
        raise CompositionError('joint tree does not span all non-root primary links')
    leg = [joint['name'] for joint in joints if 'leg_command' in joint['roles']]
    gimbal = [joint['name'] for joint in joints if 'gimbal_command' in joint['roles']]
    state = [joint['name'] for joint in joints if 'state' in joint['roles']]
    if len(leg) != 24 or gimbal != ['gimbal_yaw_joint'] or len(state) != 25:
        raise CompositionError('canonical 24+1 controller/state roles are invalid')
    visual = data['visual_policy']
    for material_name in (
        'primary_material_rgba',
        'servo_case_material_rgba',
        'servo_horn_material_rgba',
        'camera_body_material_rgba',
        'camera_hardware_material_rgba',
        'camera_optics_material_rgba',
    ):
        rgba = _vector(visual[material_name], 4, material_name)
        if any(value < 0.0 or value > 1.0 for value in rgba):
            raise CompositionError(f'{material_name} must be normalized RGBA')
    if visual.get('fidelity') != 'fusion_exact_presentation':
        raise CompositionError('canonical visual policy must use exact Fusion presentation meshes')
    inventory = visual.get('exact_mesh_inventory', {})
    if (
        inventory.get('reviewed_source_bodies') != 77
        or inventory.get('servo_models') != {'DS3235': 19, 'DS5160': 6}
        or inventory.get('tibia_components') != 6
        or inventory.get('gemini_335_exterior') != {
            'housing_and_bracket_bodies': 5,
            'hardware_bodies': 6,
            'optical_bodies': 4,
            'internal_bodies_included': 0,
        }
        or inventory.get('retained_visual_proxies') != 0
        or inventory.get('alignment_policy') != 'preserve_fusion_occurrence_no_force_alignment'
    ):
        raise CompositionError('exact Fusion visual inventory contract is invalid')
    if any('visual_mesh_uri' not in link for link in links):
        raise CompositionError('exact Fusion primary mesh does not cover every canonical link')
    auxiliary = [item for link in links for item in link.get('auxiliary_visuals', [])]
    role_counts = {
        role: sum(item.get('role') == role for item in auxiliary)
        for role in ('servo_case', 'servo_horn')
    }
    if role_counts != {'servo_case': 13, 'servo_horn': 7}:
        raise CompositionError('exact Fusion auxiliary visual coverage is invalid')
    for link in links:
        roles = [item.get('role') for item in link.get('auxiliary_visuals', [])]
        _unique(roles, f'{link["name"]} auxiliary visual role')
        if any(role not in {'servo_case', 'servo_horn'} for role in roles):
            raise CompositionError(f'{link["name"]} has an unsupported visual role')
        if any(not item.get('mesh_uri') for item in link.get('auxiliary_visuals', [])):
            raise CompositionError(f'{link["name"]} has an empty auxiliary mesh URI')
    fixed_visuals = [
        item
        for frame in data['fixed_frames']
        for item in frame.get('visuals', [])
    ]
    if [frame['child'] for frame in data['fixed_frames'] if frame.get('visuals')] != [
        'camera_link'
    ]:
        raise CompositionError('exact Gemini visuals must belong only to camera_link')
    if {item.get('role') for item in fixed_visuals} != {
        'camera_body', 'camera_hardware', 'camera_optics'
    } or len(fixed_visuals) != 3:
        raise CompositionError('exact Gemini fixed-frame visual coverage is invalid')
    if any(not item.get('mesh_uri') for item in fixed_visuals):
        raise CompositionError('camera_link has an empty visual mesh URI')


def _validate_limits(artifact: Artifact) -> None:
    data = artifact.document['data']
    for name, limits in data['classes'].items():
        lower = float(limits['lower_rad'])
        upper = float(limits['upper_rad'])
        if not lower < upper or limits['velocity_rad_s'] <= 0 or limits['effort_nm'] <= 0:
            raise CompositionError(f'invalid model limit class {name}')


def _validate_pose(artifact: Artifact) -> None:
    data = artifact.document['data']
    quaternion = _vector(data['base_pose']['orientation_xyzw'], 4, 'base quaternion')
    if abs(math.sqrt(sum(value * value for value in quaternion)) - 1.0) > 1e-9:
        raise CompositionError('nominal base quaternion is not normalized')


def _inertia_valid(inertia: dict[str, Any]) -> bool:
    ixx = float(inertia['ixx'])
    iyy = float(inertia['iyy'])
    izz = float(inertia['izz'])
    ixy = float(inertia['ixy'])
    ixz = float(inertia['ixz'])
    iyz = float(inertia['iyz'])
    a = ixx
    determinant_2 = ixx * iyy - ixy * ixy
    determinant_3 = (
        ixx * iyy * izz + 2 * ixy * ixz * iyz
        - ixx * iyz * iyz - iyy * ixz * ixz - izz * ixy * ixy
    )
    if min(a, determinant_2, determinant_3) <= 0:
        return False
    return (
        ixx + iyy >= izz - 1e-9
        and ixx + izz >= iyy - 1e-9
        and iyy + izz >= ixx - 1e-9
    )


def _validate_dynamics(artifact: Artifact) -> None:
    data = artifact.document['data']
    total = 0.0
    for name, dynamics in data['links'].items():
        if dynamics['mass_kg'] <= 1e-4:
            raise CompositionError(f'{name} mass is below Gate 0 minimum')
        _vector(dynamics['center_of_mass_xyz_m'], 3, f'{name} center of mass')
        if not _inertia_valid(dynamics['inertia_kg_m2']):
            raise CompositionError(f'{name} inertia is not positive-valid')
        total += float(dynamics['mass_kg'])
    if abs(total - float(data['total_mass_kg'])) > 1e-12:
        raise CompositionError('dynamics link masses do not sum to declared total')
    if abs(total - 3.924392774795984) > 0.001:
        raise CompositionError('Gate 0 total mass is outside rough_estimate_v0 tolerance')
    if data['rejected_fusion_all_steel_import_used']:
        raise CompositionError('rejected Fusion all-Steel dynamics cannot be used')
    proxies = data['base_proxies']
    if {proxy['id'] for proxy in proxies} != {
        'pisugar_3_plus', 'main_servo_battery', 'servo_controller'
    } or any(not proxy['include_in_base_inertia'] for proxy in proxies):
        raise CompositionError('all three documented proxies must contribute to base inertia')


def _resolve_package_uri(uri: str) -> Path:
    prefix = 'package://'
    if not uri.startswith(prefix):
        raise CompositionError(f'only package:// resources are permitted: {uri!r}')
    package, separator, relative = uri[len(prefix):].partition('/')
    if not separator:
        raise CompositionError(f'invalid package resource URI {uri!r}')
    return _installed_resource(package, relative)


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_resources(artifact: Artifact) -> None:
    for resource in artifact.document['data']['resources']:
        if resource['redistribution'] != 'allowed' or resource['license'] == 'UNKNOWN':
            raise CompositionError(f"resource {resource['id']} is not redistributable")
        path = _resolve_package_uri(resource['uri'])
        if _file_sha256(path) != resource['sha256']:
            raise CompositionError(f"resource hash mismatch for {resource['id']}")
        if resource['generated']:
            _resolve_package_uri(resource['preferred_editable_source_uri'])
            _resolve_package_uri(resource['generator_uri'])


def _validate_sources(artifact: Artifact) -> None:
    enabled = [source for source in artifact.document['data']['sources'] if source['enabled']]
    ids = [source['id'] for source in enabled]
    priorities = [source['priority'] for source in enabled]
    if any(identifier == 0 for identifier in ids):
        raise CompositionError('enabled source IDs must be non-zero')
    _unique(ids, 'enabled source ID')
    _unique(priorities, 'enabled source priority')


def _validate_qos(artifact: Artifact) -> None:
    for name, profile in artifact.document['data']['profiles'].items():
        if profile['history'] != 'keep_last' or profile['depth'] < 1:
            raise CompositionError(f'invalid QoS history for {name}')
        if profile['reliability'] not in {'best_effort', 'reliable'}:
            raise CompositionError(f'invalid QoS reliability for {name}')


def _validate_profile(artifact: Artifact) -> None:
    data = artifact.document['data']
    if data['profile_id'] not in PROFILE_PATHS:
        raise CompositionError('profile is not in the installed allow-list')
    references = data['selected_artifacts']
    ids = [reference['artifact_id'] for reference in references]
    _unique(ids, 'profile artifact identity')
    if data['deployment_class'] != 'simulator_only':
        raise CompositionError('Gate 0 permits only simulator profiles')
    if set(data['allowed_overrides']) != {
        'robot_namespace', 'gui', 'rviz', 'log_level',
        'record_on_failure', 'report_destination'
    }:
        raise CompositionError('profile override surface differs from the closed contract')


def _artifact_map(profile: Artifact) -> dict[str, Artifact]:
    resolved = {}
    for reference in profile.document['data']['selected_artifacts']:
        artifact = load_artifact(reference['package'], reference['path'])
        if artifact.artifact_id != reference['artifact_id']:
            raise CompositionError(
                f"artifact ID mismatch at {reference['package']}:"
                f"{reference['path']}"
            )
        if artifact.version != reference['artifact_version']:
            raise CompositionError(f'version mismatch for {artifact.artifact_id}')
        if artifact.artifact_id in resolved:
            raise CompositionError(f'duplicate selected artifact {artifact.artifact_id}')
        if artifact.document['deployment_scope'] == 'deployment_eligible':
            raise CompositionError('unapproved deployment-eligible artifact in simulator profile')
        resolved[artifact.artifact_id] = artifact
    for artifact in resolved.values():
        for dependency in artifact.document['dependencies']:
            selected = resolved.get(dependency['artifact_id'])
            if selected is None or selected.version != dependency['artifact_version']:
                raise CompositionError(f'unresolved exact dependency for {artifact.artifact_id}')
    return resolved


def _kind(artifacts: dict[str, Artifact], kind: str) -> Artifact:
    matches = [
        artifact for artifact in artifacts.values()
        if artifact.document['data']['kind'] == kind
    ]
    if len(matches) != 1:
        raise CompositionError(f'expected exactly one selected {kind} artifact')
    return matches[0]


def _cross_validate(artifacts: dict[str, Artifact]) -> dict[str, Any]:
    model = _kind(artifacts, 'canonical_model').document['data']
    limits = _kind(artifacts, 'joint_limits').document['data']
    pose = _kind(artifacts, 'nominal_pose').document['data']
    dynamics = _kind(artifacts, 'dynamics').document['data']
    resources = _kind(artifacts, 'resources').document['data']
    operational = _kind(artifacts, 'operational_policy').document['data']
    controllers = _kind(artifacts, 'controllers').document['data']
    gait = _kind(artifacts, 'gait').document['data']
    source_registry_artifact = _kind(artifacts, 'source_registry')
    sources = source_registry_artifact.document['data']
    safety_artifact = _kind(artifacts, 'safety_policy')
    safety = safety_artifact.document['data']
    qos = _kind(artifacts, 'qos').document['data']
    mapping = _kind(artifacts, 'teleop_mapping').document['data']
    world = _kind(artifacts, 'world').document['data']
    backend = _kind(artifacts, 'gazebo_backend').document['data']
    bridge = _kind(artifacts, 'bridge').document['data']
    sensors = _kind(artifacts, 'simulated_rgbd_imu').document['data']
    slam_artifacts = [
        artifact for artifact in artifacts.values()
        if artifact.document['data']['kind'] == 'rgbd_slam'
    ]
    if len(slam_artifacts) > 1:
        raise CompositionError('expected at most one selected RGB-D SLAM artifact')
    slam = slam_artifacts[0].document['data'] if slam_artifacts else None
    wiring = _kind(artifacts, 'wiring').document['data']
    joints = model['joints']
    names = [joint['name'] for joint in joints]
    leg_names = [joint['name'] for joint in joints if 'leg_command' in joint['roles']]
    gimbal_names = [joint['name'] for joint in joints if 'gimbal_command' in joint['roles']]
    geometry = model['kinematics']['leg_geometry_m']
    expected_origins = {
        'femur': geometry['coxa'],
        'tibia': geometry['femur'],
        'foot': geometry['tibia'],
    }
    for joint in joints:
        if joint['segment'] in expected_origins and (
            abs(float(joint['origin_xyz_m'][0]) - expected_origins[joint['segment']]) > 1e-12
            or any(abs(float(value)) > 1e-12 for value in joint['origin_xyz_m'][1:])
        ):
            raise CompositionError(
                f"canonical kinematic length disagrees with {joint['name']} origin"
            )
    foot_size = model['geometry_classes']['foot']['collision_size_m']
    foot_origin = model['geometry_classes']['foot']['collision_origin_xyz_m']
    foot_shape = model['geometry_classes']['foot'].get('collision_shape')
    foot_radius = model['geometry_classes']['foot'].get('collision_radius_m')
    if (abs(float(foot_size[0]) - geometry['foot']) > 1e-12 or
            foot_shape != 'sphere' or foot_radius is None or
            not 0.0 < float(foot_radius) <= 0.01 or
            abs(float(foot_origin[0]) - geometry['foot']) > 1e-12):
        raise CompositionError('foot collision must be a bounded sphere at the kinematic tip')
    if set(limits['assignments']) != set(names):
        raise CompositionError('model-limit assignments do not cover the canonical joints')
    if set(pose['joint_positions_rad']) != set(names):
        raise CompositionError('nominal pose does not cover the canonical joints')
    if set(dynamics['links']) != set(model['primary_link_order']):
        raise CompositionError('dynamics do not cover the canonical links')
    registered_resource_uris = {
        resource['uri'] for resource in resources['resources']
    }
    used_visual_uris = {
        link.get(
            'visual_mesh_uri',
            model['geometry_classes'][link['geometry_class']]['visual_mesh_uri'],
        )
        for link in model['links']
    }
    used_visual_uris.update(
        visual['mesh_uri']
        for link in model['links']
        for visual in link.get('auxiliary_visuals', [])
    )
    used_visual_uris.update(
        visual['mesh_uri']
        for frame in model['fixed_frames']
        for visual in frame.get('visuals', [])
    )
    if not used_visual_uris <= registered_resource_uris:
        raise CompositionError('canonical visual mesh is absent from resource registry')
    if set(operational['assignments']) != set(leg_names):
        raise CompositionError('operational limits do not cover the 24 leg joints')
    expected_body_envelope = {
        'body_xy_normal_m': 0.02,
        'body_z_normal_lower_m': -0.03,
        'body_z_normal_upper_m': 0.02,
        'body_roll_pitch_normal_rad': 0.15,
        'body_yaw_normal_rad': 0.2,
        'gimbal_yaw_normal_rad': 0.3141592653589793,
        'body_xy_hard_m': 0.035,
        'body_z_hard_lower_m': -0.045,
        'body_z_hard_upper_m': 0.035,
        'body_roll_pitch_hard_rad': 0.25,
        'body_yaw_hard_rad': 0.35,
        'gimbal_yaw_hard_rad': 0.4,
        'quaternion_norm_tolerance': 1e-6,
        'reserved_twist_tolerance': 1e-12,
        'stand_velocity_tolerance': 1e-9,
    }
    actual_body_envelope = {
        key: operational['command_envelope'][key]
        for key in expected_body_envelope
    }
    if actual_body_envelope != expected_body_envelope:
        raise CompositionError('body command envelope differs from accepted contract')
    speed_envelope = (
        operational['command_envelope']['planar_speed_normal_m_s'],
        operational['command_envelope']['planar_speed_hard_m_s'],
        operational['command_envelope']['yaw_rate_normal_rad_s'],
        operational['command_envelope']['yaw_rate_hard_rad_s'],
    )
    baseline_speed_envelope = (0.05, 0.08, 0.3, 0.5)
    responsive_speed_envelope = (0.24, 0.288, 1.2, 1.5)
    if speed_envelope not in {
        baseline_speed_envelope, responsive_speed_envelope
    }:
        raise CompositionError('velocity command envelope differs from accepted contracts')
    for joint in joints:
        model_limit = limits['classes'][limits['assignments'][joint['name']]]
        target = pose['joint_positions_rad'][joint['name']]
        if not model_limit['lower_rad'] < target < model_limit['upper_rad']:
            raise CompositionError(f"nominal target outside model limits for {joint['name']}")
        if joint['name'] in operational['assignments']:
            operation = operational['classes'][operational['assignments'][joint['name']]]
            if (
                operation['lower_rad'] < model_limit['lower_rad']
                or operation['upper_rad'] > model_limit['upper_rad']
            ):
                raise CompositionError(
                    f"operational limit widens model limit for {joint['name']}"
                )
            margin = min(target - operation['lower_rad'], operation['upper_rad'] - target)
            if margin < 0.1 - 1e-12:
                raise CompositionError(
                    'nominal operational margin below Gate 0 minimum for '
                    f"{joint['name']}"
                )
    if (
        controllers['controller_manager_rate_hz'] != 250
        or controllers['joint_state_rate_hz'] != 125
    ):
        raise CompositionError('controller rates differ from the accepted contract')
    shaping = gait['shaping']
    gait_contract = (
        gait['gait_id'], gait['base_cadence_hz'],
        gait['maximum_cadence_hz'], gait['cadence_rate_hz_s'],
        gait['preferred_maximum_stride_scale'], gait['motion_deadband_m_s'],
        gait['duty_factor'], gait['maximum_stride_m'],
        gait['swing_clearance_m'], gait['trajectory_horizon_s'],
        shaping['translation_acceleration_m_s2'],
        shaping['translation_stop_deceleration_m_s2'],
        shaping['yaw_acceleration_rad_s2'],
        shaping['yaw_stop_deceleration_rad_s2'],
        shaping['body_translation_rate_m_s'],
        shaping['body_angular_rate_rad_s'],
    )
    baseline_gait_contract = (
        'tripod_legacy_translation_rotation_blend_responsive_scheduler', 1.0, 1.5, 1.0,
        0.5, 0.005, 0.5, 0.06, 0.03, 0.04,
        0.1, 0.15, 0.6, 0.9, 0.03, 0.3,
    )
    responsive_gait_contract = (
        'tripod_legacy_translation_rotation_blend_responsive_scheduler', 1.5, 2.5, 2.0,
        0.6, 0.005, 0.5, 0.12, 0.06, 0.04,
        0.4, 0.6, 2.4, 3.6, 0.03, 0.3,
    )
    if gait_contract not in {baseline_gait_contract, responsive_gait_contract}:
        raise CompositionError(
            'gait identity, timing, or envelope differs from the accepted contract')
    accepted_sources = {
        '0.1.0': {
            'teleop': (10, 100, 50, 0.15, True),
            'navigation': (20, 50, 20, 0.3, False),
            'system_test': (250, 200, 100, 0.1, False),
        },
        '0.2.0': {
            'teleop': (10, 100, 50, 0.5, True),
            'navigation': (20, 50, 20, 0.3, False),
            'system_test': (250, 200, 100, 0.1, False),
        },
    }
    actual_sources = {
        item['name']: (
            item['id'], item['priority'], item['rate_hz'],
            item['freshness_timeout_s'], item['enabled'],
        )
        for item in sources['sources']
    }
    if actual_sources != accepted_sources.get(source_registry_artifact.version):
        raise CompositionError('source authority differs from the accepted contract')
    accepted_safety_timing = {
        '0.3.0': {
            'watchdogs_s': {
                'selected_command': 0.05, 'safe_command': 0.05,
                'joint_state': 0.1, 'locomotion_status': 0.1,
                'controller_state': 0.1, 'provenance': 1.5,
                'clock_progress': 0.25,
            },
            'maximum_detection_s': {
                'selected_command': 0.06, 'safe_command': 0.06,
                'joint_state': 0.11, 'locomotion_status': 0.11,
                'controller_state': 0.11, 'provenance': 1.51,
                'clock_progress': 0.26,
            },
        },
        '0.5.0': {
            'watchdogs_s': {
                'selected_command': 0.5, 'safe_command': 0.5,
                'joint_state': 0.5, 'locomotion_status': 0.5,
                'controller_state': 0.5, 'provenance': 1.5,
                'clock_progress': 0.5,
            },
            'maximum_detection_s': {
                'selected_command': 0.51, 'safe_command': 0.51,
                'joint_state': 0.51, 'locomotion_status': 0.51,
                'controller_state': 0.51, 'provenance': 1.51,
                'clock_progress': 0.51,
            },
        },
    }
    expected_safety_timing = accepted_safety_timing.get(safety_artifact.version)
    if expected_safety_timing is None or any(
        safety[key] != expected_safety_timing[key]
        for key in ('watchdogs_s', 'maximum_detection_s')
    ):
        raise CompositionError('safety watchdogs differ from the accepted contract')
    if set(qos['profiles']) != {
        'candidate_latest', 'trusted_command_latest', 'controller_command',
        'operational_status', 'latched_classification', 'state_sample',
        'debug_latest', 'diagnostics',
    }:
        raise CompositionError('QoS profile set differs from the accepted contract')
    common_mapping_valid = (
        mapping['source_id'] == 10
        and mapping['publication_rate_hz'] == 50
        and mapping['release_publishes_inactive']
    )
    keyboard_mapping_valid = mapping['adapter'] == 'keyboard' and (
        mapping['heartbeat_timeout_s'] == 0.12
        and
        mapping['frontend'] == 'tk_key_state_window'
        and mapping['state_protocol'] == 'araco.keyboard-state.v1'
        and mapping['state_topic'] == 'teleop/key_state'
        and mapping['deadman_key'] == 'space'
        and mapping['deadman_neutral_keeps_source_active']
    )
    joystick_mapping_valid = mapping['adapter'] == 'joystick' and (
        mapping['heartbeat_timeout_s'] == 0.5
        and
        mapping['device_name'] == 'LiteStar PXN-2113 Pro'
        and mapping['usb_id'] == '11ff:0837'
        and mapping['axis_count'] == 6
        and mapping['button_count'] == 12
        and mapping['activation_policy'] ==
        'auto_enable_once_from_fresh_neutral_standing_selection'
        and mapping['control_response'] == {
            'kind': 'legacy_p_only_time_invariant',
            'reference_period_s': 0.005,
            'normal_error_fraction': 0.02,
            'height_error_fraction': 0.01,
        }
        and not any(key in mapping for key in (
            'deadman_button', 'deadman_physical_label',
            'deadman_neutral_keeps_source_active'))
        and mapping['roll_left_button'] == 2
        and mapping['roll_left_physical_label'] == 'physical button 3'
        and mapping['roll_right_button'] == 3
        and mapping['roll_right_physical_label'] == 'physical button 4'
        and mapping['roll_button_scale_rad'] == 0.15
        and mapping['axes'] == {
            'forward': {'index': 1, 'invert': False, 'scale': 0.24},
            'lateral': {'index': 0, 'invert': False, 'scale': 0.24},
            'walking_yaw': {'index': 3, 'invert': False, 'scale': 1.2},
            'body_height': {
                'index': 2, 'positive_end_is_zero': True, 'range_m': 0.03},
            'body_pitch': {'index': 5, 'invert': True, 'scale': 0.15},
            'posture_yaw': {'index': 4, 'invert': False, 'scale': 0.2},
            'gimbal_yaw': {
                'index': 4, 'invert': False, 'scale': 0.3141592653589793},
        }
    )
    if not common_mapping_valid or not (
        keyboard_mapping_valid or joystick_mapping_valid
    ):
        raise CompositionError('teleop mapping does not match registered source authority')
    operational_class_contract = tuple(
        (
            name,
            operational['classes'][name]['lower_rad'],
            operational['classes'][name]['upper_rad'],
            operational['classes'][name]['command_rate_cap_rad_s'],
        )
        for name in ('coxa', 'femur', 'tibia', 'foot')
    )
    baseline_operational_class_contract = (
        ('coxa', -0.45, 0.45, 1.2),
        ('femur', 0.35, 1.1, 1.2),
        ('tibia', -2.35, -1.15, 1.2),
        ('foot', -0.85, 0.1, 1.2),
    )
    responsive_operational_class_contract = (
        ('coxa', -2.356194490192345, 2.356194490192345, 5.5),
        ('femur', -1.636194490192345, 3.076194490192345, 10.0),
        ('tibia', -4.256194490192345, 0.456194490192345, 12.5),
        ('foot', -2.766194490192345, 1.946194490192345, 9.0),
    )
    responsive_contract_selected = (
        gait_contract == responsive_gait_contract
        and speed_envelope == responsive_speed_envelope
        and operational_class_contract == responsive_operational_class_contract
    )
    baseline_contract_selected = (
        gait_contract == baseline_gait_contract
        and speed_envelope == baseline_speed_envelope
        and operational_class_contract == baseline_operational_class_contract
    )
    if not (responsive_contract_selected or baseline_contract_selected):
        raise CompositionError(
            'gait, velocity-envelope, and operational-limit profiles are mismatched')
    if joystick_mapping_valid != responsive_contract_selected:
        raise CompositionError(
            'joystick mapping must select the complete responsive simulator contract')
    if (
        world['physics_engine'] != 'dart'
        or world['maximum_step_s'] != 0.001
        or world['real_time_factor'] != 1.0
        or world['seed'] != 42
        or world['sensor_systems'] != ['rendering', 'imu']
    ):
        raise CompositionError('Gazebo world determinism contract differs')
    if (
        backend['plugin'] != 'gz_ros2_control/GazeboSimSystem'
        or not backend['synchronous_update']
        or backend['mapping_policy'] != 'derive_roles_from_canonical_model'
    ):
        raise CompositionError('Gazebo backend contract differs')
    if not any(endpoint['ros_topic'] == '/clock' for endpoint in bridge['endpoints']):
        raise CompositionError('Gazebo bridge does not provide /clock')
    fixed_frame_names = {frame['child'] for frame in model['fixed_frames']}
    if (
        sensors['mount_frame'] not in fixed_frame_names
        or sensors['color']['optical_frame'] not in fixed_frame_names
        or sensors['depth']['optical_frame'] not in fixed_frame_names
        or sensors['depth']['point_cloud_frame'] not in fixed_frame_names
        or sensors['imu']['frame'] not in fixed_frame_names
    ):
        raise CompositionError('simulated RGB-D/IMU frame is absent from canonical model')
    bridge_pairs = {
        (endpoint['ros_topic'], endpoint['gz_topic'], endpoint['ros_type'])
        for endpoint in bridge['endpoints']
    }
    expected_sensor_bridges = {
        (
            sensors['color']['ros_image_topic'],
            sensors['color']['gz_image_topic'],
            'sensor_msgs/msg/Image',
        ),
        (
            sensors['color']['ros_camera_info_topic'],
            sensors['color']['gz_camera_info_topic'],
            'sensor_msgs/msg/CameraInfo',
        ),
        (
            sensors['depth']['ros_depth_image_topic'],
            sensors['depth']['gz_depth_image_topic'],
            'sensor_msgs/msg/Image',
        ),
        (
            sensors['depth']['ros_camera_info_topic'],
            sensors['depth']['gz_camera_info_topic'],
            'sensor_msgs/msg/CameraInfo',
        ),
        (
            sensors['depth']['ros_points_topic'],
            sensors['depth']['gz_points_topic'],
            'sensor_msgs/msg/PointCloud2',
        ),
        (
            sensors['imu']['ros_topic'],
            sensors['imu']['gz_topic'],
            'sensor_msgs/msg/Imu',
        ),
    }
    if not expected_sensor_bridges <= bridge_pairs:
        raise CompositionError('Gazebo bridge does not cover the RGB-D/IMU contract')
    point_cloud_bridges = [
        endpoint for endpoint in bridge['endpoints']
        if endpoint['ros_topic'] == sensors['depth']['ros_points_topic']
    ]
    if (
        len(point_cloud_bridges) != 1
        or point_cloud_bridges[0].get('frame_id') != sensors['depth']['point_cloud_frame']
    ):
        raise CompositionError(
            'Gazebo point cloud must use its +X-forward camera frame, not an optical frame')
    if slam is not None:
        registered_fields = ('width', 'height', 'horizontal_fov_rad', 'update_rate_hz')
        if any(
            sensors['color'][field] != sensors['depth'][field]
            for field in registered_fields
        ):
            raise CompositionError(
                'RGB-D SLAM requires registered color and depth intrinsics')
        if slam['inputs'] != {
            'rgb': sensors['color']['ros_image_topic'],
            'depth': sensors['depth']['ros_depth_image_topic'],
            'camera_info': sensors['color']['ros_camera_info_topic'],
            'imu': sensors['imu']['ros_topic'],
        }:
            raise CompositionError('RGB-D SLAM inputs differ from selected sensors')
        if slam['frames'] != {
            'map': 'map', 'odom': 'odom', 'base': 'base_link',
            'imu': sensors['imu']['frame'],
        }:
            raise CompositionError('RGB-D SLAM frame chain differs from contract')
        if not slam['synchronization']['registered_depth_required']:
            raise CompositionError('RGB-D SLAM must require registered depth')
    expected_input_nodes = (
        {'keyboard_teleop_ui', 'teleop_adapter'} if mapping['adapter'] == 'keyboard'
        else {'joy_node', 'joystick_adapter'}
    )
    if set(wiring['nodes']) != expected_input_nodes | {
        'command_arbiter', 'safety_supervisor', 'locomotion',
    }:
        raise CompositionError('node wiring differs from the accepted contract')
    if wiring['node_rates_hz'] != {
        'teleop_adapter': 50, 'command_arbiter': 100,
        'safety_supervisor': 100, 'locomotion': 100,
    } or not wiring['use_sim_time']:
        raise CompositionError('node timing/wiring differs from the accepted contract')
    return {'leg_joints': leg_names, 'gimbal_joints': gimbal_names, 'state_joints': names}


def _behavior_fingerprint(artifacts: dict[str, Artifact]) -> str:
    records = [
        {'artifact_id': item.artifact_id, 'version': item.version, 'sha256': item.sha256}
        for item in sorted(artifacts.values(), key=lambda value: value.artifact_id)
        if item.document['deployment_scope'] != 'test_only'
    ]
    return content_sha256(records)


def _validate_profile_equivalence(
    profile_id: str, artifacts: dict[str, Artifact]
) -> None:
    if profile_id not in {'gazebo_dev_v0', 'gazebo_ci_v0'}:
        return
    peer_id = 'gazebo_ci_v0' if profile_id == 'gazebo_dev_v0' else 'gazebo_dev_v0'
    peer = load_artifact('araco_bringup', PROFILE_PATHS[peer_id])
    peer_artifacts = _artifact_map(peer)
    if _behavior_fingerprint(artifacts) != _behavior_fingerprint(peer_artifacts):
        raise CompositionError('development and CI behavior fingerprints differ')


def _xml_values(values: list[float]) -> str:
    return ' '.join(format(float(value), '.15g') for value in values)


def _render_urdf(
    artifacts: dict[str, Artifact], output_path: Path
) -> None:
    model = _kind(artifacts, 'canonical_model').document['data']
    dynamics = _kind(artifacts, 'dynamics').document['data']
    limits = _kind(artifacts, 'joint_limits').document['data']
    resources = _kind(artifacts, 'resources').document['data']
    pose = _kind(artifacts, 'nominal_pose').document['data']
    backend = _kind(artifacts, 'gazebo_backend').document['data']
    world = _kind(artifacts, 'world').document['data']
    sensors = _kind(artifacts, 'simulated_rgbd_imu').document['data']
    template = next(
        resource for resource in resources['resources']
        if resource['id'] == 'robot_xacro_macros'
    )
    template_path = _resolve_package_uri(template['uri'])
    lines = [
        '<?xml version="1.0"?>',
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="araco">',
        f'  <xacro:include filename="{template_path}"/>',
    ]
    geometry = model['geometry_classes']
    visual_policy = model['visual_policy']
    for link in model['links']:
        inertial = dynamics['links'][link['name']]
        inertia = inertial['inertia_kg_m2']
        shape = geometry[link['geometry_class']]
        macro_name = (
            'araco_dynamic_sphere_link'
            if shape.get('collision_shape') == 'sphere' else 'araco_dynamic_link')
        collision_argument = (
            f'collision_radius="{float(shape["collision_radius_m"]):.15g}"'
            if shape.get('collision_shape') == 'sphere' else
            f'collision_size="{_xml_values(shape["collision_size_m"])}"')
        lines.append(
            f'  <xacro:{macro_name} '
            f'name="{link["name"]}" mass="{inertial["mass_kg"]:.15g}" '
            f'com_xyz="{_xml_values(inertial["center_of_mass_xyz_m"])}" '
            f'ixx="{inertia["ixx"]:.15g}" ixy="{inertia["ixy"]:.15g}" '
            f'ixz="{inertia["ixz"]:.15g}" iyy="{inertia["iyy"]:.15g}" '
            f'iyz="{inertia["iyz"]:.15g}" izz="{inertia["izz"]:.15g}" '
            f'collision_xyz="{_xml_values(shape["collision_origin_xyz_m"])}" '
            f'{collision_argument}>'
        )
        lines.extend([
            '    <visuals>',
            f'      <visual name="{link["name"]}_primary_visual">',
            '        <geometry>',
            '          <mesh filename="'
            f'{link.get("visual_mesh_uri", shape["visual_mesh_uri"])}"/>',
            '        </geometry>',
            '        <material name="araco_printed">',
            f'          <color rgba="{_xml_values(visual_policy["primary_material_rgba"])}"/>',
            '        </material>',
            '      </visual>',
        ])
        for auxiliary in link.get('auxiliary_visuals', []):
            role = auxiliary['role']
            material_key = '{}_material_rgba'.format(role)
            lines.extend([
                f'    <visual name="{link["name"]}_{role}_visual">',
                f'      <geometry><mesh filename="{auxiliary["mesh_uri"]}"/></geometry>',
                f'      <material name="araco_{role}">',
                f'        <color rgba="{_xml_values(visual_policy[material_key])}"/>',
                '      </material>',
                '    </visual>',
            ])
        lines.append('    </visuals>')
        lines.append(f'  </xacro:{macro_name}>')
    for frame in model['fixed_frames']:
        visuals = frame.get('visuals', [])
        if not visuals:
            lines.append(f'  <xacro:araco_frame_link name="{frame["child"]}"/>')
            continue
        lines.append(f'  <link name="{frame["child"]}">')
        for visual in visuals:
            role = visual['role']
            material_key = '{}_material_rgba'.format(role)
            lines.extend([
                f'    <visual name="{frame["child"]}_{role}_visual">',
                f'      <geometry><mesh filename="{visual["mesh_uri"]}"/></geometry>',
                f'      <material name="araco_{role}">',
                f'        <color rgba="{_xml_values(visual_policy[material_key])}"/>',
                '      </material>',
                '    </visual>',
            ])
        lines.append('  </link>')
    for joint in model['joints']:
        limit = limits['classes'][limits['assignments'][joint['name']]]
        lines.append(
            '  <xacro:araco_revolute_joint '
            f'name="{joint["name"]}" parent="{joint["parent"]}" child="{joint["child"]}" '
            f'xyz="{_xml_values(joint["origin_xyz_m"])}" '
            f'rpy="{_xml_values(joint["origin_rpy_rad"])}" '
            f'axis="{_xml_values(joint["axis"])}" lower="{limit["lower_rad"]:.15g}" '
            f'upper="{limit["upper_rad"]:.15g}" velocity="{limit["velocity_rad_s"]:.15g}" '
            f'effort="{limit["effort_nm"]:.15g}" damping="{limit["damping_nms_rad"]:.15g}" '
            f'friction="{limit["friction_nm"]:.15g}"/>'
        )
    for frame in model['fixed_frames']:
        lines.append(
            '  <xacro:araco_fixed_joint '
            f'name="{frame["name"]}" parent="{frame["parent"]}" child="{frame["child"]}" '
            f'xyz="{_xml_values(frame["origin_xyz_m"])}" '
            f'rpy="{_xml_values(frame["origin_rpy_rad"])}"/>'
        )
    lines.extend([
        '  <ros2_control name="GazeboSimSystem" type="system">',
        '    <hardware>',
        f'      <plugin>{backend["plugin"]}</plugin>',
        '    </hardware>',
    ])
    for joint in model['joints']:
        limit = limits['classes'][limits['assignments'][joint['name']]]
        initial = pose['joint_positions_rad'][joint['name']]
        lines.extend([
            f'    <joint name="{joint["name"]}">',
            '      <command_interface name="position">',
            f'        <param name="min">{limit["lower_rad"]:.15g}</param>',
            f'        <param name="max">{limit["upper_rad"]:.15g}</param>',
            '      </command_interface>',
            '      <state_interface name="position">',
            f'        <param name="initial_value">{initial:.15g}</param>',
            '      </state_interface>',
            '      <state_interface name="velocity"/>',
            '      <state_interface name="effort"/>',
            '    </joint>',
        ])
    lines.append('  </ros2_control>')
    foot_names = [link['name'] for link in model['links'] if link['role'] == 'foot']
    for link in model['links']:
        friction = (
            world['foot_friction']
            if link['name'] in foot_names else world['nonfoot_friction']
        )
        lines.extend([
            f'  <gazebo reference="{link["name"]}">',
            f'    <mu1>{friction["mu"]:.15g}</mu1>',
            f'    <mu2>{friction["mu2"]:.15g}</mu2>',
            '  </gazebo>',
        ])
    color = sensors['color']
    depth = sensors['depth']
    imu = sensors['imu']
    lines.extend([
        f'  <gazebo reference="{sensors["mount_frame"]}">',
        f'    <sensor name="{color["sensor_name"]}" type="camera">',
        '      <always_on>true</always_on>',
        f'      <update_rate>{color["update_rate_hz"]:.15g}</update_rate>',
        f'      <topic>{color["gz_image_topic"]}</topic>',
        '      <camera>',
        f'        <camera_info_topic>{color["gz_camera_info_topic"]}</camera_info_topic>',
        f'        <horizontal_fov>{color["horizontal_fov_rad"]:.15g}</horizontal_fov>',
        '        <image>',
        f'          <width>{color["width"]}</width>',
        f'          <height>{color["height"]}</height>',
        '          <format>R8G8B8</format>',
        '        </image>',
        '        <clip>',
        f'          <near>{color["near_clip_m"]:.15g}</near>',
        f'          <far>{color["far_clip_m"]:.15g}</far>',
        '        </clip>',
        f'        <optical_frame_id>{color["optical_frame"]}</optical_frame_id>',
        '      </camera>',
        '    </sensor>',
        f'    <sensor name="{depth["sensor_name"]}" type="rgbd_camera">',
        '      <always_on>true</always_on>',
        f'      <update_rate>{depth["update_rate_hz"]:.15g}</update_rate>',
        f'      <topic>{depth["gz_base_topic"]}</topic>',
        '      <camera>',
        f'        <camera_info_topic>{depth["gz_camera_info_topic"]}</camera_info_topic>',
        f'        <horizontal_fov>{depth["horizontal_fov_rad"]:.15g}</horizontal_fov>',
        '        <image>',
        f'          <width>{depth["width"]}</width>',
        f'          <height>{depth["height"]}</height>',
        '          <format>R8G8B8</format>',
        '        </image>',
        '        <clip>',
        f'          <near>{depth["near_clip_m"]:.15g}</near>',
        f'          <far>{depth["far_clip_m"]:.15g}</far>',
        '        </clip>',
        f'        <optical_frame_id>{depth["optical_frame"]}</optical_frame_id>',
        '      </camera>',
        '    </sensor>',
        f'    <sensor name="{imu["sensor_name"]}" type="imu">',
        '      <always_on>true</always_on>',
        f'      <update_rate>{imu["update_rate_hz"]:.15g}</update_rate>',
        f'      <topic>{imu["gz_topic"]}</topic>',
        f'      <gz_frame_id>{imu["frame"]}</gz_frame_id>',
        '    </sensor>',
        '  </gazebo>',
    ])
    for link in model['links']:
        link_name = link['name']
        contact_topic = (
            f'/araco/contact_sensor/{link_name.removesuffix("_foot_link")}'
            if link['role'] == 'foot' else '/araco/contact_sensor/nonfoot'
        )
        lines.extend([
            f'  <gazebo reference="{link_name}">',
            f'    <sensor name="{link_name}_gate1_contact" type="contact">',
            '      <always_on>true</always_on>',
            '      <update_rate>50</update_rate>',
            '      <contact>',
            f'        <collision>{link_name}_collision_collision</collision>',
            f'        <topic>{contact_topic}</topic>',
            '      </contact>',
            '    </sensor>',
            '  </gazebo>',
        ])
    manager_parameters = _installed_resource(
        'araco_bringup', 'config/controllers/controller_manager_runtime.yaml'
    )
    lines.extend([
        '  <gazebo>',
        '    <plugin filename="gz_ros2_control-system" '
        'name="gz_ros2_control::GazeboSimROS2ControlPlugin">',
        f'      <parameters>{manager_parameters}</parameters>',
        '      <controller_manager_name>controller_manager</controller_manager_name>',
        '      <hold_joints>true</hold_joints>',
        '      <position_proportional_gain>'
        f'{backend["position_proportional_gain"]:.15g}'
        '</position_proportional_gain>',
        '    </plugin>',
        '    <plugin filename="gz-sim-odometry-publisher-system" '
        'name="gz::sim::systems::OdometryPublisher">',
        '      <odom_frame>odom</odom_frame>',
        '      <robot_base_frame>base_link</robot_base_frame>',
        '      <odom_topic>/araco/simulation/ground_truth/odom</odom_topic>',
        '      <odom_publish_frequency>100</odom_publish_frequency>',
        '      <dimensions>3</dimensions>',
        '    </plugin>',
        '  </gazebo>',
    ])
    lines.append('</robot>')
    generated_xacro = output_path.with_suffix('.generated.xacro')
    generated_xacro.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    try:
        document = xacro.process_file(str(generated_xacro))
        root = ET.fromstring(document.toxml())
        ET.indent(root, space='  ')
        normalized_xml = ET.tostring(root, encoding='unicode')
        output_path.write_text(
            '<?xml version="1.0"?>\n' + normalized_xml + '\n',
            encoding='utf-8',
        )
    except Exception as error:
        raise CompositionError(f'Xacro/URDF expansion failed: {error}') from error
    finally:
        generated_xacro.unlink(missing_ok=True)
    urdf_links = root.findall('link')
    urdf_joints = root.findall('joint')
    if len(urdf_links) != 29 or len(urdf_joints) != 28:
        raise CompositionError(
            'expanded URDF does not contain 26 primary links plus 3 fixed frames'
        )


def _yaml_write(path: Path, data: dict[str, Any]) -> None:
    path.write_text(
        '# Generated by araco_bringup preflight; do not edit.\n'
        + yaml.safe_dump(data, sort_keys=False),
        encoding='utf-8',
    )


def _standing_parameters(
    model: dict[str, Any],
    limits: dict[str, Any],
    operational: dict[str, Any],
    pose: dict[str, Any],
    solver: dict[str, Any],
) -> dict[str, Any]:
    """Adapt canonical description data into typed, flattened runtime values."""
    leg_joints = [
        joint for joint in model['joints'] if 'leg_command' in joint['roles']
    ]
    leg_names = list(dict.fromkeys(joint['leg'] for joint in leg_joints))
    if len(leg_joints) != 24 or len(leg_names) != 6:
        raise CompositionError('standing adapter requires six ordered four-joint legs')
    geometry_data = model['kinematics']['leg_geometry_m']
    geometry = [float(geometry_data[key]) for key in ('coxa', 'femur', 'tibia', 'foot')]
    foot_pitch = float(model['kinematics']['standing_foot_pitch_rad'])
    mounts = []
    mount_yaws = []
    targets = []
    lower = []
    upper = []
    rate_caps = []
    for leg_name in leg_names:
        joints = [joint for joint in leg_joints if joint['leg'] == leg_name]
        if [joint['segment'] for joint in joints] != ['coxa', 'femur', 'tibia', 'foot']:
            raise CompositionError(f'invalid canonical joint order for {leg_name}')
        mount = _vector(joints[0]['origin_xyz_m'], 3, f'{leg_name} mount')
        mount_yaw = float(joints[0]['origin_rpy_rad'][2])
        values = [float(pose['joint_positions_rad'][joint['name']]) for joint in joints]
        pitch_1 = values[1]
        pitch_2 = pitch_1 + values[2]
        actual_pitch = pitch_2 + values[3]
        if abs(actual_pitch - foot_pitch) > 2e-6:
            raise CompositionError(f'{leg_name} standing foot pitch violates policy')
        radial = (
            geometry[0]
            + geometry[1] * math.cos(pitch_1)
            + geometry[2] * math.cos(pitch_2)
            + geometry[3] * math.cos(foot_pitch)
        )
        local_x = radial * math.cos(values[0])
        local_y = radial * math.sin(values[0])
        local_z = (
            geometry[1] * math.sin(pitch_1)
            + geometry[2] * math.sin(pitch_2)
            + geometry[3] * math.sin(foot_pitch)
        )
        mounts.extend(mount)
        mount_yaws.append(mount_yaw)
        targets.extend([
            mount[0] + math.cos(mount_yaw) * local_x - math.sin(mount_yaw) * local_y,
            mount[1] + math.sin(mount_yaw) * local_x + math.cos(mount_yaw) * local_y,
            mount[2] + local_z,
        ])
        for joint in joints:
            model_limit = limits['classes'][limits['assignments'][joint['name']]]
            operation = operational['classes'][
                operational['assignments'][joint['name']]
            ]
            if (operation['lower_rad'] < model_limit['lower_rad'] or
                    operation['upper_rad'] > model_limit['upper_rad']):
                raise CompositionError('operational standing limit widens model limit')
            lower.append(float(operation['lower_rad']))
            upper.append(float(operation['upper_rad']))
            rate_caps.append(float(operation['command_rate_cap_rad_s']))
    if solver['algorithm'] != 'analytic-four-dof-position-and-foot-pitch':
        raise CompositionError('unsupported selected IK algorithm')
    if (solver['foot_pitch_target_policy'] !=
            'project_world_down_into_each_leg_sagittal_plane'):
        raise CompositionError('unsupported selected foot-pitch target policy')
    if solver['orientation_residual_policy'] != 'report_rejected_angle':
        raise CompositionError('unsupported selected orientation residual policy')
    branch = solver['standing_branch']
    if branch != model['kinematics']['standing_branch']:
        raise CompositionError('solver and canonical standing branches disagree')
    return {
        'leg_geometry_m': geometry,
        'leg_mount_positions_base_m': mounts,
        'leg_mount_yaw_rad': mount_yaws,
        'standing_foot_targets_base_m': targets,
        'standing_foot_pitch_rad': [foot_pitch] * 6,
        'joint_lower_rad': lower,
        'joint_upper_rad': upper,
        'joint_command_rate_cap_rad_s': rate_caps,
        'standing_branch': branch,
        'ik_position_tolerance_m': float(solver['position_tolerance_m']),
        'ik_singularity_threshold': float(solver['singularity_threshold']),
        'ik_near_limit_margin_rad': float(solver['near_limit_margin_rad']),
        'standing_oracle_tolerance_rad': float(solver['oracle_tolerance_rad']),
    }


def _node_parameters(
    profile: Artifact,
    artifacts: dict[str, Artifact],
    behavior_fingerprint: str,
    input_fingerprint: str,
) -> dict[str, dict[str, Any]]:
    selected_ids = sorted(artifacts)
    common = {
        'config.profile_id': profile.document['data']['profile_id'],
        'config.profile_version': profile.document['data']['profile_version'],
        'config.behavior_fingerprint': behavior_fingerprint,
        'config.input_selection_fingerprint': input_fingerprint,
        'config.selected_artifact_ids': selected_ids,
        'use_sim_time': True,
    }
    wiring = _kind(artifacts, 'wiring').document['data']
    rates = wiring['node_rates_hz']
    source_registry = _kind(artifacts, 'source_registry')
    safety = _kind(artifacts, 'safety_policy')
    gait = _kind(artifacts, 'gait')
    model = _kind(artifacts, 'canonical_model').document['data']
    limits = _kind(artifacts, 'joint_limits').document['data']
    operational = _kind(artifacts, 'operational_policy').document['data']
    pose = _kind(artifacts, 'nominal_pose').document['data']
    solver = _kind(artifacts, 'ik_solver').document['data']
    standing = _standing_parameters(model, limits, operational, pose, solver)
    mapping = _kind(artifacts, 'teleop_mapping')
    mapping_data = mapping.document['data']
    source_items = {item['name']: item for item in source_registry.document['data']['sources']}
    envelope = operational['command_envelope']
    shaping = gait.document['data']['shaping']
    watchdogs = safety.document['data']['watchdogs_s']
    gimbal_joint = next(
        joint for joint in model['joints'] if 'gimbal_command' in joint['roles'])
    gimbal_limit = limits['classes'][limits['assignments'][gimbal_joint['name']]]
    definitions = {
        '/araco/teleop_adapter': {
            **common,
            'loop_rate_hz': float(rates['teleop_adapter']),
            'mapping_path': str(mapping.installed_path),
            'mapping_sha256': mapping.sha256,
        },
        '/araco/joystick_adapter': {
            **common,
            'loop_rate_hz': float(rates['teleop_adapter']),
            'mapping_path': str(mapping.installed_path),
            'mapping_sha256': mapping.sha256,
        },
        '/araco/command_arbiter': {
            **common,
            'loop_rate_hz': float(rates['command_arbiter']),
            'source_registry_path': str(source_registry.installed_path),
            'source_registry_sha256': source_registry.sha256,
            'teleop_enabled': bool(
                profile.document['data']['input_selection']['keyboard_adapter']
                or profile.document['data']['input_selection'].get(
                    'joystick_adapter', False)),
            'system_test_enabled': bool(
                profile.document['data']['input_selection']['system_test_adapter']),
            'teleop_source_id': int(source_items['teleop']['id']),
            'teleop_priority': int(source_items['teleop']['priority']),
            'system_test_source_id': int(source_items['system_test']['id']),
            'system_test_priority': int(source_items['system_test']['priority']),
            'teleop_timeout_s': float(source_items['teleop']['freshness_timeout_s']),
            'system_test_timeout_s': float(
                source_items['system_test']['freshness_timeout_s']),
            'body_envelope.planar_speed_hard_m_s': float(
                envelope['planar_speed_hard_m_s']),
            'body_envelope.yaw_rate_hard_rad_s': float(
                envelope['yaw_rate_hard_rad_s']),
            'body_envelope.gimbal_yaw_hard_rad': float(
                envelope['gimbal_yaw_hard_rad']),
            'body_envelope.xy_hard_m': float(envelope['body_xy_hard_m']),
            'body_envelope.z_hard_lower_m': float(envelope['body_z_hard_lower_m']),
            'body_envelope.z_hard_upper_m': float(envelope['body_z_hard_upper_m']),
            'body_envelope.roll_pitch_hard_rad': float(
                envelope['body_roll_pitch_hard_rad']),
            'body_envelope.yaw_hard_rad': float(envelope['body_yaw_hard_rad']),
            'body_envelope.quaternion_norm_tolerance': float(
                envelope['quaternion_norm_tolerance']),
            'body_envelope.reserved_twist_tolerance': float(
                envelope['reserved_twist_tolerance']),
            'body_envelope.stand_velocity_tolerance': float(
                envelope['stand_velocity_tolerance']),
        },
        '/araco/safety_supervisor': {
            **common,
            'loop_rate_hz': float(rates['safety_supervisor']),
            'controller_manager_validation_period_s': float(
                safety.document['data']['controller_manager_validation_period_s']),
            'joint_state_topic': wiring['topics']['joint_states'],
            'safety_policy_path': str(safety.installed_path),
            'safety_policy_sha256': safety.sha256,
            'state_joint_names': [
                joint['name'] for joint in model['joints'] if 'state' in joint['roles']
            ],
            'leg_joint_names': [
                joint['name'] for joint in model['joints'] if 'leg_command' in joint['roles']
            ],
            'gimbal_joint_names': [
                joint['name'] for joint in model['joints'] if 'gimbal_command' in joint['roles']
            ],
            'selected_command_timeout_s': float(watchdogs['selected_command']),
            'joint_state_timeout_s': float(watchdogs['joint_state']),
            'locomotion_status_timeout_s': float(watchdogs['locomotion_status']),
            'controller_state_timeout_s': float(watchdogs['controller_state']),
            'clock_progress_timeout_s': float(watchdogs['clock_progress']),
            'stable_hold_dwell_s': float(envelope['stable_hold_dwell_s']),
            'startup_readiness_stable_s': float(
                safety.document['data']['startup_readiness_stable_s']),
            'auto_enable_once_from_neutral_standing_source': bool(
                mapping_data['adapter'] == 'joystick' and
                mapping_data.get('activation_policy') ==
                'auto_enable_once_from_fresh_neutral_standing_selection'),
            'body_envelope.planar_speed_normal_m_s': float(
                envelope['planar_speed_normal_m_s']),
            'body_envelope.planar_speed_hard_m_s': float(
                envelope['planar_speed_hard_m_s']),
            'body_envelope.yaw_rate_normal_rad_s': float(
                envelope['yaw_rate_normal_rad_s']),
            'body_envelope.yaw_rate_hard_rad_s': float(
                envelope['yaw_rate_hard_rad_s']),
            'body_envelope.gimbal_yaw_normal_rad': float(
                envelope['gimbal_yaw_normal_rad']),
            'body_envelope.gimbal_yaw_hard_rad': float(
                envelope['gimbal_yaw_hard_rad']),
            'body_envelope.xy_normal_m': float(envelope['body_xy_normal_m']),
            'body_envelope.z_normal_lower_m': float(envelope['body_z_normal_lower_m']),
            'body_envelope.z_normal_upper_m': float(envelope['body_z_normal_upper_m']),
            'body_envelope.roll_pitch_normal_rad': float(
                envelope['body_roll_pitch_normal_rad']),
            'body_envelope.yaw_normal_rad': float(envelope['body_yaw_normal_rad']),
            'body_envelope.xy_hard_m': float(envelope['body_xy_hard_m']),
            'body_envelope.z_hard_lower_m': float(envelope['body_z_hard_lower_m']),
            'body_envelope.z_hard_upper_m': float(envelope['body_z_hard_upper_m']),
            'body_envelope.roll_pitch_hard_rad': float(
                envelope['body_roll_pitch_hard_rad']),
            'body_envelope.yaw_hard_rad': float(envelope['body_yaw_hard_rad']),
            'body_envelope.quaternion_norm_tolerance': float(
                envelope['quaternion_norm_tolerance']),
            'body_envelope.reserved_twist_tolerance': float(
                envelope['reserved_twist_tolerance']),
            'body_envelope.stand_velocity_tolerance': float(
                envelope['stand_velocity_tolerance']),
        },
        '/araco/locomotion': {
            **common,
            'loop_rate_hz': float(rates['locomotion']),
            'gait_path': str(gait.installed_path),
            'gait_sha256': gait.sha256,
            'leg_joint_names': [
                joint['name'] for joint in model['joints'] if 'leg_command' in joint['roles']
            ],
            'gimbal_joint_name': gimbal_joint['name'],
            'gimbal_lower_rad': float(gimbal_limit['lower_rad']),
            'gimbal_upper_rad': float(gimbal_limit['upper_rad']),
            'gimbal_command_rate_cap_rad_s': float(gimbal_limit['velocity_rad_s']),
            'nominal_positions_rad': [
                pose['joint_positions_rad'][joint['name']]
                for joint in model['joints'] if 'leg_command' in joint['roles']
            ],
            **standing,
            'trajectory_horizon_s': gait.document['data']['trajectory_horizon_s'],
            'gait_base_cadence_hz': float(gait.document['data']['base_cadence_hz']),
            'gait_maximum_cadence_hz': float(
                gait.document['data']['maximum_cadence_hz']),
            'gait_cadence_rate_hz_s': float(
                gait.document['data']['cadence_rate_hz_s']),
            'gait_preferred_maximum_stride_scale': float(
                gait.document['data']['preferred_maximum_stride_scale']),
            'gait_motion_deadband_m_s': float(
                gait.document['data']['motion_deadband_m_s']),
            'gait_duty_factor': float(gait.document['data']['duty_factor']),
            'gait_maximum_stride_m': float(gait.document['data']['maximum_stride_m']),
            'gait_swing_clearance_m': float(gait.document['data']['swing_clearance_m']),
            'gait_planar_command_scale_m_s': float(
                envelope['planar_speed_normal_m_s']),
            'gait_yaw_command_scale_rad_s': float(
                envelope['yaw_rate_normal_rad_s']),
            'translation_acceleration_m_s2': float(
                shaping['translation_acceleration_m_s2']),
            'translation_stop_deceleration_m_s2': float(
                shaping['translation_stop_deceleration_m_s2']),
            'yaw_acceleration_rad_s2': float(shaping['yaw_acceleration_rad_s2']),
            'yaw_stop_deceleration_rad_s2': float(
                shaping['yaw_stop_deceleration_rad_s2']),
            'stable_hold_dwell_s': float(envelope['stable_hold_dwell_s']),
            'body_translation_rate_m_s': float(shaping['body_translation_rate_m_s']),
            'body_angular_rate_rad_s': float(shaping['body_angular_rate_rad_s']),
            'operator_input_pre_filtered': bool(
                mapping_data.get('control_response', {}).get('kind') ==
                'legacy_p_only_time_invariant'),
            'safe_command_timeout_s': float(watchdogs['safe_command']),
        },
    }
    for parameters in definitions.values():
        fingerprint_input = copy.deepcopy(parameters)
        parameters['config.node_config_fingerprint'] = content_sha256(fingerprint_input)
    return definitions


def _emit_bundle(
    temporary: Path,
    profile: Artifact,
    artifacts: dict[str, Artifact],
    partitions: dict[str, Any],
    overrides: dict[str, Any],
) -> dict[str, Any]:
    normalized = temporary / 'normalized_artifacts'
    node_dir = temporary / 'node_params'
    controller_dir = temporary / 'ros2_control'
    gazebo_dir = temporary / 'gazebo'
    description_dir = temporary / 'description'
    for directory in (normalized, node_dir, controller_dir, gazebo_dir, description_dir):
        directory.mkdir(parents=True)
    for artifact in sorted(artifacts.values(), key=lambda item: item.artifact_id):
        filename = artifact.artifact_id.replace('.', '_') + '.json'
        (normalized / filename).write_bytes(canonical_json_bytes(artifact.document) + b'\n')

    behavior_fingerprint = _behavior_fingerprint(artifacts)
    input_fingerprint = content_sha256({
        'source_adapter_presence': profile.document['data']['input_selection'],
        'accepted_overrides': overrides,
    })
    node_parameters = _node_parameters(
        profile, artifacts, behavior_fingerprint, input_fingerprint
    )
    node_filenames = {
        '/araco/teleop_adapter': 'teleop_adapter.yaml',
        '/araco/joystick_adapter': 'joystick_adapter.yaml',
        '/araco/command_arbiter': 'command_arbiter.yaml',
        '/araco/safety_supervisor': 'safety_supervisor.yaml',
        '/araco/locomotion': 'locomotion.yaml',
    }
    for node_name, parameters in node_parameters.items():
        _yaml_write(
            node_dir / node_filenames[node_name],
            {node_name: {'ros__parameters': parameters}},
        )

    controllers = _kind(artifacts, 'controllers').document['data']
    backend = _kind(artifacts, 'gazebo_backend').document['data']
    _yaml_write(controller_dir / 'controller_manager.yaml', {
        'controller_manager': {'ros__parameters': {
            'update_rate': controllers['controller_manager_rate_hz'],
            'use_sim_time': True,
            'joint_state_broadcaster': {'type': controllers['joint_state_broadcaster_type']},
            'leg_trajectory_controller': {'type': controllers['trajectory_controller_type']},
            'gimbal_trajectory_controller': {'type': controllers['trajectory_controller_type']},
        }}
    })
    _yaml_write(controller_dir / 'joint_state_broadcaster.yaml', {
        'joint_state_broadcaster': {'ros__parameters': {
            'joints': partitions['state_joints'],
            'interfaces': ['position', 'velocity', 'effort'],
            'update_rate': controllers['joint_state_rate_hz'],
            'use_local_topics': False,
        }}
    })
    trajectory_common = {
        'command_interfaces': ['position'],
        'state_interfaces': ['position', 'velocity'],
        'interpolation_method': controllers['interpolation_method'],
        'allow_partial_joints_goal': False,
        'open_loop_control': False,
        'interpolate_from_desired_state': True,
        'allow_integration_in_goal_trajectories': False,
        'allow_nonzero_velocity_at_trajectory_end': False,
        'constraints': {'goal_time': 0.0},
    }
    _yaml_write(controller_dir / 'leg_trajectory_controller.yaml', {
        'leg_trajectory_controller': {'ros__parameters': {
            **trajectory_common,
            'joints': partitions['leg_joints'],
            'cmd_timeout': controllers['leg_cmd_timeout_s'],
        }}
    })
    _yaml_write(controller_dir / 'gimbal_trajectory_controller.yaml', {
        'gimbal_trajectory_controller': {'ros__parameters': {
            **trajectory_common,
            'joints': partitions['gimbal_joints'],
            'cmd_timeout': 0.0,
        }}
    })

    bridge = _kind(artifacts, 'bridge').document['data']
    _yaml_write(gazebo_dir / 'bridge.yaml', [
        {
            'ros_topic_name': endpoint['ros_topic'],
            'gz_topic_name': endpoint['gz_topic'],
            'ros_type_name': endpoint['ros_type'],
            'gz_type_name': endpoint['gz_type'],
            'direction': endpoint['direction'],
            **({'frame_id': endpoint['frame_id']} if 'frame_id' in endpoint else {}),
        }
        for endpoint in bridge['endpoints']
    ])
    world = _kind(artifacts, 'world').document['data']
    shutil.copyfile(_resolve_package_uri(world['world_uri']), gazebo_dir / 'resolved_world.sdf')
    backend = _kind(artifacts, 'gazebo_backend').document['data']
    (gazebo_dir / 'backend_mapping.json').write_bytes(canonical_json_bytes({
        'plugin': backend,
        'leg_joints': partitions['leg_joints'],
        'gimbal_joints': partitions['gimbal_joints'],
        'state_joints': partitions['state_joints'],
    }) + b'\n')
    _render_urdf(artifacts, description_dir / 'robot.urdf')

    generated_hashes = {}
    for path in sorted(temporary.rglob('*')):
        if path.is_file() and path.name not in {'manifest.json', 'validation_report.json'}:
            generated_hashes[str(path.relative_to(temporary))] = _file_sha256(path)
    manifest = {
        'profile_id': profile.document['data']['profile_id'],
        'profile_version': profile.document['data']['profile_version'],
        'profile_source_sha256': profile.sha256,
        'artifacts': [
            {
                'artifact_id': item.artifact_id,
                'artifact_version': item.version,
                'package': item.package,
                'relative_path': item.relative_path,
                'installed_path': str(item.installed_path),
                'sha256': item.sha256,
                'schema_id': item.document['schema_id'],
                'deployment_scope': item.document['deployment_scope'],
                'evidence_class': item.document['evidence']['class'],
            }
            for item in sorted(artifacts.values(), key=lambda value: value.artifact_id)
        ],
        'compiler': {'identity': 'araco_bringup_config', 'version': '0.1.0'},
        'environment': {
            'ros_distribution': os.environ.get('ROS_DISTRO', 'unknown'),
            'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION', 'system_default'),
            'gazebo_generation': 'Harmonic',
            'python_packages': {
                name: importlib.metadata.version(name)
                for name in ('jsonschema', 'PyYAML', 'xacro')
            },
        },
        'namespace': overrides['robot_namespace'],
        'seed': world['seed'],
        'accepted_overrides': overrides,
        'controller_partitions': partitions,
        'generated_file_sha256': generated_hashes,
        'behavior_fingerprint': behavior_fingerprint,
        'input_selection_fingerprint': input_fingerprint,
    }
    manifest['run_fingerprint'] = content_sha256(manifest)
    (temporary / 'manifest.json').write_bytes(canonical_json_bytes(manifest) + b'\n')
    report = {
        'gate': 0,
        'status': 'PASS',
        'checks': [
            'strict_yaml_and_schema', 'exact_artifact_resolution',
            'canonical_26_link_25_joint_tree', 'normalized_axes_and_finite_transforms',
            'positive_proxy_inclusive_dynamics', 'nested_limits_and_nominal_margin',
            'redistributable_resources', 'derived_24_plus_1_controller_partition',
            'strict_xacro_urdf_expansion', 'deterministic_fingerprints',
        ],
        'warnings': [
            'Simulator-only joint limits, dynamics, camera pose, and standing '
            'pose remain provisional.'
        ],
        'behavior_fingerprint': behavior_fingerprint,
        'run_fingerprint': manifest['run_fingerprint'],
    }
    (temporary / 'validation_report.json').write_bytes(canonical_json_bytes(report) + b'\n')
    return manifest


def compose_profile(
    profile_id: str,
    output_directory: str | Path,
    *,
    robot_namespace: str = 'araco',
    gui: bool | None = None,
    rviz: bool | None = None,
    log_level: str = 'info',
    record_on_failure: bool = True,
    report_destination: str = 'run_directory',
) -> dict[str, Any]:
    """Validate one installed profile and atomically emit its runtime bundle."""
    if profile_id not in PROFILE_PATHS:
        raise CompositionError(f'profile {profile_id!r} is not allowed')
    if not robot_namespace or robot_namespace.startswith('/') or '/' in robot_namespace:
        raise CompositionError('robot_namespace must be one non-empty relative token')
    if log_level not in {'debug', 'info', 'warn', 'error', 'fatal'}:
        raise CompositionError('log_level is outside the closed override policy')
    if report_destination not in {'run_directory', 'none'}:
        raise CompositionError('report_destination is outside the closed override policy')
    output = Path(output_directory).resolve()
    if output.exists():
        raise CompositionError(f'output directory already exists: {output}')
    profile = load_artifact('araco_bringup', PROFILE_PATHS[profile_id])
    if profile.document['data']['profile_id'] != profile_id:
        raise CompositionError('requested profile ID does not match installed profile')
    presentation = profile.document['data']['presentation']
    overrides = {
        'robot_namespace': robot_namespace,
        'gui': presentation['gui'] if gui is None else bool(gui),
        'rviz': presentation['rviz'] if rviz is None else bool(rviz),
        'log_level': log_level,
        'record_on_failure': bool(record_on_failure),
        'report_destination': report_destination,
    }
    artifacts = _artifact_map(profile)
    partitions = _cross_validate(artifacts)
    _validate_profile_equivalence(profile_id, artifacts)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=f'.{output.name}.', dir=output.parent))
    try:
        manifest = _emit_bundle(temporary, profile, artifacts, partitions, overrides)
        os.replace(temporary, output)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return manifest
