# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT
# Author: Araco project
# Description: Fail-closed robot-description inventory and visual exporter for Autodesk Fusion.

import datetime
import hashlib
import json
import math
import os
import re
import traceback

import adsk.core
import adsk.fusion


EXPORTER_NAME = "Araco Robot Description Exporter"
EXPORTER_VERSION = "0.4.0"
SCHEMA_VERSION = 1
VISUAL_MANIFEST_SCHEMA_VERSION = 1
EXPECTED_ACTUATED_JOINTS = 25
VISUAL_EXPORT_SPEC_FILENAME = "visual_export_spec_v1.json"

CM_TO_M = 0.01
CM2_TO_M2 = 1.0e-4
CM3_TO_M3 = 1.0e-6
KG_PER_CM3_TO_KG_PER_M3 = 1.0e6
KG_CM2_TO_KG_M2 = 1.0e-4

JOINT_TYPE_NAMES = {
    0: "rigid",
    1: "revolute",
    2: "slider",
    3: "cylindrical",
    4: "pin_slot",
    5: "planar",
    6: "ball",
    7: "inferred",
}

JOINT_DIRECTION_NAMES = {
    0: "joint_local_x",
    1: "joint_local_y",
    2: "joint_local_z",
    3: "custom",
}

CALCULATION_ACCURACY_NAMES = {
    0: "low",
    1: "medium",
    2: "high",
    3: "very_high",
}


def _finite(value):
    if value is None:
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _enum_number(value):
    if value is None:
        return None
    try:
        return int(value)
    except Exception:
        return None


def _enum_data(value, names=None):
    number = _enum_number(value)
    if names and number in names:
        name = names[number]
    else:
        name = str(value) if value is not None else None
    return {"name": name, "value": number}


def _items(collection):
    if collection is None:
        return []
    try:
        return [collection.item(index) for index in range(collection.count)]
    except Exception:
        return list(collection)


def _error(errors, field, exception):
    errors.append({"field": field, "message": str(exception)})


def _get(obj, attribute, errors=None, default=None):
    if obj is None:
        return default
    try:
        return getattr(obj, attribute)
    except Exception as exception:
        if errors is not None:
            _error(errors, attribute, exception)
        return default


def _point_m(point):
    if point is None:
        return None
    return {
        "x": _finite(point.x * CM_TO_M),
        "y": _finite(point.y * CM_TO_M),
        "z": _finite(point.z * CM_TO_M),
    }


def _vector(vector):
    if vector is None:
        return None
    return {
        "x": _finite(vector.x),
        "y": _finite(vector.y),
        "z": _finite(vector.z),
    }


def _coordinate_system(matrix, errors=None, field="transform"):
    if matrix is None:
        return None
    try:
        origin, x_axis, y_axis, z_axis = matrix.getAsCoordinateSystem()
        return {
            "origin_m": _point_m(origin),
            "x_axis": _vector(x_axis),
            "y_axis": _vector(y_axis),
            "z_axis": _vector(z_axis),
        }
    except Exception as exception:
        if errors is not None:
            _error(errors, field, exception)
        return None


def _parameter(parameter, errors=None, field="parameter"):
    if parameter is None:
        return None
    local_errors = []
    result = {
        "name": _get(parameter, "name", local_errors),
        "expression": _get(parameter, "expression", local_errors),
        "unit": _get(parameter, "unit", local_errors),
        "value_fusion_internal": _finite(_get(parameter, "value", local_errors)),
    }
    if local_errors:
        result["errors"] = local_errors
        if errors is not None:
            errors.append({"field": field, "message": "See nested parameter errors"})
    return result


def _limits(limits, unit, errors=None, field="limits"):
    if limits is None:
        return None
    local_errors = []
    result = {
        "unit": unit,
        "minimum_enabled": bool(_get(limits, "isMinimumValueEnabled", local_errors, False)),
        "minimum": _finite(_get(limits, "minimumValue", local_errors)),
        "maximum_enabled": bool(_get(limits, "isMaximumValueEnabled", local_errors, False)),
        "maximum": _finite(_get(limits, "maximumValue", local_errors)),
        "rest_enabled": bool(_get(limits, "isRestValueEnabled", local_errors, False)),
        "rest": _finite(_get(limits, "restValue", local_errors)),
    }
    if local_errors:
        result["errors"] = local_errors
        if errors is not None:
            errors.append({"field": field, "message": "See nested limit errors"})
    return result


def _motion(motion, errors=None):
    if motion is None:
        return None

    local_errors = []
    joint_type_value = _get(motion, "jointType", local_errors)
    joint_type_number = _enum_number(joint_type_value)
    result = {
        "object_type": _get(motion, "objectType", local_errors),
        "joint_type": _enum_data(joint_type_value, JOINT_TYPE_NAMES),
    }

    if joint_type_number == 1:
        result.update(
            {
                "rotation_axis_definition": _enum_data(
                    _get(motion, "rotationAxis", local_errors),
                    JOINT_DIRECTION_NAMES,
                ),
                "rotation_axis_vector_root_fusion": _vector(
                    _get(motion, "rotationAxisVector", local_errors)
                ),
                "rotation_value_rad": _finite(
                    _get(motion, "rotationValue", local_errors)
                ),
                "rotation_limits": _limits(
                    _get(motion, "rotationLimits", local_errors),
                    "radian",
                    local_errors,
                    "rotationLimits",
                ),
            }
        )
    elif joint_type_number == 2:
        result.update(
            {
                "slide_direction_vector_root_fusion": _vector(
                    _get(motion, "slideDirectionVector", local_errors)
                ),
                "slide_value_m": _finite(
                    _get(motion, "slideValue", local_errors) * CM_TO_M
                )
                if _get(motion, "slideValue", local_errors) is not None
                else None,
                "slide_limits_fusion_internal_cm": _limits(
                    _get(motion, "slideLimits", local_errors),
                    "centimeter",
                    local_errors,
                    "slideLimits",
                ),
            }
        )

    if local_errors:
        result["errors"] = local_errors
        if errors is not None:
            errors.append({"field": "jointMotion", "message": "See nested motion errors"})
    return result


def _occurrence_reference(occurrence, errors=None):
    if occurrence is None:
        return {"kind": "root_component_or_unavailable", "full_path": None}

    local_errors = []
    component = _get(occurrence, "component", local_errors)
    result = {
        "name": _get(occurrence, "name", local_errors),
        "full_path": _get(occurrence, "fullPathName", local_errors),
        "component_name": _get(component, "name", local_errors),
        "component_part_number": _get(component, "partNumber", local_errors),
    }
    if local_errors:
        result["errors"] = local_errors
        if errors is not None:
            errors.append({"field": "occurrence", "message": "See nested occurrence errors"})
    return result


def _body_inventory(occurrence, errors=None):
    bodies = []
    try:
        body_collection = occurrence.bRepBodies
        for body in _items(body_collection):
            body_errors = []
            material = _get(body, "material", body_errors)
            appearance = _get(body, "appearance", body_errors)
            record = {
                "name": _get(body, "name", body_errors),
                "is_solid": bool(_get(body, "isSolid", body_errors, False)),
                "material": _get(material, "name", body_errors),
                "appearance": _get(appearance, "name", body_errors),
            }
            if body_errors:
                record["errors"] = body_errors
            bodies.append(record)
    except Exception as exception:
        if errors is not None:
            _error(errors, "bRepBodies", exception)
    return bodies


def _occurrence_inventory(occurrence, include_bodies=True):
    errors = []
    component = _get(occurrence, "component", errors)
    transform = _get(occurrence, "transform2", errors)
    child_occurrences = _get(occurrence, "childOccurrences", errors)
    result = {
        "name": _get(occurrence, "name", errors),
        "full_path": _get(occurrence, "fullPathName", errors),
        "component_name": _get(component, "name", errors),
        "component_part_number": _get(component, "partNumber", errors),
        "component_description": _get(component, "description", errors),
        "is_grounded": bool(_get(occurrence, "isGrounded", errors, False)),
        "is_visible": bool(_get(occurrence, "isVisible", errors, False)),
        "child_occurrence_count": len(_items(child_occurrences)),
        "transform_root_fusion": _coordinate_system(transform, errors, "transform2"),
    }
    if include_bodies:
        result["direct_bodies"] = _body_inventory(occurrence, errors)
    if errors:
        result["errors"] = errors
    return result


def _physical_properties(target, accuracy):
    errors = []
    try:
        properties = target.getPhysicalProperties(accuracy)
    except Exception as exception:
        return {"available": False, "errors": [{"field": "getPhysicalProperties", "message": str(exception)}]}

    if properties is None:
        return {"available": False, "errors": [{"field": "getPhysicalProperties", "message": "Fusion returned null"}]}

    center_of_mass = _get(properties, "centerOfMass", errors)
    result = {
        "available": True,
        "accuracy": _enum_data(
            _get(properties, "accuracy", errors),
            CALCULATION_ACCURACY_NAMES,
        ),
        "mass_kg": _finite(_get(properties, "mass", errors)),
        "area_m2": None,
        "volume_m3": None,
        "average_density_kg_m3": None,
        "center_of_mass_root_fusion_m": _point_m(center_of_mass),
    }

    area = _get(properties, "area", errors)
    volume = _get(properties, "volume", errors)
    density = _get(properties, "density", errors)
    result["area_m2"] = _finite(area * CM2_TO_M2) if area is not None else None
    result["volume_m3"] = _finite(volume * CM3_TO_M3) if volume is not None else None
    result["average_density_kg_m3"] = (
        _finite(density * KG_PER_CM3_TO_KG_PER_M3)
        if density is not None
        else None
    )

    try:
        success, xx, yy, zz, xy, yz, xz = properties.getXYZMomentsOfInertia()
        result["fusion_world_xyz_moments_kg_m2"] = {
            "available": bool(success),
            "xx": _finite(xx * KG_CM2_TO_KG_M2),
            "yy": _finite(yy * KG_CM2_TO_KG_M2),
            "zz": _finite(zz * KG_CM2_TO_KG_M2),
            "xy": _finite(xy * KG_CM2_TO_KG_M2),
            "yz": _finite(yz * KG_CM2_TO_KG_M2),
            "xz": _finite(xz * KG_CM2_TO_KG_M2),
        }
    except Exception as exception:
        _error(errors, "getXYZMomentsOfInertia", exception)

    try:
        success, i1, i2, i3 = properties.getPrincipalMomentsOfInertia()
        result["principal_moments_kg_m2"] = {
            "available": bool(success),
            "i1": _finite(i1 * KG_CM2_TO_KG_M2),
            "i2": _finite(i2 * KG_CM2_TO_KG_M2),
            "i3": _finite(i3 * KG_CM2_TO_KG_M2),
        }
    except Exception as exception:
        _error(errors, "getPrincipalMomentsOfInertia", exception)

    try:
        success, x_axis, y_axis, z_axis = properties.getPrincipalAxes()
        result["principal_axes_root_fusion"] = {
            "available": bool(success),
            "x_axis": _vector(x_axis),
            "y_axis": _vector(y_axis),
            "z_axis": _vector(z_axis),
        }
    except Exception as exception:
        _error(errors, "getPrincipalAxes", exception)

    if errors:
        result["errors"] = errors
    return result


def _timeline_index(item, errors):
    timeline_object = _get(item, "timelineObject", errors)
    return _get(timeline_object, "index", errors)


def _joint(joint):
    errors = []
    geometry_one_transform = _get(joint, "geometryOneTransform", errors)
    geometry_two_transform = _get(joint, "geometryTwoTransform", errors)
    parent_component = _get(joint, "parentComponent", errors)
    assembly_context = _get(joint, "assemblyContext", errors)
    result = {
        "source": "joint",
        "name": _get(joint, "name", errors),
        "entity_token_snapshot": _get(joint, "entityToken", errors),
        "timeline_index": _timeline_index(joint, errors),
        "parent_component": _get(parent_component, "name", errors),
        "assembly_context": _occurrence_reference(assembly_context, errors),
        "occurrence_one": _occurrence_reference(
            _get(joint, "occurrenceOne", errors), errors
        ),
        "occurrence_two": _occurrence_reference(
            _get(joint, "occurrenceTwo", errors), errors
        ),
        "geometry_one_root_fusion": _coordinate_system(
            geometry_one_transform, errors, "geometryOneTransform"
        ),
        "geometry_two_root_fusion": _coordinate_system(
            geometry_two_transform, errors, "geometryTwoTransform"
        ),
        "is_flipped": bool(_get(joint, "isFlipped", errors, False)),
        "is_locked": bool(_get(joint, "isLocked", errors, False)),
        "is_suppressed": bool(_get(joint, "isSuppressed", errors, False)),
        "health_state": _enum_data(_get(joint, "healthState", errors)),
        "health_message": _get(joint, "errorOrWarningMessage", errors),
        "angle_parameter": _parameter(_get(joint, "angle", errors), errors, "angle"),
        "offset_parameter": _parameter(_get(joint, "offset", errors), errors, "offset"),
        "offset_x_parameter": _parameter(_get(joint, "offsetX", errors), errors, "offsetX"),
        "offset_y_parameter": _parameter(_get(joint, "offsetY", errors), errors, "offsetY"),
        "motion": _motion(_get(joint, "jointMotion", errors), errors),
    }
    if errors:
        result["errors"] = errors
    return result


def _as_built_joint(joint):
    errors = []
    parent_component = _get(joint, "parentComponent", errors)
    assembly_context = _get(joint, "assemblyContext", errors)
    result = {
        "source": "as_built_joint",
        "name": _get(joint, "name", errors),
        "entity_token_snapshot": _get(joint, "entityToken", errors),
        "timeline_index": _timeline_index(joint, errors),
        "parent_component": _get(parent_component, "name", errors),
        "assembly_context": _occurrence_reference(assembly_context, errors),
        "occurrence_one": _occurrence_reference(
            _get(joint, "occurrenceOne", errors), errors
        ),
        "occurrence_two": _occurrence_reference(
            _get(joint, "occurrenceTwo", errors), errors
        ),
        "geometry_root_fusion": _coordinate_system(
            _get(joint, "transform", errors), errors, "transform"
        ),
        "is_suppressed": bool(_get(joint, "isSuppressed", errors, False)),
        "motion": _motion(_get(joint, "jointMotion", errors), errors),
    }
    if errors:
        result["errors"] = errors
    return result


def _data_file(document):
    errors = []
    data_file = _get(document, "dataFile", errors)
    if data_file is None:
        return {"available": False, "errors": errors}
    result = {
        "available": True,
        "name": _get(data_file, "name", errors),
        "id": _get(data_file, "id", errors),
        "version_number": _get(data_file, "versionNumber", errors),
        "file_extension": _get(data_file, "fileExtension", errors),
    }
    if errors:
        result["errors"] = errors
    return result


def _build_report(app, document, design):
    root = design.rootComponent
    accuracy = adsk.fusion.CalculationAccuracy.HighCalculationAccuracy

    joints = [_joint(item) for item in _items(root.allJoints)]
    as_built_joints = [
        _as_built_joint(item) for item in _items(root.allAsBuiltJoints)
    ]
    all_joint_records = joints + as_built_joints
    revolute_count = sum(
        1
        for record in all_joint_records
        if record.get("motion", {}).get("joint_type", {}).get("name") == "revolute"
    )

    direct_occurrences = []
    for occurrence in _items(root.occurrences):
        record = _occurrence_inventory(occurrence, include_bodies=True)
        record["physical_properties"] = _physical_properties(occurrence, accuracy)
        direct_occurrences.append(record)

    all_occurrences = [
        _occurrence_inventory(occurrence, include_bodies=True)
        for occurrence in _items(root.allOccurrences)
    ]

    detected_count = len(all_joint_records)
    if detected_count == EXPECTED_ACTUATED_JOINTS and revolute_count == EXPECTED_ACTUATED_JOINTS:
        gate_status = "expected_25_revolute_joints_detected"
    elif detected_count < EXPECTED_ACTUATED_JOINTS:
        gate_status = "joint_definitions_missing"
    else:
        gate_status = "joint_count_or_types_require_review"

    units_manager = design.unitsManager
    return {
        "schema": {
            "name": "araco_fusion_robot_description_export",
            "version": SCHEMA_VERSION,
        },
        "exporter": {
            "name": EXPORTER_NAME,
            "version": EXPORTER_VERSION,
            "generated_at_utc": datetime.datetime.now(
                datetime.timezone.utc
            ).isoformat(),
            "fusion_application_version": app.version,
            "read_only_design_access": True,
        },
        "units_and_frames": {
            "fusion_internal_length_unit": "centimeter",
            "exported_length_unit": "meter",
            "exported_angle_unit": "radian",
            "exported_mass_unit": "kilogram",
            "exported_inertia_unit": "kilogram_meter_squared",
            "root_frame": "Fusion root-component coordinate system",
            "design_default_length_unit": units_manager.defaultLengthUnits,
            "joint_geometry_note": "rootComponent.allJoints returns joints flattened into root-component assembly context",
            "physical_inertia_note": "fusion_world_xyz_moments_kg_m2 preserves Autodesk getXYZMomentsOfInertia component labels; validate tensor convention before URDF use",
        },
        "design": {
            "document_name": document.name,
            "is_modified": bool(_get(document, "isModified", default=False)),
            "root_component_name": root.name,
            "design_type": _enum_data(design.designType),
            "cloud_data_file": _data_file(document),
            "counts": {
                "direct_root_occurrences": len(direct_occurrences),
                "all_occurrences": len(all_occurrences),
                "joints": len(joints),
                "as_built_joints": len(as_built_joints),
                "all_joint_definitions": detected_count,
                "revolute_joint_definitions": revolute_count,
            },
        },
        "robot_description_gate": {
            "expected_actuated_joint_count": EXPECTED_ACTUATED_JOINTS,
            "detected_joint_definition_count": detected_count,
            "detected_revolute_joint_count": revolute_count,
            "status": gate_status,
            "does_not_validate_mechanical_safety": True,
        },
        "root_assembly_physical_properties": _physical_properties(root, accuracy),
        "joints": joints,
        "as_built_joints": as_built_joints,
        "direct_root_occurrences": direct_occurrences,
        "all_occurrences": all_occurrences,
    }


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as input_file:
        while True:
            chunk = input_file.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def _write_json(path, value):
    with open(path, "w", encoding="utf-8", newline="\n") as output_file:
        json.dump(value, output_file, ensure_ascii=False, indent=2, allow_nan=False)
        output_file.write("\n")


def _load_visual_export_spec():
    spec_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), VISUAL_EXPORT_SPEC_FILENAME)
    with open(spec_path, "r", encoding="utf-8") as spec_file:
        spec = json.load(spec_file)
    return spec_path, spec


def _validate_visual_export_spec(spec):
    errors = []
    if spec.get("schema") != {
        "name": "araco_fusion_visual_export_spec",
        "version": 1,
    }:
        errors.append("unsupported or malformed visual-export schema")

    rights = spec.get("rights_boundary", {})
    if rights.get("license") != "mixed-open-source":
        errors.append("rights_boundary.license must describe the mixed open-source set")
    if rights.get("license_policy") != "preserve_per_asset":
        errors.append("rights_boundary.license_policy must preserve per-asset licenses")
    if rights.get("redistribution_confirmed") is not True:
        errors.append("rights_boundary.redistribution_confirmed must be true")
    if rights.get("fail_closed") is not True:
        errors.append("rights_boundary.fail_closed must be true")
    if rights.get("unlisted_geometry_export_allowed") is not False:
        errors.append("unlisted geometry must not be exportable")

    mesh_options = spec.get("mesh_options", {})
    required_options = {
        "format": "binary_stl",
        "length_unit": "millimeter",
        "refinement": "high",
        "coordinate_context": "Fusion source-component-local body coordinates; occurrence transform recorded separately",
    }
    if mesh_options != required_options:
        errors.append("mesh_options must match the reviewed binary-STL contract")

    exports = spec.get("exports")
    if not isinstance(exports, list) or not exports:
        errors.append("exports must be a non-empty list")
        exports = []
    elif len(exports) != 59:
        errors.append("exports must contain exactly 59 reviewed mesh selections")

    asset_ids = set()
    selections = set()
    reviewed_body_count = 0
    for index, selection in enumerate(exports):
        prefix = "exports[{}]".format(index)
        if not isinstance(selection, dict):
            errors.append("{} must be an object".format(prefix))
            continue
        asset_id = selection.get("asset_id")
        canonical_link = selection.get("canonical_link")
        occurrence_path = selection.get("occurrence_full_path")
        export_scope = selection.get("export_scope", "body")
        visual_role = selection.get("visual_role", "primary")
        if not isinstance(asset_id, str) or re.fullmatch(r"[a-z0-9_]+", asset_id) is None:
            errors.append("{}.asset_id is not a safe file stem".format(prefix))
        elif asset_id in asset_ids:
            errors.append("duplicate asset_id: {}".format(asset_id))
        else:
            asset_ids.add(asset_id)
        if not isinstance(canonical_link, str) or re.fullmatch(r"[a-z0-9_]+_link|base_link", canonical_link) is None:
            errors.append("{}.canonical_link is invalid".format(prefix))
        if not isinstance(occurrence_path, str) or not occurrence_path.startswith("araco - "):
            errors.append("{}.occurrence_full_path is outside the project-owned prefix".format(prefix))
        if visual_role not in {
            "primary",
            "servo_case",
            "tibia_component_auto",
            "camera_body",
            "camera_hardware",
            "camera_optics",
        }:
            errors.append("{}.visual_role is invalid".format(prefix))
        if export_scope == "body":
            expected_bodies = [
                {
                    "body_name": selection.get("body_name"),
                    "expected_material": selection.get("expected_material"),
                }
            ]
            if "expected_bodies" in selection:
                errors.append("{}.expected_bodies is forbidden for body scope".format(prefix))
        elif export_scope == "occurrence":
            expected_bodies = selection.get("expected_bodies")
            if not isinstance(expected_bodies, list) or not expected_bodies:
                errors.append("{}.expected_bodies must be a non-empty list".format(prefix))
                expected_bodies = []
            if "body_name" in selection or "expected_material" in selection:
                errors.append("{}.body fields are forbidden for occurrence scope".format(prefix))
            if visual_role != "tibia_component_auto":
                errors.append("{}.occurrence scope is reserved for exact tibia components".format(prefix))
        else:
            errors.append("{}.export_scope is invalid".format(prefix))
            expected_bodies = []
        reviewed_body_count += len(expected_bodies)
        expected_names = set()
        for body_index, expected_body in enumerate(expected_bodies):
            body_prefix = "{}.expected_bodies[{}]".format(prefix, body_index)
            if not isinstance(expected_body, dict):
                errors.append("{} must be an object".format(body_prefix))
                continue
            body_name = expected_body.get("body_name")
            material = expected_body.get("expected_material")
            if not isinstance(body_name, str) or not body_name:
                errors.append("{}.body_name is missing".format(body_prefix))
            elif body_name in expected_names:
                errors.append("{}.body_name is duplicated".format(body_prefix))
            expected_names.add(body_name)
            if material not in {"PETG", "Steel"}:
                errors.append("{}.expected_material must be PETG or Steel".format(body_prefix))
            selection_key = (occurrence_path, body_name)
            if selection_key in selections:
                errors.append("duplicate occurrence/body selection: {!r}".format(selection_key))
            selections.add(selection_key)

    if reviewed_body_count != 77:
        errors.append("mesh selections must cover exactly 77 reviewed bodies")

    proxy_links = spec.get("retained_proxy_links")
    if proxy_links != []:
        errors.append("retained_proxy_links must be empty for the exact-mesh export")

    expected_exported_links = {
        "base_link",
        "gimbal_yaw_link",
        "left_front_coxa_link",
        "left_front_femur_link",
        "left_front_foot_link",
        "left_front_tibia_link",
        "left_middle_coxa_link",
        "left_middle_femur_link",
        "left_middle_foot_link",
        "left_middle_tibia_link",
        "left_rear_coxa_link",
        "left_rear_femur_link",
        "left_rear_foot_link",
        "left_rear_tibia_link",
        "right_front_coxa_link",
        "right_front_femur_link",
        "right_front_foot_link",
        "right_front_tibia_link",
        "right_middle_coxa_link",
        "right_middle_femur_link",
        "right_middle_foot_link",
        "right_middle_tibia_link",
        "right_rear_coxa_link",
        "right_rear_femur_link",
        "right_rear_foot_link",
        "right_rear_tibia_link",
        "camera_link",
    }
    actual_exported_links = {
        selection.get("canonical_link")
        for selection in exports
        if isinstance(selection, dict)
    }
    if actual_exported_links != expected_exported_links:
        errors.append("exports do not exactly cover the 26 primary links plus camera_link")

    excluded_categories = {
        entry.get("category")
        for entry in spec.get("excluded_geometry", [])
        if isinstance(entry, dict)
    }
    if excluded_categories != {"deferred_electronics_and_sensors", "unlisted_araco_bodies"}:
        errors.append("excluded_geometry must preserve both reviewed exclusion categories")

    if errors:
        raise RuntimeError(
            "Visual export specification failed validation:\n- {}".format("\n- ".join(errors))
        )


def _preflight_visual_exports(document, design, report, spec):
    expected = spec["expected_design"]
    actual_design = report["design"]
    actual_counts = actual_design["counts"]
    actual_cloud = actual_design["cloud_data_file"]
    checks = [
        (actual_design["document_name"], expected["document_name"], "document name"),
        (actual_design["root_component_name"], expected["root_component_name"], "root component name"),
        (actual_cloud.get("id"), expected["cloud_data_file_id"], "cloud data-file lineage"),
        (actual_cloud.get("version_number"), expected["cloud_version_number"], "cloud version"),
        (actual_counts["direct_root_occurrences"], expected["direct_root_occurrences"], "direct root occurrence count"),
        (actual_counts["all_occurrences"], expected["all_occurrences"], "all occurrence count"),
        (actual_counts["all_joint_definitions"], expected["joint_definitions"], "joint definition count"),
        (actual_counts["revolute_joint_definitions"], expected["revolute_joint_definitions"], "revolute joint count"),
    ]
    errors = [
        "{} differs: actual={!r}, expected={!r}".format(label, actual, required)
        for actual, required, label in checks
        if actual != required
    ]
    if bool(_get(document, "isModified", default=False)):
        errors.append("the active Fusion document has unsaved modifications; save it before exporting")

    occurrences_by_path = {}
    for occurrence in _items(design.rootComponent.allOccurrences):
        full_path = _get(occurrence, "fullPathName")
        occurrences_by_path.setdefault(full_path, []).append(occurrence)

    resolved = []
    for selection in spec["exports"]:
        occurrence_path = selection["occurrence_full_path"]
        occurrence_matches = occurrences_by_path.get(occurrence_path, [])
        if len(occurrence_matches) != 1:
            errors.append(
                "selection {} resolved to {} assembly occurrences, expected exactly 1".format(
                    occurrence_path, len(occurrence_matches)
                )
            )
            continue
        occurrence = occurrence_matches[0]
        bodies_by_name = {
            _get(body, "name"): body for body in _items(occurrence.bRepBodies)
        }
        if selection.get("export_scope", "body") == "body":
            expected_bodies = [
                {
                    "body_name": selection["body_name"],
                    "expected_material": selection["expected_material"],
                }
            ]
        else:
            expected_bodies = selection["expected_bodies"]
            expected_names = {entry["body_name"] for entry in expected_bodies}
            if set(bodies_by_name) != expected_names:
                errors.append(
                    "occurrence selection {} direct bodies differ: actual={!r}, expected={!r}".format(
                        occurrence_path, sorted(bodies_by_name), sorted(expected_names)
                    )
                )
        selected_bodies = []
        for expected_body in expected_bodies:
            body_name = expected_body["body_name"]
            body = bodies_by_name.get(body_name)
            if body is None:
                errors.append("selection {}/{} did not resolve".format(occurrence_path, body_name))
                continue
            material = _get(_get(body, "material"), "name")
            if material != expected_body["expected_material"]:
                errors.append(
                    "selection {}/{} material differs: actual={!r}, expected={!r}".format(
                        occurrence_path, body_name, material, expected_body["expected_material"]
                    )
                )
            if not bool(_get(body, "isSolid", default=False)):
                errors.append("selection {}/{} is not a solid body".format(occurrence_path, body_name))
            selected_bodies.append(body)
        if len(selected_bodies) != len(expected_bodies):
            continue
        if selection.get("export_scope") == "occurrence":
            export_entity = _get(occurrence, "component")
            if export_entity is None:
                errors.append("selection {} has no source component".format(occurrence_path))
                continue
        else:
            export_entity = selected_bodies[0]
        resolved.append((selection, occurrence, export_entity, selected_bodies))

    if errors:
        raise RuntimeError(
            "Fusion design failed the fail-closed visual-export preflight:\n- {}".format(
                "\n- ".join(errors)
            )
        )
    return resolved


def _export_visual_bundle(app, document, design, report, spec_path, spec, resolved, parent_folder):
    generated_at = datetime.datetime.now(datetime.timezone.utc)
    timestamp = generated_at.strftime("%Y%m%dT%H%M%SZ")
    design_stem = re.sub(r"[^A-Za-z0-9._-]+", "_", document.name).strip("._") or "fusion_design"
    bundle_stem = "{}_visual_export_{}".format(design_stem, timestamp)
    incomplete_path = os.path.join(parent_folder, bundle_stem + ".incomplete")
    final_path = os.path.join(parent_folder, bundle_stem)
    if os.path.exists(incomplete_path) or os.path.exists(final_path):
        raise RuntimeError("Refusing to overwrite an existing export bundle: {}".format(bundle_stem))

    os.makedirs(incomplete_path)
    meshes_path = os.path.join(incomplete_path, "meshes_root_fusion_mm")
    os.makedirs(meshes_path)
    try:
        inventory_path = os.path.join(incomplete_path, "robot_description_inventory.json")
        _write_json(inventory_path, report)
        spec_copy_path = os.path.join(incomplete_path, VISUAL_EXPORT_SPEC_FILENAME)
        _write_json(spec_copy_path, spec)

        export_manager = design.exportManager
        exported = []
        for selection, occurrence, export_entity, selected_bodies in resolved:
            filename = selection["asset_id"] + ".stl"
            output_path = os.path.join(meshes_path, filename)
            options = export_manager.createSTLExportOptions(export_entity, output_path)
            if options is None:
                raise RuntimeError("Fusion could not create STL options for {}".format(selection["asset_id"]))
            options.sendToPrintUtility = False
            options.isBinaryFormat = True
            options.isOneFilePerBody = False
            options.unitType = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
            options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
            if not export_manager.execute(options):
                raise RuntimeError("Fusion reported that STL export failed for {}".format(selection["asset_id"]))
            if not os.path.isfile(output_path) or os.path.getsize(output_path) <= 84:
                raise RuntimeError("Exported STL is missing or too small: {}".format(output_path))

            source_bodies = [
                {
                    "body_name": _get(body, "name"),
                    "body_entity_token_snapshot": _get(body, "entityToken"),
                    "material": _get(_get(body, "material"), "name"),
                    "appearance": _get(_get(body, "appearance"), "name"),
                }
                for body in selected_bodies
            ]
            exported.append(
                {
                    "asset_id": selection["asset_id"],
                    "canonical_link": selection["canonical_link"],
                    "path": "meshes_root_fusion_mm/{}".format(filename),
                    "sha256": _sha256(output_path),
                    "size_bytes": os.path.getsize(output_path),
                    "source_occurrence_full_path": selection["occurrence_full_path"],
                    "source_occurrence_transform_root_fusion": _coordinate_system(
                        _get(occurrence, "transform2")
                    ),
                    "export_scope": selection.get("export_scope", "body"),
                    "source_bodies": source_bodies,
                    "visual_role": selection.get("visual_role", "primary"),
                    "creator": selection.get("creator", "Araco Hexapod contributors"),
                    "license": selection.get("license", "MIT"),
                    "redistribution": "allowed_by_rights_holder",
                }
            )

        if bool(_get(document, "isModified", default=False)):
            raise RuntimeError("Fusion marked the design modified during export; bundle remains incomplete for review")

        manifest = {
            "schema": {
                "name": "araco_fusion_visual_export_manifest",
                "version": VISUAL_MANIFEST_SCHEMA_VERSION,
            },
            "exporter": {
                "name": EXPORTER_NAME,
                "version": EXPORTER_VERSION,
                "fusion_application_version": app.version,
                "generated_at_utc": generated_at.isoformat(),
                "read_only_design_access": True,
            },
            "source_design": report["design"],
            "rights_boundary": spec["rights_boundary"],
            "mesh_options": spec["mesh_options"],
            "visual_export_spec": {
                "path": VISUAL_EXPORT_SPEC_FILENAME,
                "sha256": _sha256(spec_copy_path),
                "source_path_sha256": _sha256(spec_path),
            },
            "robot_description_inventory": {
                "path": "robot_description_inventory.json",
                "sha256": _sha256(inventory_path),
            },
            "export_count": len(exported),
            "reviewed_body_count": sum(len(entry["source_bodies"]) for entry in exported),
            "exports": exported,
            "retained_proxy_links": spec["retained_proxy_links"],
            "excluded_geometry": spec["excluded_geometry"],
            "normalization_status": "not_started",
            "integration_status": "not_started",
        }
        _write_json(os.path.join(incomplete_path, "visual_export_manifest.json"), manifest)
        os.rename(incomplete_path, final_path)
        return final_path, manifest
    except Exception:
        try:
            with open(os.path.join(incomplete_path, "EXPORT_FAILED.txt"), "w", encoding="utf-8", newline="\n") as failure_file:
                failure_file.write(traceback.format_exc())
        except Exception:
            pass
        raise


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        document = app.activeDocument
        design = adsk.fusion.Design.cast(app.activeProduct)

        if document is None or design is None:
            ui.messageBox(
                "Open the Araco assembly in the Design workspace before running {}.".format(
                    EXPORTER_NAME
                )
            )
            return

        spec_path, spec = _load_visual_export_spec()
        _validate_visual_export_spec(spec)
        report = _build_report(app, document, design)
        resolved = _preflight_visual_exports(document, design, report, spec)

        dialog = ui.createFolderDialog()
        dialog.title = "Choose parent folder for the Araco visual export bundle"
        result = dialog.showDialog()
        if result != adsk.core.DialogResults.DialogOK:
            return

        output_path, manifest = _export_visual_bundle(
            app,
            document,
            design,
            report,
            spec_path,
            spec,
            resolved,
            dialog.folder,
        )

        counts = report["design"]["counts"]
        ui.messageBox(
            "Export complete.\n\n"
            "Bundle: {}\n"
            "Exported mesh files: {}\n"
            "Reviewed source bodies: {}\n"
            "Retained visual proxies: {}\n\n"
            "Joints: {}\n"
            "As-built joints: {}\n"
            "Revolute joints: {}\n"
            "Direct root occurrences: {}\n"
            "All occurrences: {}\n\n"
            "The Fusion design was not modified.".format(
                output_path,
                manifest["export_count"],
                manifest["reviewed_body_count"],
                len(manifest["retained_proxy_links"]),
                counts["joints"],
                counts["as_built_joints"],
                counts["revolute_joint_definitions"],
                counts["direct_root_occurrences"],
                counts["all_occurrences"],
            )
        )
    except Exception:
        if ui is not None:
            ui.messageBox(
                "{} failed:\n\n{}".format(EXPORTER_NAME, traceback.format_exc())
            )


def stop(context):
    # Fusion scripts finish when run returns; this exists for manifest compatibility.
    pass
