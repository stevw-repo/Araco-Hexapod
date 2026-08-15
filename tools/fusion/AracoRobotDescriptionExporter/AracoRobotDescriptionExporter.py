# Author: Araco project
# Description: Read-only robot-description inventory exporter for Autodesk Fusion.

import datetime
import json
import math
import os
import re
import traceback

import adsk.core
import adsk.fusion


EXPORTER_NAME = "Araco Robot Description Exporter"
EXPORTER_VERSION = "0.1.1"
SCHEMA_VERSION = 1
EXPECTED_ACTUATED_JOINTS = 25

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


def _default_filename(document_name):
    safe_name = re.sub(r"[^A-Za-z0-9._-]+", "_", document_name).strip("._")
    if not safe_name:
        safe_name = "fusion_design"
    return "{}_robot_description_export.json".format(safe_name)


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

        dialog = ui.createFileDialog()
        dialog.title = "Save Araco robot-description inventory"
        dialog.filter = "JSON files (*.json);;All files (*.*)"
        dialog.filterIndex = 0
        dialog.isMultiSelectEnabled = False
        dialog.initialFilename = _default_filename(document.name)

        result = dialog.showSave()
        if result != adsk.core.DialogResults.DialogOK:
            return

        output_path = dialog.filename
        if not output_path.lower().endswith(".json"):
            output_path += ".json"

        report = _build_report(app, document, design)
        with open(output_path, "w", encoding="utf-8", newline="\n") as output_file:
            json.dump(
                report,
                output_file,
                ensure_ascii=False,
                indent=2,
                allow_nan=False,
            )
            output_file.write("\n")

        counts = report["design"]["counts"]
        ui.messageBox(
            "Export complete.\n\n"
            "File: {}\n\n"
            "Joints: {}\n"
            "As-built joints: {}\n"
            "Revolute joints: {}\n"
            "Direct root occurrences: {}\n"
            "All occurrences: {}\n\n"
            "The Fusion design was not modified.".format(
                output_path,
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
