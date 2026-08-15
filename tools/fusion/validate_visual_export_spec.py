#!/usr/bin/env python3
# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT
"""Validate the reviewed Fusion visual-export whitelist against project evidence."""

import argparse
import json
import pathlib
import re
import sys


SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_SPEC = SCRIPT_DIR / "AracoRobotDescriptionExporter" / "visual_export_spec_v1.json"
DEFAULT_MODEL = REPO_ROOT / "src" / "araco_description" / "config" / "model" / "canonical_model_v1.yaml"


def _load_json(path):
    with path.open("r", encoding="utf-8") as input_file:
        return json.load(input_file)


def _expected_bodies(entry):
    if entry.get("export_scope", "body") == "body":
        return [
            {
                "body_name": entry.get("body_name"),
                "expected_material": entry.get("expected_material"),
            }
        ]
    return entry.get("expected_bodies", [])


def _validate(spec, model, inventory=None):
    errors = []
    if spec.get("schema") != {"name": "araco_fusion_visual_export_spec", "version": 1}:
        errors.append("unexpected spec schema")
    rights = spec.get("rights_boundary", {})
    if rights.get("license") != "mixed-open-source" or rights.get("fail_closed") is not True:
        errors.append("rights boundary must identify the mixed open-source set and fail closed")
    if rights.get("license_policy") != "preserve_per_asset":
        errors.append("rights boundary must preserve per-asset licenses")
    if rights.get("redistribution_confirmed") is not True:
        errors.append("rights boundary must record the user's redistribution confirmation")
    if rights.get("unlisted_geometry_export_allowed") is not False:
        errors.append("unlisted geometry must be forbidden")

    exports = spec.get("exports", [])
    if len(exports) != 59:
        errors.append("expected exactly 59 whitelisted mesh exports, found {}".format(len(exports)))
    asset_ids = [entry.get("asset_id") for entry in exports]
    if len(set(asset_ids)) != len(asset_ids):
        errors.append("asset_id values must be unique")
    if any(not isinstance(value, str) or re.fullmatch(r"[a-z0-9_]+", value) is None for value in asset_ids):
        errors.append("asset_id values must be safe lowercase file stems")
    selections = [
        (entry.get("occurrence_full_path"), body.get("body_name"))
        for entry in exports
        for body in _expected_bodies(entry)
    ]
    if len(set(selections)) != len(selections):
        errors.append("occurrence/body selections must be unique")
    if any(not str(path).startswith("araco - ") for path, _ in selections):
        errors.append("every selected occurrence must use the project-owned araco prefix")
    if any(
        body.get("expected_material") not in {"PETG", "Steel"}
        for entry in exports
        for body in _expected_bodies(entry)
    ):
        errors.append("every selected body must require PETG or Steel during Fusion preflight")
    if len(selections) != 77:
        errors.append("mesh selections must cover exactly 77 reviewed bodies")
    roles = [entry.get("visual_role", "primary") for entry in exports]
    expected_role_counts = {
        "primary": 25,
        "servo_case": 13,
        "tibia_component_auto": 6,
        "camera_body": 5,
        "camera_hardware": 6,
        "camera_optics": 4,
    }
    actual_role_counts = {role: roles.count(role) for role in set(roles)}
    if actual_role_counts != expected_role_counts:
        errors.append(
            "visual-role counts differ: actual={!r}, expected={!r}".format(
                actual_role_counts, expected_role_counts
            )
        )

    exported_link_contract = set(model["data"]["primary_link_order"]) | {"camera_link"}
    exported_links = {entry.get("canonical_link") for entry in exports}
    if spec.get("retained_proxy_links") != []:
        errors.append("the exact-mesh specification must retain no visual proxies")
    if exported_links != exported_link_contract:
        errors.append("exports must cover all primary links plus camera_link")

    if inventory is not None:
        expected = spec["expected_design"]
        actual = inventory["design"]
        cloud = actual["cloud_data_file"]
        counts = actual["counts"]
        comparisons = [
            (actual["document_name"], expected["document_name"], "document_name"),
            (actual["root_component_name"], expected["root_component_name"], "root_component_name"),
            (cloud.get("id"), expected["cloud_data_file_id"], "cloud_data_file_id"),
            (cloud.get("version_number"), expected["cloud_version_number"], "cloud_version_number"),
            (counts["direct_root_occurrences"], expected["direct_root_occurrences"], "direct_root_occurrences"),
            (counts["all_occurrences"], expected["all_occurrences"], "all_occurrences"),
            (counts["all_joint_definitions"], expected["joint_definitions"], "joint_definitions"),
            (counts["revolute_joint_definitions"], expected["revolute_joint_definitions"], "revolute_joint_definitions"),
        ]
        for actual_value, expected_value, field in comparisons:
            if actual_value != expected_value:
                errors.append(
                    "inventory {} differs: actual={!r}, expected={!r}".format(
                        field, actual_value, expected_value
                    )
                )

        occurrence_index = {
            occurrence["full_path"]: occurrence
            for occurrence in inventory["all_occurrences"]
        }
        for entry in exports:
            occurrence = occurrence_index.get(entry["occurrence_full_path"])
            if occurrence is None:
                errors.append("inventory lacks occurrence {}".format(entry["occurrence_full_path"]))
                continue
            bodies_by_name = {body["name"]: body for body in occurrence["direct_bodies"]}
            expected_bodies = _expected_bodies(entry)
            if entry.get("export_scope", "body") == "occurrence" and set(bodies_by_name) != {
                body["body_name"] for body in expected_bodies
            }:
                errors.append(
                    "inventory occurrence {} contains an unexpected direct-body set".format(
                        entry["occurrence_full_path"]
                    )
                )
            for expected_body in expected_bodies:
                body = bodies_by_name.get(expected_body["body_name"])
                if body is None:
                    errors.append(
                        "inventory selection {}/{} did not resolve".format(
                            entry["occurrence_full_path"], expected_body["body_name"]
                        )
                    )
                elif not body["is_solid"] or body["material"] != expected_body["expected_material"]:
                    errors.append(
                        "inventory body {}/{} does not satisfy solid/material constraints".format(
                            entry["occurrence_full_path"], expected_body["body_name"]
                        )
                    )

    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--spec", type=pathlib.Path, default=DEFAULT_SPEC)
    parser.add_argument("--model", type=pathlib.Path, default=DEFAULT_MODEL)
    parser.add_argument("--inventory", type=pathlib.Path)
    args = parser.parse_args()

    errors = _validate(
        _load_json(args.spec),
        _load_json(args.model),
        _load_json(args.inventory) if args.inventory else None,
    )
    if errors:
        for error in errors:
            print("ERROR: {}".format(error), file=sys.stderr)
        return 1
    print("Visual export specification valid: 59 mesh files covering 77 bodies, no visual proxies.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
