#!/usr/bin/env python3
# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT
"""Validate and deduplicate an approved Fusion visual-export bundle."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct


EXPECTED_SCHEMA = {"name": "araco_fusion_visual_export_manifest", "version": 1}
SUPPORTED_BUNDLE_CONTRACTS = {
    "0.3.1": {"export_count": 44, "reviewed_body_count": 62},
    "0.4.0": {"export_count": 59, "reviewed_body_count": 77},
}


def _load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_json(path: Path, value) -> None:
    path.write_text(
        json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _safe_child(root: Path, relative: str) -> Path:
    candidate = (root / relative).resolve()
    try:
        candidate.relative_to(root.resolve())
    except ValueError as exception:
        raise ValueError("bundle path escapes its root: {!r}".format(relative)) from exception
    return candidate


def _binary_stl_triangle_count(path: Path) -> int:
    size = path.stat().st_size
    if size < 84:
        raise ValueError("STL is shorter than its binary header: {}".format(path))
    with path.open("rb") as stream:
        stream.seek(80)
        count = struct.unpack("<I", stream.read(4))[0]
    if size != 84 + count * 50:
        raise ValueError(
            "STL byte length does not match its binary triangle count: {}".format(path)
        )
    return count


def _transform_key(transform) -> tuple[float, ...]:
    values = []
    for field in ("origin_m", "x_axis", "y_axis", "z_axis"):
        values.extend(float(transform[field][axis]) for axis in ("x", "y", "z"))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("occurrence transform contains a non-finite value")
    return tuple(round(value, 12) for value in values)


def _validate_bundle(bundle: Path):
    manifest_path = bundle / "visual_export_manifest.json"
    manifest = _load_json(manifest_path)
    errors = []
    if manifest.get("schema") != EXPECTED_SCHEMA:
        errors.append("unsupported visual-export manifest schema")
    exporter_version = manifest.get("exporter", {}).get("version")
    bundle_contract = SUPPORTED_BUNDLE_CONTRACTS.get(exporter_version)
    if bundle_contract is None:
        errors.append(
            "unsupported exporter version; expected one of {}".format(
                sorted(SUPPORTED_BUNDLE_CONTRACTS)
            )
        )
        bundle_contract = {"export_count": -1, "reviewed_body_count": -1}
    if manifest.get("exporter", {}).get("read_only_design_access") is not True:
        errors.append("exporter did not assert read-only design access")
    if manifest.get("source_design", {}).get("is_modified") is not False:
        errors.append("source Fusion document was modified during export")
    rights = manifest.get("rights_boundary", {})
    if rights.get("license") != "mixed-open-source" or rights.get("fail_closed") is not True:
        errors.append("bundle lacks the reviewed mixed-open-source fail-closed rights boundary")
    if rights.get("license_policy") != "preserve_per_asset":
        errors.append("bundle does not preserve per-asset licenses")
    if rights.get("redistribution_confirmed") is not True:
        errors.append("bundle lacks the user's redistribution confirmation")
    if rights.get("unlisted_geometry_export_allowed") is not False:
        errors.append("bundle permits unlisted geometry")
    expected_export_count = bundle_contract["export_count"]
    expected_reviewed_body_count = bundle_contract["reviewed_body_count"]
    if manifest.get("export_count") != expected_export_count:
        errors.append(
            "bundle must contain exactly {} mesh exports for exporter {}".format(
                expected_export_count, exporter_version
            )
        )
    if manifest.get("reviewed_body_count") != expected_reviewed_body_count:
        errors.append(
            "bundle must cover exactly {} reviewed source bodies for exporter {}".format(
                expected_reviewed_body_count, exporter_version
            )
        )
    exports = manifest.get("exports", [])
    if len(exports) != expected_export_count:
        errors.append(
            "manifest export array must contain exactly {} entries".format(
                expected_export_count
            )
        )
    if manifest.get("retained_proxy_links") != []:
        errors.append("exact-mesh bundle must retain no visual proxies")

    referenced = manifest.get("visual_export_spec", {})
    spec_path = _safe_child(bundle, referenced.get("path", ""))
    inventory_reference = manifest.get("robot_description_inventory", {})
    inventory_path = _safe_child(bundle, inventory_reference.get("path", ""))
    for path, expected_hash, label in (
        (spec_path, referenced.get("sha256"), "visual export spec"),
        (inventory_path, inventory_reference.get("sha256"), "robot-description inventory"),
    ):
        if not path.is_file():
            errors.append("missing {}".format(label))
        elif _sha256(path) != expected_hash:
            errors.append("{} hash mismatch".format(label))

    asset_ids = set()
    selections = set()
    validated = []
    for entry in exports:
        asset_id = entry.get("asset_id")
        if asset_id in asset_ids:
            errors.append("duplicate asset_id: {}".format(asset_id))
        asset_ids.add(asset_id)
        source_bodies = entry.get("source_bodies")
        if not isinstance(source_bodies, list) or not source_bodies:
            errors.append("asset has no reviewed source-body records: {}".format(asset_id))
            source_bodies = []
        for body in source_bodies:
            selection = (
                entry.get("source_occurrence_full_path"),
                body.get("body_name"),
            )
            if selection in selections:
                errors.append("duplicate occurrence/body selection: {!r}".format(selection))
            selections.add(selection)
            if body.get("material") not in {"PETG", "Steel"}:
                errors.append("unexpected source material escaped the allowlist: {}".format(asset_id))
        if not entry.get("license") or entry.get("redistribution") != "allowed_by_rights_holder":
            errors.append("asset lacks reviewed redistribution metadata: {}".format(asset_id))
        try:
            _transform_key(entry["source_occurrence_transform_root_fusion"])
            mesh_path = _safe_child(bundle, entry["path"])
        except (KeyError, TypeError, ValueError) as exception:
            errors.append("invalid asset {}: {}".format(asset_id, exception))
            continue
        if not mesh_path.is_file():
            errors.append("missing mesh: {}".format(entry.get("path")))
            continue
        actual_size = mesh_path.stat().st_size
        actual_hash = _sha256(mesh_path)
        if actual_size != entry.get("size_bytes"):
            errors.append("size mismatch for {}".format(asset_id))
        if actual_hash != entry.get("sha256"):
            errors.append("hash mismatch for {}".format(asset_id))
        try:
            triangle_count = _binary_stl_triangle_count(mesh_path)
        except ValueError as exception:
            errors.append(str(exception))
            continue
        validated.append((entry, mesh_path, triangle_count))

    spec = _load_json(spec_path) if spec_path.is_file() else {}
    def spec_signature(entry):
        if entry.get("export_scope", "body") == "body":
            bodies = [(entry.get("body_name"), entry.get("expected_material"))]
        else:
            bodies = [
                (body.get("body_name"), body.get("expected_material"))
                for body in entry.get("expected_bodies", [])
            ]
        return (
            entry.get("asset_id"), entry.get("canonical_link"),
            entry.get("occurrence_full_path"), entry.get("export_scope", "body"),
            tuple(sorted(bodies)), entry.get("visual_role", "primary"),
            entry.get("creator", "Araco Hexapod contributors"),
            entry.get("license", "MIT"),
        )

    def manifest_signature(entry):
        bodies = [
            (body.get("body_name"), body.get("material"))
            for body in entry.get("source_bodies", [])
        ]
        return (
            entry.get("asset_id"), entry.get("canonical_link"),
            entry.get("source_occurrence_full_path"), entry.get("export_scope", "body"),
            tuple(sorted(bodies)), entry.get("visual_role", "primary"),
            entry.get("creator"), entry.get("license"),
        )

    expected_selections = {spec_signature(entry) for entry in spec.get("exports", [])}
    actual_selections = {manifest_signature(entry) for entry in exports}
    if actual_selections != expected_selections:
        errors.append("manifest exports differ from the reviewed allowlist copy")

    if errors:
        raise ValueError("Fusion visual bundle failed validation:\n- " + "\n- ".join(errors))
    return manifest_path, manifest, spec_path, inventory_path, validated


def _coordinate_evidence(validated):
    by_hash = {}
    for entry, _, _ in validated:
        by_hash.setdefault(entry["sha256"], []).append(entry)
    repeated = []
    for sha256, entries in sorted(by_hash.items()):
        transforms = {
            _transform_key(entry["source_occurrence_transform_root_fusion"])
            for entry in entries
        }
        if len(entries) > 1 and len(transforms) > 1:
            repeated.append(
                {
                    "sha256": sha256,
                    "asset_ids": [entry["asset_id"] for entry in entries],
                    "distinct_occurrence_transform_count": len(transforms),
                }
            )
    if not repeated:
        raise ValueError(
            "cannot establish component-local coordinates from repeated meshes and distinct transforms"
        )
    return repeated


def import_bundle(bundle: Path, output: Path) -> None:
    manifest_path, manifest, spec_path, inventory_path, validated = _validate_bundle(bundle)
    if output.exists():
        raise ValueError("refusing to overwrite existing source import: {}".format(output))
    output.mkdir(parents=True)
    raw_output = output / "raw_by_sha256"
    raw_output.mkdir()

    imported_hashes = set()
    source_exports = []
    for entry, mesh_path, triangle_count in validated:
        sha256 = entry["sha256"]
        relative_raw = "raw_by_sha256/{}.stl".format(sha256)
        if sha256 not in imported_hashes:
            shutil.copyfile(mesh_path, output / relative_raw)
            imported_hashes.add(sha256)
        source_exports.append(
            {
                "asset_id": entry["asset_id"],
                "canonical_link": entry["canonical_link"],
                "raw_mesh_path": relative_raw,
                "raw_mesh_sha256": sha256,
                "raw_mesh_size_bytes": entry["size_bytes"],
                "raw_triangle_count": triangle_count,
                "source_occurrence_full_path": entry["source_occurrence_full_path"],
                "source_occurrence_transform_root_fusion": entry[
                    "source_occurrence_transform_root_fusion"
                ],
                "export_scope": entry.get("export_scope", "body"),
                "source_bodies": entry["source_bodies"],
                "visual_role": entry.get("visual_role", "primary"),
                "creator": entry["creator"],
                "license": entry["license"],
                "redistribution": "allowed",
            }
        )

    source_manifest = {
        "schema": {"name": "araco_fusion_visual_source", "version": 1},
        "source_export": {
            "manifest_sha256": _sha256(manifest_path),
            "spec_sha256": _sha256(spec_path),
            "inventory_sha256": _sha256(inventory_path),
            "exporter": manifest["exporter"],
            "design": manifest["source_design"],
        },
        "rights_boundary": manifest["rights_boundary"],
        "raw_mesh_encoding": {
            "format": "binary_stl",
            "length_unit": "millimeter",
            "coordinate_space": "source_component_local",
        },
        "coordinate_space_correction": {
            "declared_by_export_bundle": manifest["mesh_options"]["coordinate_context"],
            "verified_interpretation": "Fusion source-component-local body coordinates",
            "evidence": _coordinate_evidence(validated),
            "normalization_rule": "Apply the recorded occurrence transform before Fusion-to-ROS and link-local transforms.",
        },
        "unique_raw_mesh_count": len(imported_hashes),
        "export_count": len(source_exports),
        "reviewed_body_count": manifest["reviewed_body_count"],
        "exports": source_exports,
        "retained_proxy_links": manifest["retained_proxy_links"],
        "excluded_geometry": manifest["excluded_geometry"],
    }
    _write_json(output / "source_manifest.json", source_manifest)
    print(
        "Imported {} approved exports as {} unique raw meshes into {}".format(
            len(source_exports), len(imported_hashes), output
        )
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path)
    parser.add_argument("output", type=Path)
    arguments = parser.parse_args()
    import_bundle(arguments.bundle.resolve(), arguments.output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
