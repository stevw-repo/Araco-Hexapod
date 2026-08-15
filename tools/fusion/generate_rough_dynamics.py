#!/usr/bin/env python3
# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

"""Generate a non-destructive, rough dynamics snapshot from a Fusion export."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from collections import Counter
from pathlib import Path
from typing import Any


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _scale_numeric_mapping(value: Any, factor: float) -> Any:
    if isinstance(value, dict):
        return {
            key: (_scale_numeric_mapping(item, factor) if key != "available" else item)
            for key, item in value.items()
        }
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value * factor
    return value


def _component_mass(config: dict[str, Any], name: str, quantity: int) -> float:
    component = config["component_masses"][name]
    return float(component["unit_mass_kg"]) * quantity


def _petg_mass_from_binary_mixture(
    raw_mass: float,
    raw_volume: float,
    petg_density: float,
    steel_density: float,
) -> tuple[float, float]:
    steel_volume = (raw_mass - petg_density * raw_volume) / (
        steel_density - petg_density
    )
    tolerance = max(raw_volume, 1.0) * 1e-12
    if steel_volume < -tolerance or steel_volume > raw_volume + tolerance:
        raise ValueError(
            "Occurrence cannot be decomposed as a PETG/Steel binary mixture: "
            f"mass={raw_mass}, volume={raw_volume}, steel_volume={steel_volume}"
        )
    steel_volume = min(max(steel_volume, 0.0), raw_volume)
    petg_mass = petg_density * (raw_volume - steel_volume)
    steel_mass = steel_density * steel_volume
    return petg_mass, steel_mass


def _matching_rule(
    occurrence_name: str, rules: list[dict[str, Any]]
) -> dict[str, Any]:
    matches = [
        rule for rule in rules if re.search(rule["name_regex"], occurrence_name)
    ]
    if len(matches) != 1:
        raise ValueError(
            f"Expected exactly one rule for {occurrence_name!r}; found "
            f"{[rule['id'] for rule in matches]}"
        )
    return matches[0]


def _estimated_mass(
    occurrence: dict[str, Any],
    rule: dict[str, Any],
    config: dict[str, Any],
) -> tuple[float, dict[str, Any]]:
    physical = occurrence["physical_properties"]
    raw_mass = float(physical["mass_kg"])
    raw_volume = float(physical["volume_m3"])
    method = rule["method"]

    details: dict[str, Any] = {"method": method}
    if method == "keep_raw":
        return raw_mass, details
    if method == "fixed_mass":
        fixed_mass = float(rule["fixed_mass_kg"])
        components = rule.get("components", [])
        quantities = rule.get("component_quantities", [])
        if components:
            if len(components) != len(quantities):
                raise ValueError(f"Component/quantity mismatch in rule {rule['id']!r}")
            component_total = sum(
                _component_mass(config, name, int(quantity))
                for name, quantity in zip(components, quantities, strict=True)
            )
            if not math.isclose(fixed_mass, component_total, abs_tol=1e-12):
                raise ValueError(
                    f"Fixed mass/component mismatch in rule {rule['id']!r}: "
                    f"{fixed_mass} != {component_total}"
                )
            details["replacement_component_mass_kg"] = component_total
        return fixed_mass, details
    if method != "petg_plus_components":
        raise ValueError(f"Unsupported method {method!r}")

    densities = config["material_densities_kg_m3"]
    petg_mass, steel_mass = _petg_mass_from_binary_mixture(
        raw_mass,
        raw_volume,
        float(densities["PETG"]),
        float(densities["Steel"]),
    )
    components = rule.get("components", [])
    quantities = rule.get("component_quantities", [])
    if len(components) != len(quantities):
        raise ValueError(f"Component/quantity mismatch in rule {rule['id']!r}")
    replacement_mass = sum(
        _component_mass(config, name, int(quantity))
        for name, quantity in zip(components, quantities, strict=True)
    )
    details.update(
        {
            "decomposed_petg_mass_kg": petg_mass,
            "rejected_steel_derived_mass_kg": steel_mass,
            "replacement_component_mass_kg": replacement_mass,
        }
    )
    return petg_mass + replacement_mass, details


def generate(
    source_path: Path, config_path: Path, output_path: Path
) -> dict[str, Any]:
    source_hash = _sha256(source_path)
    source = json.loads(source_path.read_text(encoding="utf-8"))
    config = json.loads(config_path.read_text(encoding="utf-8"))

    expected_hash = config["source_export"]["sha256"]
    if source_hash != expected_hash:
        raise ValueError(
            f"Source SHA-256 mismatch: expected {expected_hash}, got {source_hash}"
        )

    results: list[dict[str, Any]] = []
    component_quantities_used: Counter[str] = Counter()
    raw_total = 0.0
    estimated_direct_total = 0.0
    for occurrence in source["direct_root_occurrences"]:
        name = occurrence["full_path"]
        rule = _matching_rule(name, config["occurrence_rules"])
        physical = occurrence["physical_properties"]
        if not physical.get("available"):
            raise ValueError(f"Missing physical properties for {name!r}")
        raw_mass = float(physical["mass_kg"])
        estimated_mass, details = _estimated_mass(occurrence, rule, config)
        if not math.isfinite(estimated_mass) or estimated_mass <= 0.0:
            raise ValueError(f"Invalid estimated mass for {name!r}: {estimated_mass}")
        scale = estimated_mass / raw_mass
        for component, quantity in zip(
            rule.get("components", []),
            rule.get("component_quantities", []),
            strict=True,
        ):
            component_quantities_used[component] += int(quantity)
        raw_total += raw_mass
        estimated_direct_total += estimated_mass

        results.append(
            {
                "full_path": name,
                "component_name": occurrence["component_name"],
                "rule_id": rule["id"],
                "raw_mass_kg": raw_mass,
                "raw_volume_m3": float(physical["volume_m3"]),
                "estimated_mass_kg": estimated_mass,
                "estimated_average_density_kg_m3": (
                    estimated_mass / float(physical["volume_m3"])
                ),
                "mass_scale_factor": scale,
                "method_details": details,
                "center_of_mass_root_fusion_m": physical[
                    "center_of_mass_root_fusion_m"
                ],
                "rough_scaled_fusion_world_xyz_moments_kg_m2": (
                    _scale_numeric_mapping(
                        physical["fusion_world_xyz_moments_kg_m2"], scale
                    )
                ),
                "rough_scaled_principal_moments_kg_m2": _scale_numeric_mapping(
                    physical["principal_moments_kg_m2"], scale
                ),
                "principal_axes_root_fusion": physical[
                    "principal_axes_root_fusion"
                ],
            }
        )

    proxy_total = sum(
        float(proxy["mass_kg"])
        for proxy in config["missing_base_proxies"]
        if proxy.get("include_in_total_mass")
    )
    for proxy in config["missing_base_proxies"]:
        component_quantities_used[proxy["component"]] += 1
    expected_quantities = {
        name: int(component["quantity"])
        for name, component in config["component_masses"].items()
    }
    if dict(component_quantities_used) != expected_quantities:
        raise ValueError(
            "Component quantity mismatch: "
            f"used={dict(component_quantities_used)}, expected={expected_quantities}"
        )
    output = {
        "schema": "araco.rough_dynamics_snapshot.v1",
        "estimate_id": config["estimate_id"],
        "effective_date": config["effective_date"],
        "status": config["status"],
        "source_export": {
            "path_used": str(source_path),
            "sha256": source_hash,
            "fusion_root_mass_kg": source["root_assembly_physical_properties"][
                "mass_kg"
            ],
        },
        "override_manifest": str(config_path),
        "limitations": config["method_notes"],
        "direct_root_occurrences": results,
        "missing_base_proxies": config["missing_base_proxies"],
        "component_quantities_used": dict(component_quantities_used),
        "totals": {
            "raw_direct_occurrence_mass_kg": raw_total,
            "estimated_direct_occurrence_mass_kg": estimated_direct_total,
            "missing_base_proxy_mass_kg": proxy_total,
            "estimated_robot_mass_kg": estimated_direct_total + proxy_total,
        },
        "aggregate_center_of_mass_and_inertia": {
            "available": False,
            "reason": (
                "Missing base proxy poses and per-body properties prevent an honest "
                "aggregate correction. Resolve them when producing canonical link inertias."
            ),
        },
    }
    output_path.write_text(
        json.dumps(output, indent=2, sort_keys=False) + "\n", encoding="utf-8"
    )
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="immutable Fusion exporter JSON")
    parser.add_argument("config", type=Path, help="versioned mass estimate manifest")
    parser.add_argument("output", type=Path, help="derived output JSON")
    args = parser.parse_args()
    result = generate(args.source, args.config, args.output)
    print(json.dumps(result["totals"], indent=2))


if __name__ == "__main__":
    main()
