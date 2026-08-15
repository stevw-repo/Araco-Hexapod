# Araco Robot Description Exporter

This project-authored add-in source is licensed under
`MIT`; see the repository root `LICENSE`. Autodesk Fusion and its API
are proprietary third-party software and are not included or relicensed here.

This Autodesk Fusion Python script reads the active assembly and exports a
fail-closed visual-source bundle. It does not edit the design, drive joints, or
interact with the physical robot.

## Rights boundary

`visual_export_spec_v1.json` is the reviewed allowlist. Specification revision
`3.0.0` exports 59 exact mesh files covering the 77 reviewed source bodies:

- Six project-owned base fragments.
- The project-owned gimbal body.
- The project-owned coxa, femur, and foot body for each of six legs.
- Seven DS3235 servo bodies attached to the base/frame and gimbal mount.
- Six DS5160 servo bodies attached to the six coxa links.
- All four exact Fusion bodies in each of the six tibia components; together
  these contain the printed tibia structures, twelve DS3235 servos, and their
  exact associated geometry.
- Fifteen exact Gemini 335 exterior bodies: five housing/bracket bodies, six
  pads/fasteners, and four externally visible optical elements.

The user confirmed on 2026-08-16 that all selected mechanical and vendor CAD is
open source and may be used for simulator and presentation visuals. The bundle
preserves MIT for project-authored bodies and uses
`LicenseRef-UserConfirmed-Open-Source-CAD` for vendor bodies pending more
specific upstream attribution metadata. The Gemini's eight-body internal
connector/PCB assembly, Raspberry Pi Camera Module 3, battery, controller, and
all other unlisted electronics remain outside this bounded export. An
`araco - ...` component name by itself is not enough to authorize export;
unlisted bodies always fail closed.

## What the bundle contains

- 59 high-refinement binary STL files in millimetres, exported in each source
  component's local coordinates. The occurrence transform is recorded
  separately so Ubuntu can place and normalize every body deterministically.
  The six tibia files each contain the four reviewed direct bodies from one
  complete tibia occurrence. This occurrence-level packaging avoids a Fusion
  API defect where `Body2` passes solid-body preflight but produces no
  individual STL.
- A visual-export manifest with SHA-256 hashes, exact source occurrence/body,
  material, appearance, root transform, target ROS link, creator, and license.
- A copy of the reviewed allowlist.
- The complete robot-description JSON inventory described below.

The raw STL files are source artifacts, not link-local ROS meshes. Do not copy
them directly into `araco_description`; they must be normalized and verified on
Ubuntu first.

The inventory includes:

- All regular and as-built joints flattened into the root assembly context.
- Joint occurrence pairs, geometry coordinate systems, motion axes, current
  values, rest values, and configured limits.
- Full occurrence paths and root-frame transforms.
- Direct body material and appearance names for every occurrence.
- High-accuracy physical properties for the complete assembly and each direct
  root occurrence: mass, center of mass, principal axes/moments, and Fusion XYZ
  moments of inertia.

The report uses metres, radians, kilograms, and `kg·m²` except where a field is
explicitly marked as a Fusion internal value. Fusion entity tokens are included
only as snapshot identifiers; they are not durable names.

## Run it on Windows

1. Copy the complete `AracoRobotDescriptionExporter` folder to Windows. The
   Python file, manifest, and `visual_export_spec_v1.json` must stay together.
2. Open and save `araco - assembly ros-description v1 v2` in Fusion, then leave
   its root assembly active. The script rejects unsaved changes.
3. Choose **Utilities → Scripts and Add-Ins**.
4. In **Scripts**, use the `+` command to link an existing script and select the
   `AracoRobotDescriptionExporter` folder. If Fusion asks for a file instead,
   select `AracoRobotDescriptionExporter.py`.
5. Select **AracoRobotDescriptionExporter** and choose **Run**.
6. In the folder dialog, choose this existing parent folder on the shared
   `DATA-ST` drive:

   ```text
   DATA-ST\New folder
   ```

7. Wait for the completion dialog. Physical-property calculations for the
   detailed Gemini assembly may take some time.
8. The script creates a new timestamped folder such as
   `araco_-_assembly_ros-description_v1_v2_visual_export_20260816T120000Z`.
   It never overwrites an existing bundle.
9. Confirm that the completion dialog reports **59 exported mesh files**,
   **77 reviewed source bodies**, and **0 retained visual proxies**.
10. Return to Ubuntu and provide the new bundle-folder path to Codex.

Expected for the reviewed Fusion cloud version 2: 24 regular joints, one
as-built joint, 25 revolute definitions, 32 direct root occurrences, and 269
total occurrences. If Fusion reports an error, capture the entire error dialog
and do not alter the assembly to work around it. A failed mesh operation leaves
a timestamped `.incomplete` folder with `EXPORT_FAILED.txt`; preserve that folder
for diagnosis.

## Ubuntu-side allowlist validation

Before copying the script to Windows, or after changing the allowlist, validate
it against the last Fusion inventory and the canonical model:

```bash
python3 tools/fusion/validate_visual_export_spec.py \
  --inventory '/media/stevw-s14/DATA-ST/New folder/<latest_visual_export_bundle>/robot_description_inventory.json'
```

This validation does not prove that Fusion can tessellate the selected bodies;
the actual export must still be run in Fusion.

## Alternative installation location

Fusion automatically discovers scripts placed in this Windows directory:

```text
%APPDATA%\Autodesk\Autodesk Fusion\API\Scripts\AracoRobotDescriptionExporter
```

The folder, Python file, and manifest share the same base name as required by
Fusion's script loader.

## Generate the rough simulation dynamics snapshot

The raw Fusion JSON is preserved as evidence. On Ubuntu, generate the separate
`rough_estimate_v0` snapshot with:

```bash
python3 tools/fusion/generate_rough_dynamics.py \
  '/media/stevw-s14/DATA-ST/New folder/araco_-_assembly_ros-description_v1_v2_robot_description_export.json' \
  tools/fusion/rough_mass_estimates_v0.json \
  tools/fusion/araco_rough_dynamics_v0.json
```

The estimate replaces Steel-derived servo and electronics mass with researched
or explicitly rough component values. It retains occurrence centers of mass and
uniformly scales their inertia values. Missing base electronics contribute to
the total mass only because their poses are not yet known. This output is an
initial Gazebo input, not physically validated dynamics.
