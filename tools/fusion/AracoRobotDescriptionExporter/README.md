# Araco Robot Description Exporter

This project-authored add-in source is licensed under
`MIT`; see the repository root `LICENSE`. Autodesk Fusion and its API
are proprietary third-party software and are not included or relicensed here.

This Autodesk Fusion Python script reads the active assembly and writes one JSON
inventory. It does not edit the design, drive joints, export meshes, or interact
with the physical robot.

## What it exports

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

1. Copy the complete `AracoRobotDescriptionExporter` folder to Windows.
2. Open `araco - assembly v25` in Fusion and leave the root assembly active.
3. Choose **Utilities → Scripts and Add-Ins**.
4. In **Scripts**, use the `+` command to link an existing script and select the
   `AracoRobotDescriptionExporter` folder. If Fusion asks for a file instead,
   select `AracoRobotDescriptionExporter.py`.
5. Select **AracoRobotDescriptionExporter** and choose **Run**.
6. In the Save dialog, write the JSON to the shared `DATA-ST` drive. A useful
   destination is:

   ```text
   DATA-ST\New folder\araco_v25_robot_description_export.json
   ```

7. Wait for the completion dialog. Physical-property calculations for the
   detailed Gemini assembly may take some time.
8. Return to Ubuntu and provide the JSON path to Codex.

Expected for Fusion design version 25: 24 regular revolute joints, zero as-built
joints, 32 direct root occurrences, and 269 total occurrences. If Fusion reports
an error, capture the entire error dialog and do not alter the assembly to work
around it.

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
