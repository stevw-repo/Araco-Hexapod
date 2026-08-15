# Araco Robot Description Manifest

Status: **PROPOSED — not approved for implementation or hardware use**  
Inventory date: 2026-08-15  
Semantic confirmations: 2026-08-15  
Target convention: ROS REP-103, ROS 2 Jazzy, Gazebo Harmonic

## Purpose and gate

This document reconciles the current Fusion export with the legacy URDF, inverse
kinematics, and servo driver. It proposes the canonical ROS link and joint
contract. It does not authorize mesh conversion, URDF/Xacro generation,
controller configuration, or physical actuation.

Implementation may begin only after the naming, base frame, kinematic tree, and
open items at the end of this document are reviewed and accepted.

## User-confirmed semantic facts

The following facts were confirmed by the user on 2026-08-15:

- Robot forward is toward the gimbal and cameras.
- `L1`/`R1` are the front legs, `L2`/`R2` are the middle legs, and `L3`/`R3`
  are the rear legs.
- Raspberry Pi Camera Module 3 is physically installed and is included in the
  operational robot description.
- Gemini 335 rotates with the yaw gimbal.
- The descriptive canonical link and joint names proposed here are accepted.

## Source snapshot

| Source | Role | SHA-256 |
|---|---|---|
| `araco - assembly.f3z` | Fusion CAD authority; root design version 25 | `51589fd704eb06b0ef7c4cefef5509b2bfebb29367338e4f5dabbc158debde6d` |
| `araco - assembly.step` | Read-only AP214 product, geometry, and placement inventory | `cfe5774fb9f66f6c2adae8afc182e68a86a6a1b4fd5318eb7af5d6c6c76150d9` |
| Legacy `Araco.urdf` | Prior 26-link/25-joint topology, transforms, and inertial estimates | Not a new-model authority |
| Legacy `algo.py` | Prior link lengths, leg ordering, and commanded joint order | Behavioral reference only |
| Legacy `servodriver.py` | Prior servo IDs, signs, offsets, and PWM calibration | Unverified hardware reference |

The F3Z is a valid archive containing one root assembly and 13 referenced Fusion
designs. Its root is `7d6edd50-b012-46ae-8382-1c0dc6719a38.f3d`. The referenced
designs cover the frame, covers, battery holder, four leg components, gimbal,
gimbal mount, Raspberry Pi 5, Gemini 335, and Raspberry Pi camera.

## STEP inventory findings

- Schema: STEP AP214 (`AUTOMOTIVE_DESIGN`).
- Length unit: millimetre. ROS values must be converted to metres.
- One root product: `araco - assembly`, version 25.
- 236 products and 255 mapped assembly occurrences.
- 32 direct occurrences under the robot root.
- The root contains 24 leg-component occurrences: four stages × three right
  occurrences plus four mirrored stages × three left occurrences.
- The remaining root occurrences are the body/frame parts, Raspberry Pi,
  battery holders, gimbal mount, and moving gimbal.
- The moving gimbal contains two child designs: Gemini 335 and Raspberry Pi
  Camera Module 3.
- The Gemini model is highly detailed down to housings, PCB assemblies,
  connectors, pins, and electronic parts. Those internal products must be
  collapsed into one visual/inertial sensor assembly, not emitted as ROS links.
- No STEP kinematic-pair, mechanism, mass, inertia, density, or material
  entities were found. STEP therefore cannot define the robot joints or trusted
  dynamics by itself.
- The STEP assembly is a placement graph, not an articulated leg tree: all 24
  leg components are direct children of the assembly root.

## Rigid-body classification

| CAD content | Occurrences | Proposed ROS ownership |
|---|---:|---|
| Frame, top cover, bottom cover, two battery holders, Raspberry Pi 5, gimbal mount | 7 | `base_link` rigid body; separate fixed semantic frames only where useful |
| Right coxa, femur, tibia, end | 3 each | Right front/middle/rear leg links |
| Mirrored coxa, femur, tibia, end | 3 each | Left front/middle/rear leg links |
| Gimbal | 1 | `gimbal_yaw_link` |
| Gemini 335 | 1 under gimbal | Fixed visual/sensor frames under `gimbal_yaw_link` |
| Raspberry Pi Camera Module 3 | 1 under gimbal | Fixed visual/sensor frames under `gimbal_yaw_link`; inclusion confirmed |
| Gemini internal products | 223 nested occurrences, including intermediate assemblies | Collapse; never expose as independent robot links |

The primary articulated model is therefore 26 rigid bodies: `base_link`, 24 leg
links, and `gimbal_yaw_link`. Fixed sensor and optical frames may increase the
URDF link count but do not add actuated degrees of freedom.

## Proposed base-frame conversion

The CAD and legacy data indicate that Fusion/STEP uses `+X` toward the robot's
right side, `+Y` toward the front, and `+Z` upward. The proposed conversion to
REP-103 is:

```text
x_ros =  y_step
y_ros = -x_step
z_ros =  z_step
```

All six STEP coxa occurrences share a CAD X offset of approximately
`-0.313602 mm`. Removing that common offset makes their XY positions agree with
the legacy mount pattern to the available precision.

For continuity with the legacy coxa joint height, a provisional `base_link`
datum in STEP coordinates is:

```text
[-0.313602, 0.0, 33.322807] mm
```

This maps the STEP coxa occurrence Z coordinate `6.572807 mm` to the legacy
coxa-joint Z coordinate `-26.75 mm`. This datum is an inference, not a Fusion
joint definition, and must be visually checked against the intended body frame.

### Proposed coxa mount origins

These ROS-frame positions match both the normalized STEP placements and the
legacy URDF. They remain provisional until the REP-103 front direction and body
datum are confirmed.

| Leg | Legacy code | Proposed coxa origin in `base_link` (m) | STEP occurrence |
|---|---|---|---|
| Left front | `L1` | `[0.081819805, 0.066819805, -0.02675]` | mirrored coxa `:2` (`#1043`) |
| Left middle | `L2` | `[0.0, 0.09, -0.02675]` | mirrored coxa `:1` (`#1042`) |
| Left rear | `L3` | `[-0.081819805, 0.066819805, -0.02675]` | mirrored coxa `:3` (`#1044`) |
| Right front | `R1` | `[0.081819805, -0.066819805, -0.02675]` | right coxa `:2` (`#1031`) |
| Right middle | `R2` | `[0.0, -0.09, -0.02675]` | right coxa `:1` (`#1030`) |
| Right rear | `R3` | `[-0.081819805, -0.066819805, -0.02675]` | right coxa `:3` (`#1032`) |

The current CAD gimbal component origin does not agree with the legacy gimbal
yaw origin after applying the same Z datum: it gives approximately `56.6 mm`
while the legacy yaw origin is `72.35 mm`. A component origin is not necessarily
a joint pivot. The gimbal yaw origin must therefore come from the Fusion joint
or a direct measurement, not from the STEP component origin.

## Proposed canonical tree

```text
base_link
├── left_front_coxa_link
│   └── left_front_femur_link
│       └── left_front_tibia_link
│           └── left_front_foot_link
├── left_middle_coxa_link
│   └── left_middle_femur_link
│       └── left_middle_tibia_link
│           └── left_middle_foot_link
├── left_rear_coxa_link
│   └── left_rear_femur_link
│       └── left_rear_tibia_link
│           └── left_rear_foot_link
├── right_front_coxa_link
│   └── right_front_femur_link
│       └── right_front_tibia_link
│           └── right_front_foot_link
├── right_middle_coxa_link
│   └── right_middle_femur_link
│       └── right_middle_tibia_link
│           └── right_middle_foot_link
├── right_rear_coxa_link
│   └── right_rear_femur_link
│       └── right_rear_tibia_link
│           └── right_rear_foot_link
└── gimbal_yaw_link
    ├── camera_link
    │   ├── camera_color_optical_frame
    │   └── camera_depth_optical_frame
    └── rpi_camera_link
```

Proposed canonical axes use a consistent leg-local convention:

- Coxa yaw: local `+Z`.
- Femur, tibia, and foot pitch: one common leg-plane axis, proposed local `+Y`.
- Gimbal yaw: local `+Z`.
- Physical servo reversal belongs in calibration/hardware configuration rather
  than inconsistent URDF axes where possible.

The precise leg-mount rotations and the relationship between canonical positive
motion and legacy positive motion remain review items.

## Proposed 25-joint manifest

All entries are proposed finite `revolute` joints. Lower/upper position limits,
velocity limits, and effort limits are intentionally unset until validated.

In the `Legacy map` column, the expression is the input passed to
`numpy.interp`; the two PWM values correspond to interpolation input
`-π/2 → +π/2`. They are historical calibration evidence, not safe limits.

| Canonical joint and parent → child | Legacy | CAD occurrence | Servo | Legacy map | Evidence status |
|---|---|---|---|---|---|
| `left_front_coxa_joint`: `base_link` → `left_front_coxa_link` | `L1C` | mirrored coxa `:2` (`#1043`) | ID 16, DS3235 | `-q`, `2142 → 893` | Occurrence/ID verified from files; sign and calibration unverified physically |
| `left_front_femur_joint`: `left_front_coxa_link` → `left_front_femur_link` | `L1F` | mirrored femur `:2` (`#1046`) | ID 17, DS5160 | `q`, `2172 → 818` | Same |
| `left_front_tibia_joint`: `left_front_femur_link` → `left_front_tibia_link` | `L1T` | mirrored tibia `:2` (`#1049`) | ID 18, DS3235 | `-q - π/2`, `2125 → 863` | Same |
| `left_front_foot_joint`: `left_front_tibia_link` → `left_front_foot_link` | `L1E` | mirrored end `:2` (`#1052`) | ID 19, DS3235 | `q`, `2131 → 865` | Same |
| `left_middle_coxa_joint`: `base_link` → `left_middle_coxa_link` | `L2C` | mirrored coxa `:1` (`#1042`) | ID 22, DS3235 | `-q`, `2132 → 879` | Same |
| `left_middle_femur_joint`: `left_middle_coxa_link` → `left_middle_femur_link` | `L2F` | mirrored femur `:1` (`#1045`) | ID 23, DS5160 | `q`, `2164 → 815` | Same |
| `left_middle_tibia_joint`: `left_middle_femur_link` → `left_middle_tibia_link` | `L2T` | mirrored tibia `:1` (`#1048`) | ID 24, DS3235 | `-q - π/2`, `2109 → 856` | Same |
| `left_middle_foot_joint`: `left_middle_tibia_link` → `left_middle_foot_link` | `L2E` | mirrored end `:1` (`#1051`) | ID 25, DS3235 | `q`, `2127 → 846` | Same |
| `left_rear_coxa_joint`: `base_link` → `left_rear_coxa_link` | `L3C` | mirrored coxa `:3` (`#1044`) | ID 27, DS3235 | `-q`, `2138 → 885` | Same |
| `left_rear_femur_joint`: `left_rear_coxa_link` → `left_rear_femur_link` | `L3F` | mirrored femur `:3` (`#1047`) | ID 28, DS5160 | `q`, `2167 → 815` | Same |
| `left_rear_tibia_joint`: `left_rear_femur_link` → `left_rear_tibia_link` | `L3T` | mirrored tibia `:3` (`#1050`) | ID 29, DS3235 | `-q - π/2`, `2108 → 860` | Same |
| `left_rear_foot_joint`: `left_rear_tibia_link` → `left_rear_foot_link` | `L3E` | mirrored end `:3` (`#1053`) | ID 30, DS3235 | `q`, `2139 → 880` | Same |
| `right_front_coxa_joint`: `base_link` → `right_front_coxa_link` | `R1C` | right coxa `:2` (`#1031`) | ID 10, DS3235 | `-q`, `2132 → 867` | Same |
| `right_front_femur_joint`: `right_front_coxa_link` → `right_front_femur_link` | `R1F` | right femur `:2` (`#1034`) | ID 11, DS5160 | `-q`, `2168 → 797` | Same |
| `right_front_tibia_joint`: `right_front_femur_link` → `right_front_tibia_link` | `R1T` | right tibia `:2` (`#1037`) | ID 12, DS3235 | `q + π/2`, `2127 → 871` | Same |
| `right_front_foot_joint`: `right_front_tibia_link` → `right_front_foot_link` | `R1E` | right end `:2` (`#1040`) | ID 13, DS3235 | `-q`, `2134 → 881` | Same |
| `right_middle_coxa_joint`: `base_link` → `right_middle_coxa_link` | `R2C` | right coxa `:1` (`#1030`) | ID 5, DS3235 | `-q`, `2113 → 862` | Same |
| `right_middle_femur_joint`: `right_middle_coxa_link` → `right_middle_femur_link` | `R2F` | right femur `:1` (`#1033`) | ID 6, DS5160 | `-q`, `2172 → 830` | Same |
| `right_middle_tibia_joint`: `right_middle_femur_link` → `right_middle_tibia_link` | `R2T` | right tibia `:1` (`#1036`) | ID 7, DS3235 | `q + π/2`, `2161 → 906` | Same |
| `right_middle_foot_joint`: `right_middle_tibia_link` → `right_middle_foot_link` | `R2E` | right end `:1` (`#1039`) | ID 8, DS3235 | `-q`, `2125 → 877` | Same |
| `right_rear_coxa_joint`: `base_link` → `right_rear_coxa_link` | `R3C` | right coxa `:3` (`#1032`) | ID 1, DS3235 | `-q`, `2117 → 854` | Same |
| `right_rear_femur_joint`: `right_rear_coxa_link` → `right_rear_femur_link` | `R3F` | right femur `:3` (`#1035`) | ID 2, DS5160 | `-q`, `2174 → 794` | Same |
| `right_rear_tibia_joint`: `right_rear_femur_link` → `right_rear_tibia_link` | `R3T` | right tibia `:3` (`#1038`) | ID 3, DS3235 | `q + π/2`, `2154 → 872` | Same |
| `right_rear_foot_joint`: `right_rear_tibia_link` → `right_rear_foot_link` | `R3E` | right end `:3` (`#1041`) | ID 4, DS3235 | `-q`, `2103 → 847` | Same |
| `gimbal_yaw_joint`: `base_link` → `gimbal_yaw_link` | `Gimbal_Yaw` | gimbal `:1` (`#1029`) | ID 31, DS3235 | `-q`, `2121 → 867` | Occurrence/ID verified; joint pivot, sign, and calibration unverified physically |

STEP entity numbers are identifiers for the hashed STEP snapshot only. Future
exports may renumber them; occurrence paths and product names are the durable
mapping keys.

## Legacy geometry and dynamics evidence

- Legacy IK link lengths: coxa `43 mm`, femur `120 mm`, tibia `120 mm`, terminal
  segment `50 mm`.
- Legacy joint publication order is `L1`, `L2`, `L3`, `R1`, `R2`, `R3`, with
  `C/F/T/E` per leg, followed by gimbal yaw.
- The legacy URDF has 26 links and 25 joints, which agrees with the proposed
  primary articulated topology.
- Every legacy joint is `continuous`; there are no finite position, velocity,
  or effort limits. This must not be carried forward.
- The legacy visual mesh is reused as collision geometry for every link. This is
  unsuitable as the final Gazebo collision model.
- The sum of legacy inertial masses is approximately `7.824629 kg`, but it does
  not represent the current CAD assembly with sufficient provenance.
- Five legacy foot/end links have mass `0.020435 kg`; `L1E1` alone is
  `0.141960 kg`. This asymmetry is suspicious and must not be copied without a
  physical explanation.
- The current STEP contains no mass or material records, so updated inertias must
  come from validated Fusion physical properties or measured masses.

## Fields intentionally unresolved

The following facts cannot be safely inferred from this STEP placement graph:

1. Visual confirmation that Fusion `+Y` points toward the gimbal/cameras and
   therefore maps to ROS `+X`; the physical forward direction itself is
   confirmed.
2. Exact joint pivot origins and axes from the current Fusion assembly,
   especially gimbal yaw.
3. Definition of zero angle and positive motion for every joint in the canonical
   local frames.
4. Safe mechanical lower and upper joint limits.
5. Maximum credible velocity and effort limits for simulation and control.
6. Validation of every servo ID, model, direction, and PWM calibration against
   the physical robot.
7. Intended neutral/home/standing poses and startup pose transition.
8. Current per-rigid-body mass, center of mass, and inertia tensor.
9. Exact Raspberry Pi Camera Module 3 frame transforms.
10. Exact Gemini 335 optical-frame transforms and whether the yaw joint remains
    locked at gimbal yaw zero for the first SLAM milestone.
11. Foot collision/contact shape and self-collision exclusions.

## Evidence needed from Fusion before mesh generation

Open the hashed F3Z snapshot in Fusion 360 and obtain, preferably through one
repeatable export rather than manual transcription:

- Full occurrence path for each of the 32 root occurrences.
- The 25 intended revolute joint definitions, including parent/child occurrence,
  joint origin, axis, and rest angle.
- Any configured motion limits.
- Physical properties for each proposed rigid group after confirming materials:
  mass, center of mass, and inertia tensor with coordinate frame and units.
- Named reference points or joint origins for camera and gimbal frames.

If Fusion joints are missing or stale, the same data must be established by
explicit design dimensions and physical measurements before the model is called
hardware-accurate.

## Acceptance checklist

Before creating `araco_description`, confirm or correct:

- [x] Descriptive joint/link names in this manifest.
- [x] Mapping `L1/L2/L3` to left front/middle/rear and `R1/R2/R3` to right
  front/middle/rear.
- [ ] REP-103 base-frame conversion and `base_link` datum. Physical forward is
  confirmed toward the gimbal/cameras; the CAD-axis mapping and datum still need
  visual/dimensional verification.
- [ ] Four-link chain order `coxa → femur → tibia → foot` for all legs.
- [x] Inclusion of the Raspberry Pi camera.
- [x] Gimbal and Gemini frame ownership.
- [ ] Joint origins, axes, zeros, and finite safe limits.
- [ ] Servo IDs, models, directions, and calibration evidence.
- [ ] Mass/inertia evidence and resolution of the `L1E1` mass outlier.

After acceptance, the next implementation stage is a canonical Xacro package
with one six-instance leg macro, separate visual and simplified collision
geometry, simulator-neutral joints/frames, and isolated Gazebo/
`gz_ros2_control` overlays.
