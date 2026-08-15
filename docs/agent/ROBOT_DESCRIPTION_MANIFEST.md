# Araco Robot Description Manifest

Status: **ACCEPTED AS SIMULATOR-AUTHORING EVIDENCE — not hardware validated**
Inventory date: 2026-08-15  
Semantic confirmations: 2026-08-15  
Target convention: ROS REP-103, ROS 2 Jazzy, Gazebo Harmonic

## Purpose and gate

This document reconciles the current Fusion export with the legacy URDF, inverse
kinematics, and servo driver. It records the accepted simulator-authoring
baseline for the canonical ROS link and joint contract. It does not itself
authorize mesh conversion, URDF/Xacro generation, controller configuration, or
physical actuation.

The accepted topology, naming, base frame, joint origins/axes, nominal pose,
and rough dynamics may be implemented and validated in the simulator after the
separate phase authorization. Unresolved physical zeros, directions, limits,
calibration, and inertial fidelity remain explicit hardware blockers rather
than blockers to Phase 0 scaffolding or provisional Gazebo authoring.

## User-confirmed semantic facts

The following facts were confirmed by the user on 2026-08-15:

- Robot forward is toward the gimbal and cameras.
- `L1`/`R1` are the front legs, `L2`/`R2` are the middle legs, and `L3`/`R3`
  are the rear legs.
- Raspberry Pi Camera Module 3 is physically installed, but its software support
  is deferred and it may be omitted entirely. It is not required for simulator
  locomotion or the operational robot description.
- Gemini 335 rotates with the yaw gimbal.
- The descriptive canonical link and joint names proposed here are accepted.
- The legacy-compatible `base_link` datum at Fusion root coordinates
  `[-0.313602, 0, 8.072807] mm` is accepted.
- The current Fusion pose represents the intended safe standing/home pose, but
  the user explicitly identifies it as approximate rather than physically exact.

## Source snapshot

| Source | Role | SHA-256 |
|---|---|---|
| `araco - assembly.f3z` | Fusion CAD authority; root design version 25 | `51589fd704eb06b0ef7c4cefef5509b2bfebb29367338e4f5dabbc158debde6d` |
| `araco - assembly.step` | Read-only AP214 product, geometry, and placement inventory | `cfe5774fb9f66f6c2adae8afc182e68a86a6a1b4fd5318eb7af5d6c6c76150d9` |
| Initial Fusion API JSON export | Read-only version-25 inventory before gimbal joint | `54c6b29580641dd56b63c2912185a8132895e42ba09bd947c8bb09d41d19d582` |
| `araco - assembly ros-description v1.step` | AP214 export of the Fusion working copy after adding gimbal yaw | `4e1c14424110d750c2290d1556cfbb3f521503b0eff847f33f3814402732ea04` |
| Working-copy Fusion API JSON export | Read-only 25-joint inventory after adding gimbal yaw | `17be93c81e8773dff7ddfba7c48ec3938c40286a567877eb77b91117a16f057e` |
| `araco - assembly ros-description v1.step`, Fusion version 2 re-export | Geometry/placement comparison after partial material correction | `fbfe11ebba5ef3cb914293f267454aa009adec1a7307fbbf334af1188ec39c9a` |
| Fusion version 2 API JSON export | Partial material-correction inventory; latest snapshot | `843974cc090e76d17c691d4e097c602ccd4e966e07e158ba392ae1bef54b7352` |
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
- The working-copy STEP re-export still has 236 products and 255 mapped assembly
  occurrences, and still contains no kinematic-pair or mechanism entities. This
  is expected: the new Fusion as-built joint is evidenced by the API JSON, not
  by AP214 STEP.
- The later Fusion version 2 STEP re-export again has 236 products and 255
  assembly occurrences and contains no kinematic-pair, material-designation, or
  material-property entities. Its changed file hash does not provide new
  dynamics evidence.

## Fusion API inventory findings

The read-only exporter ran against Fusion application `2704.1.53`. The initial
cloud design version 25 reported 24 regular joints and zero as-built joints. The
`ros-description v1` working copy and its latest version 2 snapshot report 24
regular joints plus one revolute as-built joint, with 32 direct root
occurrences and 269 total Fusion occurrences.

- All 24 joints are healthy revolute joints with coincident geometry origins;
  the largest reported separation between the two sides of a joint is about
  `3.1e-15 m`, numerical noise only.
- Every joint connects one leg stage to its expected predecessor. The export
  therefore confirms all six `coxa → femur → tibia → foot` chains.
- The new as-built joint `Revolute 27` connects `araco - gimbal v5 v40:1` to
  `araco - gimbal_mount v1 v25:1`. It maps canonically to
  `gimbal_yaw_joint`; its raw Fusion name is not part of the ROS interface.
- No minimum or maximum motion limit is enabled on any joint. Rest values are
  enabled on 23 of 25 joints, but they are assembly-pose metadata rather than
  validated mechanical limits.
- The first physical-property snapshot reported all 732 B-Rep body occurrences
  as `Steel`, producing an invalid `15.552194 kg` assembly mass.
- The latest version 2 snapshot has no exporter errors and preserves all 25
  joint definitions. All 32 direct-root physical-property records are present;
  their positive masses sum to the root mass and their principal inertia
  triples are positive and satisfy the rigid-body triangle inequalities.
- Material correction remains incomplete: 707 body occurrences still report
  `Steel` and only 25 report `PETG`. The root mass is still an implausible
  `9.804328 kg`, compared with the user's `2–4 kg` physical estimate.
- The largest suspect subtotals are six tibia assemblies at `4.158309 kg`, six
  coxa assemblies at `2.734060 kg`, the base frame at `1.422909 kg`, and the
  gimbal plus mount at `0.773974 kg`. These dynamics remain rejected for URDF.
- The user identifies the remaining Steel bodies in the coxa, tibia, frame,
  gimbal-mount, and gimbal groups as servo geometry. They must not be relabeled
  PETG; their effective density must reproduce the real servo mass. Composite
  electronics likewise need measured or manufacturer-mass-informed effective
  properties rather than a literal homogeneous material guess.

### Gimbal yaw from the Fusion working copy

The as-built joint supplies the previously missing geometry:

```text
Fusion root origin: [-0.000313602045, -0.004999999925, 0.092422806813] m
Fusion root axis:   [0, 0, +1]
```

Using the accepted base datum and Fusion-to-ROS rotation, the
canonical joint is:

```text
gimbal_yaw_joint origin in base_link: [-0.005, 0, 0.08435] m
gimbal_yaw_joint axis:                [0, 0, +1]
```

Its Fusion position limits and rest value are disabled. The CAD geometry is now
complete, but safe limits and the canonical zero remain unresolved.

## Nominal standing pose evidence

The Fusion working-copy joint values reproduce the legacy controller's default
standing solution after applying the evident CAD-to-canonical offsets and signs:

```text
q_coxa  = fusion_rotation - fusion_rest
q_femur = fusion_rotation
q_tibia = -fusion_rotation
q_foot  = fusion_rotation
q_gimbal_yaw = fusion_rotation
```

The legacy solution is generated from a `280 mm` foot radius, `-80 mm` body
height, terminal vector `[0, 0, 50] mm`, and link lengths
`43/120/120/50 mm`. The largest numerical difference between that solution and
the converted Fusion pose is below `4.5e-10 rad`.

| Leg | Coxa (rad / deg) | Femur (rad / deg) | Tibia (rad / deg) | Foot (rad / deg) |
|---|---:|---:|---:|---:|
| Left front `L1` | `0.166907 / 9.563°` | `0.749681 / 42.954°` | `-1.947935 / -111.608°` | `-0.372542 / -21.345°` |
| Left middle `L2` | `0 / 0°` | `0.694188 / 39.774°` | `-1.791009 / -102.617°` | `-0.473975 / -27.157°` |
| Left rear `L3` | `-0.166907 / -9.563°` | `0.749681 / 42.954°` | `-1.947935 / -111.608°` | `-0.372542 / -21.345°` |
| Right front `R1` | `-0.166907 / -9.563°` | `0.749681 / 42.954°` | `-1.947935 / -111.608°` | `-0.372542 / -21.345°` |
| Right middle `R2` | `0 / 0°` | `0.694188 / 39.774°` | `-1.791009 / -102.617°` | `-0.473975 / -27.157°` |
| Right rear `R3` | `0.166907 / 9.563°` | `0.749681 / 42.954°` | `-1.947935 / -111.608°` | `-0.372542 / -21.345°` |

Gimbal yaw is `0 rad`. This vector is accepted as
`nominal_standing_reference_v0` for visualization, kinematic regression, and
simulator initialization. Because the user states that the CAD pose is not
fully accurate and the physical open-loop robot has not been revalidated, it
must not yet be treated as an authorized hardware startup command.

### Fusion joint-name reconciliation

| Stage | Right middle `R2` | Right front `R1` | Right rear `R3` | Left middle `L2` | Left front `L1` | Left rear `L3` |
|---|---|---|---|---|---|---|
| Coxa | `Revolute 2` | `Revolute 3` | `Revolute 4` | `Revolute 5` | `Revolute 6` | `Revolute 7` |
| Femur | `Revolute 8` | `Revolute 9` | `Revolute 10` | `Revolute 11` | `Revolute 12` | `Revolute 13` |
| Tibia | `Revolute 14` | `Revolute 15` | `Revolute 16` | `Revolute 17` | `Revolute 18` | `Revolute 19` |
| Foot | `Revolute 20` | `Revolute 21` | `Revolute 22` | `Revolute 23` | `Revolute 24` | `Revolute 25` |

The raw Fusion names remain snapshot evidence only. The accepted descriptive ROS
names remain the canonical interface.

## Rigid-body classification

| CAD content | Occurrences | Proposed ROS ownership |
|---|---:|---|
| Frame, top cover, bottom cover, two battery holders, Raspberry Pi 5, gimbal mount | 7 | `base_link` rigid body; separate fixed semantic frames only where useful |
| Right coxa, femur, tibia, end | 3 each | Right front/middle/rear leg links |
| Mirrored coxa, femur, tibia, end | 3 each | Left front/middle/rear leg links |
| Gimbal | 1 | `gimbal_yaw_link` |
| Gemini 335 | 1 under gimbal | Fixed visual/sensor frames under `gimbal_yaw_link` |
| Raspberry Pi Camera Module 3 | 1 under gimbal | Optional fixed visual/sensor frames under `gimbal_yaw_link`; omission is allowed |
| Gemini internal products | 223 nested occurrences, including intermediate assemblies | Collapse; never expose as independent robot links |

The primary articulated model is therefore 26 rigid bodies: `base_link`, 24 leg
links, and `gimbal_yaw_link`. Fixed sensor and optical frames may increase the
URDF link count but do not add actuated degrees of freedom.

The Fusion archive also embeds detailed vendor/component CAD for the Gemini,
Raspberry Pi, camera, servos, and electronics whose redistribution terms have
not been established. Their geometry is placement/mass evidence only and must
not be copied automatically into the public ROS package. Phase 1 either records
rights-compatible source/license/attribution metadata for a bundled asset or
uses project-authored simplified proxy geometry. The repository's MIT
selection cannot grant rights in those imported models.

## Accepted base-frame conversion

The top-view screenshot and Fusion joint coordinates confirm that Fusion uses
`+X` toward the robot's right side, `+Y` toward the confirmed physical front,
and `+Z` upward. The accepted axis conversion to REP-103 is:

```text
x_ros =  y_fusion
y_ros = -x_fusion
z_ros =  z_fusion
```

The actual Fusion coxa joint centers confirm the common CAD X offset of
approximately `-0.313602 mm`. Removing it makes their XY positions agree with
the legacy mount pattern to the available precision.

For continuity with the legacy coxa joint height, the accepted `base_link`
datum in Fusion root coordinates is:

```text
[-0.313602, 0.0, 8.072807] mm
```

This maps the actual Fusion coxa joint-center Z coordinate `-18.677193 mm` to
the legacy coxa-joint Z coordinate `-26.75 mm`. It supersedes the earlier
placement-only Z inference. The datum is now accepted and may be changed only
through an explicit robot-description contract revision.

### Accepted coxa mount origins

These ROS-frame positions now match both the exported Fusion joint centers and
the legacy URDF under the accepted `base_link` datum.

| Leg | Legacy code | Proposed coxa origin in `base_link` (m) | Fusion joint | STEP occurrence |
|---|---|---|---|---|
| Left front | `L1` | `[0.081819805, 0.066819805, -0.02675]` | `Revolute 6` | mirrored coxa `:2` (`#1043`) |
| Left middle | `L2` | `[0.0, 0.09, -0.02675]` | `Revolute 5` | mirrored coxa `:1` (`#1042`) |
| Left rear | `L3` | `[-0.081819805, 0.066819805, -0.02675]` | `Revolute 7` | mirrored coxa `:3` (`#1044`) |
| Right front | `R1` | `[0.081819805, -0.066819805, -0.02675]` | `Revolute 3` | right coxa `:2` (`#1031`) |
| Right middle | `R2` | `[0.0, -0.09, -0.02675]` | `Revolute 2` | right coxa `:1` (`#1030`) |
| Right rear | `R3` | `[-0.081819805, -0.066819805, -0.02675]` | `Revolute 4` | right coxa `:3` (`#1032`) |

The current CAD gimbal component origin does not agree with the legacy gimbal
yaw origin after applying the same Z datum: it gives approximately `56.6 mm`
while the legacy yaw origin is `72.35 mm`. A component origin is not necessarily
a joint pivot. The gimbal yaw origin must therefore come from the Fusion joint
or a direct measurement, not from the STEP component origin.

## Accepted canonical tree

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
    └── rpi_camera_link [optional]
```

The accepted simulator-authoring axis convention is:

- Coxa yaw: local `+Z`.
- Femur, tibia, and foot pitch: one common leg-plane axis, local `-Y`.
- Gimbal yaw: local `+Z`.
- Physical servo reversal belongs in calibration/hardware configuration rather
  than inconsistent URDF axes where possible.

Gate 1 simulator measurement corrected the earlier proposed `+Y` pitch axis:
with the accepted legacy/Fusion joint values it placed tibia geometry below the
feet. Local `-Y` reproduces the intended feet-down home pose. This remains a
simulator convention; physical servo direction is still unverified.

The expanded model must derive and Gate 0 must verify the precise leg-mount
rotations against the immutable Fusion joint evidence. The relationship between
canonical positive motion and the legacy physical servo direction remains a
hardware-validation item and is not inferred from simulator success.

## Accepted 25-joint topology

All entries are finite `revolute` joints in the simulator topology. The
simulator uses the explicitly provisional ranges, velocity caps, and effort
caps accepted in `RUNTIME_TIMING_AND_SIMULATION_CONTRACT.md`; those values are
not installed mechanical or actuator limits and are forbidden in a future
physical profile.

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

## Detailed Fusion visual evidence

The approved Fusion version 2 visual bundle was generated on 2026-08-16 with
Fusion `2704.1.53`. It contains 25 whitelisted project-owned PETG exports: six
base fragments, one gimbal asset, and coxa/femur/foot assets for all six legs.
The six tibia links deliberately retain project-authored proxies because the
current source evidence cannot safely separate printed tibia structure from
embedded servo CAD. Vendor sensor, computer, servo, battery, and controller CAD
is not included.

Repeated leg instances have byte-identical STL content but distinct recorded
occurrence transforms. This verifies that Fusion emitted component-local body
coordinates, not root-assembly coordinates. The deterministic normalization
pipeline is therefore:

1. source-component-local binary STL in millimetres to metres;
2. recorded Fusion occurrence transform to Fusion root;
3. accepted Fusion-to-ROS rotation and base datum to `base_link`;
4. inverse canonical nominal link transform to ROS link-local coordinates.

The imported source cache contains 12 unique immutable raw meshes. Normalized
output contains 20 link-local detailed meshes with 193,424 triangles and zero
degenerate triangles. Every source and output file has recorded size and SHA-256
evidence. Detailed meshes are visual-only: collision, inertia, mass, joints,
controllers, safety behavior, and nominal targets are unchanged. Fresh Gate 0
and Gate 1 evidence passes, and the live physics metrics match the earlier
proxy-visual baseline within numerical precision.

## Legacy geometry and dynamics evidence

- Legacy IK link lengths: coxa `43 mm`, femur `120 mm`, tibia `120 mm`, terminal
  segment `50 mm`.
- Legacy joint publication order is `L1`, `L2`, `L3`, `R1`, `R2`, `R3`, with
  `C/F/T/E` per leg, followed by gimbal yaw.
- The legacy URDF has 26 links and 25 joints, which agrees with the accepted
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
- The latest Fusion API export is structurally and mathematically complete but
  cannot yet supply trusted inertias: 707 of 732 body occurrences remain Steel,
  producing an implausible `9.804328 kg` assembly mass.

## Fields intentionally unresolved for physical use or higher fidelity

The following facts remain unresolved after reconciling STEP and the Fusion API
export:

1. Physical validation of the derived canonical zero offsets and positive-motion
   directions for all 25 joints.
2. Safe mechanical lower and upper joint limits.
3. Maximum credible velocity and effort limits for simulation and control.
4. Validation of every servo ID, model, direction, and PWM calibration against
   the physical robot.
5. Refinement of `nominal_standing_reference_v0` and definition of a safe startup
   transition after physical validation.
6. Current per-rigid-body mass, center of mass, and inertia tensor after
   correcting the all-Steel material assignments.
7. Exact Raspberry Pi Camera Module 3 frame transforms, only if that optional
   camera is later included.
8. Exact Gemini 335 optical-frame transforms and whether the yaw joint remains
   locked at gimbal yaw zero for the first SLAM milestone.
9. Foot collision/contact shape and self-collision exclusions.

## Researched component-mass evidence

Research on 2026-08-15 established the following candidate masses. Published
mass does not include unmodeled mounting hardware, cables, heatsinks, cases, or
other installed additions unless the source explicitly includes them.

| Component | Quantity | Candidate unit mass | Evidence and status |
|---|---:|---:|---|
| DS3235 35 kg servo | 19 | `0.060 kg` | DS3235-270 datasheet copies and multiple product listings consistently report `60 g`; weigh an installed-equivalent unit if practical |
| DS5160 60 kg servo | 6 | `0.158 kg` | Manufacturer-formatted DS5160 datasheet reports `158 g`; direct manufacturer hosting was not located, so measurement remains preferable |
| ORBBEC Gemini 335 | 1 | `0.097 kg` | Orbbec Gemini 330-series datasheet and store both specify `97 g` |
| Raspberry Pi Camera Module 3 | 1 | `0.004 kg` | Official Raspberry Pi hardware table specifies `4 g` |
| Hiwonder LSC-32 controller | 1 | about `0.026 kg` | Official Hiwonder manual specifies about `26 g`; physical controller identity remains to be checked |

The 19 DS3235 and 6 DS5160 servos contribute a candidate total of
`2.088 kg` before horns, brackets, cabling, or fasteners not included in the
published unit masses.

Authoritative published masses were not found for the installed Raspberry Pi 5
configuration or PiSugar 3 Plus, and the exact model of the separate 7.4 V
7200 mAh battery is unknown. Those installed assemblies should be weighed. The
Fusion inventory also has no clearly named PiSugar, main battery, or LSC-32
occurrence; their geometry/location must be confirmed before their mass can be
included in a corrected rigid-body model.

Sources:

- DS3235-270 datasheet copy:
  `https://github.com/microrobotics/DS3235-270/blob/master/DS3235-270_datasheet.pdf`
- DS5160 manufacturer-formatted datasheet copy:
  `https://robojax.com/file_download.php?id=521&tid=720`
- Orbbec Gemini 330-series datasheet:
  `https://www.orbbec.com/wp-content/uploads/2025/06/Gemini-330-series-Datasheet-V1.6.pdf`
- Raspberry Pi camera hardware table:
  `https://www.raspberrypi.com/documentation/accessories/camera.html`
- Hiwonder LSC-32 manual:
  `https://docs.hiwonder.com/projects/32-Channel-Servo-Controller/en/latest/docs/1_User_Manual_checked.html`

The raw Fusion JSON remains immutable evidence. A derived dynamics snapshot is
generated separately from a versioned mass-override file and retains source
URLs, evidence status, target masses, occurrence rules, and the raw-export
hash. The current exporter provides body material names but not per-body
volume, center of mass, or inertia, so it supports only a coarse occurrence-
level correction. An enhanced per-body physical-property export would be
required to preserve the changed mass distribution accurately.

## `rough_estimate_v0` simulation dynamics

On 2026-08-15 the user accepted rough mass properties for initial simulator
development. The immutable Fusion source remains rejected as direct dynamics
input, but an explicitly provisional derived estimate is now available:

- Override manifest: `tools/fusion/rough_mass_estimates_v0.json`
- Reproducible generator: `tools/fusion/generate_rough_dynamics.py`
- Derived snapshot: `tools/fusion/araco_rough_dynamics_v0.json`
- Raw source SHA-256:
  `843974cc090e76d17c691d4e097c602ccd4e966e07e158ba392ae1bef54b7352`

The correction decomposes mixed occurrences from their exported mass and
volume using PETG at `1270 kg/m³` and Steel at `7850 kg/m³`. It keeps the PETG
contribution, rejects the Steel-derived contribution, and inserts the following
component budget:

| Assembly group | Quantity | Raw mass | Estimated mass | Replacement interpretation |
|---|---:|---:|---:|---|
| Printed covers/holders | 4 | `0.120036 kg` | `0.120036 kg` | Keep exported PETG mass |
| Raspberry Pi 5 | 1 | `0.145767 kg` | `0.050000 kg` | Round installed-Pi estimate |
| Gimbal mount | 1 | `0.238234 kg` | `0.090906 kg` | PETG plus one DS3235 |
| Frame | 1 | `1.422909 kg` | `0.538934 kg` | PETG plus six DS3235 |
| Moving gimbal | 1 | `0.535740 kg` | `0.277432 kg` | PETG plus Gemini 335 and Pi camera |
| Coxa links | 6 | `2.734060 kg` | `1.101811 kg` | PETG plus six DS5160 total |
| Femur links | 6 | `0.312462 kg` | `0.312462 kg` | Keep exported PETG mass |
| Tibia assemblies | 6 | `4.158309 kg` | `0.720000 kg` | Two DS3235 per occurrence |
| Foot links | 6 | `0.136811 kg` | `0.136811 kg` | Keep exported PETG mass |

The 32 represented occurrences total `3.348393 kg`. Three unmodeled base
proxies add `0.576 kg`: PiSugar `0.150 kg`, the unknown 7.4 V 7200 mAh battery
`0.400 kg`, and the probable LSC-32 `0.026 kg`. The central whole-robot estimate
is therefore `3.924393 kg`.

For each represented occurrence, the derived snapshot retains the exported
center of mass and uniformly scales its inertia by the mass ratio. This is a
deliberate coarse approximation. Aggregate center of mass and inertia are left
unavailable rather than fabricating poses for the three missing base proxies.
The estimate may seed Gazebo but is not valid for hardware safety, actuator
sizing, structural analysis, or a claim of sim-to-real fidelity.

## Remaining evidence for hardware validation or higher fidelity

The repeatable exporter now supplies the complete occurrence inventory and all
25 joint origins and axes. These remaining items do not block Phase 0 or
provisional Gazebo authoring, but Fusion or direct measurement is still needed
before the corresponding hardware or high-fidelity claims:

- Validated finite joint limits; current lower and upper limits are disabled.
- If higher-fidelity dynamics become necessary, complete the remaining
  material/effective-mass assignments followed by a new physical-property
  export. The current derived estimate is intentionally simulator-only.
- A material-only correction requires rerunning the Fusion API exporter, which
  reads material, mass, center of mass, and inertia. It does not require another
  AP214 STEP export; re-export STEP only after geometry, placement, joint, or
  assembly-structure changes.
- Purchased composite assemblies and infill-printed parts should use measured
  mass-informed effective properties where practical; assigning a generic
  homogeneous library material can still produce misleading dynamics.
- For a modeled electrical/servo volume `V`, use an effective density
  `rho = measured_mass / V`, apply it only to that component's bodies, and
  verify the resulting component mass before rerunning the exporter.
- Named reference points or joint origins for the Gemini optical frames and,
  only if later included, the optional Raspberry Pi camera frames.

If Fusion joints are missing or stale, the same data must be established by
explicit design dimensions and physical measurements before the model is called
hardware-accurate.

## Simulator-authoring acceptance and physical deferrals

The checked items are accepted for the first simulator model. Unchecked items
remain physical-validation or fidelity work and do not block creation of the
package skeleton:

- [x] Descriptive joint/link names in this manifest.
- [x] Mapping `L1/L2/L3` to left front/middle/rear and `R1/R2/R3` to right
  front/middle/rear.
- [x] REP-103 axis conversion; Fusion `+Y` is physical forward and maps to ROS
  `+X`.
- [x] `base_link` datum `[-0.313602, 0, 8.072807] mm` in Fusion.
- [x] Four-link chain order `coxa → femur → tibia → foot` for all legs.
- [x] Raspberry Pi camera policy: physically installed but software support and
  description frames are deferred/optional; omission is allowed.
- [x] Gimbal and Gemini frame ownership.
- [x] Parent/child pairs, origins, and axes for all 25 actuated joints.
- [x] Approximate simulation home pose `nominal_standing_reference_v0`.
- [ ] Canonical joint zeros and finite safe limits.
- [ ] Servo IDs, models, directions, and calibration evidence.
- [x] Rough simulator mass estimate (`rough_estimate_v0`).
- [x] Approved detailed visuals for 20 links, with explicit retained tibia
  proxies and full source/output provenance.
- [ ] Physical mass/inertia validation and resolution of the legacy `L1E1`
  outlier.

Phase 0, Phase 1 / Gate 0, Phase 2 / Gate 1, and the bounded detailed-visual
integration are implemented and validated. Phase 3 / Gate 2 and later work
remain separately authorized phases. Nothing in this manifest authorizes
physical motion.
