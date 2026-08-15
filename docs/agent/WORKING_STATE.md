# Araco Hexapod — Working State

Updated: 2026-08-16

## Current state

Phase 1 / Gate 0 and Phase 2 / Gate 1 are implemented and pass. Exact
Fusion/vendor tibia, servo, and Gemini 335 exterior presentation geometry is
integrated. A headed Gazebo preview from the validated copied install is running
for user inspection and reached `HOLDING` without motion enablement. Collision
geometry, dynamics, joints, controllers, safety behavior, camera simulation
configuration, optical frames, and the nominal standing target remain
unchanged.

Phase 3 / Gate 2 and every later phase remain unauthorized. The Phase 1,
Phase 2, and detailed-visual worktree is intentionally uncommitted and
unpushed. `HEAD` and `origin/main` remain at
`73830cc27ab39efb0f51cfe4ec89962d5df8039c` on `main`. The active headed preview
uses `/tmp/araco_camera_copy_build.tJM6Dg/install`, with immutable runtime bundle
`/tmp/araco_camera_copy_build.tJM6Dg/headed_runtime`.

## Fusion visual integration

- Active Fusion bundle:
  `/media/stevw-s14/DATA-ST/New folder/araco_-_assembly_ros-description_v1_v2_visual_export_20260815T222414Z`.
  It records Fusion `2704.1.53`, exporter `0.4.0`, specification `3.0.0`, cloud
  version 2, 59 mesh exports, 77 reviewed bodies, and zero retained proxies.
- Fail-closed validation covers source identity, structural inventory, exact
  allowlist, per-asset rights, paths, sizes, hashes, and triangle counts. The
  import is preserved under `meshes/source/fusion_v2_exact_camera/` as 34
  deduplicated immutable STL blobs; both earlier source captures remain
  untouched.
- Each full tibia source STL reproducibly contains five connected solids:
  6,448 printed-part triangles, two 48,436-triangle servo cases, and two
  3,864-triangle horns. Direct DS3235 meshes also split deterministically into
  separate case/horn shells. Each DS5160 is one connected vendor solid and is
  kept intact.
- `normalize_fusion_exact_visuals.py` applies the recorded occurrence transform,
  Fusion-to-ROS/base datum, and inverse nominal link or fixed-frame transform.
  It emits 49 meter-scale link-local meshes: 26 primary, 13 servo-case, seven
  servo-horn, and three camera role assets. The output contains 2,066,740
  rendering triangles and zero degenerates after removing 36 zero-area source
  triangles.
- The generated URDF uses all 49 exact mesh visuals and no visual box, cylinder,
  or tibia proxy. Printed, servo-case, horn, camera-body, camera-hardware, and
  camera-optics roles retain distinct presentation materials. Every URI and
  SHA-256 is registered with preserved MIT, upstream open-source, or
  mixed-open-source provenance.
- The user's reported tibia offset is confirmed quantitatively. For both middle
  legs, the Fusion tibia occurrence frame is approximately `9.7567 mm` from the
  canonical kinematic tibia origin (`-7.88235 mm` and `-5.74996 mm` in the two
  transverse Fusion coordinates) and differs by `90 deg` around the link's
  longitudinal axis. The replacement remains centered on the ideal
  joint-to-joint line at the user's explicit direction not to force-align the
  frames. This does not invalidate the joint axis; the visual and collision
  proxies must not be presented as exact physical fidelity.
- The earlier tibia-frame offset remains preserved exactly as directed: no
  visual, joint, collision, or inertial frame is force-aligned. The exact
  meshes are visual-only and must not be interpreted as upgraded collision or
  dynamics fidelity.
- The active bundle's 15 Gemini selections are five housing/bracket bodies, six
  pads/fasteners, and four optics. They normalize into three `camera_link`
  visuals while the eight-body connector/PCB assembly and Raspberry Pi Camera
  Module 3 remain excluded. The local allowlist validates against the active
  Fusion version-2 inventory.
- The four exporter files in
  `/media/stevw-s14/DATA-ST/New folder/AracoRobotDescriptionExporter/` are
  byte-identical to the validated repository copies. No CAD file or existing
  export bundle was modified.

## Current visual-fix validation

- Fresh copied build at `/tmp/araco_camera_copy_build.tJM6Dg`: all nine packages
  passed.
- Full copied-install test result: 182 tests, 0 errors, 0 failures, and three
  expected `cppcheck` skips.
- Focused composed-model contract: 49 mesh visuals, comprising 26 primary,
  13 servo-case, seven servo-horn, and three Gemini role meshes; zero primitive
  visual proxies. `camera_link` remains visual-only with no collision or
  inertial block and is fixed to `gimbal_yaw_link`.
- `check_urdf` passes and `gz sdf -k` reports `Valid.`
- The active development composition has behavior fingerprint
  `5589ed99314a08253aea8ca59dfdc0e830b9b58c10d88162a1c7c050d1027771`.
- Headed Gazebo reached `HOLDING` without entering motion.
- The pre-existing workspace `build/araco_interfaces` contains a stale generated
  symlink-directory conflict. Validation and preview therefore use fresh copied
  build/install roots under `/tmp`; no generated workspace directory was
  deleted or reset.

## Verified evidence

Passing detailed-visual Gate 0:
`log/gate_0_20260816_detailed_visuals/`

- Result: `PASS`.
- Behavior fingerprint:
  `87eef7bca155569674450e9342856ed89fb7e9f74e3e3df2c11311d8c9a937d7`.
- CI run fingerprint:
  `3904f27bff4b5b380e8dbee598af01b088f871230be03bd567c7ad4bc460bd40`.

Passing detailed-visual Gate 1:
`log/gate_1_20260816_detailed_visuals_hold_host/`

- Result and evidence validation: `PASS`; startup to `HOLDING` was
  `14.844930884 s` against the `30 s` limit.
- Maximum leg error `9.29443e-11 rad`; RMS leg error `5.71404e-11 rad`.
- Maximum base-height error `6.31993e-7 m`; maximum pitch
  `4.46465e-9 rad`; maximum roll `6.62839e-19 rad`.
- Maximum base linear speed `0.0 m/s`; maximum penetration
  `9.11246e-6 m`; all six foot-contact and safety checks passed.
- These physics metrics match the pre-visual Gate 1 baseline within numerical
  precision, confirming the change is visual-only.

`log/gate_1_20260816_detailed_visuals_hold/` is intentionally preserved as
failed environment evidence. The sandbox denied interface discovery
(`getifaddrs`), so no clock, ROS, or simulator samples existed. The new atomic
host run passed without changing code or configuration.

Earlier accepted pre-visual evidence remains at
`log/gate_0_20260816_phase2_regression/` and
`log/gate_1_20260816_phase2_hold_pass/` for comparison.

## Validation

- All 9 packages build successfully in a fresh copied build/install root.
- `colcon test-result --all --verbose`: 182 tests, 0 errors, 0 failures, and
  3 expected `cppcheck` skips.
- Exact-mesh tests reproduce every output and manifest byte-for-byte, enforce
  59-export / 77-body source coverage and 49-output role coverage, verify all
  hashes and bounds, and confirm zero output degenerate triangles.
- Installed composition contains exactly 49 mesh visual URIs and zero primitive
  visual proxies. `check_urdf` passes and `gz sdf -k` reports `Valid.`
- `rosdep check --from-paths src --ignore-src`: all dependencies satisfied.

## Authorization boundary and exact next step

- Simulator evidence does not authorize physical motion. Physical servo zero,
  direction, safe limits, dynamics, collision/contact fidelity, camera poses,
  gains, and electrical integration remain provisional or unresolved.
- Do not begin Phase 3 / Gate 2 without explicit authorization.
- Do not commit or push without a separate explicit instruction.
- The immediate next step is user visual acceptance of the running headed
  preview. If the exact Gemini exterior is correctly placed and readable, stop
  this visual task; any later commit/push or Phase 3 work requires separate
  explicit authorization.
- Do not commit or push without a separate explicit instruction.
