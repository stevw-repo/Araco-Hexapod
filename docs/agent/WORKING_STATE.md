# Araco Hexapod — Working State

Updated: 2026-08-16

## Current state

Phase 0 repository foundation is complete and validated. It establishes only
the ROS workspace, package boundaries, accepted interfaces, metadata,
licensing, and structural tests. **No Gazebo gate is claimed.** The repository
still has no robot model, runtime configuration, Gazebo world/launch path,
motion-capable node, physical profile, or hardware backend.

The user authorized the completed MIT Phase 0 tree to be committed and pushed
to `origin/main` on 2026-08-16. This working-state file is part of that
checkpoint. The earlier GPL-licensed pre-Phase-0 architecture commit
`868d7fc8bff4d2f99c1e3bb88665735667d83b72` remains in Git history; the current
rights holder authorized the MIT relicense and no history rewrite is required.

## Completed Phase 0 scope

- Created all nine accepted ROS 2 Jazzy `ament_cmake` packages under `src/`:
  `araco_interfaces`, `araco_description`, `araco_kinematics`,
  `araco_locomotion`, `araco_supervision`, `araco_teleop`, `araco_gazebo`,
  `araco_bringup`, and `araco_system_tests`.
- Used package version `0.1.0` and the verified Git identity
  `stevw <steven060520@gmail.com>` as public maintainer metadata.
- Added the accepted seven messages: `MotionIntent`, `CommandCandidate`,
  `SelectedCommand`, `SafeCommand`, `JointStateProvenance`,
  `LocomotionStatus`, and `SafetyStatus`.
- Added the accepted `SafetyTransition` action.
- Added exact dependency declarations/exports, optional future-resource install
  rules, package metadata tests, generated-interface introspection tests, and
  an exact acyclic project-dependency-graph test.
- Added root workspace hygiene and build/test documentation. Generated
  `build/`, `install/`, `log/`, Python caches, editor state, and common local
  artifacts are ignored.
- Relicensed current project-authored content to MIT. Root and package-local
  license texts are identical; each package installs its license alongside its
  manifest; package source/manifests carry exact MIT SPDX/license metadata.
- Completed `docs/agent/PHASE_0_LICENSE_AUDIT.md`. No third-party code, binary,
  mesh, CAD, font, logo, or data requiring a bundled notice is present in the
  Phase 0 ROS packages, so no `THIRD_PARTY_NOTICES.md` is needed yet.

## Final validation evidence

Validated from a clean generated-output state on 2026-08-16:

- `rosdep check --from-paths src --ignore-src --rosdistro jazzy`:
  all system dependencies satisfied.
- `colcon build --symlink-install`: 9 packages finished successfully.
- `colcon test` plus `colcon test-result --verbose`: 111 tests, 0 errors,
  0 failures, 0 skipped.
- The test suite covers flake8, PEP 257, CMake lint, XML validation, package
  metadata/license policy, generated IDL fields/types/constants, and exact
  project dependency edges/acyclicity.
- A clean Bash shell sourced `/opt/ros/jazzy/setup.bash` and
  `install/setup.bash`, resolved all nine packages to this repository's install
  tree, found their installed manifests/licenses, and introspected all seven
  messages plus the action.
- Every source and installed package `LICENSE` matched the root MIT file.
- Both Fusion dynamics JSON files parsed, and regenerating from the external
  version-2 Fusion export reproduced `araco_rough_dynamics_v0.json` byte for
  byte at `3.924392774795984 kg`.

The first installed-shell probe enabled Bash nounset before sourcing ROS and
therefore hit ROS setup's expected unset-variable access. The corrected clean
probe sources ROS before enabling nounset and passed; this is a shell harness
ordering issue, not a package failure.

## License and distribution boundary

- Current project-authored content uses MIT (`SPDX-License-Identifier: MIT`).
- The former Apache-2.0 plan and GPL-3.0-only selection are superseded history.
  The pushed GPL checkpoint remains valid for copies received at that commit.
- Referenced Jazzy build/interface dependencies are installed Apache-2.0
  packages and are not bundled in the repository.
- Autodesk Fusion/API code and future Isaac SDKs remain proprietary external
  systems. MIT licenses only the project-authored integration source.
- Unknown-rights vendor CAD remains excluded. Future assets require provenance,
  redistribution permission, and attribution before bundling.

## Constraints and unresolved physical blockers

- Phase 1 is not authorized. Do not create Xacro/URDF/meshes, runtime
  YAML/schemas, Gazebo world/launch implementation, control nodes, or gate
  evidence until it is authorized.
- No physical commands are authorized. Simulator evidence never implicitly
  clears physical deployment.
- Installed safe mechanical limits, canonical physical zero/direction,
  controller identity, local-stop behavior, collapse-safe shutdown, and
  measured state remain unresolved.
- `rough_estimate_v0` is accepted only as an initial Gazebo estimate. Aggregate
  base inertia and missing electronics' poses remain unresolved.
- Final Pi OS/runtime acceptance remains open; Ubuntu Server 24.04 arm64 with
  native ROS 2 Jazzy remains the recommendation.

## Exact next step

Verify that the authorized MIT Phase 0 checkpoint is present on `origin/main`.
After that, stop before Phase 1. Phase 1 / Gate 0 model-and-configuration
implementation still requires separate authorization.
