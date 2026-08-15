# Araco Hexapod — Working State

Updated: 2026-08-16

## Current goal

Hold at the completed architecture boundary and await an explicit Phase 0
repository-foundation authorization. No ROS package or robot implementation is
authorized yet.

## Current phase constraints

- Architecture and decision work only; no ROS packages are authorized yet.
- Development remains simulator-first. Gazebo must exercise the canonical
  control contracts before a physical-hardware backend is enabled.
- No physical robot commands are authorized.
- The legacy project is behavioral/calibration evidence, not the new
  architecture.

## Accepted direction

- Greenfield ROS 2 Jazzy repository on Ubuntu 24.04.
- Gazebo Harmonic for local functional simulation and CI; Isaac Sim/Isaac Lab
  later for higher-fidelity work and deferred RL.
- Deterministic kinematics, tripod gait, body control, teleoperation, state
  estimation, RGB-D SLAM, and flat-ground Nav2 precede RL.
- Pi/workstation split accepted on 2026-08-15:
  - Raspberry Pi eventually owns the servo interface, `ros2_control`, local
    safety/watchdogs, kinematics, gait generation, and physical lifecycle.
  - Workstation initially owns Gazebo, RViz/development, RGB-D SLAM, and Nav2.
  - Wi-Fi/workstation loss must be handled locally on the Pi.
- Simulator control pipeline accepted on 2026-08-15:
  - Commands flow through arbitration and an independent safety supervisor into
    one deterministic locomotion process.
  - Locomotion owns body motion, tripod-gait phase, foot trajectories, and IK;
    kinematics is a pure, independently testable library within that process.
  - Separate `ros2_control` paths command the 24 leg joints and one gimbal joint.
  - `gz_ros2_control` is the initial backend; physical hardware must later
    preserve the same higher-level boundary.
- Repository/package boundaries accepted on 2026-08-15:
  - Initial packages: `araco_interfaces`, `araco_description`,
    `araco_kinematics`, `araco_locomotion`, `araco_supervision`,
    `araco_teleop`, `araco_gazebo`, `araco_bringup`, and
    `araco_system_tests`.
  - Hardware, perception, navigation, and Isaac adapters are phased packages.
  - Full responsibilities and dependency direction are recorded in
    `docs/agent/REPOSITORY_ARCHITECTURE.md`.
- Command-path contract accepted on 2026-08-15:
  - `MotionIntent`, `CommandCandidate`, `SelectedCommand`, and `SafeCommand`
    separate source, arbitration, safety, and locomotion authority.
  - Velocity, absolute body-pose offset, and stand/tripod intent are atomic.
  - Source identity, priority, and timeout are configured; source publishers do
    not claim them. Steady receipt time determines freshness.
  - Field and validation semantics are recorded in
    `docs/agent/INTERFACE_CONTRACTS.md`.
- Feedback/controller contract accepted on 2026-08-15:
  - Standard 25-joint state is accompanied by `JointStateProvenance`; physical
    open-loop state is labeled command-derived rather than measured.
  - `LocomotionStatus` reports deterministic gait/IK/output validity.
  - `joint_state_broadcaster`, a 24-joint leg trajectory controller, and a
    separate one-joint gimbal trajectory controller form the accepted
    `ros2_control` set.
  - Locomotion publishes complete named, positions-only, one-point,
    short-horizon leg trajectories through the topic interface.
- Safety/lifecycle contract accepted on 2026-08-15:
  - Eight software safety states remain independent of ROS lifecycle state.
  - Motion requires readiness, explicit enable, and a fresh source activation
    edge; source loss cannot automatically resume or fall back to another
    moving source.
  - Source changes cross a controlled-stop and verified stable-hold barrier;
    locomotion state commits transactionally across all six legs and 24 joints.
  - Layered watchdogs, typed reason/fault reporting, reset rules, and strict
    Gazebo startup/shutdown ordering are recorded in
    `docs/agent/SAFETY_ARCHITECTURE.md`.
- Configuration/validation contract accepted on 2026-08-15:
  - Every configuration class has one package owner; bringup composes values
    without becoming a second source of truth.
  - Simulator estimates, physical calibration, and operational policy remain
    distinct evidence classes.
  - Motion-affecting changes require hold, reconfiguration, readiness
    revalidation, and a fresh enable.
  - A nested limit hierarchy and seven ordered blocking Gazebo gates define the
    simulator baseline in
    `docs/agent/CONFIGURATION_AND_VALIDATION_ARCHITECTURE.md`.
- Runtime/timing/QoS/simulator-values contract accepted on 2026-08-15:
  - `1000 Hz` physics, `250 Hz` controller manager, `100 Hz` control-domain
    loops, `50 Hz` teleop, and a `0.040 s` leg-trajectory horizon form the
    initial rate hierarchy.
  - Steady-time watchdogs remain independent of ROS simulation-time gait and
    trajectories; a paused simulator revokes motion readiness.
  - Concrete topics, QoS, source IDs/priorities/timeouts, slow gait/command
    envelopes, provisional joint/dynamics values, and Gate 0–6 thresholds are
    frozen in `docs/agent/RUNTIME_TIMING_AND_SIMULATION_CONTRACT.md`.
- Parameter/artifact/runtime-composition contract accepted on 2026-08-15:
  - Package-owned strict artifacts, typed generated node parameters, exact
    profile selection, closed overrides, deterministic preflight, immutable
    runtime bundles, and non-circular fingerprints are frozen.
  - Controller joint lists are generated from the canonical model registry;
    Jazzy controller files are passed to each spawner with `--param-file`.
  - Full details are recorded in
    `docs/agent/PARAMETER_AND_CONFIGURATION_COMPOSITION.md`.
- Phased simulator delivery plan accepted on 2026-08-15:
  - One repository-foundation phase is followed by one blocking phase for each
    Gate 0–6; each exit reruns all prior gates and retains typed evidence.
  - Gate 1 nominal hold precedes Gate 2 computed IK; Gate 3 onward uses the
    production command path; Gate 6 requires three clean no-retry runs.
  - Full details are recorded in `docs/agent/PHASED_DELIVERY_PLAN.md`.

## Completed evidence and environment

- Workstation now runs Ubuntu 24.04.4 with NVIDIA driver, ROS 2 Jazzy,
  `rosdep`, Gazebo Harmonic, `ros_gz`, `ros2_control`, and `gz_ros2_control`
  validated.
- A headless ROS–Gazebo controller integration test passed without robot access.
- Fusion/legacy reconciliation produced a proposed 26-primary-link/25-joint
  robot-description contract.
- Fusion working copy now contains 24 regular leg revolutes plus one as-built
  gimbal revolute; all 25 parent/child pairs, pivots, and axes were exported.
- User accepted forward direction, leg naming, descriptive joint/link names,
  camera/gimbal ownership, and the `base_link` datum.
- `nominal_standing_reference_v0` matches the legacy standing IK numerically but
  remains simulation-only and physically approximate.
- Fusion version 2 material/API exports were validated on 2026-08-15. Joint and
  occurrence structure is intact, all direct physical-property records are
  mathematically valid, and no exporter errors were found.
- The raw Fusion dynamics remain rejected: 707 of 732 body occurrences still
  report Steel and Fusion calculates `9.804328 kg`, above the user's `2–4 kg`
  estimate.
- User clarified that remaining Steel bodies in the leg/frame/gimbal groups are
  servos. They require measured/manufacturer-mass-based effective density, not
  PETG assignment. Composite electronics require the same mass-equivalent
  treatment.
- Component research found candidate masses of `60 g` per DS3235, `158 g` per
  DS5160, `97 g` for Gemini 335, `4 g` for Camera Module 3, and about `26 g` for
  the probable LSC-32. The 25 servos total `2.088 kg` before unmodeled installed
  additions. Sources and evidence status are recorded in
  `docs/agent/ROBOT_DESCRIPTION_MANIFEST.md`.
- The user accepted rough initial simulator dynamics. The immutable raw JSON is
  now paired with versioned `rough_estimate_v0` inputs and a reproducibly
  generated snapshot under `tools/fusion/`.
- The 32 represented occurrences total `3.348393 kg`; missing PiSugar, main
  battery, and LSC-32 proxies add `0.576 kg`, giving a central robot estimate of
  `3.924393 kg`.
- Occurrence centers of mass are retained and inertias uniformly mass-scaled.
  Aggregate center of mass and inertia remain intentionally unavailable because
  the proxy poses and per-body properties are unresolved. An enhanced Fusion
  export or measurements are deferred until greater fidelity is needed.
- Final architecture validation passed on 2026-08-15: both package maps contain
  all nine packages; both numerical contracts contain Gates 0–6; the safety
  contract contains eight states and reason codes 0–30; Markdown fences and
  document references are valid; `git diff --check` passes; both dynamics JSON
  files parse; and regenerating from the hashed external Fusion snapshot exactly
  reproduces `araco_rough_dynamics_v0.json` at `3.924392774795984 kg`.
- `src/`, `build/`, `install/`, and `log/` remain absent, as required before
  explicit Phase 0 authorization. `README.md` remains the original minimal
  stub.
- License-revision validation passed on 2026-08-16: every current architecture
  authority uses `GPL-3.0-only`; Apache-2.0 appears only in explicitly
  superseded history; and `git diff --check` passes.
- The user separately authorized a licensed pre-Phase-0 checkpoint on
  2026-08-16. Root `LICENSE` now contains the unmodified official GPLv3 text.
  This authorization covers committing and pushing the accumulated coherent
  checkpoint, but it does not authorize Phase 0 or package scaffolding.

## In progress

- No implementation work is in progress.
- The final architecture consistency review passed and is recorded in
  `docs/agent/FINAL_ARCHITECTURE_REVIEW.md`.
- GNU GPL version 3 only (`GPL-3.0-only`) is selected for project-authored
  repository and package content as of 2026-08-16, superseding the earlier
  Apache-2.0 plan. Root `LICENSE` is present; package-local license copies and
  source SPDX headers wait for Phase 0 authorization.
- Phase 0 must audit linked/bundled dependency compatibility and document GPLv3
  Corresponding Source and third-party attribution obligations for distributed
  artifacts.
- The audit specifically includes publishable editable source for covered
  generated meshes and the proprietary Autodesk API boundary used by the
  existing Fusion add-in. The later Isaac adapter requires the same review.
- The exact permitted and forbidden scope of Phase 0 is frozen in the final
  review. A public maintainer name/email must be confirmed when package
  manifests are about to be written.
- Vendor CAD with unknown redistribution rights is excluded from the public
  package; Phase 1 must use cleared assets or project-authored simplified
  proxies.

## Open decisions

1. Detailed later hardware-interface design.
2. Verified physical low-power, local-stop, support/lowering, startup, and
   shutdown behavior.
3. Final Pi OS/runtime acceptance; Ubuntu Server 24.04 arm64 with native Jazzy
   remains the recommendation and Camera Module 3 no longer blocks it.

## Physical-hardware blockers

- Installed safe mechanical limits and canonical zero/direction validation are
  incomplete. Reported `270 deg` servo travel is not a safe joint range.
- Servo controller identity and serial details require physical verification.
- Open-loop servos provide no measured joint state or contact feedback.
- Servo power-off causes collapse; “hold” versus “remove power” is not yet a
  defined safe fault response.
- Physical mass/inertia validation, camera optical frames, and collision
  geometry remain incomplete. `rough_estimate_v0` is accepted only as an
  initial Gazebo estimate.
- Installed Raspberry Pi/PiSugar mass, exact 7.4 V battery identity/mass, and
  the presence/location of PiSugar, main battery, and LSC-32 geometry in CAD
  remain unresolved.

## Licensed checkpoint contents

- `AGENTS.md`
- `LICENSE`
- `docs/agent/CONTEXT.md`
- `docs/agent/CONFIGURATION_AND_VALIDATION_ARCHITECTURE.md`
- `docs/agent/DECISIONS.md`
- `docs/agent/FINAL_ARCHITECTURE_REVIEW.md`
- `docs/agent/INTERFACE_CONTRACTS.md`
- `docs/agent/PARAMETER_AND_CONFIGURATION_COMPOSITION.md`
- `docs/agent/PHASED_DELIVERY_PLAN.md`
- `docs/agent/REPOSITORY_ARCHITECTURE.md`
- `docs/agent/ROBOT_DESCRIPTION_MANIFEST.md`
- `docs/agent/RUNTIME_TIMING_AND_SIMULATION_CONTRACT.md`
- `docs/agent/SAFETY_ARCHITECTURE.md`
- `docs/agent/WORKING_STATE.md`
- `tools/fusion/AracoRobotDescriptionExporter/README.md`
- `tools/fusion/rough_mass_estimates_v0.json`
- `tools/fusion/generate_rough_dynamics.py`
- `tools/fusion/araco_rough_dynamics_v0.json`

The user authorized these accumulated changes to be committed and pushed to
`origin/main` as the pre-Phase-0 checkpoint. No package scaffolding exists.

## Exact next steps

1. Await the user's explicit Phase 0 authorization; do not infer it from any
   prior architecture approval.
2. At Phase 0 entry, confirm the public maintainer name/email, preserve the
   checkpointed documentation/evidence, and implement only the exact
   repository-foundation scope in `FINAL_ARCHITECTURE_REVIEW.md`, including the
   GPLv3 compatibility/source-obligation audit.
3. Accept or revise the Pi OS/native-runtime recommendation before the later
   physical deployment phase.
4. Do not begin Phase 1 model/configuration work until Phase 0 is separately
   completed and its checkpoint is accepted.
