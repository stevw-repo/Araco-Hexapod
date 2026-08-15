# Araco Hexapod — Decisions

## 2026-08-14 — Rebuild is greenfield

Status: accepted from the user's stated goal.

Decision: Redesign and rebuild the ROS 2 project from scratch. Use the legacy code, models, calibration constants, and documentation as evidence and migration inputs, not as an architectural base.

Rationale: The legacy locomotion behavior works, but the project is unstructured, undocumented in code, unsafe by design, difficult to test or deploy, and couples most control responsibilities into one node. A clean architecture is an explicit requirement.

Consequences:

- Compatibility with legacy topics and package names is not assumed.
- Useful geometry, mesh, calibration, and behavioral data must be validated before migration.
- Architecture decisions remain open until hardware, safety, operational, and acceptance requirements are captured.
- Proven legacy locomotion behavior and servo calibration are regression evidence and migration inputs; greenfield architecture does not mean discarding behavior that is known to work.

## 2026-08-14 — Initial functional scope and simulator direction

Status: accepted from the user's answers; the secondary-simulator question was reopened on 2026-08-14 and resolved on 2026-08-15.

Decisions:

- The first locomotion implementation needs only the tripod gait.
- Height, roll, pitch, yaw, and translation controls are essential.
- Isaac Sim/Isaac Lab is the advanced high-fidelity and learning environment; Gazebo Harmonic is the local/CI functional simulator under the accepted portfolio below.
- Development is simulator-first; physical-robot integration is deferred rather than treated as the first milestone.
- Cloud GPU execution is a first-class option for Isaac Sim and Isaac Lab because the laptop's local simulator performance is inadequate.
- The implementation may use C++ for timing-sensitive components and Python for suitable higher-level/tooling components.
- This repository is the final project repository.
- Do not write implementation code or scaffold packages until the user authorizes it.

Rationale: These choices preserve the known working behavior, constrain the first milestone, align simulation with the intended Isaac Lab direction, and avoid premature implementation during discovery.

## 2026-08-14 — Supported workstation, release, and cloud constraints

Status: accepted from the user's answers.

Decisions:

- Support Windows/Ubuntu dual boot: Windows for Fusion 360 and Ubuntu 24.04 for ROS 2 development.
- Target the current supported Isaac Sim/Isaac Lab generation when implementation begins; treat Isaac Sim 4.5 assets as migration references rather than a compatibility requirement.
- Treat approximately USD 30 per month as the preferred starting cloud-compute budget, not a hard ceiling; the user is willing to increase it when justified.
- Design for roughly three-hour interactive cloud sessions and optionally longer headless training jobs. Establish an explicit cap from actual provider pricing before provisioning.
- Use CAD-derived mass and inertia estimates initially, then validate them against the physical robot.
- Make simulator acceptance criteria a prerequisite to implementation.
- Prepare the final public repository as a reproducible personal-project showcase with a write-up.

Rationale: Ubuntu 24.04 aligns with the intended ROS 2 Jazzy toolchain, Windows remains necessary for the current CAD source, and current Isaac releases avoid building a new system around obsolete simulator interfaces. An explicit approved budget and validation gates prevent cloud costs and simulator fidelity from remaining vague.

Consequences:

- Exact Brev/provider pricing must be checked before provisioning; recommend a concrete budget and obtain the user's approval before incurring cloud cost.
- Public-repository hygiene, licensing, reproducible setup, and architecture documentation are required from the start.
- The simulator workflow should be capable of interactive streaming and headless jobs if practical; the user's preferred default is still undecided.

## Pending decisions

- Detailed later `ros2_control` physical-hardware-interface design
- Verified physical low-power, local-stop, support/lowering, startup, and shutdown behavior
- Detailed local-versus-cloud workflow, GPU instance class, persistence, remote rendering, and exact cost controls

## 2026-08-15 — Physical compute ownership and simulator-first sequence

Status: accepted by the user on 2026-08-15; Pi OS/container choice remains open.

Decisions:

- The Raspberry Pi will eventually own the physical servo interface,
  `ros2_control`, command watchdogs and safety supervision, kinematics, gait
  generation, and physical startup/shutdown behavior.
- The workstation will initially own Gazebo, RViz and development tooling,
  RGB-D SLAM, and Nav2.
- The project remains simulator-first. The same high-level command and joint
  contracts must run against a simulator backend before a physical-hardware
  backend is enabled.
- Loss of the workstation or Wi-Fi must be handled locally by the Pi; physical
  safety must never depend on receiving the next offboard command.
- Raspberry Pi Camera Module 3 support is deferred and may be omitted entirely;
  it does not constrain the simulator architecture or Pi OS choice. Gemini 335
  remains the primary planned RGB-D sensor.

Rationale: keeping the complete robot-control and safety path onboard removes
Wi-Fi latency and disconnection from the actuator-control dependency chain,
while offloading compute-heavy perception and development workloads from the
4 GiB Pi. Simulator-first sequencing lets these boundaries be tested without
actuating the robot.

## 2026-08-15 — Simulator control pipeline and locomotion boundary

Status: accepted by the user on 2026-08-15.

Decisions:

- Use the canonical runtime path: command sources → command arbitration → an
  independent safety supervisor → locomotion → `ros2_control` controllers → a
  replaceable simulator or physical backend.
- Keep body-motion generation, tripod-gait phase, foot-trajectory generation,
  and inverse kinematics in one deterministic locomotion process.
- Implement kinematics as a pure, independently testable library used by the
  locomotion process, not as a separately scheduled ROS node.
- Use separate `ros2_control` control paths for the 24 leg joints and the one
  gimbal-yaw joint, with joint-state publication alongside them.
- Use `gz_ros2_control` as the first backend. The later Raspberry Pi/servo
  hardware interface must preserve the same higher-level command/state
  boundary.
- Return simulated joint, IMU, and contact state to locomotion, safety,
  TF/visualization, diagnostics, and tests as appropriate. Ground truth is
  permitted for scoring and diagnostics, not as an input to a claimed state-
  estimation result.

Rationale: gait phase, body motion, foot trajectories, and IK form one tightly
coupled deterministic calculation. Splitting them across ROS processes would
introduce avoidable scheduling, synchronization, and intermediate-interface
complexity. Keeping the safety supervisor independent permits it to gate or
stop motion without depending on the locomotion process, while a replaceable
`ros2_control` backend preserves simulator-to-hardware portability.

Consequences:

- Package names were resolved by the following repository-boundary decision.
  ROS messages/actions, loop rates, controller types, and lifecycle/fault
  semantics remain to be designed.
- This approval authorizes architecture progression only; it does not authorize
  package scaffolding or implementation.

## 2026-08-15 — Repository and ROS package boundaries

Status: accepted by the user on 2026-08-15.

Decisions:

- Use nine initial packages: `araco_interfaces`, `araco_description`,
  `araco_kinematics`, `araco_locomotion`, `araco_supervision`, `araco_teleop`,
  `araco_gazebo`, `araco_bringup`, and `araco_system_tests`.
- Keep canonical model data in `araco_description`; keep kinematics as a pure
  C++ library without a ROS node.
- Place command arbitration and safety supervision in the cohesive
  `araco_supervision` package but run them as separate lifecycle processes.
- Keep core control independent of simulator and physical-hardware APIs.
  `araco_gazebo` and the later `araco_hardware` package are backend adapters.
- Add `araco_hardware`, `araco_perception`, `araco_navigation`, and
  `araco_isaac` only when their prerequisite phases begin.
- Keep unit tests with their owning packages and cross-package launch/simulator
  acceptance tests in `araco_system_tests`.

Rationale: this split gives each package a cohesive domain without creating a
package for every class. It prevents a monolithic control package, preserves
one-way dependencies, keeps Gazebo replaceable, and avoids adding hardware,
autonomy, or Isaac complexity to the first simulator milestone.

Consequences:

- Package responsibility and dependency rules in
  `docs/agent/REPOSITORY_ARCHITECTURE.md` are now accepted architecture.
- ROS interface fields, topic/service/action names, QoS, controller types,
  rates, lifecycle transitions, and fault behavior remain open.
- This approval does not authorize package scaffolding or implementation.

## 2026-08-15 — High-level command interface and authority contract

Status: accepted by the user on 2026-08-15.

Decisions:

- Use four stage-specific project messages: `MotionIntent`,
  `CommandCandidate`, `SelectedCommand`, and `SafeCommand`.
- Carry planar velocity, absolute body-pose offset, and stand/tripod selection
  atomically in `MotionIntent`, using SI units and canonical REP-103 axes.
- Give every source a separate candidate input. Trusted bringup configuration,
  not the publisher, assigns numeric identity, priority, and timeout.
- Use source generation stamps for provenance and local steady-clock receipt
  time for motion-authority freshness. A source cannot extend its own lease.
- Preserve source sequence and selection/safety epochs through arbitration and
  supervision for observability and transition detection.
- Only the arbiter may publish `SelectedCommand`; only the safety supervisor may
  publish `SafeCommand`; locomotion consumes only `SafeCommand`.
- Exclude direct joint/PWM/controller commands, active gimbal control, safety-
  state requests, source-supplied priority, and source-supplied validity
  duration from the high-level source contract.

Rationale: one atomic intent avoids cross-topic synchronization errors, while
separate message types encode each component's authority instead of trusting a
generic envelope. Receiver-owned freshness prevents stale or malformed sources
from granting themselves continued motion authority.

Consequences:

- Exact fields and semantics in `docs/agent/INTERFACE_CONTRACTS.md` are accepted
  architecture.
- Feedback, diagnostics, and controller output were resolved by the following
  decision. Safety-state meanings, disposition reason codes, rates, concrete
  topic names, and QoS remain open.
- This approval does not authorize IDL creation, package scaffolding, or
  implementation.

## 2026-08-15 — Feedback truthfulness and controller contract

Status: accepted by the user on 2026-08-15.

Decisions:

- Keep standard `sensor_msgs/JointState` for all 25 joints and add the
  `JointStateProvenance` project message to classify position, velocity, and
  effort as unavailable, simulated physics, hardware sensed, command derived,
  or estimator produced.
- On the open-loop physical robot, publish only command-derived joint position;
  leave velocity and effort unavailable until supported. Never describe
  command-derived TF or controller error as measured tracking.
- Add `LocomotionStatus` for locomotion mode, gait phase/cycle, processed
  command epochs, per-leg kinematic validity, and whole-trajectory validity.
- Use `joint_state_broadcaster` plus two separate
  `joint_trajectory_controller/JointTrajectoryController` instances:
  `leg_trajectory_controller` for 24 joints and
  `gimbal_trajectory_controller` for `gimbal_yaw_joint`.
- Use position command interfaces, require complete named trajectories, disable
  partial goals, interpolate continuously replaced references from desired
  state, and configure a non-zero controller command timeout.
- Send the leg controller one positions-only point at a positive short horizon
  with zero header stamp (“start now”) through the topic interface. Do not use
  `FollowJointTrajectory` actions for the continuous gait loop.
- Keep gimbal yaw out of the leg trajectory and held at zero in the first
  simulator milestone. This does not authorize physical gimbal startup.
- Use typed controller/lifecycle state for machine decisions and
  `diagnostic_msgs/DiagnosticArray` for observability; never parse diagnostic
  text as a safety-control input.
- Keep Gazebo contacts and base-pose ground truth in explicit simulation/test
  interfaces rather than making feedback unavailable on the current physical
  robot part of the core locomotion dependency.

Rationale: named, time-interpolated standard trajectories avoid order-only
controller commands and support smooth streamed gait references. Explicit
provenance prevents simulated or command-derived values from being presented as
physical measurements. Separate leg and gimbal ownership preserves the
accepted 24+1 control boundary.

Consequences:

- Exact accepted fields, validation rules, controller parameters, and command
  semantics are maintained in `docs/agent/INTERFACE_CONTRACTS.md`.
- Safety states and reason codes are resolved by the following decision. Rates,
  horizons, timeout values, QoS, topic names, provisional simulation limits,
  and physical startup behavior remain open.
- This approval does not authorize IDL, controller configuration, package
  scaffolding, or implementation.

## 2026-08-15 — Safety state, handover, lifecycle, and watchdog contract

Status: accepted by the user on 2026-08-15.

Decisions:

- Use eight software safety states independent of ROS lifecycle state:
  `INITIALIZING`, `INACTIVE`, `HOLDING`, `ENABLING`, `MOTION_ENABLED`,
  `STOPPING`, `FAULT_HOLD`, and `SHUTTING_DOWN`.
- Add one guarded `SafetyTransition` action and typed `SafetyStatus` state,
  with the accepted readiness/fault masks and common reason codes `0–30`.
- Require readiness, an explicit trusted enable request, and a fresh source
  activation edge before motion. Never restore prior motion permission after
  startup, reset, source loss, Wi-Fi loss, or process/publisher restart.
- Quarantine stale or invalid sources until a valid release and fresh
  activation edge. Never automatically execute a lower-priority source after
  loss of the selected source.
- Permit deliberate higher-priority preemption only through a controlled-stop,
  verified stable six-foot hold, and hold-dwell barrier; failure ends in
  `HOLDING`.
- Make locomotion updates transactional across all six legs and 24 joints.
  Normal `HOLDING` continuously commands the last validated stable stance;
  controller timeout is a fallback rather than the hold mechanism.
- Latch kinematic, control-component, backend/time, internal, and trusted
  software-hold faults. Quarantine an invalid ordinary source without globally
  latching the robot when the trusted control path remains healthy.
- Use strict Gazebo lifecycle ordering and layered steady-time watchdogs at the
  source, selection, safety, locomotion, controller, and backend boundaries.
- Treat software hold as distinct from an emergency stop. Never assume that
  cutting physical servo power is safe, because the standing robot collapses
  when unpowered.

Rationale: motion permission must be explicit, fail closed, and unable to
resume unexpectedly across source or process discontinuities. Controlled
handover and transactional gait updates preserve deterministic whole-robot
state, while truthful fault reporting avoids claiming safety that open-loop
physical hardware cannot provide.

Consequences:

- The complete accepted contract is maintained in
  `docs/agent/SAFETY_ARCHITECTURE.md`.
- Exact rates, timeouts, priorities, stop profiles, hold dwell, simulator
  limits, QoS, and topic names were later accepted in the runtime/timing
  decision. Physical safety and lifecycle behavior remain later design gates.
- This approval does not authorize IDL, configuration, package scaffolding,
  implementation, or physical actuation.

## 2026-08-15 — Configuration, calibration, and simulator validation architecture

Status: accepted by the user on 2026-08-15.

Decisions:

- Give every model, algorithm, supervision, simulator, controller-composition,
  test, and future hardware-calibration value one owning package. Bringup
  selects and composes owned artifacts rather than maintaining competing
  values.
- Distinguish CAD-supported design facts, canonical model parameters,
  provisional simulator estimates, simulator identification, measured physical
  calibration, and operational policy. None is promoted into another evidence
  class without explicit validation.
- Require schema/version identity, SI/REP-103 conventions, fail-closed static
  validation, reproducible configuration fingerprints, and lifecycle
  reconfiguration plus a fresh enable after any motion-affecting change.
- Use a nested joint-limit hierarchy: canonical model range intersected with a
  verified actuator range for physical deployments and then with the narrower
  operational range. Provisional simulator limits are forbidden in a physical
  profile.
- Keep `gazebo_dev_v0` and `gazebo_ci_v0` behaviorally equivalent and use seed
  `42` in both. CI may differ only in presentation, logging, recording,
  rendering, reporting, and closed input-adapter presence recorded outside the
  behavior fingerprint. Test-only fault injection cannot be selected by normal
  bringup.
- Require seven ordered blocking gates: model/configuration integrity;
  spawn/controller/stable hold; kinematics/standing validity; static body pose;
  tripod locomotion/controlled stop; supervision/fault injection; and a
  reproducible headless baseline.
- Record machine-readable outcomes, source/dependency/configuration identities,
  seeds, physics settings, metrics, and focused failure evidence. Passing the
  gates demonstrates functional simulator behavior, not physical safety or
  sim-to-real fidelity.

Rationale: explicit ownership prevents duplicated or hidden configuration,
while evidence classes prevent simulator estimates from being represented as
physical calibration. Ordered blocking gates make progress and failure
objective before implementation begins.

Consequences:

- The complete accepted contract is maintained in
  `docs/agent/CONFIGURATION_AND_VALIDATION_ARCHITECTURE.md`.
- Exact parameter schemas, values, rates, timeouts, QoS, topic names, test
  tolerances, physical calibration procedures, and implementation mechanisms
  remain later decisions.
- This approval does not authorize configuration creation, test
  implementation, package scaffolding, or physical actuation.

## 2026-08-15 — Two-simulator portfolio

Status: accepted by the user on 2026-08-15.

Decisions:

- Keep the ROS description, frames, limits, and control contracts simulator-neutral.
- Use Gazebo Harmonic locally and in headless CI for fast functional development with ROS 2 Jazzy and `gz_ros2_control`.
- Use Isaac Sim/Isaac Lab for advanced perception, synthetic data, high-fidelity validation, portfolio demonstrations, and reinforcement learning.
- Do not add Webots initially. It is a valid fallback, not a current requirement.
- Treat the physical robot as the eventual sim-to-real authority and compare simulators with bounded invariants rather than identical trajectories.
- Select a mutually supported stable Isaac Sim/Isaac Lab pair when implementation starts. Do not default to Isaac Lab 3.0 Beta while its documentation still warns of missing features and breaking changes.

Rationale: Gazebo has a direct supported pairing with the chosen Ubuntu 24.04/ROS 2 Jazzy environment and can run server-only on the local laptop. That preserves rapid, low-cost iteration while cloud Isaac remains available for workloads that justify its GPU and fidelity. A third simulator would multiply model conversion, tuning, launch, testing, and documentation work without a present unique need.

## 2026-08-15 — RGB-D SLAM and Nav2 boundary

Status: accepted by the user on 2026-08-15; terrain scope and final SLAM implementation remain open.

Decisions:

- Simulate a Gemini-335-like RGB-D camera and IMU in Gazebo, publishing standard ROS 2 sensor messages through `ros_gz`.
- Evaluate RTAB-Map first for RGB-D odometry, six-DoF localization, and 3D mapping.
- Provide Nav2 with the SLAM system's 2D occupancy projection and `map → odom` transform; feed live depth point clouds into a Nav2 voxel obstacle layer.
- Require the first SLAM milestone to produce six-DoF localization, pose graph and loop closure, a saved/reloadable mapping database, a Nav2-compatible 2D occupancy grid, live 3D voxel obstacles, and a downsampled colored 3D point cloud.
- Retain full six-DoF state estimation internally while projecting the pose and map into Nav2's planar navigation representation.
- Defer dense global volumetric maps, textured meshes, and elevation/traversability maps.
- Treat Nav2 as the body-level ground navigator, not as the future foothold or uneven-terrain planner.
- Scope the first autonomous-navigation milestone to flat ground; defer slopes, steps, uneven terrain, and terrain-aware foothold planning.
- Keep the yaw gimbal fixed during initial SLAM and navigation tests.
- Use simulated ground-truth odometry only as a pipeline diagnostic, never as evidence that SLAM or state estimation works.

Rationale: This separates 3D perception/localization from Nav2's principally 2D planning model, establishes realistic state-estimation tests, and avoids making an unmeasured open-loop gimbal transform part of the first SLAM milestone.

Initial acceptance direction:

- Build a recognizable colored 3D map from simulated RGB-D and IMU data on flat ground.
- Detect loop closure and correct drift.
- Publish a valid `map → odom → base_link` transform chain and usable 2D occupancy grid.
- Save, reload, and relocalize against the mapping database.
- Complete Nav2 goals while avoiding obstacles from live depth data.
- Use simulator ground truth only for scoring estimation error, never as an input to a claimed SLAM result.

## 2026-08-15 — RL simulator boundary

Status: accepted for the future RL phase; RL was explicitly deferred on 2026-08-15.

Decisions:

- Use Isaac Lab as the primary high-throughput training environment.
- Use Gazebo for RL environment debugging, small-scale experiments, policy playback, robustness testing, and sim-to-sim validation.
- Define simulator-neutral action, observation, reward, termination, randomization, timing, and unit contracts, with separate Gazebo and Isaac adapters.
- Avoid promising physical deployment of a raw 24-joint policy while the robot lacks measured joint and contact feedback. Prefer high-level gait adaptation or residual control for the first credible sim-to-real RL target.

Rationale: Gazebo has the simulation control primitives needed for RL but lacks first-class GPU vectorization and a mature official Gym workflow. Isaac Lab is designed for parallel GPU training. Cross-simulator evaluation is useful for detecting simulator overfitting, while the current open-loop physical actuator boundary makes low-level learned control difficult to validate and unsafe to overclaim.

## 2026-08-15 — Algorithms before reinforcement learning

Status: accepted by the user on 2026-08-15.

Decision: Defer all RL deliverables. First make the robot system work through deterministic, conventional algorithms, including kinematics, tripod gait generation, body control, teleoperation, state estimation, RGB-D SLAM, and flat-ground Nav2. Revisit RL only after those baselines and their acceptance tests are reliable.

Rationale: A working algorithmic baseline provides debuggable behavior, regression oracles, safety boundaries, measurable performance, and a comparison point for any later learned policy. It also avoids conflating simulator, model, control, and reward-design failures.

Consequences:

- Isaac Lab remains in the planned simulator portfolio but is not on the critical path for initial functionality.
- No initial RL task, reward, observation space, action space, training pipeline, or policy deployment is required.
- Later RL should demonstrate measurable improvement over the algorithmic baseline rather than replace it without comparison.

## 2026-08-15 — Rough initial simulator dynamics

Status: accepted by the user on 2026-08-15.

Decision: Use an explicitly provisional `rough_estimate_v0` for initial Gazebo
development. Preserve the raw Fusion JSON unchanged, replace Steel-derived
servo and electronics contributions with researched or clearly labeled round
masses, and keep the result separate and reproducible. A fully corrected Fusion
material model, enhanced per-body exporter, and physical weighing are deferred
until higher-fidelity or physical validation requires them.

Rationale: the user does not require exact mass properties for the first
simulator milestone. A bounded, traceable estimate is more useful than the
known-invalid `9.804328 kg` all/mixed-Steel result and avoids unnecessary Fusion
rework before the kinematic and control architecture is exercised.

Consequences:

- The central whole-robot estimate is `3.924393 kg`, including `0.576 kg` of
  missing base-electronics proxies.
- Per-occurrence centers of mass are retained and inertia values are uniformly
  scaled by mass ratio; this is not a body-accurate reconstruction.
- Aggregate center of mass and inertia remain unresolved until proxy poses and
  better body-level evidence exist.
- The estimate is simulator-only and cannot be promoted into a physical profile
  or used as hardware-safety evidence.

## 2026-08-15 — Runtime timing, QoS, topics, and simulator values

Status: accepted by the user on 2026-08-15.

Decision:

- Use the concrete project topics, source registry, QoS profiles, dual-clock
  rules, rates, horizons, watchdogs, motion envelopes, provisional simulator
  joint/dynamics values, and Gate 0–6 thresholds in
  `docs/agent/RUNTIME_TIMING_AND_SIMULATION_CONTRACT.md`.
- Use `1000 Hz` Gazebo physics, a synchronous `250 Hz` controller manager,
  `100 Hz` arbitration/safety/locomotion, `50 Hz` teleop, and a `0.040 s`
  one-point trajectory horizon for the initial simulator baseline.
- Use steady time for motion-authority/readiness expiry and ROS simulation time
  for gait/trajectory progression. A Gazebo pause revokes readiness and cannot
  resume prior motion automatically.
- Keep source candidates best-effort/latest-value while selected, safe, and
  controller commands are reliable/latest-value.
- Limit initial simulator motion to the accepted slow envelope and treat all
  joint limits, effort/velocity caps, damping, friction, mass/inertia, and
  controller gains as provisional simulator values only.

Rationale: the chosen hierarchy leaves multiple controller updates per
trajectory and physics steps per controller update, keeps watchdog expiry ahead
of the JTC fallback, prevents ROS-time pauses from extending authority, and
provides objective pass/fail thresholds without pretending the rough dynamics
are physically calibrated.

Consequences:

- The runtime values are now architecture targets and cannot be changed merely
  to make a test pass; revisions require explicit evidence and review.
- Configuration schemas and composition must encode these accepted values
  without introducing duplicate authorities.
- Position-only JTC interpolation remains an acknowledged velocity-continuity
  limitation monitored by Gate 4.
- Acceptance does not authorize IDL, ROS packages, configuration files, Xacro,
  launch code, tests, or physical commands.

## 2026-08-15 — Parameter, artifact, and runtime composition

Status: accepted by the user on 2026-08-15.

Decision:

- Use strict, JSON-compatible package-owned YAML artifacts with a common
  versioned envelope, package-owned schemas, evidence/deployment labels, exact
  dependencies, and fail-closed offline validation.
- Use the C++ or Python form of `generate_parameter_library` for typed project
  node parameters. Motion-affecting values have no silent defaults and are
  immutable for the v0 process lifetime.
- Make deployment profiles exact artifact-selection graphs. Do not permit
  generic deep merge, arbitrary motion parameter overrides, or source-tree
  resource resolution in accepted launch paths.
- Have `araco_bringup` run deterministic preflight before Gazebo, derive joint
  lists and upstream representations from the canonical model registry, and
  emit an immutable per-run bundle plus non-circular configuration identities.
- Require `gazebo_dev_v0` and `gazebo_ci_v0` to resolve the same production
  behavior fingerprint; only closed presentation/reporting choices may differ.
- Follow Jazzy controller integration rules: separate controller-manager and
  controller parameter files, pass each controller file to its spawner with
  `--param-file`, and deliver the robot description through the supported topic
  mechanism.

Rationale: this preserves one authority per value while still producing the
duplicated file shapes required by upstream ROS and Gazebo components. Strict
selection and fingerprints make simulator results reproducible and prevent
hidden launch overrides from becoming unreviewed motion policy.

Consequences:

- Exact artifact paths, profile roles, runtime-bundle contents, validation
  layers, and fingerprint semantics are frozen in
  `docs/agent/PARAMETER_AND_CONFIGURATION_COMPOSITION.md`.
- A motion-affecting change requires controlled hold, lifecycle deactivation,
  process replacement with a newly composed bundle, readiness revalidation,
  and a fresh enable/source edge.
- Acceptance does not authorize schemas, YAML, Xacro, launch code, ROS package
  scaffolding, tests, or physical commands.

## 2026-08-15 — Phased simulator delivery plan

Status: accepted by the user on 2026-08-15.

Decision:

- Use one non-gate repository-foundation phase followed by one blocking
  implementation phase for each accepted Gazebo Gate 0–6.
- Gate 1 establishes the real simulator/controller/lifecycle hold path using
  the single accepted nominal standing reference. Gate 2 replaces transitional
  target production with computed FK/IK while retaining that reference as the
  validation oracle.
- Gate 3 onward scores commands only through the production system-test
  candidate → arbitration → safety → locomotion → controller → Gazebo path.
- Write tests with each increment, rerun every prior gate at a phase exit, and
  retain typed evidence for successes and failures. Required gates have no
  retry, expected-failure, easier-CI, or silent-threshold-relaxation escape.
- Use narrow test-only dependency injection for impossible internal fault
  branches without adding production fault backdoors.
- Treat Gate 6's three clean headless no-retry runs as the functional Gazebo
  baseline that unlocks later simulator work, not physical deployment.

Rationale: the order proves model/configuration, plant/control ownership,
kinematics, body behavior, gait, supervision, and reproducibility separately so
a visually plausible later behavior cannot conceal a broken earlier contract.

Consequences:

- Package maturation, phase deliverables, gate exit boundaries, evidence,
  regression invalidation, failure classification, and handoff rules are frozen
  in `docs/agent/PHASED_DELIVERY_PLAN.md`.
- An affected change invalidates its earliest gate and all later evidence.
- Acceptance completes the planned simulator architecture sequence but does not
  authorize Phase 0, `src/`, package scaffolding, implementation, commits,
  external CI mutation, publishing, or physical commands.

## 2026-08-15 — Architecture closeout and repository/package license

Status: **superseded on 2026-08-16 by the GPL-3.0-only decision below**. Retained
as decision history; Apache-2.0 was never applied through a root `LICENSE` or
package manifest.

Decision:

- The final cross-contract review passes after the reconciliations recorded in
  `docs/agent/FINAL_ARCHITECTURE_REVIEW.md`.
- License project-authored code, configuration, documentation, tests, and
  original assets under the Apache License 2.0 using SPDX identifier
  `Apache-2.0`.
- During authorized Phase 0, add the unmodified full license text at the root
  and in every package, use
  `<license file="LICENSE">Apache-2.0</license>` in each initial
  `package.xml`, and add SPDX source headers where supported.
- Do not treat the repository license as permission to redistribute vendor CAD
  or other third-party assets. Bundled resources require explicit creator,
  source, license/attribution, modification, and redistribution metadata;
  unknown-rights assets are excluded or replaced with project-authored
  simplified proxies.
- Add `NOTICE` only when included content or attribution actually requires it.

Rationale: Apache-2.0 is permissive and public-portfolio friendly while adding
an explicit contributor patent grant absent from simpler permissive choices.
The SPDX identity is unambiguous for source and ROS package metadata. Separate
asset provenance prevents detailed imported component models from being
silently relicensed.

Consequences:

- License selection is no longer a pre-scaffolding open decision.
- The architecture is ready for a separate explicit Phase 0 authorization.
- This decision does not itself create `LICENSE`, `src/`, package files, code,
  commits, external CI, or a public release.
- A public maintainer name/email must be confirmed before package manifests are
  written.

## 2026-08-16 — License changed to GNU GPL version 3 only

Status: **superseded later on 2026-08-16 by the MIT decision below**. Retained
as decision and published-checkpoint history.

Decision:

- Supersede the planned Apache-2.0 repository/package license with the GNU
  General Public License version 3 only, SPDX identifier `GPL-3.0-only`.
- Interpret “GPL-3.0” as version 3 specifically, not the distinct
  `GPL-3.0-or-later` grant. The deprecated SPDX identifier `GPL-3.0` is not used.
- During authorized Phase 0, add the unmodified official GPLv3 text at the root
  and in every package, use
  `<license file="LICENSE">GPL-3.0-only</license>` in each initial
  `package.xml`, and add `SPDX-License-Identifier: GPL-3.0-only` to
  project-authored source where supported.
- Audit direct linked and bundled dependencies for GPLv3 compatibility before
  Phase 0 completes. Preserve third-party licenses and attributions and plan
  for the applicable GPLv3 Corresponding Source obligations of distributed
  object-code/combined works.
- Include the preferred editable source and generation tooling when distributing
  covered generated mesh/model forms. Review the existing Fusion add-in's
  Autodesk API boundary and the future Isaac adapter's proprietary SDK boundary
  rather than presuming they form GPL-compatible distributable combinations.
- Continue excluding vendor CAD and other third-party assets with unknown
  redistribution rights; the GPL selection cannot relicense them.

Rationale: the user explicitly prefers GPLv3's strong-copyleft terms over the
previously selected permissive license. The `-only` SPDX form precisely encodes
version 3 without granting automatic use under a future GPL version.

Consequences:

- The current local repository still has no root `LICENSE`, `src/`, or package
  manifests, so this is a pre-application change rather than a relicense of a
  published GPL/Apache release.
- Distributed covered modifications and combined works must comply with GPLv3,
  including applicable Corresponding Source requirements. Private use and
  modification without distribution do not require public source release.
- GitHub has no repository license setting that must be changed. After an
  authorized Phase 0 adds, commits, and pushes the root `LICENSE` to the default
  branch, GitHub should detect it from the repository contents.
- This decision updates documentation only and does not authorize Phase 0,
  commits, pushes, publication, or any GitHub mutation.

## 2026-08-16 — Licensed architecture checkpoint before Phase 0

Status: authorized explicitly by the user on 2026-08-16.

Decision:

- Add the unmodified official GNU GPL version 3 text at root `LICENSE` before
  Phase 0.
- Commit the accumulated architecture, continuity, Fusion exporter
  documentation, and rough-dynamics evidence as one coherent checkpoint.
- Push that checkpoint to `origin/main`.
- Do not treat this checkpoint authorization as Phase 0 authorization; do not
  create `src/`, ROS package skeletons, package manifests, package-local
  license copies, or source SPDX headers yet.

Rationale: establish a recoverable, remotely backed-up architecture baseline
under the already selected license before repository scaffolding begins.

Consequences:

- Root `LICENSE` now carries the GPLv3 full text. Package-local license copies,
  manifest declarations, source headers, and compatibility/source-obligation
  audit remain Phase 0 deliverables.
- The checkpoint may be committed and pushed despite the earlier general
  no-commit/no-push boundary because the user granted specific authorization
  for this checkpoint.

## 2026-08-16 — Phase 0 repository foundation authorized

Status: authorized explicitly by the user on 2026-08-16.

Decision:

- Begin only the Phase 0 scope frozen in
  `docs/agent/FINAL_ARCHITECTURE_REVIEW.md` and
  `docs/agent/PHASED_DELIVERY_PLAN.md`.
- Create the nine package skeletons, accepted seven messages and one action,
  package metadata/build/install/test structure, package-local license copies,
  root workspace hygiene and build README, and the required Phase 0 licensing
  and dependency validation evidence. The then-current GPLv3 license portion
  was superseded later the same day by the MIT decision below.
- Do not begin Phase 1 model/configuration authoring, Gazebo runtime work,
  motion-capable nodes, physical profiles, servo/UART integration, hardware
  commands, commits, pushes, releases, or hosted CI changes.

Rationale: the licensed architecture checkpoint is complete and the user has
now crossed the separately defined implementation authorization boundary.

Consequences:

- Phase 0 may claim repository integrity only and must explicitly claim no
  Gazebo gate.
- Public package manifests remain blocked until the user confirms the
  maintainer name and email intended for publication.

## 2026-08-16 — License changed to MIT and maintainer identity confirmed

Status: accepted explicitly by the user on 2026-08-16.

Decision:

- Supersede the earlier Apache-2.0 plan and GPL-3.0-only application with the
  MIT License, SPDX identifier `MIT`, for project-authored repository and ROS
  package content.
- Use `Copyright (c) 2026 Araco Hexapod contributors` in the MIT full text and
  `SPDX-License-Identifier: MIT` in project-authored source files whose formats
  support comments.
- Use `<license file="LICENSE">MIT</license>` and the exact package-local MIT
  text in every initial package.
- Publish `stevw <steven060520@gmail.com>` as the maintainer in all nine
  `package.xml` manifests, as explicitly authorized by the user.
- Continue preserving third-party licenses, attributions, and redistribution
  restrictions. MIT does not relicense Autodesk Fusion/API materials, vendor
  CAD, ROS/Gazebo dependencies, or future proprietary SDKs.

Rationale: GPLv3 was workable but added avoidable friction for the project's
planned proprietary Fusion/Isaac integration boundaries and did not fit the
Jazzy copyright-lint template without special handling. MIT is a standard
permissive open-source license that preserves attribution and warranty
disclaimers while allowing broader integration. “No license” was rejected
because default copyright would not grant the reuse, modification, and
distribution permissions expected for this public repository.

Consequences:

- The Phase 0 license audit checks notices and redistribution terms rather than
  GPL compatibility/Corresponding Source for combined works.
- The prior GPL checkpoint remains in Git history as historical licensing; the
  same rights holder has authorized the current MIT relicense. No commit or
  push of Phase 0 is authorized by this decision.
- The public-maintainer input is resolved and package manifests may be written.

## 2026-08-16 — MIT Phase 0 checkpoint commit and push

Status: authorized explicitly by the user on 2026-08-16.

Decision:

- Commit the completed and validated Phase 0 repository foundation under MIT.
- Push the checkpoint to `origin/main` using the configured Git identity
  `stevw <steven060520@gmail.com>`.
- Keep generated `build/`, `install/`, `log/`, and cache artifacts untracked.
- Do not infer authorization for Phase 1 from this checkpoint operation.

Rationale: preserve the clean Phase 0 result as a recoverable remote baseline
before any Gate 0 model/configuration implementation begins.

Consequences:

- The remote default branch will move beyond the historical GPL pre-Phase-0
  checkpoint and contain the current MIT license and package foundation.
- The prior GPL commit remains in history; no history rewrite is authorized or
  required.
