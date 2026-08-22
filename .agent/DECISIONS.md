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
  `.agent/REPOSITORY_ARCHITECTURE.md` are now accepted architecture.
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

- Exact fields and semantics in `.agent/INTERFACE_CONTRACTS.md` are accepted
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
  semantics are maintained in `.agent/INTERFACE_CONTRACTS.md`.
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
  `.agent/SAFETY_ARCHITECTURE.md`.
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
  `.agent/CONFIGURATION_AND_VALIDATION_ARCHITECTURE.md`.
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
  `.agent/RUNTIME_TIMING_AND_SIMULATION_CONTRACT.md`.
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

Correction recorded 2026-08-16:

- The joint-state broadcaster rate is `125 Hz`, not `100 Hz`. Jazzy requires a
  controller rate below the `250 Hz` manager rate to be an integer divisor for
  constant-period updates; the live Harmonic run reduced the impossible
  `100 Hz` request to `83 Hz` and warned about non-constant periods. The
  `100 Hz` safety, arbitration, and locomotion rates remain unchanged.

- Jazzy `gz_ros2_control` leaves the hardware-component `type` and
  `plugin_name` service fields empty. Runtime backend readiness therefore
  requires exactly one active, synchronous `GazeboSimSystem`, the exact 25
  available-and-claimed position command interfaces, and the exact 75
  available position/velocity/effort state interfaces. Plugin class and gain
  identity remain enforced by the integrity-checked composed URDF and Gazebo
  startup validation; an empty service field is not treated as plugin evidence.

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
  `.agent/PARAMETER_AND_CONFIGURATION_COMPOSITION.md`.
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
  in `.agent/PHASED_DELIVERY_PLAN.md`.
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
  `.agent/FINAL_ARCHITECTURE_REVIEW.md`.
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

Status: authorized and completed on 2026-08-16.

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
  `.agent/FINAL_ARCHITECTURE_REVIEW.md` and
  `.agent/PHASED_DELIVERY_PLAN.md`.
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

Status: authorized and completed on 2026-08-16.

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
- The Phase 0 implementation checkpoint is
  `2a1b3dcd2545b95570c3b8428b0dabfac37f6f95`; it was pushed to `origin/main`
  and verified against the remote branch.

## 2026-08-16 — Phase 1 / Gate 0 authorized

Status: authorized explicitly by the user on 2026-08-16.

Decision:

- Implement only Phase 1 / Gate 0 model-and-configuration integrity as frozen
  in `.agent/PHASED_DELIVERY_PLAN.md`.
- Create the package-owned strict artifacts and schemas, canonical description,
  reproducible generated integration forms, deterministic preflight/runtime
  bundle, and all blocking static/negative/installed-space validation.
- Keep the canonical model registry as the sole numeric topology authority;
  Xacro/URDF, visual meshes, collision forms, controller partitions, and
  Gazebo mappings are derived or rendering forms.
- Do not start Gazebo, claim Gate 1, implement motion-capable nodes, introduce
  physical profiles, command hardware, commit, or push without separate
  authorization.

Rationale: Phase 0 is remotely checkpointed and Gate 0 is the next accepted
blocking milestone before any live simulator process is treated as valid.

## 2026-08-16 — Phase 1 / Gate 0 completed

Status: implemented and validated on 2026-08-16; uncommitted and unpushed.

Decision:

- Accept the 20 package-owned artifacts and their exact development/CI profile
  graphs as the first static simulator configuration baseline.
- Accept the canonical registry as the only topology/transform/role authority;
  the normalized URDF, controller joint lists, Gazebo mappings, and primitive
  visual forms are generated from it.
- Accept `rough_estimate_v0` at `3.924392774795984 kg` with explicit PiSugar,
  main-battery, and servo-controller base proxies and positive-valid aggregate
  inertia, strictly as `simulator_estimate` evidence.
- Replace the over-broad base collision enclosure with a project-authored
  `0.19 × 0.15 × 0.05 m` proxy. The earlier enclosure falsely intersected four
  femur proxies; mass/inertia remain independently owned by the dynamics
  artifact.
- Declare Gate 0 passed from static and installed-space evidence. Do not infer
  Gate 1, live-spawn, stable-hold, motion, or hardware readiness.

Rationale: clean build/test, strict negative fixtures, deterministic repeated
composition, exact equal behavior fingerprints, resource/inertia/collision
checks, and independent URDF parsing all pass without starting Gazebo.

Consequences:

- The next technical packet is Phase 2 / Gate 1 launch, spawn, controller
  ownership, and stable hold, but it remains unauthorized.
- All limits, standing targets, collision fidelity, camera transforms, gains,
  and dynamics remain provisional until later simulator and physical evidence.

## 2026-08-16 — Phase 2 / Gate 1 authorized

Status: authorized explicitly by the user on 2026-08-16; implementation is in
progress and remains uncommitted and unpushed.

Decision:

- Implement only Phase 2 / Gate 1 spawn, controller ownership, readiness
  sequencing, and stable nominal hold as frozen in
  `.agent/PHASED_DELIVERY_PLAN.md`.
- Use the Gate 0 canonical model and generated configuration as the source for
  the Gazebo backend overlay, controller partitions, nominal 24-leg-joint hold,
  and zero gimbal target.
- Permit only the startup bootstrap needed to establish controller hold and
  reach software safety state `HOLDING`; Gate 1 must never request or enter
  `MOTION_ENABLED`.
- Do not implement computed IK, gait, body-pose motion, physical profiles,
  hardware actuation, Phase 3 or later work, or claim physical safety.
- Do not commit or push without a separate explicit user instruction.

Rationale: Gate 0 passed and the user explicitly advanced the project to
Phase 2. A live, scored non-moving simulator hold is the next blocking proof
before computed kinematics or walking may be trusted.

## 2026-08-16 — Correct Jazzy JTC interpolation enum

Status: implementation correction discovered during Phase 2 installed-API
audit; Gate 0 regression validation is required.

Decision:

- Replace generated JTC `interpolation_method: linear` with the Jazzy-supported
  enum value `splines` in the controller artifact and owner schema.
- Retain positions-only one-point trajectories. Jazzy's variable-degree spline
  interpolation reduces these inputs to linear position interpolation, so the
  accepted behavioral intent is unchanged.
- Regenerate Gate 0 fingerprints and evidence; do not preserve the earlier
  fingerprints as though the effective controller input were valid.

Rationale: installed `joint_trajectory_controller` 4.40.1 exposes only `none`
and `splines`. The earlier value described the resulting interpolation behavior
but was not a valid upstream parameter enum.

## 2026-08-16 — Gate 1 simulator geometry and contact corrections

Status: accepted from live Gate 1 measurement; physical direction remains open.

Decision:

- Correct every femur, tibia, and foot joint axis from the proposed local `+Y`
  to local `-Y`, because the former placed the tibias below the feet in the
  accepted standing reference.
- Set the Gate 1 nominal simulator spawn height to `0.10675 m`, derived from
  live model/contact measurement rather than retaining the legacy algorithm's
  provisional `0.080 m` body-height value as a spawn coordinate.
- Use one 50 Hz Gazebo contact-sensor topic per foot and one shared non-foot
  alarm topic, with a 100 Hz ROS aggregate. Reference the SDF-converted
  `<link>_collision_collision` names and put each transport topic inside the
  sensor's `<contact>` element as required by Harmonic.

Rationale: live Gazebo inspection showed the proposed pitch-axis sign and
contact-topic assumptions were wrong. The corrected model reaches its intended
stationary pose with all six feet in contact and no non-foot collision. These
values remain simulator evidence; they do not establish physical servo zero,
direction, or safe limits.

## 2026-08-16 — Phase 2 / Gate 1 completed

Status: implemented and validated; uncommitted and unpushed.

Decision:

- Declare Gate 1 passed from the consecutive Gate 0 regression and atomic live
  Gate 1 evidence in `log/gate_0_20260816_phase2_regression/` and
  `log/gate_1_20260816_phase2_hold_pass/`.
- Accept the hold-only locomotion, exact 24+1 controller/backend ownership,
  ordered lifecycle readiness, typed fail-closed safety action, independent
  contact streams, preflight negative fixtures, scorer, and atomic evidence
  harness as the Gate 1 implementation baseline.
- Preserve `log/gate_1_20260816_phase2_hold/` as failed audit evidence: it
  exposed the unquoted workspace path and timeout-log bytes defect that were
  corrected before the passing evidence run.
- At this checkpoint, keep Phase 3 / Gate 2 and later phases unauthorized. This
  authorization boundary was superseded by the later Phase 3 authorization;
  the prohibition on `MOTION_ENABLED` and hardware actuation remains active.

Rationale: the passing run reached `HOLDING` in `15.340814665 s`, never entered
motion or fault states, held exact controller ownership, maintained all six
foot contacts with no non-foot contact, and passed every accepted error,
velocity, pose, and penetration threshold. A clean rebuild/test produced 175
tests with 0 errors and 0 failures; dependency, URDF, SDF, and independent
installed-composition fingerprint checks also pass.

Consequences:

- The accepted behavior fingerprint is
  `c1c2b51d4e082bcfab0e5d09618566c3a9245ce79d46833295c1eb9ec7922283`;
  the accepted CI run fingerprint is
  `e435b0244f9412f9b88e693f247d6f5f0773cc80c4b02842eebdff9287a4b35b`.
- Phase 3 may replace only the transitional target producer with computed
  standing FK/IK after separate authorization, retaining the Gate 1 scorer and
  nominal standing artifact as regression oracle.

## 2026-08-16 — Fusion visual-mesh rights and exporter-first workflow

Status: explicitly accepted by the user; exporter/import/normalization/runtime
integration implemented and validated.

Decision:

- Treat Fusion mechanical components named `araco - ...` as user-owned project
  designs whose derived visual meshes may be published under MIT.
- Do not infer the same permission for embedded servo, Raspberry Pi, Gemini
  335, Raspberry Pi Camera, battery, controller, or other vendor CAD; retain
  project-authored proxies unless separate redistribution rights are verified.
- Extend the read-only Fusion exporter with an explicit reviewed body/component
  whitelist and provenance manifest rather than converting the complete
  mixed-rights STEP assembly.
- Normalize approved exports into canonical ROS link-local visual meshes while
  leaving collision geometry, dynamics, joints, controllers, and safety
  behavior unchanged.
- Preserve the current proxy visual set for rollback, make Gazebo headed
  resource discovery self-contained, then rerun Gates 0 and 1 because visual
  resource hashes and configuration fingerprints will change.

Rationale: an exporter-side whitelist uses the authoritative Fusion design and
prevents hundreds of detailed vendor solids from being accidentally published.
It also provides stronger source/version/body provenance than brittle filtering
of the 98.8 MB AP214 STEP file on Ubuntu.

Implementation boundary:

- The reviewed allowlist contains 25 exact PETG body selections: six base
  fragments, one gimbal body, and coxa/femur/foot bodies for all six legs.
- All six tibia links retain proxies because the current Fusion inventory marks
  all four bodies in each tibia occurrence as Steel and cannot safely establish
  which geometry is project-owned printed structure versus embedded servo CAD.
- Fusion `0.2.0` actually emitted source-component-local STL in millimetres,
  with the occurrence-to-root transform recorded separately. Byte-identical
  meshes for repeated instances plus distinct occurrence transforms established
  this interpretation. Exporter `0.2.1` corrects the coordinate declaration.
- The Fusion version 2 bundle contains all 25 expected exports. Ubuntu-side
  validation deduplicates them to 12 immutable source blobs and normalization
  emits 20 deterministic ROS link-local meshes; the six tibias remain proxies.
- The runtime uses detailed meshes only for visual geometry. Collision,
  dynamics, joint, controller, safety, and standing contracts are unchanged.
- Headed resource lookup is self-contained and fresh Gate 0 / Gate 1 evidence
  passes. Gate 1 physics metrics match the prior proxy-visual baseline within
  numerical precision.

## 2026-08-16 — Publishable tibia/servo proxy rendering

Status: implemented and awaiting user visual acceptance; uncommitted and
unpushed.

Decision:

- Do not move or force-align any canonical joint, link frame, collision shape,
  inertial property, controller, or standing target to compensate for the
  observed Fusion tibia-frame offset.
- Replace each rectangular tibia visual slab with a project-authored two-rail
  visual proxy centered on the existing canonical joint-to-joint line. Keep
  the original tibia collision box unchanged.
- Render project-authored box-and-cylinder servo proxies for the known
  inventory: 19 DS3235 and six DS5160. Derive their link ownership from the
  canonical joints and explicit segment rules; label all dimensions and poses
  `simulator_estimate` rather than exact vendor CAD.
- Use contrasting printed-part, servo-case, and servo-horn materials and add an
  explicit directional world light, ambient light, and shadows.
- Continue excluding exact vendor servo CAD unless redistribution rights are
  established separately.

Rationale: the visual offset is not evidence that the accepted kinematic frame
is wrong. Changing the frame to make an approximate mesh look aligned would
corrupt control and physics. Auditable project-authored proxies make the tibias
and actuators readable in Gazebo without importing third-party geometry or
changing robot behavior.

## 2026-08-16 — Exact Fusion tibia and servo presentation visuals

Status: implemented and validated; awaiting user visual acceptance; supersedes
the proxy-rendering decision for visual geometry only.

Decision:

- Accept the user's confirmation that all CAD they possess in the Fusion
  assembly is open source and may be used for the simulator. Preserve
  per-asset licensing rather than claiming that upstream vendor CAD becomes
  MIT merely by inclusion in this repository.
- Replace the six two-rail tibia visuals and all 25 box/cylinder servo visuals
  with exact high-refinement Fusion body meshes. Presentation-quality recorded
  simulator output is a requirement, so visual fidelity takes priority over
  the smaller proxy asset set.
- Use exporter `0.3.1` and specification `2.1.0`, with 44 mesh exports covering
  exactly 62 reviewed bodies: 25 existing project mechanical bodies, 13
  directly attached servo bodies, and one complete four-body component export
  for each of the six tibia occurrences. Retain no visual proxy. The
  occurrence/component packaging supersedes individual tibia-body export
  because Fusion accepted `right_middle_tibia/Body2` as solid but silently
  produced no STL for it.
- Keep collision geometry, inertia, joints, controllers, safety behavior, and
  the nominal standing target unchanged. Preserve the actual Fusion occurrence
  transforms; do not force-align the tibia visual to the canonical centerline.
- Keep unrelated Raspberry Pi, battery, controller, camera, and nested sensor
  internals out of this bounded export. They can be handled as a separate
  presentation enhancement if requested.
- Classify the reproducible connected solids inside each complete tibia STL
  fail-closed: one printed tibia, two servo cases, and two horns. Also separate
  DS3235 case/horn shells where topology permits; retain each connected DS5160
  exactly as exported. Remove only 36 zero-area source triangles, which have no
  rendered area, during deterministic normalization.
- Compose 46 link-local mesh visuals (26 primary, 13 servo-case, seven
  servo-horn) and remove every prior visual box/cylinder/tibia proxy. Keep the
  three role materials for presentation readability.

Rationale: the previous proxies solved missing geometry but do not meet the
user's presentation requirement. The current Fusion inventory identifies every
required direct occurrence/body pair, so exact geometry can be exported without
weak name matching or changes to simulation behavior.

## 2026-08-16 — Exact Gemini 335 exterior presentation visual

Status: implemented, statically validated, and running in headed Gazebo for
user visual acceptance; uncommitted and unpushed.

Decision:

- Add the exact vendor Gemini 335 exterior to the presentation model on
  `camera_link`; it rotates with `gimbal_yaw_link` through the existing fixed
  frame relationship.
- Export 15 explicitly reviewed nested bodies: five housing/bracket bodies,
  six pads/fasteners, and four externally visible optical bodies.
- Exclude the eight-body internal connector/PCB assembly, the independently
  installed Raspberry Pi Camera Module 3, and all other unlisted electronics.
- Keep the existing simulated camera pose and optical frames, collision,
  estimated mass/inertia, and sensor configuration unchanged. The new CAD is
  visual-only and must not be treated as sensor-extrinsic calibration.
- Preserve separate camera-body, camera-hardware, and camera-optics roles so
  presentation materials can make the exact solid geometry readable.

Rationale: the user requires an attractive recorded simulator presentation and
confirmed that the available CAD is open source. Body-level allowlisting gives
the needed exterior fidelity without publishing internal PCB geometry or
changing simulation behavior.

Implementation evidence:

- Fusion exporter `0.4.0` / specification `3.0.0` produced a valid 59-export,
  77-reviewed-body bundle with the 15 allowlisted Gemini exterior bodies.
- The normalized presentation set contains three `camera_link` meshes for body,
  hardware, and optics, bringing the complete model to 49 exact mesh visuals
  and 2,066,740 triangles with zero output degenerates.
- A fresh copied install built all nine packages; 182 tests passed with zero
  errors or failures and three expected `cppcheck` skips. `check_urdf` and
  `gz sdf -k` pass.
- Headed Gazebo reached `HOLDING` without motion enablement using the new camera
  visuals.

## 2026-08-16 — Phase 3 / Gate 2 authorization

Status: implemented and validated. Its original Phase 4 authorization boundary
was superseded by the separate Phase 4 decision recorded below.

Decision:

- Begin the accepted Phase 3 scope: pure typed deterministic FK/IK, explicit
  branch/reachability/singularity/finite/limit results, canonical-description
  geometry adaptation, and transactional six-leg standing target generation.
- Replace only the transitional Gate 1 direct target producer. Retain the
  nominal standing artifact as the branch/regression oracle and reuse the Gate
  1 simulator scorer.
- Require Gates 0 and 1 to regress cleanly before Gate 2 can pass. Do not enable
  body-pose commands, gait motion, physical actuation, or Phase 4 behavior.

Rationale: the user explicitly requested moving to the next phase after the
completed Phase 1/2 and detailed-visual checkpoint was committed and pushed as
`7b44c96`.

Implementation evidence:

- A fresh installed-space build at `/tmp/araco_gate2_final.n0smqm/` built all
  nine packages and passed 213 tests with zero failures or errors and nine
  expected `cppcheck` skips.
- The pure solver passed 40,000 seeded reachable round trips plus explicit
  branch, boundary, singular, unreachable, non-finite, invalid-configuration,
  and limit cases. Whole-body tests prove complete ordered output and rejection
  without partial commit.
- Gate 0 regression, Gate 1 live regression, and Gate 2 computed standing pass
  at `log/gate_0_20260816_phase3_regression/`,
  `log/gate_1_20260816_phase3_regression/`, and
  `log/gate_2_20260816_computed_standing/` respectively. They share behavior
  fingerprint
  `f656ab6f81655d32e3d386d856c56baed64d5d0dfaf3af2c6cc9e7290c0f653c`.
- Gate 2 proves the analytic solver and all typed six-leg inputs were selected,
  the transaction committed, all six foot contacts remained valid, and no
  motion-enabled or execute state occurred. Maximum computed-to-oracle error
  was `3.27e-7 rad`.

## 2026-08-16 — Phase 4 / Gate 3 authorization

Status: implemented and validated; Phase 5 and later remain unauthorized.

Decision:

- Implement the accepted simulator-only static body-pose scope: absolute body
  X/Y/Z offsets and roll/pitch/posture-yaw while the six nominal feet remain
  fixed in the ground/body transform calculation.
- Exercise the real system-test candidate → arbiter → safety → locomotion →
  controller → Gazebo path, including release, trusted enable, and a fresh
  activation edge.
- Enforce command envelopes and final joint limits transactionally, keep
  `GAIT_STAND` phase fixed, and keep production nodes isolated from simulation
  ground truth.
- Require Gates 0–2 to regress cleanly before Gate 3 may pass. Do not implement
  walking, controlled gait transitions, physical actuation, or Phase 5 work.

Rationale: the user explicitly requested the next phase after reviewing the
completed Gate 2 capabilities.

Outcome:

- Gates 0–3 pass on 2026-08-16. Gate 3 passes all 14 accepted zero, ±50%
  single-axis, and combined-35% cases through the production command path.
- The exact Fusion/vendor foot visuals remain unchanged. The physical collision
  approximation is a 4 mm-radius sphere at each kinematic foot tip. Earlier
  oriented box proxies caused artificial contact loss during roll and pitch;
  an orientation-independent point contact matches the current planted-foot
  kinematic abstraction without claiming measured physical geometry.
- Phase 3/4 remain uncommitted and unpushed until the user separately requests
  a checkpoint.

## 2026-08-16 — Phase 5 / Gate 4 authorization

Status: implemented and validated; Phase 6 and physical deployment remain
unauthorized.

Decision:

- Implement the accepted simulator-only deterministic tripod gait and
  controlled-stop scope through the production command, safety, locomotion,
  controller, and Gazebo path.
- Cover bounded forward, reverse, lateral, yaw, and combined motion; ordinary
  manual hold and source loss; planned return to six-foot stance; stable-hold
  dwell; and no-surprise resume.
- Require Gates 0–3 to remain green before Gate 4 may pass. Full fault,
  restart, and handover matrices remain Phase 6 work.
- Preserve the uncommitted Phase 3/4 work and do not commit or push without a
  separate user request.

Rationale: after the successful headed static body-pose demonstration, the user
explicitly requested the next phase.

Outcome:

- Gates 0–4 pass on 2026-08-16 with behavior fingerprint
  `4f5d37e91c937543fae18dc76793b57eb58adabacba3c72eba91fd1677f14dc8`.
- Gate 4 passes all seven accepted forward/reverse/lateral/yaw/combined cases
  with five complete cycles each, minimum three support contacts, full support
  contact duty, controlled stops below `0.892 s`, manual hold, and active
  `GAIT_STAND`.
- A separate `gazebo_dev_v0` keyboard-adapter smoke test entered motion,
  advanced the tripod gait cycle, and returned through controlled stopping to
  holding when input expired.
- Phase 3–5 remain uncommitted and unpushed pending explicit checkpoint
  authorization.

## 2026-08-16 — Versioned Gazebo position-response revision

Status: accepted and validated under the Phase 5 no-silent-tuning rule; final
Gates 0–4 regression passes.

Decision:

- Retain `gz_ros2_control_v0.yaml` as historical version `0.1.0` with gain
  `0.040`.
- Add `gz_ros2_control_v1.yaml` as version `0.2.0` with simulator-only position
  proportional gain `0.100`, and select v1 consistently in the development,
  CI, Gate 3, and Gate 4 profiles.
- Do not change the Gate 4 contact, tracking, or stop thresholds. Re-run every
  gate through Gate 4 with the revised behavior fingerprint.

Rationale:

- Phase 5 contact evidence repeatedly showed the outgoing tripod retained all
  three contacts while the intended incoming tripod retained zero at phases
  `0.050–0.067` after each handover. This matched the documented approximate
  `0.100 s` first-order response of gain `0.040` and could not satisfy the
  frozen `0.050 s` transition exclusion.
- Gain `0.100` gives an approximate `0.040 s` simulator response at the
  accepted 250 Hz controller-manager rate. This addresses the incompatible
  simulator response rather than weakening the evidence or falsifying gait
  phase. It remains a provisional simulator value and supplies no physical
  servo safety evidence.

Outcome:

- The unchanged Gate 4 thresholds pass for all seven direction cases. Support
  contact duty is `1.0`, swing contact duty is `0.08`, and worst joint tracking
  error is `0.04256 rad`.
- The gain remains explicitly provisional and simulator-only.

## 2026-08-16 — Legacy foot-path curve with phase-0.75 continuity fix

Status: accepted by the user, implemented, and validated through Gates 0–4.

Decision:

- Replace the initial generic smoothstep horizontal swing and piecewise-linear
  lift/touchdown shapes with an independent C++ expression of the legacy
  function-defined horizontal and vertical foot paths.
- Normalize the legacy functions to the accepted configurable 60 mm maximum
  stride and 30 mm swing clearance, and phase-shift them onto the new
  state-machine swing interval without changing tripod membership or handover
  timing.
- Correct the legacy horizontal branch at phase `0.75`: retain the `+0.5`
  plateau value at the boundary, then decrease linearly to zero at phase
  `1.0`. This makes position continuous and preserves the constant
  supporting-foot slope across the cycle wrap.
- Preserve the new architecture, velocity shaping, transactional IK, joint
  limits, runtime joint-rate limiter, controlled stop, stable hold, and safety
  state machine. Do not port the legacy counter/controller architecture.
- Version the gait artifact as `0.2.0` with identity
  `tripod_slow_legacy_curve_phase075_fix`. Extend only Gate 4's observation
  duration from `6.2 s` to `7.6 s`; keep every physical, contact, tracking,
  stop, and scored-window threshold unchanged.

Rationale:

- The user judged the function-defined legacy curve smoother in practice and
  rejected a proposed fully smoothed horizontal replacement because its
  support-foot speed varied and appeared to pause near mid-cycle.
- The raw legacy polynomial is sharper than the per-update joint-motion cap at
  some samples. Retaining the production rate limiter is safer than weakening
  it or distorting the approved path merely to match the nominal phase clock.
  The longer evidence observation window measures the resulting five complete
  cycles without changing the acceptance limits.

Outcome:

- A clean nine-package build and 248 tests pass with zero failures.
- Gates 0–4 pass with behavior fingerprint
  `fa45b732e967a4f780e297c56b2c736b09d518dc3a10001dbe45160ae1664de9`.
- All seven Gate 4 direction cases complete five cycles, retain at least three
  support contacts with `1.0` support-contact duty, avoid non-foot contact,
  and return to a stable six-foot hold. Worst joint tracking error is
  `0.04508 rad`; controlled-stop time is at most `1.478 s`.
- The curve source records its behavioral provenance from the project's
  Apache-2.0 legacy `algo.py`; no legacy Python source lines were copied.

## 2026-08-16 — Precision-biased speed scheduling direction

Status: accepted by the user, implemented, and validated through Gates 0–4.

Decision:

- Keep the ROS locomotion control loop fixed at 100 Hz and keep gait cadence
  and stride as separate, observable parameters owned by one scheduler.
- Prioritize responsive teleoperation and fine gait-level displacement: while
  moving at low and normal speeds, retain a relatively high cadence in a narrow
  tested range and use short stride length as the primary speed variable.
- Increase cadence further only when higher requested velocity would otherwise
  require excessive stride; retain the existing joint-rate shaper as final
  authority and saturate infeasible velocity requests.
- Apply the same normalized amplitude factor `a` to horizontal stride and swing
  clearance: `S=a*S_max`, `H=a*H_max`. At zero command enter stand. Validate an
  active-command deadband so the smallest nonzero proportional clearance still
  clears the ground rather than adding an unapproved independent lift floor.

Rationale:

- The user prefers the shorter displacement per gait cycle and faster arrival
  of safe tripod boundaries because these make tiny teleoperated moves and
  command release more responsive.
- Keeping cadence separate retains headroom for geometric stride limits and
  measured joint/contact constraints without coupling gait speed to the ROS
  callback frequency.

Consequences:

- The earlier proposal to slow cadence substantially at low speed while
  retaining a non-tiny stride is superseded.
- Exact physical displacement will still be limited by servo resolution,
  backlash, compliance, contact, and slip; a later odometry-closed finite-nudge
  command is the stronger solution for guaranteed millimetre-scale moves.

Implementation values selected for simulator validation:

- Fixed locomotion callback: `100 Hz` (unchanged).
- Responsive baseline cadence: `1.0 Hz`; maximum cadence: `1.5 Hz`; cadence
  slew limit: `1.0 Hz/s`.
- Cadence remains at baseline until the largest required leg stride would
  exceed `0.5` of the `60 mm` maximum, then rises only as needed.
- Local-foot-speed deadband: `0.005 m/s`. At the smallest admitted speed, the
  proportional maximum swing clearance begins near `1.25 mm`; Gate 4 must
  prove that a just-above-deadband command clears the simulated ground.
- Each leg uses its own stride fraction for clearance, exactly
  `H_i = H_max * |S_i| / S_max`. There is no independent clearance floor.
- Requests that still exceed maximum stride at maximum cadence are scaled
  uniformly. Cadence, maximum stride scale, maximum clearance, and applied
  velocity scale are exposed in locomotion status.

These numerical values are provisional simulator operating policy. They do
not authorize physical actuation or assert that the open-loop servos can
reproduce the same minimum effective motion.

Outcome:

- Gait artifact `araco.locomotion.gait-tripod-slow` is version `0.3.0` with
  identity `tripod_legacy_curve_responsive_scheduler`.
- An admitted command ramps stride and lift continuously from zero; velocity
  shaped below the admission deadband is no longer hidden and released as a
  discontinuous first step. A zero or below-deadband request still stands.
- `LocomotionStatus` exposes applied cadence, maximum stride scale, maximum
  proportional clearance, and uniform velocity saturation scale.
- A clean nine-package build passes 255 tests with zero failures and 15
  expected skips. Gates 0–4 pass with behavior fingerprint
  `6f9678fd5aae3b832b2afede9710d187c3252a326a46a382d5cc74b937b58bda`.
- Gate 4 passes a `0.006 m/s` precision case at exactly `1.0 Hz`, `0.05`
  stride scale, and `0.0015 m` clearance, plus all seven prior movement cases.
  The combined case rises to about `1.293 Hz` at the preferred `0.5` stride
  scale. All eight cases retain at least three support contacts.

## 2026-08-16 — Simulator response revision for responsive cadence

Status: implemented and validated through Gates 0–4.

Decision:

- Retain the historical backend artifacts v0 (`0.040`) and v1 (`0.100`).
- Add `gz_ros2_control_v2.yaml`, artifact version `0.3.0`, with provisional
  simulator-only position proportional gain `0.150`, and select it consistently
  in development, CI, Gate 3, and Gate 4 profiles.
- Preserve Gate 4's physical, contact, tracking, and stop thresholds. Treat the
  mathematically exact `0.050 s` handover-exclusion boundary as excluded with a
  floating-point tolerance; do not extend the exclusion duration.

Rationale:

- At the responsive 1.0–1.293 Hz measured cadences, gain v1 trailed the
  commanded handover by about 50–60 ms even though the robot remained stable.
  The written Gate 4 contract freezes a 50 ms transition exclusion and requires
  intended support contacts after it.
- The v2 response aligns simulated tracking with that existing contract rather
  than weakening contact evidence or modifying the approved legacy foot curve.
  It remains only a functional simulator estimate.

Outcome:

- Gates 0–4 pass unchanged acceptance limits. In final Gate 4 evidence, all
  eight cases have minimum three intended support contacts and support-contact
  duty `1.0`; worst joint tracking error is `0.02639 rad`.
- This tuning is not a physical-servo response model, calibration, or hardware
  motion authorization.

## 2026-08-16 — Ground-relative foot orientation during body tilt

Status: accepted by the user, implemented, and validated through Gates 0–4.

Decision:

- Retain the new analytic, fail-closed four-DOF IK and its whole-robot
  transactional commit. Do not restore the legacy unchecked IK formulas.
- For body roll and pitch, express world-down in the commanded body frame and
  project it into each leg's instantaneous sagittal plane. Supply the resulting
  per-leg pitch to the existing IK.
- Report the angular component rejected by that projection as the unavoidable
  orientation residual. A four-DOF leg controls foot-tip position and one foot
  pitch, so it cannot independently realize every component of a full 3D foot
  orientation.
- Preserve the accepted constant standing pitch at the level pose and preserve
  yaw-only behavior. Reject the complete six-leg transaction if a projection
  is non-finite or degenerate.
- Extend unit tests and simulator Gate 3 so roll/pitch evidence checks actual
  foot-link orientation against the closest realizable projection, checks the
  excess over the unavoidable world-vertical residual, and retains all six
  foot-contact requirements.

Rationale:

- The legacy algorithm adjusted the fourth-link direction during body tilt,
  while the current caller always supplied `-pi/2`. Keeping the modern solver
  and correcting only its orientation input recovers the useful behavior
  without losing explicit limits, reachability checks, FK validation, or
  atomic failure handling.

Outcome:

- The versioned `araco.kinematics.ik` policy is `0.2.0`; composition requires
  `project_world_down_into_each_leg_sagittal_plane` and explicit rejected-angle
  residual reporting.
- Level and yaw-only poses retain the accepted `-pi/2` pitch. Roll/pitch poses
  produce separate per-leg pitch targets, and non-finite or degenerate
  projection input rejects the full 24-joint transaction.
- A clean nine-package build passes 251 tests with zero failures and 15
  expected skips. Gates 0–4 pass with behavior fingerprint
  `14cc9acbea9d31ca7c5feb4ab7aa0ab13d84f9dd615227099808d001fc0d2cdd`.
- Gate 3 reconstructs each actual foot-link axis from Gazebo joint feedback and
  ground-truth body attitude. All 14 pose cases pass: foot-axis projection
  error and vertical error above the unavoidable residual are numerically
  negligible, all six contact duties are `1.0`, and no non-foot collision is
  observed. The largest measured unavoidable residual is about `0.075 rad` in
  the accepted 50% pitch cases.

## 2026-08-16 — Focused keyboard-control window

Status: accepted by the user, implemented, and validated.

Decision:

- Replace terminal stdin capture with a development-only Tk window that owns
  real key press/release and focus state.
- Publish the complete pressed-key set at `50 Hz` using internal protocol
  `araco.keyboard-state.v1`. The adapter requires current window focus, the
  `space` deadman, and a heartbeat no older than `0.120 s`.
- Permit simultaneous `W/S`, `A/D`, and `Q/E` axes; equal opposite inputs
  cancel. Focus loss, closing the window, malformed input, adapter lifecycle
  deactivation, and heartbeat loss all release the command fail-closed.
- Put guarded Enable Motion and Controlled Hold action controls plus the current
  safety/readiness state in the window. Do not bypass the arbiter, safety
  supervisor, locomotion, or controller path.
- Select versioned keyboard mapping v1 (`0.3.0`) and single-robot wiring v1
  (`0.2.0`) in every simulator profile so behavior fingerprints remain
  comparable, while only the development launch starts the UI and adapter.

Rationale:

- A headed test proved `ros2 launch` owns the PTY and does not forward its stdin
  to the adapter child process. The former two independent 120 ms key pulses
  also could not represent a normally held `Space + W` chord reliably.
- A focused full-state heartbeat provides observable simultaneous input and a
  bounded disconnect/focus-loss stop without weakening the existing source and
  safety watchdogs.

Consequences:

- Keyboard control will require the `python3-tk` runtime package and a graphical
  session. Headless CI and scored gates do not start the window.
- This does not add joystick support; the PXN-2113 Pro remains a separate input
  adapter and device-mapping task.
- Interactive usability is validated for the headed Gazebo development profile
  using a real `Space + W` press/release. Joystick support remains separate.

Outcome:

- Clean root `/tmp/araco_keyboard_ui_final_20260816_02/` builds all nine
  packages and passes 258 tests with zero failures and 15 expected skips.
- Gates 0–4 pass with behavior fingerprint
  `8ac1afc7650e37c5bfafb464cb06bb4ca3bc6e0f2552494a20885d895173280d`.
- The final headed run used bundle
  `/tmp/araco_keyboard_ui_demo_20260816_02/`. A real `Space + W` state produced
  an active `0.05 m/s` forward candidate through the teleop adapter. Releasing
  it produced an inactive stand candidate, and safety returned to `HOLDING`
  with readiness `127/127`, zero fault mask, and no reset requirement.
- The orderly safety/Gazebo shutdown completed with every project process,
  including the UI, exiting cleanly. An earlier UI cleanup race and its
  publisher-context traceback were corrected before this final run.

Follow-up correction accepted on 2026-08-16:

- Releasing one direction key must not surrender the already accepted teleop
  source while the `space` deadman remains held. A fresh, focused deadman with
  no net direction now publishes `active=true`, zero velocity, and
  `GAIT_STAND`; it therefore stays motion-authorized while the operator changes
  direction keys. Releasing `space`, losing focus/heartbeat, or closing the
  window still releases the source immediately.
- Tk key releases are committed after a 30 ms repeat-suppression interval and
  cancelled by a matching press, preventing X11 key-repeat release/press pairs
  from creating a false gap. Focus loss bypasses that delay and releases all
  input immediately.
- This supersedes the earlier headed-test observation that releasing the sole
  direction key intentionally produced `active=false`. Mapping v1 is bumped
  from `0.2.0` to `0.3.0`.
- Clean root `/tmp/araco_keyboard_ui_final_20260816_03/` builds all nine
  packages and passes 259 tests with zero failures and 15 expected skips.
  Gates 0–4 pass with behavior fingerprint
  `35574c357af798bc014d5de8fdf8909ba02af07331e002fd1fd8ae2052c452db`.
  The final Gate 2 evidence is
  `/tmp/araco_keyboard_ui_gate2_20260816_04/`; an earlier attempt passed every
  scored check but lost its lifecycle helper to the normal shutdown race.
- Headed bundle `/tmp/araco_keyboard_ui_demo_20260816_03/` observed, in order:
  active tripod `vx=0.05, vy=0.05` for `Space+W+A`; active tripod `vy=0.05`
  after releasing only `W`; active neutral `GAIT_STAND` after releasing only
  `A`; and inactive only after releasing `Space`. The safety trace remained in
  `MOTION_ENABLED` throughout the direction-key releases and entered controlled
  stopping only after the final deadman release. Gazebo and all project
  processes then exited cleanly.

## 2026-08-16 — Connected joystick authority and Phase 6 supervision policy

Status: implemented and accepted; Gate 5 and Gate 6 pass. The joystick held-
deadman clauses below are superseded by the later no-deadman decision; the
timeout and all supervision controls remain.

Decision:

- Preserve the legacy PXN-2113 Pro axis roles and polarity as behavioral
  evidence, but do not preserve its positional float-array transport or lack of
  a deadman/watchdog.
- Add a separate `gazebo_joystick_v0` profile. It selects the connected device
  by exact Linux name, launches ROS `joy`, and uses a lifecycle adapter on the
  registered teleop candidate input. It does not alter keyboard development or
  headless CI input selection.
- Require a held deadman, a 120 ms raw-report timeout, an 8% centered deadzone,
  complete finite six-axis/12-button reports, and fail-closed release. A held
  deadman with neutral motion retains active stand authority so control changes
  do not require another enable action.
- Use a typed `ArbitrationStatus` beside `SelectedCommand` so safety can decide
  release, quarantine, epochs, and deliberate higher-priority handover without
  parsing log text or trusting a source-declared reason.
- Put all eight software safety states and their guards in a deterministic pure
  `SafetyMachine`. Keep an independent steady-time safe-command guard inside
  locomotion so loss or corruption of the supervisor stream cannot silently
  continue gait.
- Safety actions are long-running: hold, enable, reset, latched hold, and
  shutdown complete only when the requested target state is actually reached.
  The controlled-stop availability guard is explicit rather than inferred from
  aggregate readiness.

Evidence and limitations:

- Linux identifies the controller as `LiteStar PXN-2113 Pro`, USB `11ff:0837`,
  SDL GUID `0300b14bff1100003708000010010000`; ROS reports six axes and 12
  buttons. The axis sweep is live-verified.
- Live isolated ROS Joy observation verified that the main trigger / physical
  button 1 is ROS index 0 and physical button 2 is ROS index 1. Mapping artifact
  `0.2.0` therefore assigns the trigger as held deadman and button 2 as the
  roll/pitch modifier without provisional labeling.
- Phase 6 Gate 5 and formal Phase 7 Gate 6 pass. The accepted Gate 6 evidence is
  `/tmp/araco_gate6_final_20260816_06/`.
- This is simulator software supervision, not a physical emergency stop or a
  certified safety system. It does not authorize physical actuation.

## 2026-08-16 — Simplified joystick posture controls

Status: implemented. The sentence retaining the joystick held deadman is
superseded by the later no-deadman decision. Physical buttons 3/4 retain an
explicit sequential-index assumption eligible for a later live spot-check.

Decision:

- Supersede only the roll/pitch-modifier portion of joystick mapping `0.2.0`.
  The held deadman and the rest of the Phase 6 supervision policy are unchanged
  by this decision.
- Map axis 5 directly to planted-body pitch and keep inverted axis 4 as
  planted-body posture yaw.
- Leave physical button 2 unassigned. Assign physical buttons 3 and 4 to
  dedicated roll-left and roll-right commands respectively. If both are held,
  command zero roll instead of choosing an arbitrary priority.
- Record physical buttons 3/4 as sequential ROS indices 2/3. They remain
  eligible for a later non-blocking live spot-check; simulator development does
  not require another multi-control calibration exercise.

Rationale:

- The user considers roll the least useful posture function and rejected a
  modifier chord as unnecessarily complicated. Pitch is the more useful HAT
  function, while digital buttons are sufficient for rare roll adjustment.
- A direct one-control/one-function layout is easier to remember and avoids a
  mode-dependent HAT axis.

## 2026-08-16 — Joystick control has no deadman button

Status: implemented and validated in clean tests, Gate 0 composition, and a
headed joystick launch.

Decision:

- Supersede the joystick held-deadman requirement. Mapping/profile `0.5.0`
  grants joystick source authority whenever a complete finite Joy report is
  fresh; no button is required.
- Remove the separate operator Enable Motion step for this simulator-only
  profile. Automatically enable once per lifecycle activation, but only after
  readiness is complete and the selected joystick command is fresh, valid,
  neutral, and standing. Translation, walking yaw, roll, pitch, and posture yaw
  must be neutral; body-height trim may remain at its current position. Do not
  auto-enable a walking or non-neutral posture command.
- Emit one internal inactive candidate immediately after joystick-adapter
  activation, then publish fresh device state normally. This establishes the
  arbiter's new-session release edge without adding an operator control.
- Leave physical buttons 1 and 2 unassigned. Preserve buttons 3/4 as dedicated
  roll left/right.
- Retain the 120 ms report timeout, finite/dimension validation, inactive output
  on timeout/disconnect/malformed reports, controlled hold action, independent
  command freshness guards, and the safety supervisor. Consume automatic enable
  after its first use so hold, fault/reset, or source loss cannot auto-resume.
- Do not change the keyboard UI's separate `Space` deadman in this decision.

Rationale and tradeoff:

- The user explicitly rejected a joystick deadman and prefers the legacy-style
  direct control model.
- Once startup finishes, moving a joystick control can command motion immediately
  without holding the trigger or issuing an enable command. The trigger is not
  an emergency stop; controlled hold and the existing timeout/supervisor remain
  the stopping mechanisms.
- Live mapping `0.4.0` proved no-trigger candidate authority but exposed that
  the earlier explicit-enable fresh-edge rule made a continuously active source
  impossible to arm. The temporary preselected explicit-enable compatibility
  path was removed rather than retained alongside automatic enable.
- Final focused regression root
  `/tmp/araco_joystick_auto_final_20260816_01` reports 132 checks, zero errors,
  zero failures, and 11 expected skips. Final joystick Gate 0 composition at
  `/tmp/araco_joystick_auto_final_profile_v050_03` passes with behavior
  fingerprint `683ad02b5cbc0112f3e1b674795b5f015eb89fc87c869fc07fe79594b99b8adf`.
  A headed launch observed the automatic safety transition to
  `MOTION_ENABLED` with source 10 and no button/action input.

## 2026-08-16 — Controller-manager evidence and service-response timing

Status: implemented; the latest staggered-polling revision is awaiting three
fresh Gate 5 repetitions and replacement Gate 6 validation.

Decision:

- Treat controller-manager service polling as a typed, validated latest-value
  mailbox. Do not require a new successful service response inside the
  controller-state stream watchdog window.
- Poll the typed controller and hardware validation services at `1 Hz` each,
  staggered so they are never requested simultaneously. Keep the independently
  published controller-state, joint-state, and simulation-clock liveness
  watchdogs at their accepted `100 ms` bounds.
- Keep a validated controller or hardware identity/state result while its typed
  service remains available. Invalidate it on an explicit invalid reply or
  service disappearance.
- Use fresh controller-state, joint-state, and simulation-clock streams as the
  bounded ongoing liveness evidence. Controller interface claims are checked as
  controller ownership evidence, not as hardware-component identity evidence.

Rationale:

- The first formal Gate 6 attempt showed a healthy controller and Gazebo backend
  while command-arbiter activation delayed a controller-manager service reply.
  Expiring the previous reply after 110 ms conflated request scheduling latency
  with loss of controller/backend liveness and latched a false fault.
- The accepted architecture already describes nonblocking polling into a
  validated mailbox and separately fresh corroborating streams. This correction
  makes the implementation match that evidence model without weakening explicit
  invalid-response, service-loss, stream-loss, or clock-loss handling.
- Polling both controller-manager services every 50 ms generated 40 service
  requests per second and coincided with a measured controller/joint-state
  publication gap during a Gate 5 startup. Versioned safety policy `0.2.0`
  removes that avoidable service pressure without relaxing stream liveness.

Evidence boundary:

- The failed attempt remains immutable at
  `/tmp/araco_gate6_final_20260816_01/`; no automatic retry was performed.
- The complete `araco_supervision` suite reports 73 checks, zero errors, and
  zero failures after the correction. Three fresh unrestricted Gate 5 runs pass
  at `/tmp/araco_phase6_gate5_mailbox_fix_unrestricted_rep{1,2,3}/`.
- A sandbox-blocked infrastructure run is retained separately at
  `/tmp/araco_phase6_gate5_mailbox_fix_rep1/`; it failed before spawn because
  the managed sandbox denied DDS sockets, `getifaddrs`, and Gazebo logging. It
  is not behavioral evidence and was not relabeled as a passing repetition.
- Replacement Gate 6 evidence remains required before Phase 7 is complete.

Gate-test harness corrections retained on 2026-08-16:

- A second formal attempt is retained at
  `/tmp/araco_gate6_final_20260816_02_runner_crash/`. All three repetitions and
  every Gate 0–5 report passed, but the Gate 6 comparator crashed because it
  assumed Gate 0 used the same validation-report shape as Gates 1–5. The
  comparator now normalizes both documented schemas and has regression tests.
- The preserved physical metrics satisfied every repeatability threshold, but
  exact discrete paths exposed two scorer races. Isolated controller/joint/
  clock fault cases now keep the command source fresh, so they test only the
  named component. Duplicate injection now observes the first exact sequence
  through typed `SelectedCommand` before sending the duplicate.
- These are test-harness corrections, not changes to the accepted robot
  behavior. The replacement formal Gate 6 run must use a new evidence path;
  failed or crashed attempts remain immutable and are never relabeled.
- Formal attempt `/tmp/araco_gate6_final_20260816_03/` stopped at preflight
  Gate 1. The robot reached fully ready `HOLDING` with no fault and every
  motion/contact/physics check passed, but the Gate 1 scorer had polled both
  controller-manager services continuously before initialization and retained
  incomplete startup replies. Gate 1 now uses fully ready `HOLDING` as its
  typed initialization barrier and queries each evidence service only once.
- The correction passes standalone Gate 1 at
  `/tmp/araco_phase7_gate1_ready_barrier_20260816_01/`; its controller ownership
  and Gazebo backend identity checks are both true.
- Gate 6 compares the exact terminal safety epoch/state/reason path. Gate 5
  separately proves the required fault-mask matrix and reset rules. It does not
  compare the union of extra fault bits accumulated after a fault is already
  latched while a controller-manager recovery operation runs, because that
  union can vary with service scheduling without creating a new safety state or
  reason outcome. Runtime fault detection and watchdog thresholds are unchanged.
- Formal attempt `/tmp/araco_gate6_final_20260816_04/` passed package tests,
  sanitizers, all preflight gates, all 18 repeated Gates 0–5, physical
  repeatability, fingerprints, real-time factor, and wall budgets. It failed
  only exact discrete outcomes: deactivating `joint_state_broadcaster` sometimes
  also aged controller-state publication, legitimately changing joint-loss
  handling between controlled-stop-then-fault and direct fault hold.
- Use the Phase 6 plan's test-only relay for the joint-state-loss scenario.
  Production profiles still connect safety directly to `/joint_states`; only
  `gazebo_gate5_v0` selects versioned test wiring through
  `/araco/system_test/joint_states`. Pausing that relay removes the joint-state
  stream without switching controllers, so the scenario tests one boundary.

## 2026-08-16 — Synchronize selection and arbitration epochs at safety boundary

Status: implemented and accepted by Gate 6.

Decision:

- Treat `SelectedCommand` and `ArbitrationStatus` as one epoch-correlated
  decision even though ROS transports them on separate topics.
- If their selection epochs temporarily differ, publish HOLD rather than
  execute, and do not commit a source-loss or handover transition until the
  matching arbitration epoch supplies its typed reason.
- Require the synchronized pair before publishing an executable `SafeCommand`
  or an executing `SafetyStatus` disposition.

Rationale:

- Formal Gate 6 attempt `/tmp/araco_gate6_final_20260816_05/` passed every
  criterion except exact discrete outcomes. In repetition 3, the no-selection
  command arrived before its matching arbitration status, so safety used the
  preceding epoch's empty reason and reported `SOURCE_RELEASED` instead of the
  actual `SOURCE_INVALID` and `SOURCE_STALE` initiating causes.
- The command-selection epochs and all individual Gate 5 checks were identical;
  the defect was cross-topic arrival order at the safety boundary. Holding
  during a mismatch remains fail-closed while epoch correlation makes the
  transition reason deterministic and truthful.

Validation:

- Three corrected Gate 5 runs at
  `/tmp/araco_phase7_gate5_epoch_sync_rep{1,2,3}/` have identical arbitration
  and normalized safety signatures.
- Formal Gate 6 attempt `/tmp/araco_gate6_final_20260816_06/` passes all 21
  acceptance checks, including exact discrete outcomes across all three
  no-retry repetitions.

## 2026-08-16 — Correct joystick polarity and classify workspace saturation as limiting

Status: partially superseded on 2026-08-17 by mapping `0.7.0`; workspace-
limiting behavior remains accepted.

Decision:

- Preserve the user-verified forward/reverse polarity and reverse lateral,
  walking yaw, body height, pitch, posture yaw, and dedicated roll-button
  directions in joystick mapping/profile version `0.6.0`.
- When a finite bounded operator request cannot advance within the IK workspace,
  retain the last complete valid 24-joint trajectory, report shared reason 11
  (`REASON_COMMAND_LIMITED`), and retry on later ticks.
- Continue to report reason 16 and enter the existing latched safety path if the
  locomotion node loses its last committed trajectory invariant. Do not relax
  watchdogs, controller validation, or the safety latch.

Rationale:

- The user verified in the headed simulator that only forward/reverse had the
  intended direction.
- The reported Gazebo freeze was a controller stop, not a rendering hang: the
  live log showed locomotion readiness drop for one tick with reason 16, then
  recover, while safety correctly remained latched. A rejected candidate is an
  ordinary workspace saturation because it is never published and the prior
  full trajectory remains valid; classifying it as loss of kinematic integrity
  made routine joystick saturation unnecessarily terminal.

Validation:

- Focused affected-package results contain 129 checks, zero errors/failures,
  and 10 expected skips. The complete eight-package dependency set builds, and
  profile `0.6.0` composes with behavior fingerprint
  `854549de4a7a7fcf7a7d65aa85c17c8b2306aa0b54ea29eb8c1efc2e7f21c931`.
- During a headed joystick run, one gait/posture and two body-pose workspace
  rejections were limited nonfatally. The post-event safety sample remained
  `MOTION_ENABLED`, reason 0, readiness `127/127`, fault mask 0, source 10.

## 2026-08-17 — Restore height and roll polarity; separate speed-profile choice

Status: polarity mapping implemented and focused validation passed. The speed
choice was subsequently selected and is recorded below.

Decision:

- Restore the version `0.5.0` height-axis and dedicated roll-button polarities,
  because the user found those two controls inverted after the broad `0.6.0`
  reversal.
- Retain the `0.6.0` corrections for lateral, walking yaw, pitch, and posture
  yaw, and retain the previously correct forward/reverse direction.
- Publish the correction as joystick mapping/profile `0.7.0`.
- Keep walking top speed, cadence, stride, acceleration, and planted-body pose
  rates unchanged until the user chooses among explicit speed profiles.

Validation:

- `araco_teleop` and `araco_bringup` report 57 checks, zero errors/failures,
  and zero skips after the `0.7.0` correction.
- The joystick profile composes at
  `/tmp/araco_joystick_polarity_profile_v070_20260817_01` with behavior
  fingerprint `cbd3bcf19d976fa6269bca75d8dd1194fb62eb93bdba13b7f3f35f2a0ed4c0e8`.
- Live visual confirmation of height and roll remains pending until the next
  headed launch.

## 2026-08-17 — Add an opt-in Responsive joystick simulator profile

Status: implemented and configuration-validated; headed operator validation
pending.

Decision:

- Keep the accepted CI, Gate, and keyboard simulator profiles on the existing
  slow gait and velocity-envelope artifacts.
- Make `gazebo_joystick_v0` select a separate Responsive simulator gait and
  operational policy, together with joystick mapping/profile version `0.8.0`.
- Set maximum joystick translation to `0.100 m/s` and walking yaw to
  `0.600 rad/s`. Use a `1.5–2.5 Hz` cadence range with `2.0 Hz/s` cadence slew.
- Retain the `0.060 m` maximum stride and scale each leg's clearance by the
  same normalized factor as its stride, up to `0.030 m`.
- Use `0.200 m/s²` translation acceleration, `0.300 m/s²` stop deceleration,
  `1.200 rad/s²` yaw acceleration, and `1.800 rad/s²` yaw stop deceleration.
  Keep planted-body pose rates at `0.030 m/s` and `0.300 rad/s`.
- Treat this as simulator-only tuning. It is not evidence for safe physical
  servo speed, acceleration, contact behavior, or stability.

Rationale:

- The user selected the Responsive option after reviewing explicit speed
  choices and asked for faster interactive movement.
- Versioned profile-specific artifacts prevent presentation tuning from
  silently changing the established automated regression baseline.
- Scaling mapping limits, safety envelopes, cadence, and shaping together
  avoids hidden clipping or a partially applied speed profile.

Validation:

- The clean eight-package dependency graph builds at
  `/tmp/araco_joystick_responsive_build_20260817_01`.
- Affected-package tests report 205 checks, zero errors/failures, and 21
  expected static-analysis skips.
- Profile `gazebo_joystick_v0` composes at
  `/tmp/araco_joystick_responsive_profile_v080_20260817_01` with behavior
  fingerprint
  `07d0049377d65cd61a2a4d1784bdc59c5c280fe2d9bea4009d88754173f1d564`.
- Generated teleop, safety, and locomotion parameters contain the complete
  Responsive values. All five non-joystick profiles still compose; their
  established behavior fingerprints remain unchanged.

## 2026-08-17 — Double Responsive speed and make workspace saturation recoverable

Status: superseded for joint-angle range by the following wide-envelope
decision and for re-arm behavior by the later first-step/automatic-retry
decision. The doubled stride, speed, and cadence selections remain active.

Decision:

- Supersede the `0.8.0` Responsive speed values above for the joystick profile.
  The user's intent is approximately twice the robot speed by doubling stride
  at the same cadence, not twice the visible stride at unchanged body speed.
- Keep the Responsive cadence at `1.5–2.5 Hz`, raise maximum stride from
  `0.060 m` to `0.120 m`, translation from `0.100 m/s` to `0.200 m/s`, and
  walking yaw from `0.600 rad/s` to `1.200 rad/s`. Double the corresponding
  acceleration and stop-deceleration values so the response time does not
  become twice as slow.
- For this simulator-only Responsive profile, use the complete existing
  provisional simulator joint ranges and their model velocity of `2.0 rad/s`:
  coxa `[-0.70,+0.70]`, femur `[+0.15,+1.35]`, tibia
  `[-2.65,-0.75]`, and foot `[-1.25,+0.35]`. Do not widen the underlying
  model limits, and do not apply these values to physical hardware.
- On a gait workspace rejection, freeze the phase that was moving outward and
  decelerate the current curve amplitude toward the nominal six-foot stance.
  Keep reason 11 while recovering. Require centered planar controls before
  re-arming the gait, so a held saturated command cannot immediately drive the
  controller back into the same boundary.
- Continue using the previous complete 24-joint trajectory transaction while
  evaluating recovery. A lost committed trajectory remains reason 16 and a
  safety fault; workspace saturation does not.

Rationale:

- The former narrow operational ranges, not the provisional model limits,
  rejected several otherwise reachable gait-plus-posture combinations.
- The indefinite lock had a separate state-machine cause: normal stopping
  advanced gait phase. At an exact boundary every positive step could continue
  outward, so retrying with smaller time steps could never begin the retreat.
- A `0.120 m` stride cap with half cadence would preserve body speed and would
  contradict the user's clarified objective. Retaining cadence and doubling
  velocity/stride produces the intended speed increase.

Validation:

- Clean build root:
  `/tmp/araco_joystick_stride_recovery_build_20260817_02`; affected-package
  results contain 208 checks, zero errors/failures, and 21 expected static-
  analysis skips.
- The exact kinematic regression sweeps forward/reverse, lateral both ways,
  walking yaw both ways, and maximum diagonal translation plus maximum yaw
  through multiple gait cycles at the doubled-speed profile; all six-leg
  transactions remain reachable at a neutral body pose.
- The recovery regression proves phase freeze, monotonic amplitude reduction,
  nominal foot placement, and stable-hold completion. An isolated ROS exercise
  forced reason 11, observed a valid complete neutral trajectory during
  recovery, then centered the command and observed reason 0 with all six legs
  valid.
- Profile `0.9.0` composes at
  `/tmp/araco_joystick_double_speed_profile_v090_20260817_01` with behavior
  fingerprint
  `15277c3a7fbc20e7315dc665bfab482bf0d8f9380059c5cdf55a694bdf9ab1bd`.
  All five non-joystick behavior fingerprints remain unchanged.

## 2026-08-17 — Isolate a wide 270-degree simulator joint envelope to joystick control

Status: implemented and validated by clean build, affected-package tests, exact
configuration composition, and non-joystick fingerprint isolation; renewed
headed operator validation is pending.

Decision:

- Supersede only the earlier instruction not to widen the Responsive
  joystick model limits. The user reports 270-degree servos and intentionally
  designed the mechanical parts with substantial collision clearance.
- Add `araco.description.provisional-sim-limits` `0.2.0` as a separate artifact
  selected only by joystick profile `0.10.0`. Keep the original `0.1.0` limits
  and all existing non-joystick profile selections unchanged.
- Give coxa, femur, tibia, and foot intervals a `270 degree` (`3*pi/2`) span,
  centered near the corresponding nominal standing coordinate: coxa
  `[-2.356194,+2.356194]`, femur `[-1.636194,+3.076194]`, tibia
  `[-4.256194,+0.456194]`, and foot `[-2.766194,+1.946194]` radians.
  Keep the gimbal limit unchanged because this request concerns the leg-control
  saturation and the gimbal has separate cable/extrinsic considerations.
- Make the Responsive operational policy use the complete wide leg envelope at
  the existing `2.0 rad/s` simulator command-rate cap.
- Select a matching Gazebo backend artifact `0.4.0` whose exact dependency is
  the wide model-limit artifact, so URDF and `ros2_control` enforce the same
  bounds without altering the baseline backend.
- Retain the knee-down IK branch, singularity rejection, geometric reach test,
  complete six-leg transaction rule, and phase-frozen boundary recovery.

Rationale:

- A separate selected artifact gives the user a permissive presentation and
  joystick test profile without weakening the deterministic CI/Gate baseline.
- Servo travel alone does not identify calibrated physical endpoints. Centering
  the provisional range near the nominal coordinate is suitable for simulation
  exploration but is explicitly ineligible for a physical deployment profile.
- Wider angular limits remove artificial joint-limit rejections but cannot make
  a geometrically unreachable foot target reachable. The existing recovery
  behavior remains necessary.

Validation:

- All nine packages build cleanly at
  `/tmp/araco_joystick_wide_limits_build_20260817_01`; affected-package results
  contain 181 tests, zero errors/failures, and 10 expected skips.
- Profile `0.10.0` composes with `PASS` at
  `/tmp/araco_joystick_wide_limits_profile_v0100_20260817_01`; its behavior
  fingerprint is
  `f75f419871a7df381ea88bf1bbf8c3938b049b228da96c5409afaec2fb7e13bf`.
- Generated locomotion parameters, URDF limits, and `ros2_control` min/max
  interfaces contain the same wide values. A regression verifies a 270-degree
  span for all four leg-joint classes and nominal-pose containment.
- All five non-joystick profiles retain their established behavior
  fingerprints: `866f756334259bd34e2d3960948f69af92ccaadb1b5719c4ff567ca6c048e829`
  for development, CI, Gate 3, and Gate 4; and
  `4a803eed16ea0203bfa59ea0c545d90865b18c1ff58cf3691f3318ec9357caa6`
  for Gate 5.

## 2026-08-17 — Restore the legacy first-step curve and remove planar re-arm lock

Status: implemented; clean build, complete test suite, all profile compositions,
and Gazebo contact-gate validation pass. Headed operator validation is pending.

Decision:

- Supersede the assumption that acceleration-scaled stride alone reproduced
  the legacy smooth first step. The legacy algorithm begins tripod A at counter
  `-50` and tripod B at `0`; the previous normalized port entered the repeating
  curve directly and therefore began at nonzero horizontal endpoints.
- Restore only the legacy negative-counter horizontal and vertical functions
  and execute `-50 → -25` / `0 → 25` as a dedicated first quarter-cycle on
  every newly admitted walk. Join the existing repeating curve at phase zero,
  retaining the user-approved phase-0.75 horizontal correction and the new
  scheduler/state-machine architecture.
- Version the baseline and Responsive gait artifacts as `0.4.0` and `0.6.0`
  with identity `tripod_legacy_curve_warm_start_responsive_scheduler`.
- Supersede the recovery rule that held all gait output until planar controls
  were centered. After a complete phase-frozen retreat, retry planar gait
  automatically at neutral body posture. If the incompatible request included
  body posture, suppress walking posture offsets and keep reason 11 visible
  until posture controls are centered; do not weaken IK reachability,
  singularity, or complete-transaction checks.

Rationale:

- The negative interval is the mechanism that makes the legacy first tripod
  travel from the standing target to its first half-stride instead of appearing
  instantly at a periodic endpoint. It also gives the initial lift the legacy
  counter-speed ramp rather than starting a leg in an already active swing.
- The 270-degree simulator envelopes remove joint-angle clipping but cannot
  resolve incompatible combined body-pose and gait geometry. Requiring planar
  centering after recovery made a recoverable reason-11 event look like an
  indefinite freeze. Prioritizing planar gait while temporarily dropping the
  conflicting posture preserves useful control without creating a latched
  software fault.

Validation:

- `/tmp/araco_first_step_build_20260817_01` builds the locomotion dependency
  chain. `araco_locomotion` reports 66 tests, zero errors/failures, and 10
  expected static-analysis skips, including exact warm-start and restart
  regressions.
- `/tmp/araco_warm_start_recovery_build_20260817_01` builds all nine packages;
  340 tests report zero errors/failures and 24 expected skips.
- All six profiles compose at
  `/tmp/araco_warm_start_recovery_profiles_20260817_01`. Development, CI,
  Gate 3, and Gate 4 share behavior fingerprint
  `44c8c502bc5976cc579cf32edd4ca57454c63647e3d81813d58e4cec1aedd398`;
  joystick uses
  `0a28c82cf911b2cc0e81245584ad01673ce348b5eda307281a94b48e56146093`;
  Gate 5 uses
  `fe5169518dd0c49a9645bc7872c31d0dab497c23e3ab037bc367b0de849e387d`.
- Full Gazebo Gate 4 passes at
  `/tmp/araco_warm_start_gate4_20260817_02`, including startup, active stand,
  every movement direction, support contacts, manual hold, controlled stop,
  and absence of non-foot ground contact.

## 2026-08-17 — Reclassify the current result as a simulator baseline, not legacy parity

Status: audit complete; no behavior change authorized by this review.

Decision:

- Correct earlier reporting that allowed completion of simulator Gates 0–6 to
  sound like completion of the full robot system. The implemented result is a
  tested Gazebo control baseline.
- Treat legacy behavior and capability parity as an explicit unfinished work
  stream. Preserve the detailed comparison in
  `.agent/LEGACY_PARITY_AUDIT.md`.
- Record a confirmed cross-layer defect: independent `0.20 m/s` joystick axes
  produce a `0.2828427 m/s` diagonal above the `0.24 m/s` hard radial envelope;
  invalid-source quarantine then cannot clear because the no-deadman adapter
  remains active even with centered controls.
- Do not attribute every apparent freeze to IK. The latest headed session
  logged reason 10 (`SOURCE_INVALID`) and no reason-11 event. Retain the
  separately observed combined gait/posture IK usability problem as unfinished.

Rationale:

- The existing gates validate the contracts selected by their profiles. Gate 4
  uses the slow system-test profile and does not exercise the Responsive
  joystick stack or complete combined operator-input space.
- The legacy system omitted important validation, but that does not justify a
  successor that turns ordinary joystick input or reachable subsets of a
  command into unexplained persistent holds.

Validation:

- Pure mapping reproduction gives diagonal magnitude `0.282842712 m/s` against
  hard limit `0.24 m/s`, and confirms a fresh centered joystick report still
  has `active=true`.
- Launch log
  `/home/stevw-s14/.ros/log/2026-08-17-19-12-43-934839-stevw-s14-Stealth-14Studio-A13VF-659450`
  records `MOTION_ENABLED`, reason 10, holding, and later backend-loss shutdown.

## 2026-08-17 — Restore advanced legacy locomotion input, mixing, and gimbal

Status: implemented and validated in package tests, profile composition,
Gazebo Gate 4, and an isolated live joystick/gimbal run.

Decision:

- Supersede the independently scaled joystick X/Y mapping. Shape the left
  stick as one radially normalized vector so a full diagonal remains at the
  configured planar magnitude.
- Port the inspected legacy `rotation()` foot-path function separately from
  the horizontal translation function. Apply its yaw output as exact arcs
  around the base origin.
- Port the legacy translation/yaw mix: normalize each magnitude to its command
  scale, use their relative magnitudes as weights, and use the larger magnitude
  as the overall request.
- Preserve cumulative gait-cycle telemetry while using a separate per-walk
  startup marker. Every new walk gets the negative-counter warm start and
  group-A rotational handover suppression.
- Couple joystick axis 4 to planted-body posture yaw and gimbal yaw. Supervise
  `gimbal_yaw_rad` through the typed command path and publish it through the
  separate gimbal trajectory controller at a `1.5 rad/s` rate cap.
- Double Responsive maximum swing clearance from `0.030 m` to `0.060 m`, using
  the same normalized factor as stride. Keep conservative CI/Gate clearance at
  `0.030 m` because its separately selected narrow joint envelope does not pass
  the yaw matrix at 60 mm.
- Fix invalid-source recovery without adding a deadman. After all motion,
  posture, and gimbal controls center, emit one inactive session release and
  resume from the next fresh report.
- Expose typed source, safety, and locomotion reasons as operator status
  transitions. Keep info/warning calls on stable Python source lines and ignore
  only `1e-12`-scale roundoff when classifying a normal-boundary clamp.

Rationale:

- These changes restore the user-valued smooth legacy path behavior without
  removing the current system's checked IK, atomic transactions, supervision,
  controlled stop, or simulator profile isolation.
- The conservative and Responsive artifacts serve different evidence goals.
  Doubling all profiles would invalidate the conservative contact gate rather
  than prove the Responsive presentation behavior.
- A source rejection and an IK/command limit must remain diagnosable as
  different causes; neither should appear as an unexplained Gazebo freeze.

Validation:

- All six profiles compose at
  `/tmp/araco_legacy_port_profiles_20260817_final`. Baseline fingerprint is
  `e18abb8bf7ae8233c68612ee58ad8e93646e5f9d52a297e18d56550363401e0e`;
  Responsive joystick fingerprint is
  `20743b137720f53cd6622da81f86b23fec7c3b4923828dd1c3ca9a4aab30e226`.
- Gate 4 passes at `/tmp/araco_legacy_port_gate4_20260817_07`; all eight cases
  complete seven monotonic cycles with at least three intended support
  contacts and unchanged physical/tracking thresholds.
- Live axis-4 validation in
  `/tmp/araco_legacy_port_joystick_live_20260817_03` produced a selected
  `0.3141592653589793 rad` gimbal request and
  `0.31415926535897787 rad` Gazebo feedback with safety state 4 and no fault.
- `.agent/LEGACY_PARITY_AUDIT.md` supersedes its earlier defect report and
  records the remaining parity boundary.

## 2026-08-17 — Reclassify axis-4 smoothing parity as unfinished

Status: confirmed by operator observation and direct legacy/current source
comparison; no runtime change is authorized yet.

Correction:

- The preceding axis-4 work restored the typed command route and simulated
  gimbal actuation, but did not restore legacy response shaping.
- Legacy `algo.py` is nominally a PID layer, but its active gains are P-only:
  `Kp=0.02`, `Ki=0`, and `Kd=0` at 200 Hz. Body yaw and gimbal yaw are both
  derived from the same filtered `dx` state, at `±pi/8` and `±pi/10`.
- The current implementation independently slews body posture at `0.3 rad/s`
  and the gimbal at `1.5 rad/s`, while body yaw is limited to `±0.2 rad`.
  This makes the gimbal visibly outrun the body and is not legacy parity.
- A later correction should filter the normalized axis once using a
  time-step-independent equivalent of the legacy first-order response, then
  scale that shared result into both targets. Physical joint velocity and
  checked-IK rate limits remain final safety guards rather than operator-feel
  shaping.

## 2026-08-17 — Restore legacy-equivalent smoothing for every joystick control

Status: implemented and validated; supersedes the preceding unfinished-runtime
classification.

Decision:

- Apply a time-step-independent form of the legacy P-only response to every
  current joystick control: radial translation X/Y, walking yaw, body height,
  dedicated-button roll, body pitch, and axis 4.
- Preserve the legacy coefficients as response fractions at a 5 ms reference
  interval: `0.02` for normal controls and `0.01` for height. This produces the
  same response curve even though the current adapter publishes at 50 Hz.
- Filter physical axis 4 exactly once in normalized space, then scale that
  shared state into body yaw and gimbal yaw. Keep the approved current ranges:
  body yaw `±0.2 rad` and gimbal yaw `±pi/10`. Do not restore the legacy
  `±pi/8` body-yaw range.
- Mark only the joystick composition as `operator_input_pre_filtered`. Its
  ordinary executable commands bypass the locomotion gait/body/gimbal feel
  limiters so the legacy response is not filtered twice. Keyboard, CI, and
  Gate inputs keep their existing shaping.
- Preserve controlled deceleration, non-executable return shaping, checked IK,
  complete six-leg transaction validation, joint command-rate checks,
  controller limits, source freshness, quarantine recovery, and supervision.

Rationale:

- The previous independent body and gimbal slew limits changed their normalized
  timing and made the gimbal appear disconnected from posture control.
- One normalized axis state guarantees equal response progress for both
  outputs. Their physical angles are intentionally different because the user
  chose to keep their current unequal ranges.
- Applying the same response policy to all controls restores the behavior the
  user meant by the legacy PID layer; height retains its deliberately slower
  coefficient.

Validation:

- Clean build root: `/tmp/araco_smoothing_build_20260817_01`.
- Full result: `355 tests, 0 errors, 0 failures, 24 skipped`.
- All six profiles compose at
  `/tmp/araco_smoothing_profiles_20260817_01`.
- Joystick profile `0.13.0` resolves
  `operator_input_pre_filtered: true`; development profile resolves `false`.
- Unit tests cover time-step equivalence, every current joystick control,
  shared axis-4 normalized response, timeout reset, and centered recovery.

## 2026-08-17 — Increase Responsive preferred stride and planar speed by 20%

Status: implemented and regression-tested; operator Gazebo evaluation pending.

Decision:

- Interpret “increase the stride a bit more to get more speed” as a coordinated
  20% Responsive-only increase. Raising stride without requested velocity would
  merely lower cadence at the same body speed.
- Increase preferred stride scale from `0.5` to `0.6`. With the unchanged
  `0.120 m` absolute maximum, preferred full-stick stride rises from 60 mm to
  72 mm.
- Increase joystick planar scales and the Responsive normal planar envelope
  from `0.200` to `0.240 m/s`. Preserve the 1.2 hard/normal ratio by increasing
  the hard envelope from `0.240` to `0.288 m/s`.
- Keep `1.5–2.5 Hz` cadence, `0.060 m` maximum clearance, proportional
  stride/clearance scaling, yaw speed, smoothing coefficients, checked IK,
  joint-rate validation, and all supervision behavior unchanged.
- Version the Responsive gait as `0.8.0`, Responsive operational policy as
  `0.6.0`, PXN mapping as `0.12.0`, and joystick profile as `0.14.0`.

Risk boundary:

- This is a presentation-profile tuning change, not a physical calibration.
- The preceding headed run logged recoverable workspace retreats during some
  combined commands. The higher-speed/stride profile may reach workspace
  limits more often and still requires a headed operator check plus an
  exhaustive Responsive contact/workspace matrix.

Validation:

- Clean build/test root: `/tmp/araco_stride_speed_build_20260817_01`.
- Full result: `355 tests, 0 errors, 0 failures, 24 skipped`.
- All six profiles compose at
  `/tmp/araco_stride_speed_profiles_20260817_01`; conservative profile behavior
  fingerprint remains unchanged.
- Responsive joystick behavior fingerprint:
  `5cf383effae1ac9ed603d4d1e2f72603114c957b3245a34e70cc6aca53c6db96`.

## 2026-08-17 — Remove hidden Responsive cadence retiming at the validated envelope

Status: implemented and regression-tested; headed operator evaluation remains
useful but is not required for the configuration contract.

Decision:

- Treat the reported larger-but-slower gait as a real control effect, not an
  optical illusion. The 10 ms locomotion transaction was bisecting its gait
  time whenever the previous uniform `2.0 rad/s` rate cap rejected a candidate;
  this slowed phase while status continued to report nominal cadence.
- Do not raise the cap to an initially guessed `2.4 rad/s`. A 100 Hz
  exact-geometry sweep measured hard-envelope steady peaks of approximately
  `4.95`, `8.30`, `10.97`, and `7.42 rad/s` for coxa, femur, tibia, and foot.
- Select simulator-only model and operational caps of `5.5`, `10.0`, `12.5`,
  and `9.0 rad/s` respectively. Preserve every angle/reach/singularity check,
  atomic transaction, and workspace recovery path.
- Model the actual 50 Hz joystick publication and the legacy-equivalent 2%-per-
  5-ms response in regression tests. Require all normal and hard translation,
  lateral, yaw, and mixed commands to advance at the complete 10 ms phase step.
- Version the wide limit and pose artifacts as `0.3.0`, Responsive operational
  policy as `0.7.0`, wide Gazebo backend as `0.5.0`, and joystick profile as
  `0.15.0`.

Rationale:

- Raising only to `2.4 rad/s` would leave the hidden retiming active. Raising
  every joint to the unfiltered instantaneous-command peak would instead encode
  an input discontinuity that the production joystick never emits. Per-class
  caps based on the filtered hard envelope retain margin without pretending to
  be physical-servo calibration.

Consequences:

- These caps are presentation-simulator values only. Physical DS3235/DS5160
  loaded-speed measurements, PWM mapping, power limits, and safety validation
  must define any later hardware profile independently.
- Out-of-envelope inputs, an IK/workspace failure, or a future gait change can
  still invoke transaction bisection. The new regression prevents the current
  supported joystick envelope from silently doing so.

## 2026-08-17 — Establish the simulator Gemini RGB-D/IMU and RViz baseline

Status: implemented, regression-tested, and validated with live Gazebo/ROS
messages on the development laptop.

Decision:

- Activate the previously deferred `araco_perception` package now that RGB-D
  simulation has begun. It owns a strict Gemini-like sensor artifact and the
  project RViz display layout; Gazebo world/plugins and bridges remain owned by
  `araco_gazebo`, and canonical frames/meshes remain owned by
  `araco_description`.
- Add separate Gazebo color-camera, RGB-D-camera, and IMU sensors at
  `camera_link`. Keep the yaw gimbal fixed for initial perception validation.
- Publish standard ROS messages on six stable endpoints: color `Image` and
  `CameraInfo`, depth `Image` and `CameraInfo`, organized `PointCloud2`, and
  camera `Imu`. Use `camera_color_optical_frame` and
  `camera_depth_optical_frame` for optical products and `camera_link` for IMU.
- Use a performance-safe baseline of 424 x 240 at 15 Hz for color/depth and
  100 Hz for IMU, with 0.15–20 m clipping. Treat the generated pinhole
  calibration as simulator data, not a physical Gemini calibration.
- Make `gazebo.launch.py` honor the composed profile's `rviz` setting and load
  `gemini_rgbd_v0.rviz`, which contains robot, TF, RGB, depth, and point-cloud
  displays.
- Require the Gazebo Sensors/Ogre2 and IMU systems in the selected world and
  cross-validate sensor frames and all bridge mappings during composition.

Rationale and rejected draft:

- The first 640 x 480 at 30 Hz draft produced valid messages but delivered only
  about 7 Hz in wall time while competing with the exact-mesh 1 kHz physics and
  controller workload. It also exposed a transient locomotion-staleness fault
  during full-stack startup. Keeping that draft would recreate the user's
  simulator-freeze problem.
- The selected mode reduces organized point-cloud traffic by about sixfold and
  preserves useful RGB-D development while keeping control supervision healthy.
  Higher-fidelity camera modes may be added later as separate profiles after
  measured performance evidence; they must not silently replace this control-
  safe baseline.

Validation:

- Fresh build/test root: `/tmp/araco_rgbd_build_20260817_01`; the final
  post-tuning full result is `372 tests, 0 errors, 0 failures, 24 skipped`.
- Live full-stack headless launch reached and remained in `HOLDING` with
  readiness `127/127`, `fault_mask=0`, and `reset_required=false`.
- Measured delivery was approximately 13.4 Hz for color and depth and 87 Hz for
  IMU. One-shot messages confirmed `rgb8`, `32FC1`, organized point-cloud data,
  populated `CameraInfo`, correct dimensions, and correct frame IDs.
- RViz loaded on the real desktop display, subscribed to the configured point
  cloud, and exited cleanly after the bounded smoke test. The failed offscreen
  attempt was an expected GLX parent-window limitation, not a configuration
  failure.

## 2026-08-17 — Qualify startup readiness and label Gazebo point clouds truthfully

Status: implemented, regression-tested, and live-validated in headed Gazebo
with RViz.

Decision:

- Require continuous complete simulator readiness for `1.0 s` before the safety
  machine transitions from `INACTIVE` to `HOLDING`. A readiness gap during this
  startup dwell restarts the dwell instead of latching a fault. Once `HOLDING`
  is reached, all existing clock, controller, joint, backend, locomotion, and
  provenance fault behavior remains unchanged.
- Label the Gazebo RGB-D point cloud as `camera_link`. Gazebo rendering sensors
  generate +X-forward point coordinates, whereas ROS optical frames are
  +Z-forward. Images and calibration remain in their optical frames; relabeling
  the unrotated point values as optical was the source of the RViz axis error.
- Cross-validate the perception artifact and bridge `frame_id` at composition
  time. Version the safety policy and bridge as `0.3.0`, perception artifact as
  `0.2.0`, simulator profiles as `0.3.0`, and joystick profile as `0.17.0`.

Validation:

- Full result: `373 tests, 0 errors, 0 failures, 24 skipped` in
  `/tmp/araco_rgbd_build_20260817_01`.
- Headed Gazebo/RViz reached `HOLDING` only after the observed one-second dwell,
  then reported readiness `127/127`, `fault_mask=0`, and
  `reset_required=false` after the renderer and point-cloud stream were active.
- The live `PointCloud2` header was `camera_link`; the live
  `camera_link -> camera_depth_optical_frame` transform remained the canonical
  `[-90 deg, 0, -90 deg]` optical rotation.

## 2026-08-18 — Separate locomotion health from simulation time and qualify headed timing

Status: implemented, regression-tested, and live-validated under joystick
motion with Gazebo GUI, exact meshes, RGB-D rendering, and RViz.

Decision:

- Publish locomotion status from a `50 Hz` steady-time heartbeat rather than
  deriving health publication from every second ROS-time motion tick. Keep gait
  and trajectory progression on ROS time.
- Instrument motion-loop wall gap/execution time and heartbeat callback gaps;
  log exact locomotion receive age when readiness changes.
- Make joint, controller, locomotion, and clock watchdogs generated read-only
  safety parameters. The prior policy file declared these values, but runtime
  still used hard-coded `100/250 ms` values; that mismatch is removed.
- Select a headed-only simulator safety/source policy for
  `gazebo_joystick_v0`: `0.500 s` for joystick reports, teleop candidates,
  selected/safe command, joint/controller/locomotion state, and clock progress.
  Keep provenance at `1.500 s`.
- Preserve strict `gazebo_dev_v0`, CI, and Gate timing artifacts and behavior
  fingerprints. Do not infer a physical watchdog policy from desktop evidence.

Evidence and rationale:

- The original freeze was state `4 -> 6`, reason 15
  (`LOCOMOTION_STALE`), after only locomotion readiness crossed `100 ms`; the
  next status arrived roughly one safety tick later, too late for the latched
  fault.
- Instrumented headed startup then measured whole-process scheduling gaps of
  `280–327.409 ms`, during which joints, controllers, locomotion, and clock all
  aged together. This is host rendering/RViz contention, not an IK failure or
  component crash.
- A first fixed-policy run exposed the same missing margin in the independent
  `120 ms` joystick-report and `150 ms` source-candidate windows, causing a
  recoverable source quarantine. The final policy carries one coherent margin
  through the full joystick-to-safety chain.
- The final headed run remained in `MOTION_ENABLED` for over 110 seconds while
  accepting active joystick gait transitions and logging repeated motion-loop
  gaps up to `153.983 ms`. No source quarantine, readiness drop, safety fault,
  or reset-required transition occurred. One deliberate workspace-boundary
  event used the existing `COMMAND_LIMITED` retreat and recovered to reason
  `NONE`, proving IK protection remained active rather than being loosened.

## 2026-08-18 — Add a deterministic registered-RGB-D RTAB-Map baseline

Status: implemented, statically tested, and live-validated for online map
growth; loop closure and saved-map relocalization remain unqualified.

Decision:

- Keep all accepted flat-ground/control profiles unchanged. Add the dedicated
  `gazebo_perception_v0` profile and `rgbd_validation_v0` arena for perception
  work, with asymmetric colored geometry and a checkerboard to provide stable
  visual/depth features.
- Add a registered simulator sensor variant in which color and depth use the
  same 424 x 240, 15 Hz, 90-degree pinhole geometry. Do not silently mutate the
  earlier generic sensor artifact.
- Activate `araco_navigation` and make it own explicit RTAB-Map RGB-D sync,
  frame-to-map visual-inertial odometry, pose-graph SLAM, the
  `map -> odom -> base_link` chain, a 5 cm occupancy grid, and accumulated 3D
  cloud outputs. Ground truth is forbidden as an estimator input.
- Persist the default mapping database at `~/.ros/araco_rgbd_map.db`; callers
  may select another path through the launch argument.
- Retain dynamic IMU transform checking. Although RTAB-Map occasionally sees a
  2-8 ms future extrapolation under headed render load, this profile can still
  command the yaw gimbal, so treating `camera_link -> base_link` as static would
  be incorrect. Initial mapping runs should keep the gimbal centered. A later
  measured/timestamped IMU preprocessing path or explicit mapping-mode gimbal
  lock must precede claiming dynamic-gimbal SLAM quality.

Evidence:

- The live stack reached safety state 4 with readiness `127/127`, reason 0,
  fault mask 0, and no reset requirement.
- RGB-D odometry tracked active motion with typical feature quality 32-54.
  RTAB-Map grew from one node to working memory 66 with five active local-map nodes,
  published odometry, a 90 x 159 occupancy grid at 0.05 m resolution, an
  accumulated cloud in `map`, and a resolvable `map -> base_link` transform.
- Shutdown saved `/home/stevw-s14/.ros/araco_rgbd_map.db` at approximately
  31 MB. The affected five-package suite reports `369 tests, 0 errors,
  0 failures, 23 skipped`.

## 2026-08-18 — Make brief tracking loss non-destructive and score a closed loop

Status: implemented and live-wiring validated. Operator route 03 proved route,
loop-closure, graph-growth, and cloud-continuity behavior but exposed a scorer
defect; the first route using corrected scorer `0.2.0` is pending.

Decision:

- Set `Odom/ResetCountdown=0`. A single odometry-quality failure must pause
  usable pose updates rather than reset odometry and replace the active cloud
  with a disconnected map segment.
- Add visual-only east/north/west/south/origin route markers to the dedicated
  RGB-D arena. They affect presentation and operator repeatability, not
  collision or robot physics.
- Score one ordered closed loop with `araco_slam_score`. Require route/path
  completion, returned heading, RTAB-Map working-memory growth, at least one
  loop closure, a substantial cloud with no catastrophic replacement, and
  bounded estimated closure error.
- Permit simulator ground truth only inside this observer/scorer. It remains
  forbidden from the RGB-D odometry and SLAM launch graph.
- Treat the initial thresholds as provisional until a first complete route
  produces evidence. Do not tune thresholds from an incomplete or failed run.

Rationale: gimbal motion can plausibly trigger feature-tracking loss, but it was
the one-frame automatic reset—not RViz decay—that turned the loss into the
visible cloud wipe. Disabling destructive reset addresses that failure mode;
the marked route and immutable score make loop closure and continuity claims
repeatable instead of visual impressions.

Smoke evidence:

- The live node reported `Odom/ResetCountdown=0` and recovered from a startup
  `quality=0` frame to normal feature quality without an odometry-reset event.
- The scorer received all five observer streams after its map and cloud
  subscriptions were aligned with RTAB-Map's reliable transient-local QoS.
  No route was driven, so this is wiring evidence only and not a SLAM pass.

Scorer correction after operator route 03:

- Route 03 revealed that acceptance artifact `0.1.0` incorrectly gated closure
  on raw `odom -> base_link`. RTAB-Map applies loop corrections in
  `map -> odom`, so that value cannot measure final graph-corrected SLAM error.
- Artifact `0.2.0` gates translation and yaw closure on `map -> base_link` and
  retains raw odometry translation/yaw drift as explicitly named diagnostics.
- Record exact final simulator truth translation and yaw, rather than only a
  Boolean heading check. Subscribe to `/araco/perception/odom_info` to record
  odometry inlier range, tracking-loss event count, recovery count, total and
  longest loss duration, and whether tracking is available at finish. Tracking
  availability at finish is an acceptance gate.
- Live smoke `/tmp/araco_slam_scorer_v2_smoke_20260818_01/metrics.json`
  received all six observer topics and `map -> base_link`, captured the origin,
  and wrote all corrected/raw/tracking fields. It was interrupted without
  driving, so its `FAIL` is expected and is not route acceptance.

## 2026-08-18 — Isolate the camera-IMU timing defect and use visual RGB-D operationally

Status: implemented, controlled live diagnostics completed, regression-tested;
a fresh complete route remains pending.

Decision:

- Supersede the operational visual-inertial selection in the 2026-08-18 RTAB-
  Map baseline decision. `gazebo_perception_v0` now selects six-DoF RGB-D
  visual odometry without IMU fusion (`araco.navigation.rtabmap-rgbd-sim`
  `0.4.0`). Keep publishing the simulated camera IMU.
- Preserve exact dynamic-gimbal-IMU, fixed-gimbal-IMU, and visual-only
  estimator artifacts and bringup profiles for controlled diagnostics. Do not
  represent the fixed latest-transform fallback as repaired inertial fusion.
- Add synchronized stationary, translation, body-yaw, and gimbal-yaw recording.
  The recorder must publish its command on a dedicated 50 Hz wall-time thread,
  record safety state/reason, and invalidate stale-command or no-motion runs.
- Require final XY and starting yaw together for a two-second dwell, followed
  by ten seconds of tracking-healthy stable corrected pose. Add a visible +X
  arena arrow. A position-only return or an unconverged five-second snapshot
  cannot complete acceptance.

Evidence and rationale:

- Timestamped camera-IMU transform lookup succeeded in only 99–135 samples and
  failed in 1475–1514 samples across representative controlled runs. The old
  dynamic profile therefore had a real timing defect; the fixed profile hid it
  by accepting the latest transform.
- Over about 0.585 m of controlled translation, visual-only corrected error was
  `0.08229 m` versus `0.10386 m` for fixed-IMU. Large body-yaw trials were
  effectively tied at about `0.022 rad` yaw error, with no tracking loss.
- A visual-only gimbal sweep to `0.28 rad` with the body fixed invented only
  `0.0000470 m` translation and `0.000399 rad` yaw. This supports the dynamic
  RGB-D extrinsic and contradicts gimbal motion as the primary drift source.
- All four operational/diagnostic profiles compose from installed artifacts.
  The four affected package suites report `386 tests, 0 errors, 0 failures,
  23 skipped`.

Tradeoff:

- Removing unqualified IMU input is not a claim that visual-only is the final
  physical architecture. It is the more accurate and auditable simulator
  baseline under current evidence. Inertial fusion may return only after its
  timestamp path and gimbal policy pass the same controlled trials.

## 2026-08-18 — Move agent continuity files from `docs/agent/` to `.agent/`

Status: implemented and staged; not committed.

Decision:

- Agent continuity files move from `docs/agent/` to `.agent/`. This supersedes
  the previous `docs/agent/` location convention recorded in `AGENTS.md`.
- Rationale: keeps working state out of documentation builds and out of
  human-facing documentation.
- All 14 files moved with `git mv`, preserving file history as exact renames.
- `docs/` retains `SIMULATOR_DEVELOPER_RUNBOOK.md`, which is operator-facing
  documentation and not agent continuity state.

Scope boundary (the configuration-artifact deferral below is superseded by the
2026-08-18 evidence-source repoint entry):

- Only prose references were repointed: `README.md`, `AGENTS.md`,
  `.agent/CONTEXT.md`, and `.agent/DECISIONS.md` (29 references).
- 48 configuration artifacts under `src/` still carry `docs/agent/...` strings
  in their `evidence.sources` fields. These are deliberately unchanged.
  `load_artifact` hashes the whole document, and `_behavior_fingerprint` folds
  that hash over every artifact whose `deployment_scope` is not `test_only`.
  Rewriting those strings would change the recorded operational fingerprint
  `d7d55a9774692baf62ae4f57c1272f782f0b26e59fc612b97c16c5eeb668b03c` and the
  Gate 6 baseline fingerprints without any behavioral change.
- Those references are therefore stale until a separate change repoints them
  with artifact version bumps, a rebuild, and fresh fingerprint evidence.

Tradeoff:

- The stated rationale is only half-applicable today: this repository has no
  documentation build (no MkDocs, Sphinx, Docusaurus, or CI workflow), so
  nothing was excluded from publication. The move stands on keeping continuity
  state out of human-facing `docs/`.
- Moving the architecture references into a hidden directory reduces their
  discoverability for a repository intended as a public showcase. `README.md`
  still points readers to `.agent/`.

## 2026-08-18 — Repoint configuration evidence sources without version bumps

Status: implemented and verified by composition; staged, not committed.
Supersedes the configuration-artifact deferral in the preceding entry.

Decision:

- Repoint `evidence.sources` in 48 configuration artifacts from `docs/agent/...`
  to `.agent/...` (57 strings). No other field changes.
- Do not bump `artifact_version` for these artifacts. This knowingly relaxes
  the rule in `PARAMETER_AND_CONFIGURATION_COMPOSITION.md` that any changed
  value requires a version change, for provenance-only edits that carry no
  behavioral or numeric change.

Evidence and rationale:

- A version bump is not free here. `composer.py` keys accepted source-authority
  and safety contracts to exact artifact version strings at lines 684, 689,
  705, and 719, and three tests assert exact artifact versions
  (`test_slam_scoring.py`, `test_rtabmap_contract.py`, `test_gate0_bundle.py`).
  A trial bump of all 48 artifacts made every profile fail composition with
  `source authority differs from the accepted contract`.
- Completing the bump would therefore place a documentation-path change inside
  the composer's accepted-contract tables. That is a poor trade for an edit
  with no behavioral effect, so the bump was reverted.
- Artifact identifiers are not unique across files: nine identifiers exist in
  multiple versioned variants, and `araco.bringup.wiring` has two files at
  `0.3.0`. Any future bump must therefore update `selected_artifacts` and
  `dependencies` pins keyed on the exact identifier and version pair.

Fingerprint effect:

- The operational behavior fingerprint moves from
  `d7d55a9774692baf62ae4f57c1272f782f0b26e59fc612b97c16c5eeb668b03c` to
  `32dd967509420327c135167abefa9b2dfc2f5ef0754c5727d2f37db12f7a7aa2`.
- The prior value was reproduced exactly from the unmodified tree before the
  edit, which confirms the recorded fingerprint was accurate and that
  fingerprints are recomputable from source at any time. They were never lost
  with the deleted `/tmp` evidence; only gate run logs were.
- `gazebo_dev_v0`, `gazebo_ci_v0`, `gazebo_joystick_v0`, `gazebo_perception_v0`,
  `gazebo_gate3_v0`, `gazebo_gate4_v0`, and `gazebo_gate5_v0` all compose
  successfully after the change, so development and CI equivalence still holds.

Known defect, not fixed:

- `load_artifact` hashes the whole document, so the provenance `evidence` block
  sits inside behavior identity and a documentation edit shifts a behavioral
  fingerprint. Excluding `evidence` from the artifact hash would fix this
  permanently. It was considered and deliberately not taken in this change
  because it modifies the composer.

## 2026-08-20 — Record the gz-sim teardown failure as a named non-blocking upstream defect

Status: implemented and committed as `8865e3c`; recorded here 2026-08-22. Not
yet exercised by a gate run — no gate has been run since 2026-08-19, so the
classification is covered by unit tests only.

This is option 3 of the three recorded in the Defect C section of
`WORKING_STATE.md`. It is a gate-contract change and was the operator's call,
not the agent's.

Decision:

- Gates 1-5 and Gate 6 split launch-log error lines into a named
  `gz_sim_shutdown` upstream defect and everything else. Only the second kind
  fails a gate.
- Options 1 (symbolize the core and file upstream) and 2 (upgrade or patch
  Gazebo) remain open and unchosen. This change records the defect; it does not
  fix or close it.
- The runner's 5 s shutdown wait was **not** enlarged. That was explicitly
  rejected in the Defect C options because it would hide a real crash.

Rationale:

- The defect is established as an upstream shutdown race in gz-sim `8.11.0`
  reached through `gz_ros2_control` `1.2.19`, reproducible only when the robot
  model is spawned, and absent from both a stock Gazebo world and our own
  `resolved_world.sdf` without the robot. No Araco source change fixes it.
- Both failure modes occur strictly after all scored behavior has completed and
  after `metrics.json` has been written. Conflating them with
  `launch_log_clean` reported a clean, fully scored run as a failure and
  blocked Gates 1, 2, 4, 5 and 6 on a condition no Araco change can clear.

Implementation:

- `gate1.py` gains `classify_launch_log(text, scored_complete, stop_requested,
  escalated)`, returning `shutdown_defect` and `unclassified` line lists.
- `araco_gate1_evidence` is the shared runner for Gates 1-5, so one change
  covers all five. `launch_log_clean` is now `not unclassified`. `launch_exit`
  accepts a non-zero return code only when scoring completed, a stop was
  requested, a shutdown-defect line was actually seen, and nothing is
  unclassified.
- Both `validation_report.json` and `gate_result.json` carry an
  `upstream_defects.gz_sim_shutdown` block with `observed`, `blocking: false`,
  the launch return code, whether the runner had to escalate, and every
  attributed line. The condition is tracked in evidence, not suppressed.
- `gate6.py` gains `is_gz_shutdown_defect`; `classify_logs` routes matching
  lines to a `shutdown_defect` list that `no_unclassified_error_or_fatal` does
  not count. The full list is written to the gate 6 result under
  `log_classification`.

Guards against masking a real failure:

- Attribution in Gates 1-5 requires **both** that `metrics.json` exists and
  that a valid server stop was requested. A crash before scoring completes
  stays blocking.
- Only `[gazebo-1]` may be excused for a crash signal (`139`, `-11`, `-9`) or a
  `Segmentation fault` / `failed to terminate` signature. A crash in any other
  process stays blocking.
- Group-signal deaths (`-2`, `-15`) are excused only when the runner actually
  had to escalate, which is recorded as `escalated` in `process_outcomes.json`.
  Without escalation a signalled death is unexplained and blocks.
- `Traceback` is never attributable to the defect.
- Six unit tests in `test_gate1_scoring.py` pin each of these, including the
  negative cases.

Known limitation, accepted deliberately:

- `gate6.is_gz_shutdown_defect` has no `scored_complete` / `stop_requested`
  guard, because it scans a whole log tree rather than one scored run. It
  therefore excuses a `[gazebo-1]` crash signature or a signalled death
  wherever it appears, including mid-repetition. The protection is that Gate
  6's preflight and per-repetition sub-gate results fail independently of this
  classification, so a mid-run crash still fails Gate 6 through the affected
  gate rather than through `no_unclassified_error_or_fatal`. This is weaker
  than the Gates 1-5 guard and is the place to tighten first if a real crash is
  ever missed.
- Consequence to accept openly: a Gate 1/2/4/5 PASS no longer means the
  simulator exited cleanly. It means every scored check passed and any unclean
  exit matched the recorded upstream signature. Read
  `upstream_defects.gz_sim_shutdown.observed` to tell the two apart.

## 2026-08-22 — Reap orphaned simulators and assign Gate 6 sub-gate domains explicitly

Status: implemented, tests green, uncommitted. Operator asked for both changes
after the 2026-08-22 Gate 6 run failed its last two checks.

Decision:

- `araco_gate6_evidence` reaps leftover `gz sim` servers after every sub-gate,
  and assigns each sub-gate an explicit `--domain-id`.
- Neither change touches the gate contract. No check was added, removed, or
  relaxed, and the reaping is recorded as evidence rather than hidden.

The fix, and the hardening, are not the same thing:

- **Reaping is the fix.** The upstream gz-sim teardown hang leaves the real
  server alive — it is a child of the `/bin/sh` wrapper, so signalling the
  launch process group misses it, and being deadlocked in `futex_do_wait` it
  ignores SIGTERM. Four such servers accumulated during one Gate 6 run, one at
  75% CPU. They starved the next sub-gate until its own `/clock` publishing
  stuttered and the supervisor latched `REASON_TIME_DISCONTINUITY`, which broke
  repetition 1 at gate 3 and failed both remaining checks.
- **Explicit domains are hardening only.** An earlier reading of this failure
  blamed `/clock` crosstalk on a shared DDS domain. **That was wrong and is
  withdrawn.** `araco_gate1_evidence` already isolates every run with
  `ROS_DOMAIN_ID = 100 + os.getpid() % 100` and a unique `GZ_PARTITION`, and the
  domains recorded in the failing run were all distinct. What the explicit
  assignment removes is a narrow residual risk: `pid % 100` collides whenever
  two sub-gate pids differ by exactly 100, which is reachable across 24
  sub-gates that each spawn dozens of processes. Worth closing, but it was not
  the cause.

Implementation:

- `gate6.sub_gate_domain_id(index, base)` assigns ids in the existing 100-199
  band, distinct for any attempt under 100 sub-gates. A full attempt is 24: six
  preflight plus six in each of three repetitions.
- `gate6.orphan_server_pids(output, exclude)` parses `pgrep -f` output. Matching
  is on the **argument list**, never the process name: the server's name is
  `ruby`, so `pgrep -x "gz sim"` silently matches nothing. An orphan sampler
  written that way reported zero servers through a run that leaked four.
- `reap_orphan_servers()` sends SIGTERM with a 2 s grace, then SIGKILL with
  another 2 s, and records which signal worked per pid. The grace is short
  because a deadlocked server never answers SIGTERM, and the wait is only paid
  when a server actually leaked.
- Reaping runs only after a sub-gate has returned, so any surviving server is
  leaked by definition and safe to kill.
- Gate 0 is skipped for `--domain-id`: it composes configuration only and its
  parser does not accept the flag.
- Every reaped pid reaches the result under `metrics.orphan_servers_reaped`
  with its scope, gate, and signal. Reaping stops a leak from poisoning the next
  sub-gate, but it is still an occurrence of the upstream defect and must stay
  visible.
- Four unit tests added in `test_gate6.py`. Package suite: 71 tests, 0 failures.

Deliberately not done:

- No new pass/fail check for orphan reaping. That would be a contract change,
  and the contract question was settled separately on 2026-08-20.
- No change to `suite_wall_budget` or its thresholds. See the open question
  recorded in `WORKING_STATE.md`: `planned_complete_suite_sim_s` is 100.0 while
  Gate 4 alone now consumes 77.8 simulated seconds, so the budget may be stale
  relative to how much the suite has grown. That is a threshold decision and
  needs its own evidence.
