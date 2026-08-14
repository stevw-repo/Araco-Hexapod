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

- System boundary and compute/deployment topology
- ROS 2 package and node boundaries
- Command, state, diagnostics, calibration, and safety interfaces
- Control-loop ownership, rates, timing model, and language(s)
- Kinematics/dynamics libraries versus custom implementations
- `ros2_control` adoption and hardware-interface design
- Source-of-truth and conversion workflow between Fusion 360, URDF/Xacro, and Isaac USD
- Navigation, perception, and autonomy scope
- 3D SLAM implementation and map products, odometry source, Nav2 costmap projection, and camera/gimbal operating policy
- Testing gates and phased delivery plan
- Safe behavior on command loss, process failure, Wi-Fi loss, low power, invalid commands, startup, and shutdown
- Detailed local-versus-cloud workflow, GPU instance class, persistence, remote rendering, and exact cost controls

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
