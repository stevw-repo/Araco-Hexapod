# Araco Hexapod — Project Context

Last verified: 2026-08-16

## Goal

Build a new ROS 2 software system for the Araco hexapod from the ground up, including architecture, control logic, packages, interfaces, simulation, hardware integration, tests, and documentation. The legacy project is evidence and a behavioral reference, not an architecture to preserve.

## Repositories and source material

- New writable repository: `/home/stevw-s14/Desktop/Araco Project/araco-hexapod-temp`
- Legacy workspace (read-only reference): `/home/stevw-s14/Desktop/Araco`
- Documentation repository: `https://github.com/stevw-repo/Araco-Hexapod`
- Documentation repository was inspected at commit `cacdaddab38624e96855ba56a17da0e5a7f7fb5b` (2026-03-05).

## Verified product facts

- Araco is a third-generation, 3D-printed hexapod project begun in late 2022.
- Each of six legs has four actuated joints. The fourth joint is intended to keep the foot/end segment vertical relative to the ground for grip and contact area.
- The robot description also contains a one-axis yaw gimbal, for 25 modeled/commanded joints total.
- Current locomotion is a function-defined tripod gait. Translation can be rotated into arbitrary planar directions and blended with a separate yaw-rotation trajectory.
- Intended longer-term capabilities include depth vision, SLAM, Nav2, and potentially reinforcement learning with Isaac Lab.
- The documentation identifies ROS 2 Jazzy, Ubuntu 24.04 on the development laptop, ROS 2 in a Docker image on Raspberry Pi OS, and Isaac Sim 4.5.0.
- The legacy physical system is behaviorally successful, not currently failing: it can stand, walk forward/backward, strafe, turn, combine translation with turning, adjust body height/orientation, and control the gimbal.
- The published demo shows the latest physical behavior and was produced by the inspected legacy code.
- Basic RViz robot rendering/visualization and Isaac Sim operation were also achieved. Depth-camera integration was not implemented.
- The rebuild is therefore primarily an architecture, safety, maintainability, portability, observability, and extensibility project—not recovery of a presently broken locomotion behavior.

## Confirmed physical hardware

- Raspberry Pi 5B
- 18 × DS3235 35 kg leg servos: coxa, tibia, and fourth/foot joints
- 6 × DS5160 60 kg servos: femur joints
- 1 × DS3235 35 kg servo: gimbal yaw
- Hiwonder 32-channel digital servo controller
- PiSugar 3 Plus Raspberry Pi battery module
- 7.4 V 7200 mAh lithium battery
- ORBBEC Gemini 335, installed on and rotating with the yaw gimbal
- Raspberry Pi Camera Module 3, physically installed

The Raspberry Pi is powered by the PiSugar battery module. The servo controller and servos are powered by the separate 7.4 V battery. Exact power distribution, protection, grounding, and regulator details remain unknown.

Neither camera has been integrated into the ROS system. The Gemini IMU may be
usable but has not been integrated. No actuator, joint, force/contact, power, or
other robot-state feedback is currently integrated. The servo system is
believed to be strictly open-loop.

The Raspberry Pi Camera Module 3 is explicitly deferred and may be omitted from
the operational software entirely. It must not constrain simulator development
or the Pi OS/runtime choice. Gemini 335 remains the primary planned RGB-D sensor.

For presentation, the exact Gemini 335 vendor exterior is integrated as three
visual-only meshes on `camera_link`. The imported Fusion `0.4.0` / specification
`3.0.0` bundle covers 59 exports and 77 reviewed bodies. Its camera scope is 15
exterior bodies (housing/bracket, pads/fasteners, and four optics); internal
connector/PCB geometry and the separate Raspberry Pi camera remain excluded.
The complete presentation model has 49 exact mesh visuals and 2,066,740
triangles. This visual upgrade does not calibrate or change simulated optical
frames, collision, dynamics, or sensor behavior.

The physical robot is confirmed unchanged from the hardware used with the legacy code and March 2026 documentation. Total mass is not measured; the user's estimate is 2–4 kg.

The raw Fusion version 2 physical-property export remains unsuitable as direct
dynamics input. Although its 32 direct occurrence records are complete and
mathematically valid, 707 of 732 body occurrences are still assigned Steel and
the calculated assembly mass is `9.804328 kg`.

The user confirms that the remaining Steel bodies in the coxa, tibia, frame,
gimbal-mount, and gimbal groups represent servos rather than printed structure.
They must receive measured/manufacturer-mass-informed effective density, not be
changed to PETG. The same mass-equivalent treatment applies to composite
electrical assemblies such as the Raspberry Pi and cameras.

Initial component-mass research found candidate masses of `60 g` for each of 19
DS3235 servos, `158 g` for each of six DS5160 servos, `97 g` for the Gemini 335,
`4 g` for Camera Module 3, and about `26 g` for the probable Hiwonder LSC-32.
The candidate servo total is `2.088 kg`. The user explicitly accepts a rough
initial simulator estimate rather than requiring full physical accuracy. The
derived `rough_estimate_v0` preserves the raw export, replaces Steel-derived
servo/electronics contributions, and estimates total robot mass at
`3.924393 kg`. It uses round assumptions of `0.050 kg` for the installed Pi,
`0.150 kg` for PiSugar, and `0.400 kg` for the unknown main battery. Phase 1
adds explicit PiSugar, main-battery, and servo-controller proxy poses and a
positive-valid aggregate `base_link` center of mass and inertia. Those values
are simulator estimates, not measurements. Offline derivation evidence remains
under `tools/fusion/`; the runtime snapshot is owned by `araco_description`.

## Legacy software inventory

The legacy workspace mixes source, generated output, launch files, robot assets, and simulator assets at its root:

- `src/Araco`: one `ament_python` package named `Araco`, version `0.0.0`
- `launch`: four near-duplicate, non-installed launch files
- `urdf`: URDF and STL meshes
- `isaacsim`: USD assets and configuration layers
- `models`: a Gazebo Sim world referencing the URDF by an absolute path
- `Rviz`: RViz configuration
- `build`, `install`, `log`: generated colcon artifacts retained in the workspace

Legacy package executables:

- `receiver`: `/joy` (`sensor_msgs/Joy`) → `/received` (`std_msgs/Float32MultiArray`)
- `algo`: `/received` → gait/body/IK pipeline → `/joint_states`, six foot-point debug topics, and `base_link → ground` TF
- `servodriver`: `/joint_states` → calibrated PWM values for 25 channels → Hiwonder serial protocol on `/dev/ttyAMA0` at 9600 baud
- `depthtocloud`: camera depth image plus intrinsics → point cloud

Nominal legacy control path:

```text
joy_node
  /joy
    → receiver
      /received (positional float array)
        → algo (nominal 200 Hz timer)
          /joint_states (24 leg joints + gimbal yaw)
            → servodriver
              Hiwonder serial frame (/dev/ttyAMA0, 9600 baud)
```

## Verified legacy geometry and calibration

- Leg link lengths hard-coded by IK: 43 mm, 120 mm, 120 mm, 50 mm.
- Nominal foot radius from body center: 280 mm.
- Nominal body height in the algorithm: 80 mm, represented as `z = -80 mm`.
- Leg attachment offsets hard-coded in the algorithm are approximately `(±66.82, ±81.82, 0)` mm and `(±90, 0, 0)` mm.
- Servo IDs used: right-side groups `1–13`, left-side groups `16–30`, gimbal `31`; the exact gaps and ordering are encoded in `servodriver.py`.
- Each servo has a separate measured PWM endpoint pair in the legacy driver. Preserve these values as migration data until they are revalidated; do not treat them as safe without a hardware calibration procedure.
- The user confirms those PWM calibrations remain valid, were produced using calibration tools, and no servo or horn position has changed.
- The user reports nominal `270 deg` servo travel. Installed mechanical joint
  limits remain unknown; servo travel alone is not a safe joint limit.
- From the six nominal foot-center positions at a 280 mm radius, the code implies a nominal foot-center bounding box of approximately 560 mm × 448 mm. This excludes the physical foot dimensions and is not a measured overall footprint.

## Verified legacy deployment and operation

- Laptop: joystick connection and substantive locomotion algorithm.
- Raspberry Pi: received servo positions and drove the servo controller.
- Inter-computer transport used Wi-Fi. A robot access point is available but inconvenient.
- The exact controller model/revision, serial-device identity, and baud rate are not currently confirmed despite `/dev/ttyAMA0` and 9600 baud being hard-coded in the legacy driver.
- The servo controller is connected directly to the Raspberry Pi with TX, RX, and GND.
- If the command stream stops while power remains on, the servos retain their last commanded positions.
- Switching servo-controller power off while the robot is standing causes the robot to collapse.
- There have been no known damaging incidents, near misses, overheated servos, broken parts, unexpected full-range movements, or battery/power problems.
- A physical switch on the servo controller can cut or control servo operation; its exact electrical behavior and suitability as an emergency stop remain to be verified.
- The available hardware-test setup is the robot suspended with its legs unloaded.

The legacy protocol and vendor documentation strongly indicate that the controller is a Hiwonder LSC-32: the documented LSC-32 uses the same 32 PWM channels, 500–2500 position range, 5–8.4 V input, 7.4 V battery support, serial protocol, and 9600 baud. This is a high-confidence identification, not yet physically verified from its label or PCB.

## Current development workstation

The workstation was reinstalled and revalidated on 2026-08-15:

- MSI Stealth 14 Studio-class host (`Stealth-14Studio-A13VF` hostname)
- Windows/Ubuntu dual boot with Ubuntu 24.04.4 LTS, x86-64
- Intel Core i9-13900H, 14 cores / 20 logical CPUs
- 61 GiB RAM and 8 GiB swap
- Intel Iris Xe plus NVIDIA GeForce RTX 4060 Laptop GPU
- Validated NVIDIA driver
- ROS 2 Jazzy Desktop installed and automatically sourced in new Bash terminals
- `rosdep`, Gazebo Harmonic, `ros_gz`, `ros2_control`, and `gz_ros2_control`
  installed and validated

There is no project need to install Isaac Sim, Isaac Lab, or a standalone CUDA toolkit as part of the initial workstation bootstrap. The initial post-install gate is a working NVIDIA driver, ROS 2 Jazzy, Gazebo Harmonic through the supported Jazzy integration, RViz, and basic ROS/Gazebo/joystick smoke tests.

Past local Isaac Sim performance was only about 10–15 FPS, and the laptop is not expected to run useful Isaac Lab training. Cloud GPU development, including NVIDIA Brev, must be evaluated as a first-class option.

## Current Raspberry Pi platform

- Raspberry Pi 5B with 4 GiB RAM
- Debian GNU/Linux 13.2 (`trixie`)
- Accepted deployment split: the Pi will eventually own the physical servo
  interface, `ros2_control`, safety/watchdogs, kinematics, gait generation, and
  physical startup/shutdown behavior. The workstation initially owns Gazebo,
  RViz/development tools, RGB-D SLAM, and Nav2.
- Development remains simulator-first. The Pi and physical hardware are not on
  the critical path until simulator milestones and safety contracts pass.
- Heavy simulation and training remain offboard.
- Onboard OS/container choice remains open. Current recommendation is Ubuntu
  Server 24.04 LTS arm64 with native ROS 2 Jazzy for the robot-core processes,
  but the user has not yet accepted it.
- Important OS tradeoff: ROS 2 Jazzy supports Ubuntu 24.04 arm64 as Tier 1,
  whereas current Debian 13.2 is not a Jazzy target. Canonical documents the
  Raspberry Pi CSI camera stack as non-operational on Ubuntu releases before
  25.04, but Camera Module 3 is now deferred/optional and therefore does not
  block the recommended Noble host. Raspberry Pi OS provides the supported
  `rpicam`/Picamera2 stack if this camera is reconsidered later.
- Storage capacity and performance budget remain undecided.

## Accepted simulator control architecture

- The canonical control path is command sources → command arbitration → an
  independent safety supervisor → locomotion → `ros2_control` → a replaceable
  backend.
- One deterministic locomotion process owns body-motion generation, tripod-gait
  phase, foot trajectories, and inverse kinematics. Kinematics remains a pure,
  independently testable library used inside that process rather than a
  separately scheduled ROS node.
- `ros2_control` uses separate control paths for the 24 leg joints and the one
  gimbal-yaw joint, plus joint-state publication.
- Gazebo Harmonic connects through `gz_ros2_control` during simulator
  development. A later Raspberry Pi/servo hardware interface must implement the
  same command/state boundary without changing higher-level locomotion.
- Gazebo provides simulated joint, IMU, and contact state to control,
  supervision, TF, diagnostics, and tests. Simulator ground truth may score or
  diagnose behavior but must not be used as evidence that an estimator works.
- The accepted initial ROS packages are `araco_interfaces`,
  `araco_description`, `araco_kinematics`, `araco_locomotion`,
  `araco_supervision`, `araco_teleop`, `araco_gazebo`, `araco_bringup`, and
  `araco_system_tests`.
- `araco_hardware`, `araco_perception`, `araco_navigation`, and `araco_isaac`
  are phased packages added only when their prerequisite milestone begins.
- The complete accepted responsibility and dependency map is maintained in
  `docs/agent/REPOSITORY_ARCHITECTURE.md`.
- The accepted command path uses four stage-specific interfaces:
  `MotionIntent`, `CommandCandidate`, `SelectedCommand`, and `SafeCommand`.
- Candidate sources use separate configured inputs. Source identity, priority,
  and timeout are assigned by trusted configuration rather than claimed by a
  publisher. Receipt freshness uses local steady time; ROS timestamps remain
  provenance for simulation, logs, and replay.
- The accepted command fields atomically carry planar velocity, an absolute
  body-pose offset, and stand/tripod intent. Sources cannot directly command
  joints, the gimbal, controller topics, or safety state.
- Full field and semantic definitions are maintained in
  `docs/agent/INTERFACE_CONTRACTS.md`.
- `/joint_states` remains the standard 25-joint state interface. The accepted
  `JointStateProvenance` contract identifies each position/velocity/effort
  channel as unavailable, simulated physics, hardware sensed, command derived,
  or estimator produced. Missing velocity/effort stays absent rather than being
  published as misleading zeros.
- The accepted `LocomotionStatus` reports locomotion mode, gait phase/cycle,
  processed safety/selection epochs, per-leg kinematic validity, and whole-
  trajectory validity without claiming controller acceptance or tracking.
- The accepted controller set is `joint_state_broadcaster`, a 24-joint
  `leg_trajectory_controller`, and a separate one-joint
  `gimbal_trajectory_controller`; both controllers use the standard
  `joint_trajectory_controller/JointTrajectoryController` with position
  command interfaces.
- Locomotion sends complete named 24-joint, positions-only, one-point,
  short-horizon `JointTrajectory` messages through the topic interface. Partial
  trajectories and gimbal commands in the leg stream are forbidden.
- Physical joint state remains command-derived until sensors exist. Controller
  feedback/error based on it is not evidence that a servo reached its target.
- Software safety uses eight states independent of ROS lifecycle state, with
  explicit readiness, enable, fresh-source-edge, controlled-stop, stable-hold,
  reset, and shutdown gates.
- Source loss never silently transfers motion to a lower-priority source or
  restores prior permission. Deliberate higher-priority preemption crosses a
  controlled-stop and stable six-foot hold barrier.
- Locomotion commits gait/IK state transactionally across all six legs and 24
  joints. Ordinary hold continuously commands the last validated stance.
- Watchdogs are layered across arbitration, supervision, locomotion,
  controllers, and the backend. Software hold is not an emergency stop, and
  physical servo power-off is not assumed safe.
- The accepted safety states, action/status contracts, readiness/fault masks,
  reason codes, handover semantics, and lifecycle ordering are maintained in
  `docs/agent/SAFETY_ARCHITECTURE.md`.
- Configuration ownership, evidence classes, motion-affecting immutability,
  named profile composition, and fail-closed validation are accepted. Bringup
  composes values from their owning packages rather than redefining them.
- Effective joint limits are the intersection of the canonical model range,
  the later verified actuator range for physical deployment, and a narrower
  operational range. Provisional simulator limits cannot authorize hardware.
- Gazebo progression uses seven ordered blocking gates from static model checks
  through a reproducible headless control/safety baseline. A pass proves
  functional simulator behavior, not physical safety or sim-to-real fidelity.
- The complete configuration and validation contract is maintained in
  `docs/agent/CONFIGURATION_AND_VALIDATION_ARCHITECTURE.md`.
- The accepted simulator runtime baseline uses `1000 Hz` physics, a `250 Hz`
  controller manager, a `125 Hz` joint-state broadcaster, `100 Hz`
  arbitration/safety/locomotion loops, and `50 Hz` teleop publication. It uses
  a `0.040 s` leg-trajectory horizon and layered
  steady-time watchdogs, with a JTC last-line hold at about `0.144 s` after the
  last trajectory receipt.
- Gait/trajectory progression uses ROS simulation time; motion-authority and
  readiness watchdogs use steady time. Pausing Gazebo therefore revokes motion
  readiness and cannot cause surprise gait resumption.
- Source candidates are best-effort latest-value streams. Trusted selected and
  safe commands are reliable latest-value streams. Concrete project topics,
  source IDs/priorities/timeouts, and status QoS are accepted.
- Initial simulator motion is deliberately slow: planar speed is limited to
  `0.050 m/s`, yaw rate to `0.300 rad/s`, the tripod cycle is `1.200 s`, and a
  healthy controlled stop must reach stable hold within `1.500 s`.
- Provisional joint ranges, command-rate/effort caps, damping, friction, DART
  physics settings, and quantitative Gate 0–6 thresholds are simulator-only and
  forbidden as physical calibration or safety evidence.
- The complete accepted numerical contract is maintained in
  `docs/agent/RUNTIME_TIMING_AND_SIMULATION_CONTRACT.md`.

## Accepted configuration composition

- A complete parameter, artifact, profile, override, deterministic-preflight,
  runtime-bundle, and fingerprint contract is accepted in
  `docs/agent/PARAMETER_AND_CONFIGURATION_COMPOSITION.md`.
- The contract keeps human-reviewed source artifacts in their owning packages,
  uses generated typed ROS parameters for project nodes, and has bringup emit a
  validated immutable runtime bundle for each run.
- Joint lists and controller partitions are generated from the canonical
  model registry rather than maintained in controller YAML.
- `gazebo_dev_v0` and `gazebo_ci_v0` select identical behavior artifacts;
  only presentation/reporting fields and closed input-adapter presence recorded
  in the input-selection/run identity may differ. Both use seed `42`, the same
  physics, and the same source registry.
- Phase 1 and Phase 2 subsequently implemented the accepted composition. A
  fresh installed-space CI composition must reproduce the accepted evidence
  fingerprints before a live result is trusted.

## Accepted phased simulator delivery

- `docs/agent/PHASED_DELIVERY_PLAN.md` defines one repository-foundation phase
  followed by one blocking implementation phase for each accepted Gate 0–6.
- Gate 1 establishes the real simulator/controller/lifecycle hold path from the
  single nominal standing reference. Gate 2 then replaces transitional
  target production with typed computed FK/IK using that reference as its
  oracle; it does not introduce another standing authority.
- Gate 3 onward uses the real system-test candidate → arbitration → safety
  → locomotion → controller → Gazebo path for scored behavior.
- Each gate retains structured evidence, reruns all prior gates, forbids
  silent threshold/configuration relaxation, and invalidates all later evidence
  after an affected earlier-gate change.
- Gate 6 requires three clean no-retry headless runs with the accepted
  reproducibility thresholds. It unlocks later simulator phases, not
  hardware.
- This delivery-plan acceptance did not itself authorize Phase 0. The user
  subsequently authorized Phase 0 on 2026-08-16, and the foundation is now
  complete. Phase 1 / Gate 0 and Phase 2 / Gate 1 were later separately
  authorized and completed on 2026-08-16.

## Completed architecture closeout and license selection

- The final cross-contract review is complete in
  `docs/agent/FINAL_ARCHITECTURE_REVIEW.md`. The user subsequently authorized
  Phase 0, and its repository foundation is complete.
- Project-authored code, configuration, documentation, tests, and original
  assets use the MIT License (`MIT`) as of 2026-08-16. This supersedes the
  earlier Apache-2.0 and GPL-3.0-only selections. Root and package-local
  `LICENSE` files carry the MIT text and Phase 0 source uses exact MIT SPDX
  headers.
- Package manifests use `<license file="LICENSE">MIT</license>` and Phase 0
  project source files carry MIT SPDX identifiers where their format supports
  comments.
- The completed Phase 0 audit records linked/bundled dependency, attribution,
  and source-distribution boundaries in
  `docs/agent/PHASE_0_LICENSE_AUDIT.md`.
- Project-authored generated meshes should retain publishable preferred
  editable source and generation tooling. The existing Fusion add-in's
  Autodesk API boundary and the future Isaac adapter's proprietary SDK boundary
  still require terms and redistribution review; MIT does not license those
  third-party systems.
- Vendor CAD and other third-party assets are not relicensed by the repository
  MIT license. Preserve per-asset provenance and upstream license/attribution
  metadata.
- The user confirmed on 2026-08-16 that all CAD they have in the Fusion
  assembly is open source, that licensing is not a blocker, and that exact
  vendor meshes may be used. Presentation-quality recorded simulator visuals
  are an explicit requirement.
- Fusion exporter `0.3.1` established the fail-closed 62-body allowlist packaged as 44
  mesh files: the existing 25 project mechanical bodies, 13 directly attached
  servo bodies, and six complete tibia-component files containing all 24 tibia
  bodies. This covers all 25 servos and every canonical link while retaining no
  visual proxy. Project-authored
  assets remain MIT; vendor bodies are currently marked
  `LicenseRef-UserConfirmed-Open-Source-CAD` until more specific upstream
  attribution metadata is recorded. Unrelated nested electronics and sensor
  internals remain outside this bounded export.
- The current reviewed exporter `0.4.0` / specification `3.0.0` is mirrored at
  `/media/stevw-s14/DATA-ST/New folder/AracoRobotDescriptionExporter`. The
  exact 59-export / 77-reviewed-body bundle was produced, validated, imported,
  and integrated on 2026-08-16. The earlier 25-body and 44-export bundles remain
  preserved as superseded source evidence.
  Fusion emits these bodies in source-component-local coordinates; the bundle's
  recorded occurrence transforms are required to reconstruct the assembly.
- The review reconciled source-registry ownership, profile naming/seed/input
  selection, startup watchdog arming, gimbal hold ownership, backend readiness,
  orderly shutdown, and the simulator-versus-physical robot-description gate.
- The user explicitly authorized and pushed the licensed pre-Phase-0
  architecture/evidence checkpoint on 2026-08-16. They later authorized Phase
  0, which now provides all nine package skeletons under `src/`. The completed
  MIT Phase 0 implementation was committed as
  `2a1b3dcd2545b95570c3b8428b0dabfac37f6f95`, pushed to `origin/main`, and
  verified against the remote on 2026-08-16.

## Completed Phase 0 repository foundation

- Phase 0 completed on 2026-08-16 and claims repository integrity only. It
  claims no Gazebo gate and creates no runnable robot.
- All nine accepted `ament_cmake` packages exist at version `0.1.0` and use the
  verified Git identity `stevw <steven060520@gmail.com>` as public maintainer
  metadata.
- `araco_interfaces` generates the accepted seven messages and one
  `SafetyTransition` action. Generated Python type introspection is tested
  against the accepted fields, types, fixed arrays, and constants.
- Exact project dependency edges and acyclicity are tested. No Phase 1 model,
  configuration, launch, Gazebo runtime, executable, or hardware content was
  introduced.
- Root and package-local MIT licenses are identical and each package installs
  its `LICENSE` beside `package.xml`.
- Clean validation passed: rosdep satisfied; 9 packages built; 111 tests passed
  with 0 errors, failures, or skips; all packages and all eight interfaces were
  discoverable from the install space.

## Completed Phase 1 and Phase 2 simulator baseline

- Phase 1 / static Gate 0 and Phase 2 / live Gate 1 both pass as of 2026-08-16.
- The Gate 1 runtime uses Gazebo Harmonic with DART, `gz_ros2_control`, exact
  24-leg + 1-gimbal trajectory-controller claims, the complete 25-joint state,
  simulated provenance, robot-state TF, odometry, six foot-contact streams, and
  a non-foot ground-contact alarm.
- Locomotion is intentionally hold-only at Gate 1. It publishes the accepted
  nominal 24-joint standing reference at 100 Hz; no computed IK, gait, body-pose
  command, physical profile, or hardware path exists yet.
- Safety follows `INITIALIZING -> INACTIVE -> HOLDING`, refuses motion enable,
  and exposes the typed `/araco/safety/transition` action for fail-closed hold,
  latch/reset, and orderly shutdown.
- Gate 1 model measurement corrected all femur/tibia/foot pitch axes from the
  proposed local `+Y` to local `-Y` and sets the simulator nominal spawn height
  to `0.10675 m`. This is a simulator geometry correction, not physical servo
  direction or zero calibration.
- Gazebo collision sensors publish at 50 Hz on six independent foot topics and
  one shared non-foot alarm topic. A ROS filter produces the fixed 100 Hz
  aggregate used for scoring. URDF collision references account for Gazebo's
  `_collision` suffix conversion.
- The passing evidence directories are
  `log/gate_0_20260816_phase2_regression/` and
  `log/gate_1_20260816_phase2_hold_pass/`. Both share behavior fingerprint
  `c1c2b51d4e082bcfab0e5d09618566c3a9245ce79d46833295c1eb9ec7922283`;
  the CI run fingerprint is
  `e435b0244f9412f9b88e693f247d6f5f0773cc80c4b02842eebdff9287a4b35b`.
- Clean validation after deleting/recreating generated build/install output is
  9 built packages and 175 tests with 0 errors, 0 failures, and 3 expected
  `cppcheck` skips. Dependency closure, independent URDF parsing, Gazebo SDF
  validation, and fresh fingerprint reproduction also pass.
- The exact post-Phase-2 visual-fidelity task imports 59 approved Fusion
  exports covering 77 reviewed bodies as 34 deduplicated source blobs. It
  deterministically generates 49 meter-scale link-local presentation meshes:
  26 primary, 13 servo-case, seven separable servo-horn, and three Gemini
  exterior role assets. No visual proxy remains; collision, dynamics, joint,
  controller, safety, optical-frame, sensor, and standing artifacts remain
  unchanged.
- Headed Gazebo resource discovery is self-contained: bringup prepends the
  installed `araco_description` share parent to `GZ_SIM_RESOURCE_PATH` before
  launch. The headed run resolves all detailed `package://` resources.
- Fresh detailed-visual evidence passes at
  `log/gate_0_20260816_detailed_visuals/` and
  `log/gate_1_20260816_detailed_visuals_hold_host/`, with behavior fingerprint
  `87eef7bca155569674450e9342856ed89fb7e9f74e3e3df2c11311d8c9a937d7`.
  Gate 1 physics metrics match the proxy-visual baseline within numerical
  precision. The exact-visual configuration builds all nine packages and passes
  182 tests with 0 errors, 0 failures, and 3 expected skips. Fresh formal Gate
  0 / Gate 1 evidence is deferred until user visual acceptance.
- The rejected flat-blue/proxy preview has been replaced by exact Fusion/vendor
  solids, six material roles, directional lighting, ambient light, and
  shadows. The current generated URDF contains 49 meshes and zero visual
  primitives. A headed preview has reached `HOLDING`; final visual acceptance
  is pending.
- The tibia proxy offset seen by the user is real rather than a lighting
  illusion. The Fusion tibia occurrence and canonical kinematic tibia frame
  differ by about `9.7567 mm` transversely and `90 deg` about the longitudinal
  axis. The box intentionally follows the joint-to-joint centerline; exact
  Fusion normalization would preserve the assembly offset. This currently
  limits visual and collision fidelity, but does not alone disprove the
  accepted joint origins or Gate 1 hold behavior.

## Cloud simulation evidence (2026-08-14)

- NVIDIA officially documents Isaac Sim cloud deployment on Brev with streamed access to the desktop.
- NVIDIA's current Brev-specific Isaac Sim instructions select one L40S GPU and run the Isaac Sim container through Docker Compose with a web viewer.
- The documented livestream ports must be restricted to the user's IP because the streaming endpoints do not provide authentication or encryption.
- Brev instances provide GPU drivers, CUDA, Docker, JupyterLab, SSH, and remote-IDE access.
- Brev bills running instances hourly. Stopping releases the GPU and preserves `/home/ubuntu/workspace` with storage charges, but restart can fail when that GPU type is unavailable in the same provider/region.
- Deleting an instance permanently deletes its instance data. Important work must be synchronized to the project Git repository or separate durable storage.
- Current Brev guidance treats 20 GiB VRAM, 500 GiB disk, and Ampere-or-newer compute as general smart defaults; NVIDIA's Isaac Sim instructions are more specific and recommend an L40S.
- Proposed operating pattern: keep Isaac Sim/Isaac Lab and the simulator-side ROS graph together on the cloud VM; use Git for source synchronization and livestream/SSH for human access. Do not expose raw ROS 2 DDS discovery over the public internet.
- Cloud pricing and capacity are dynamic. A spending limit and shutdown/stop discipline are required before provisioning anything.
- Initial cloud-compute budget preference: approximately USD 30 per month, but the user is willing to increase it when justified. A typical interactive session may be about three hours; longer unattended training runs are acceptable. Set an explicit cap from actual provider pricing before provisioning.
- Measured user network connection: approximately 56 Mbit/s download and 27.4 Mbit/s upload. This is adequate for streamed interaction in principle, but latency and stability remain unmeasured.
- Interactive streaming versus headless-only execution is not yet decided; the workflow should support both if practical.
- A June 2026 NVIDIA Developer Forum example showed an AWS `g6e.2xlarge` Brev L40S instance at USD 2.69/hour compute plus USD 0.04/hour storage, and an NVIDIA representative confirmed that shape was appropriate for an interactive Isaac Sim workflow. This is only an indicative example, not a quote for this user, region, or provisioning date. At that example rate, a three-hour session is about USD 8.07 compute and USD 30 buys about 11.2 compute hours before stopped-storage charges.

## Simulator portfolio evaluation (2026-08-14)

Verified current support:

- Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic is the officially recommended ROS/Gazebo pairing. Harmonic is an LTS release with currently documented support through May 2029.
- Gazebo Harmonic runs on Ubuntu 24.04, can run server-only without its GUI, integrates through `ros_gz`, and has a maintained Jazzy `gz_ros2_control` system plugin for URDF/SDF joint command/state interfaces.
- Webots supports Ubuntu 24.04, ROS 2 Jazzy packages, a `webots_ros2_control` plugin, and batch/no-rendering execution. It is a technically viable simulator.
- Isaac Sim 6.0 is generally available and its supported Ubuntu 24.04 path uses ROS 2 Jazzy. Isaac Lab's current release line must be matched to the selected Isaac Sim version at setup time.
- Isaac Lab 3.0 Beta and its kit-less Newton backend are promising but explicitly under active development with missing features and possible breaking changes. Do not make the beta a baseline merely because it is newer.

Accepted simulator portfolio (accepted 2026-08-15):

- Keep one canonical, simulator-neutral ROS robot description and command/state contract.
- Use Gazebo Harmonic as the everyday local functional simulator and headless CI/regression environment.
- Use Isaac Sim/Isaac Lab for advanced sensors, synthetic data, higher-fidelity release validation, demonstrations, and reinforcement learning, usually on cloud GPU.
- Do not initially maintain Webots. Reconsider it only if Gazebo proves materially unreliable for this robot or a specific Webots-only capability is identified.
- Do not demand numerically identical motion across physics engines. Cross-simulator tests should enforce interfaces, frames, joint limits, reachability, gait sequencing, stability envelopes, and bounded behavioral metrics.
- Ultimately, physical measurements and robot behavior—not any simulator—are the authority for sim-to-real fidelity.

## Reinforcement-learning simulator assessment (2026-08-15)

Verified capability:

- Gazebo Harmonic can serve as an RL environment: it supports headless execution, pause/run, controlled stepping, reset, state access, actuator commands, sensors, and programmatic server APIs.
- Gazebo does not currently provide an Isaac-Lab-equivalent, polished and GPU-vectorized RL workflow. The upstream Gazebo RL tracking issue remains open and explicitly identifies Gym API exposure, multi-instance vectorization, randomized environments, deterministic closed-loop sensor access, and improved reset APIs as work still needed.
- A project can wrap Gazebo behind the standard Gymnasium `reset()` / `step()` contract and train through libraries such as Stable-Baselines3, RLlib, or CleanRL, but Araco would own that integration and its reproducibility/performance testing.
- Isaac Lab is purpose-built around many parallel environments and GPU rollout collection; it remains substantially better suited to high-throughput legged-locomotion training.

Accepted RL role split (accepted 2026-08-15):

- Use Gazebo for RL environment debugging, reward/action/observation validation, small experiments, policy playback, robustness tests, and cross-simulator policy evaluation.
- Use Isaac Lab or a later stable Newton/Isaac Lab workflow for primary high-throughput training.
- Preserve a simulator-neutral specification of actions, observations, termination conditions, rewards, randomization ranges, and policy input/output units; implement simulator-specific adapters rather than assuming identical physics or APIs.
- Do not treat a raw 24-joint locomotion policy as a near-term physical deployment target. The current robot has open-loop position servos and no measured joint state or foot contact. Until feedback is added, learned high-level gait modulation or residual commands are more credible physical targets; raw joint policies may remain simulation research.

## Depth perception, SLAM, and navigation direction (2026-08-14)

Confirmed goal:

- The user specifically intends to integrate 3D SLAM and Nav2, initially in simulation and later with the planned ORBBEC Gemini 335.

Verified capability:

- Gazebo Harmonic supports depth cameras, RGB-D cameras, GPU lidar, IMU, contact, force-torque, and segmentation-camera sensors.
- `ros_gz_bridge` supports the ROS 2 sensor interfaces needed for this pipeline, including `sensor_msgs/Image`, `CameraInfo`, `PointCloud2`, `Imu`, and `LaserScan`. The upstream RGB-D bridge example publishes color, depth, camera calibration, and point-cloud streams.
- Nav2 accepts `PointCloud2` from depth or 3D sensors through its voxel layer. That layer maintains limited 3D obstacle data and projects it into Nav2's 2D planning costmap; standard Nav2 is not a general volumetric 3D planner.
- Nav2 requires a valid `map → odom → base_link → sensor` TF chain. The mapping/localization system owns `map → odom`, while the odometry system owns `odom → base_link`.
- RTAB-Map is the accepted first RGB-D SLAM candidate to evaluate on ROS 2 Jazzy: its upstream ROS 2 repository includes RGB-D/3D-lidar sensor examples and Nav2 integration demos. Evaluation does not guarantee final adoption if acceptance tests expose a better option.
- The Gemini 335 is an RGB + active/passive stereo depth camera with a six-axis accelerometer/gyroscope IMU. Orbbec's current ROS 2 wrapper supports Jazzy and the Gemini 335, including RGB, depth, point-cloud, accelerometer, and gyroscope configuration.

Important limitations and implications:

- Gazebo can approximate Gemini 335 resolution, frame rate, field of view, range, mounting pose, update rate, and generic noise. It does not automatically reproduce the camera's stereo/IR failure modes, lighting dependence, transparent/reflective-surface errors, motion artifacts, firmware filters, or exact latency. Isaac Sim and later physical datasets should provide higher-fidelity validation.
- A useful initial navigation architecture is 3D RGB-D SLAM/localization feeding a 2D occupancy projection and `map → odom` transform to Nav2, while live depth `PointCloud2` feeds a local voxel obstacle layer.
- Accepted first SLAM outputs: six-DoF pose estimation, pose graph with loop closure, saved/reloadable mapping database, `map → odom`, Nav2-compatible 2D occupancy grid, live 3D voxel obstacles, and a downsampled colored 3D point cloud for RViz/debugging/showcase use.
- The first SLAM system should estimate six-DoF motion even on flat ground because gait produces vertical, roll, and pitch motion. Nav2 consumes the planar projection while SLAM retains the full pose.
- Dense global volumetric mapping, textured meshing, and elevation/traversability maps are deferred.
- Standard Nav2 is suitable for body-level navigation on mostly traversable ground. It does not by itself determine leg footholds, slope/step traversability, or full 3D paths over uneven terrain; those require an elevation/traversability representation and eventually a terrain-aware or footstep planning layer.
- The physical robot currently lacks measured leg/joint odometry and foot contact. In simulation, ground-truth odometry may be used to test wiring but must not be used to claim SLAM performance. A realistic evaluation needs visual/RGB-D-inertial odometry or another noisy estimated odometry source.
- Mounting the camera on the yaw gimbal creates a dynamic camera extrinsic. In simulation that transform can be exact, but on the real open-loop gimbal it would be inferred from the command and subject to servo error/backlash. Initial SLAM should therefore keep the gimbal fixed/locked unless a measured gimbal angle is added.

## Legacy design and correctness concerns

- One node owns unrelated responsibilities: input filtering, gait state, trajectory generation, body pose, IK, TF, debug visualization, and joint command publication.
- Controller intent is encoded by positional indices in `Float32MultiArray`; no semantic interface, validation, deadman signal, or freshness contract exists.
- `algo` catches all exceptions during input processing and silently continues. Before the first controller message, its timer repeatedly relies on this exception path.
- Most values are hard-coded: geometry, gait curves, controller mapping, rates, topic names, frames, device path, baud, servo IDs, direction signs, and PWM calibration.
- The serial port is opened anew for every received joint message. The command duration is fixed at 300 ms even though joint messages can be produced at a nominal 5 ms interval.
- `/joint_states` is used both as desired actuator command transport and robot-state visualization, conflating command and measured state.
- There is no explicit hardware enable/disable lifecycle, deadman, watchdog, stale-command behavior, emergency stop, startup pose transition, shutdown pose behavior, or joint/velocity/acceleration safety layer visible in the code.
- Analytic IK does not explicitly reject unreachable targets or singularities before inverse trigonometric/division operations, and does not enforce physical joint limits.
- URDF joints are modeled as `continuous` despite finite servo motion; physical limits are not represented.
- The point-cloud message declares fields in `x,z,y` order while serialized data is arranged `x,y,z`, so consumers can interpret axes incorrectly.
- Launch and simulator files contain machine-specific absolute paths, including a different username/path than the inspected legacy workspace.
- Launch files set `use_sim_time: true` even for hardware-oriented launch combinations.
- Package metadata and tests are largely generated placeholders; runtime dependency declarations are incomplete/inaccurate.
- The robot model lacks clearly named foot/contact frames and has no verified simulator-control architecture in the inspected ROS package.

## Current unknowns

The following still require later decisions, implementation evidence, or
physical measurements:

- Physical acceptance criteria beyond the accepted simulator Gate 0–6 baseline
- Exact Hiwonder controller model/revision and physical power/signal behavior beyond the confirmed UART link and hold-last-command behavior
- Physical real-time behavior and acceptable Pi/controller/network latency
- Desired future feedback additions, if any
- Power-distribution/protection details and software safety behavior
- Physical joint-zero verification, safe mechanical limits, and verified
  kinematic dimensions
- Quantified terrain, speed, payload, and endurance goals
- Required Isaac Sim/Isaac Lab fidelity and sim-to-real boundary
- Depth-camera mounting location and whether the gimbal will remain fixed during mapping/navigation
- Odometry source for simulation acceptance and for the later physical robot
- Later physical deployment, networking, calibration, and update workflows
- Verified physical mass/inertia, collision fidelity, and electronics proxy poses
- Cloud simulator/training provider, GPU class, storage/persistence model, remote visualization method, and spending controls
- Exact current Isaac Sim/Isaac Lab release selected at implementation time and the effort required to migrate useful legacy Isaac Sim 4.5 assets
- Initial RL scope: high-level gait adaptation/residual control versus raw 24-joint locomotion, required observations, and whether physical deployment is expected before feedback sensors are added

## Confirmed rebuild priorities and scope

- Initial gait scope: tripod gait only.
- Essential body controls: height, roll, pitch, yaw, and translation.
- Desired later body capabilities may include IMU stabilization, foot-placement adjustment, and gimbal/camera tracking.
- Priority group 1: reliable teleoperation, remote monitoring, Isaac Sim, and Isaac Lab.
- Priority group 2: depth perception, odometry/localization, and SLAM.
- Priority group 3: Nav2 autonomous navigation.
- The first autonomous-navigation milestone targets flat ground. Slopes, stairs/steps, uneven-terrain traversability, and terrain-aware foothold planning are deferred.
- Priority group 4: reinforcement learning and uneven-terrain adaptation.
- Priority group 5: research/data collection.
- The user is open to C++ for timing-sensitive components and Python elsewhere.
- The accepted simulator portfolio uses Gazebo Harmonic as the lightweight local/CI functional simulator and Isaac Sim/Isaac Lab as the advanced high-fidelity and learning environment. Webots is excluded initially unless a later unique requirement or Gazebo failure justifies it.
- Development is simulator-first. The physical robot and its hardware adapter are not an immediate delivery priority.
- Cloud execution should be considered for Isaac Sim and especially Isaac Lab because local performance is inadequate.
- Teleoperation device: PXN-2113 Pro flight joystick.
- Coordinate convention: ROS REP-103 body convention (`+x` forward, `+y` left, `+z` up) is accepted.
- Essential yaw behavior includes twisting the body while the feet remain planted; walking rotation is already part of legacy behavior.
- Initial remote monitoring should aim to include commanded/estimated pose, gait phase, node/process health, CPU temperature/load, Wi-Fi/command freshness, serial status, power information when available, and emergency/fault state.
- Future sensing direction includes the IMU in the planned depth camera and planned foot-contact sensors. Simulation substitutes for these during the initial phase.
- The source Fusion 360 assembly is available only through Windows dual boot. The legacy URDF was exported with a `fusion2urdf` script; whether that remains the right source-to-model workflow is undecided.
- Preliminary model-workflow direction: use the Fusion 360 assembly as CAD truth, preserve a version-controlled and reviewed URDF/Xacro as ROS kinematic/interface truth, and derive/tune USD as the Isaac Sim artifact. An automatic Fusion exporter may bootstrap this, but its joint axes, inertias, limits, collision geometry, naming, and frames must not be accepted without validation.
- CAD-derived mass and inertial estimates are acceptable for the initial simulator model, provided they are clearly labeled as estimates and later validated against the physical robot.
- The current repository is intended to become the final project repository.
- The final repository will be public and used as a personal-project showcase, with a project write-up. Public documentation quality, reproducibility, licensing, and removal of machine-specific/private data are therefore product requirements.
- The rebuild targets the current supported Isaac Sim/Isaac Lab generation at implementation time. Legacy Isaac Sim 4.5 assets are migration references, not a compatibility target.
- Simulator acceptance criteria are required before implementation; the user approved defining explicit model, control, determinism, real-time-factor, sensor, and regression gates.
- Reinforcement learning is explicitly deferred. Initial milestones must use deterministic, conventional algorithms for kinematics, tripod gait generation, body control, teleoperation, state estimation, SLAM, and Nav2. RL may later adapt or improve a proven algorithmic baseline.
- Phase 0 repository/package scaffolding, Phase 1 / static Gate 0, and Phase 2 /
  live Gate 1 are complete. Phase 3 / Gate 2 and every later phase require
  separate authorization.

## Evidence boundary

Facts above marked as legacy behavior describe inspected files, not necessarily
the current physical robot or desired future behavior. The new simulator
architecture, configuration-composition contract, and phased implementation
plan recorded above are accepted. Physical-hardware behavior and later
deployment decisions remain unresolved.
