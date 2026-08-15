# Araco Hexapod — Working State

Updated: 2026-08-15

## Current goal

Complete and review the robot-description contract from the restored Fusion and legacy evidence before beginning the greenfield implementation.

## Completed

- Confirmed the new repository is nearly empty and on branch `main`.
- Inspected the full legacy Python package, package metadata, launch files, URDF structure, RViz config, Gazebo world, Isaac Sim asset inventory, and generated workspace layout.
- Inspected the documentation repository README and local render at documentation commit `cacdaddab38624e96855ba56a17da0e5a7f7fb5b`.
- Reconstructed the legacy ROS graph, gait/IK pipeline, hard-coded geometry, servo channel map, and principal architectural and safety concerns.
- Created the initial handoff-ready continuity files.
- Completed the first user interview round and confirmed the unchanged physical revision, servo allocation, open-loop nature, successful legacy behaviors, compute split, priorities, test setup, simulator direction, and current no-code constraint.
- Corrected the initial characterization: locomotion is not presently failing; the rebuild targets architecture and future capability while preserving verified behavior as regression evidence.
- Inspected the current workstation read-only and found Ubuntu 26.04, i9-13900H, 61 GiB RAM, RTX 4060 Laptop GPU, no `/opt/ros` installation, and a presently non-functional `nvidia-smi` connection.
- Compared the legacy servo protocol to official Hiwonder documentation; the controller is likely an LSC-32 but awaits physical confirmation.
- Completed the second interview round: confirmed direct UART wiring, hold-last-command behavior, collapse on servo power-off, PXN-2113 Pro joystick, REP-103 coordinates, 4 GiB Pi running Debian 13.2, Fusion 360/Windows dependency, planned IMU/contact sensing, and simulator-first development.
- Confirmed that local Isaac Sim previously achieved only about 10–15 FPS and local Isaac Lab is impractical; cloud GPU execution must be evaluated.
- Reviewed current official Isaac Sim and Brev cloud guidance: NVIDIA documents a one-L40S Brev deployment, Docker Compose/web streaming, IP-restricted ports, persistent-on-stop workspace behavior, hourly compute billing, and capacity/data-loss caveats.
- Reviewed current Isaac Sim URDF import behavior and recorded the preliminary Fusion 360 → reviewed URDF/Xacro → tuned USD model pipeline direction.
- Confirmed Windows/Ubuntu 24.04 dual boot as the supported workstation direction, current Isaac Sim/Isaac Lab as the target, and legacy Isaac Sim 4.5 assets as migration references only.
- Captured the preferred initial USD 30/month cloud budget (explicitly flexible when justified), approximately three-hour interactive-session expectation, allowance for longer training, and measured 56/27.4 Mbit/s network connection.
- Confirmed that CAD-derived mass estimates may bootstrap simulation, simulator acceptance criteria are required, and the repository will be a public personal-project showcase with a write-up.
- Compared current Gazebo Harmonic and Webots support. Both support Ubuntu 24.04, ROS 2 Jazzy, `ros2_control`, and headless operation; Gazebo was selected as the local/CI companion because it is the official Jazzy pairing and Webots adds no current unique requirement.
- Verified Isaac Sim 6.0's Ubuntu 24.04/ROS 2 Jazzy direction and recorded that Isaac Lab 3.0 Beta/Newton must not be adopted as a stable baseline without an explicit risk decision.
- Found an indicative June 2026 Brev L40S example at USD 2.69/hour compute plus USD 0.04/hour storage; an actual quote is still required before provisioning.
- Confirmed the user's explicit 3D SLAM and Nav2 goal. Verified Gazebo Harmonic RGB-D/depth/IMU/point-cloud support, `ros_gz` standard ROS message bridges, and Nav2 `PointCloud2` voxel integration.
- Recorded the accepted perception/navigation boundary: RTAB-Map evaluation for RGB-D/IMU mapping and localization, 2D map projection plus live voxel obstacles for Nav2, fixed gimbal initially, and no ground-truth odometry in SLAM acceptance claims.
- Assessed Gazebo Harmonic for RL: it has the required headless, step, reset, state, command, and sensor primitives, but no mature official GPU-vectorized Gym workflow comparable to Isaac Lab.
- Recorded the accepted RL split: Isaac Lab for primary training; Gazebo for environment debugging, small experiments, policy playback, robustness, and cross-simulator validation. Raw 24-joint sim-to-real control is not a credible near-term target without physical joint/contact feedback.
- User accepted the complete two-simulator portfolio, RGB-D SLAM/Nav2 boundary, and RL simulator boundary on 2026-08-15.
- User scoped the first autonomous-navigation milestone to flat ground on 2026-08-15; uneven terrain, slopes, steps, and foothold planning are deferred.
- User accepted the first SLAM deliverables on 2026-08-15: six-DoF localization, loop-closing pose graph, saved/reloadable database, 2D Nav2 occupancy, live 3D voxel obstacles, and a downsampled colored 3D map. Dense volumes, meshes, and terrain maps are deferred.
- User explicitly deferred reinforcement learning on 2026-08-15. Initial functionality must come from deterministic algorithmic baselines; RL may be reconsidered only after locomotion, state estimation, SLAM, and navigation work reliably.
- Inspected the workstation's existing dual-boot partition layout on 2026-08-15. The current Ubuntu root is `/dev/nvme0n1p6` (149.5 GiB); Windows, EFI, and recovery data occupy `p1` through `p5` and must be preserved.
- Established that Ubuntu 24.04 installation should occur before ROS package implementation, but only after an off-partition backup and restore verification.
- Identified two installation blockers: the new repository's continuity files are untracked, and the approximately 2.2 GiB legacy workspace is not version-controlled.
- Completed the clean transition to Ubuntu 24.04.4 while preserving Windows and restored the new repository, legacy workspace, and Fusion exports.
- Installed and validated the NVIDIA driver, ROS 2 Jazzy Desktop, `rosdep`, Gazebo Harmonic, `ros_gz`, `ros2_control`, and `gz_ros2_control`; ROS Jazzy is sourced automatically in new Bash terminals.
- Passed a headless ROS–Gazebo integration test with simulated joint-state feedback and commanded trajectory motion; no physical robot commands were issued.
- Inventoried the current F3Z/STEP assembly and reconciled it against the legacy URDF, inverse kinematics, and servo mapping.
- Produced a proposed 26-primary-link/25-joint naming and ownership contract in `ROBOT_DESCRIPTION_MANIFEST.md`; no meshes, URDF/Xacro, controllers, or implementation packages have been generated.
- User confirmed forward toward the gimbal/cameras, front/middle/rear leg numbering, inclusion of Raspberry Pi Camera Module 3, Gemini ownership under the yaw gimbal, and the proposed descriptive names.

## In progress

- Review and resolve the proposed robot-description manifest before authorizing implementation.

## Blockers

- The simulator-side architecture can be designed before hardware details are complete, but the later hardware boundary still requires controller identification, joint limits, and safety behavior.
- Hardware actuation must not be tested until servo mappings, joint limits, safe poses, power isolation, emergency-stop behavior, and test procedure are explicitly validated.
- “Freeze on fault” is not yet a defined safe state: an open-loop hexapod holding its last pose may remain loaded, while removing PWM/servo power may cause collapse.
- The STEP export contains placements and geometry but no joint, material, mass, or inertia definitions. Exact pivots, axes, zeros, safe limits, and physical properties require Fusion joint data or direct measurements.
- The proposed base datum and STEP-to-REP-103 conversion require visual/user confirmation; the STEP gimbal component origin cannot be treated as its yaw pivot.

## Files changed in the new repository

- `docs/agent/CONTEXT.md`
- `docs/agent/DECISIONS.md`
- `docs/agent/WORKING_STATE.md`
- `docs/agent/ROBOT_DESCRIPTION_MANIFEST.md`

## Validation performed

- Read-only source and asset inventory of `/home/stevw-s14/Desktop/Araco`
- Read-only inspection of package, launch, Python, URDF, SDF, and RViz files
- Public documentation repository cloned to temporary storage and inspected
- Read-only workstation OS/CPU/RAM/GPU/ROS/disk inventory
- Current ROS 2 Jazzy, `ros2_control`, Isaac Sim, Isaac Lab, and Hiwonder primary documentation reviewed
- Current NVIDIA Brev instance lifecycle, GPU catalog, Isaac Sim Brev deployment, livestream security, and URDF importer documentation reviewed
- NVIDIA driver, ROS 2 Jazzy, `rosdep`, Gazebo Harmonic, and ROS–Gazebo integration validated on Ubuntu 24.04.4
- Read-only F3Z metadata and STEP product/occurrence/placement inventory of `/home/stevw-s14/Desktop/araco-assembly-files`
- Legacy 25-joint tree, geometry, servo-ID/model, and calibration mapping reconciled with the CAD occurrences
- Proposed manifest checked for exactly 25 unique legacy joint names, servo IDs, and STEP occurrences
- No legacy files changed; no robot commands issued; no build or hardware test performed

## Exact next steps

1. Open the hashed F3Z snapshot in Fusion 360 and inspect both `Joints` and `As-Built Joints`; capture their complete contents without modifying the assembly.
2. Depending on whether usable Fusion joints exist, export or establish the 25 revolute joint pivots, axes, rest angles, and any motion limits.
3. Confirm CAD materials and obtain mass, center-of-mass, and inertia data for each proposed rigid body; resolve the legacy `L1E1` mass outlier.
4. Validate servo IDs, models, directions, finite safe limits, neutral pose, and startup behavior without commanding the physical robot.
5. Accept or revise the robot-description contract.
6. Only after explicit authorization, scaffold `araco_description` and create reviewed Xacro, visual meshes, simplified collision geometry, and simulator overlays.
