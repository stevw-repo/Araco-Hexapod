# Araco Hexapod — Working State

Updated: 2026-08-15

## Current goal

Preserve all project material, replace the workstation environment that is incompatible with the selected ROS baseline with Ubuntu 24.04, validate ROS 2 Jazzy/Gazebo Harmonic, and then complete the architecture brief before implementing the greenfield rebuild.

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

## In progress

- Prepare a handoff-safe workstation transition to Ubuntu 24.04.4; no installation or data movement has yet been authorized or performed.

## Blockers

- The simulator-side architecture can be designed before hardware details are complete, but the later hardware boundary still requires controller identification, joint limits, and safety behavior.
- Hardware actuation must not be tested until servo mappings, joint limits, safe poses, power isolation, emergency-stop behavior, and test procedure are explicitly validated.
- “Freeze on fault” is not yet a defined safe state: an open-loop hexapod holding its last pose may remain loaded, while removing PWM/servo power may cause collapse.
- Ubuntu reinstallation is blocked until the new repository, untracked continuity files, legacy workspace, and other required Linux-side data are backed up outside `/dev/nvme0n1p6` and the backup is verified.

## Files changed in the new repository

- `docs/agent/CONTEXT.md`
- `docs/agent/DECISIONS.md`
- `docs/agent/WORKING_STATE.md`

## Validation performed

- Read-only source and asset inventory of `/home/stevw-s14/Desktop/Araco`
- Read-only inspection of package, launch, Python, URDF, SDF, and RViz files
- Public documentation repository cloned to temporary storage and inspected
- Read-only workstation OS/CPU/RAM/GPU/ROS/disk inventory
- Current ROS 2 Jazzy, `ros2_control`, Isaac Sim, Isaac Lab, and Hiwonder primary documentation reviewed
- Current NVIDIA Brev instance lifecycle, GPU catalog, Isaac Sim Brev deployment, livestream security, and URDF importer documentation reviewed
- No legacy files changed; no robot commands issued; no build or hardware test performed

## Exact next steps

1. Copy the complete new repository (including untracked `docs/agent/`) and `/home/stevw-s14/Desktop/Araco` to storage that will survive replacement of `/dev/nvme0n1p6`; preserve any other required Linux-side personal/configuration data and verify restoration from the copy.
2. Confirm Windows still boots, preserve the Windows/BitLocker recovery information if applicable, download the official Ubuntu 24.04.4 desktop amd64 image, verify its checksum, and live-boot it to test essential laptop hardware.
3. Clean-install Ubuntu 24.04 only into the existing Linux partition while preserving EFI, Windows, and recovery partitions. Do not select an erase-entire-disk option.
4. Install and validate the NVIDIA driver, ROS 2 Jazzy, Gazebo Harmonic/Jazzy integration, RViz, development tools, and joystick support. Do not install Isaac Sim/Lab or a standalone CUDA toolkit yet.
5. Restore the repositories and verify the continuity and legacy files before any project implementation begins.
6. Resolve the Fusion 360 → repository model → simulator artifacts workflow and define simulator-model validation criteria.
7. Define the deterministic locomotion, body-control, command, state, and safety architecture, then finish quantified simulator, SLAM, and navigation acceptance criteria.
8. Produce the simulator-first architecture and phased delivery brief without writing implementation code; scaffold packages only after the user explicitly authorizes code.
