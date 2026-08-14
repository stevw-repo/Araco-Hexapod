# Araco Hexapod — Project Context

Last verified: 2026-08-15

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

The Raspberry Pi is powered by the PiSugar battery module. The servo controller and servos are powered by the separate 7.4 V battery. Exact power distribution, protection, grounding, and regulator details remain unknown.

Planned but not installed:

- ORBBEC Gemini 335 depth camera; its IMU may be usable but has never been integrated
- Raspberry Pi Camera Module 3

No actuator, joint, force/contact, power, or other robot-state feedback is currently integrated. The servo system is believed to be strictly open-loop.

The physical robot is confirmed unchanged from the hardware used with the legacy code and March 2026 documentation. Total mass is not measured; the user's estimate is 2–4 kg.

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

Read-only inspection on 2026-08-14 found:

- MSI Stealth 14 Studio-class host (`Stealth-14Studio-A13VF` hostname)
- Ubuntu 26.04 LTS, x86-64
- Intel Core i9-13900H, 14 cores / 20 logical CPUs
- 61 GiB RAM and 8 GiB swap
- Intel Iris Xe plus NVIDIA GeForce RTX 4060 Laptop GPU
- Approximately 112 GiB free on the repository filesystem
- No ROS distribution under `/opt/ros`
- NVIDIA kernel modules are loaded, but `nvidia-smi` cannot currently communicate with the driver

This is the intended development laptop. Ubuntu 26.04 was installed because it was the newest Ubuntu release, not because the project required it. The user is willing to return to Ubuntu 24.04. No environment mutation is authorized yet.

The supported workstation direction is now approved as Windows/Ubuntu dual boot: Windows remains available for Fusion 360, while Ubuntu 24.04 is the intended ROS 2 development environment. Reinstallation or other environment mutation has not yet been authorized.

Read-only reinstall preflight on 2026-08-15 found:

- The 1 TB NVMe drive already has a Windows/Ubuntu dual-boot layout.
- `/dev/nvme0n1p6` is the current 149.5 GiB ext4 Ubuntu root partition and contains both the new repository and the legacy workspace.
- `/dev/nvme0n1p1` through `p5` contain the EFI, Microsoft reserved, Windows, Windows recovery, and vendor recovery partitions; these must be preserved during any Ubuntu reinstall.
- The new repository's `docs/agent/` continuity files are currently untracked and therefore are not protected by Git or the remote.
- The legacy workspace at `/home/stevw-s14/Desktop/Araco` is approximately 2.2 GiB and is not a Git repository.
- A clean Ubuntu 24.04 reinstall should not begin until the new repository, continuity files, legacy workspace, and any other required Linux-side data have been copied to storage outside `p6` and a restore check has succeeded.
- The current official Ubuntu 24.04 desktop point release is 24.04.4. Use the official amd64 image and verify its published checksum when installation is authorized.

There is no project need to install Isaac Sim, Isaac Lab, or a standalone CUDA toolkit as part of the initial workstation bootstrap. The initial post-install gate is a working NVIDIA driver, ROS 2 Jazzy, Gazebo Harmonic through the supported Jazzy integration, RViz, and basic ROS/Gazebo/joystick smoke tests.

Past local Isaac Sim performance was only about 10–15 FPS, and the laptop is not expected to run useful Isaac Lab training. Cloud GPU development, including NVIDIA Brev, must be evaluated as a first-class option.

## Current Raspberry Pi platform

- Raspberry Pi 5B with 4 GiB RAM
- Debian GNU/Linux 13.2 (`trixie`)
- The Pi is intended for lightweight onboard work; heavy simulation, training, and possibly higher-level perception/planning will be offboard.
- Final onboard ROS installation/container strategy, storage capacity, and performance budget remain undecided.

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

The following must be established with the user and/or physical measurements before architecture is finalized:

- Exact measurable acceptance criteria beyond “works”
- Exact Hiwonder controller model/revision and physical power/signal behavior beyond the confirmed UART link and hold-last-command behavior
- Final compute split between laptop, Raspberry Pi, servo controller, and simulators
- Required real-time behavior and acceptable control/communications latency
- Desired future feedback additions, if any
- Power-distribution/protection details and software safety behavior
- Final leg/joint naming, joint zero definitions, safe mechanical limits, and verified kinematic dimensions
- Quantified terrain, speed, payload, and endurance goals
- Required Isaac Sim/Isaac Lab fidelity and sim-to-real boundary
- Depth-camera mounting location and whether the gimbal will remain fixed during mapping/navigation
- Odometry source for simulation acceptance and for the later physical robot
- Future command arbitration
- Deployment, networking, container, launch, logging, diagnostics, calibration, and update workflows
- Available Fusion 360 export/parameter data, joint limits, masses, inertias, and collision geometry
- Exact Ubuntu 24.04 restoration/dual-boot/container workflow and disk allocation
- Exact RTX 4060 VRAM and which Isaac Sim version/performance level is feasible on the laptop
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
- No implementation or project scaffolding is authorized yet; continue discovery and design discussion only.

## Evidence boundary

Facts above marked as legacy behavior describe inspected files, not necessarily the current physical robot or desired future behavior. No new architecture has been selected yet.
