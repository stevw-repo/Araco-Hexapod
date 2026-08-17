# Araco Simulator Developer Runbook

<!-- SPDX-License-Identifier: MIT -->

This runbook covers the validated Ubuntu 24.04, ROS 2 Jazzy, and Gazebo
Harmonic simulator workflow. It is simulator-only. It does not authorize or
describe physical servo actuation.

## Prepare a shell

Run from the repository root:

```bash
cd "/home/stevw-s14/Desktop/Araco Project/araco-hexapod-temp"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install
source install/setup.bash
```

Every new terminal must source both ROS and the workspace. The ROS line is
already suitable for `~/.bashrc`; the workspace line is intentionally kept
explicit so an old build is not selected accidentally.

## Headed keyboard simulator

```bash
ros2 launch araco_bringup gazebo.launch.py profile:=gazebo_dev_v0
```

Wait for the **Araco Keyboard Control** window to show `HOLDING` and `ready`.
Click **Enable Motion**, focus that window, and hold `Space` with:

- `W/S`: forward/reverse
- `A/D`: left/right
- `Q/E`: walking yaw left/right

Multiple direction keys may be held. Releasing only direction keys keeps an
active stand while `Space` remains held. Releasing `Space`, pressing `Esc`,
changing focus, closing the window, or clicking **Controlled Hold** releases
authority fail-closed.

## Headed joystick simulator

The installed profile targets the exact Linux device name
`LiteStar PXN-2113 Pro`:

```bash
ros2 launch araco_bringup gazebo.launch.py profile:=gazebo_joystick_v0
```

Wait for the robot to finish startup, then move a control. No joystick deadman
button or separate Enable Motion action is required. Automatic enable occurs
once per launch after the stack is ready and translation, walking yaw, roll,
pitch, and posture yaw are neutral. Body-height trim may remain at its current
position. The adapter sends one internal inactive startup sample to establish a
new arbiter session; this is automatic and is not an operator control.
The current calibrated roles are:

- axis 1: forward/reverse
- axis 0: left/right
- axis 3: walking yaw
- axis 2: body height, from nominal to 30 mm lower
- inverted axis 5: planted-body pitch
- axis 4: planted-body posture yaw
- main trigger / physical button 1 (ROS index 0): unassigned
- physical button 2 (ROS index 1): unassigned
- physical button 3 (expected ROS index 2): planted-body roll left
- physical button 4 (expected ROS index 3): planted-body roll right

All six axis roles and physical buttons 1/2 were live-verified on the connected
controller on 2026-08-16. Mapping/profile version `0.7.0` intentionally leaves
both buttons 1 and 2 unused. A fresh complete joystick report keeps the source
active; a report timeout/disconnect or malformed report stops it. Automatic
enable is one-shot: Controlled Hold, a fault, or source loss cannot unexpectedly
resume motion during the same launch. The user-approved dedicated roll layout
assigns the
device's sequential physical buttons 3/4 to ROS indices 2/3. A live Gazebo
direction checks preserved forward/reverse, corrected lateral, walking yaw,
pitch, and posture yaw, and restored the prior height and roll-button signs.

## Controlled stop and shutdown

Request a controlled hold:

```bash
ros2 action send_goal /araco/safety/transition \
  araco_interfaces/action/SafetyTransition "{request: 1}"
```

Request orderly simulator shutdown before closing the launch terminal:

```bash
ros2 action send_goal /araco/safety/transition \
  araco_interfaces/action/SafetyTransition "{request: 4}"
```

## Package and simulator validation

```bash
colcon test
colcon test-result --verbose
```

Each evidence destination must not already exist:

```bash
ros2 run araco_system_tests araco_gate0_evidence log/gate_0_local
ros2 run araco_system_tests araco_gate1_evidence log/gate_1_local
ros2 run araco_system_tests araco_gate2_evidence log/gate_2_local
ros2 run araco_system_tests araco_gate3_evidence log/gate_3_local
ros2 run araco_system_tests araco_gate4_evidence log/gate_4_local
ros2 run araco_system_tests araco_gate5_evidence log/gate_5_local
```

Gate 6 needs explicit build and install roots and can take roughly 20 minutes:

```bash
ros2 run araco_system_tests araco_gate6_evidence \
  --build-base build --install-base install log/gate_6_local
```

The accepted local baseline is retained at
`/tmp/araco_gate6_final_20260816_06`. It passed package tests, sanitizer tests,
preflight Gates 0–5, three no-retry Gate 0–5 repetitions, physical
repeatability limits, timing budgets, log checks, matching behavior
fingerprints, and exact discrete safety outcomes.

## Troubleshooting

- **Package or executable not found:** source `/opt/ros/jazzy/setup.bash`, then
  the exact workspace `install/setup.bash` used for the build.
- **Launch says an output directory already exists:** choose a new evidence
  path. Evidence tools never overwrite or silently retry an attempt.
- **Joystick does nothing:** center translation, walking yaw, roll, pitch, and
  posture yaw, then verify `ros2 topic echo /joy --once` produces a six-axis,
  12-button report. The joystick profile enables automatically when the neutral
  report is fresh.
- **Enable request is rejected:** this applies to keyboard/system-test profiles,
  not the joystick profile. Wait for `HOLDING`, full readiness, and all sources
  released before requesting explicit enable.
- **Gazebo is slow:** the accepted laptop median real-time factor is about
  `0.957`; do not change physics, controller, or watchdog values to mask host
  load. Close unrelated heavy applications and rerun into a new evidence path.
- **Robot visuals appear flat or incomplete:** use the development or joystick
  profile from the current installed workspace. Both select the exact
  Fusion/vendor presentation meshes; CI presentation choices are separate.
- **A fault remains after the condition recovers:** release all sources and use
  the explicit reset request (`{request: 3}`). Faults never auto-reset into
  motion.

Build, install, log, and local evidence directories are generated outputs and
must not be committed.
