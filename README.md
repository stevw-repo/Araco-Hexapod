# Araco Hexapod

<!-- SPDX-License-Identifier: MIT -->

Simulator-first ROS 2 software for the Araco 25-joint hexapod. The project
targets Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic.

## Current status

Phase 7 / Gate 6 is complete. The repository contains the strict configuration
substrate, canonical 26-link/25-joint model, provisional simulator dynamics,
Gazebo Harmonic integration, exact 24-leg + 1-gimbal `ros2_control` ownership,
ordered lifecycle supervision, deterministic four-DOF leg FK/IK, transactional
six-leg standing, bounded planted-foot body-pose control, deterministic slow
responsive stride/cadence tripod locomotion, controlled stopping, and atomic
headless scoring evidence.
The real arbitration and safety path supports body X/Y/Z, roll, pitch, posture
yaw, and bounded forward/reverse/lateral/yaw/combined walking. Physical
hardware control is not implemented or authorized. The accepted no-retry
simulator baseline is retained at `/tmp/araco_gate6_final_20260816_06`.

The accepted architecture, interface contracts, safety model, configuration
rules, and phased delivery plan are maintained under `.agent/`.
Verified simulator operation and troubleshooting commands are in
[`docs/SIMULATOR_DEVELOPER_RUNBOOK.md`](docs/SIMULATOR_DEVELOPER_RUNBOOK.md).

## Packages

| Package | Foundation responsibility |
|---|---|
| `araco_interfaces` | Project messages and actions |
| `araco_description` | Canonical robot-description resources |
| `araco_kinematics` | Pure forward/inverse kinematics library |
| `araco_locomotion` | Body, gait, foot-trajectory, and joint-command logic |
| `araco_supervision` | Command arbitration and software safety supervision |
| `araco_teleop` | Operator-input adapters and mappings |
| `araco_gazebo` | Gazebo-specific worlds and integration |
| `araco_bringup` | Profile composition and lifecycle ordering |
| `araco_system_tests` | Installed-space integration and simulator tests |

## Build and test

Install dependencies and build from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

After a successful installed-space build, emit a static Gate 0 evidence bundle:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run araco_system_tests araco_gate0_evidence log/gate_0_local
```

Run the headless Gate 1 hold, scorer, and orderly-shutdown evidence workflow:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run araco_system_tests araco_gate1_evidence log/gate_1_local
```

Run Gate 2 with the same physics scorer plus analytic-IK contract checks:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run araco_system_tests araco_gate2_evidence log/gate_2_local
```

Run the 14-case Gate 3 planted-foot body-pose matrix:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run araco_system_tests araco_gate3_evidence log/gate_3_local
```

Run the precision case plus seven-direction Gate 4 tripod gait and
controlled-stop matrix:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run araco_system_tests araco_gate4_evidence log/gate_4_local
```

Run Gate 5 supervision and fault-injection validation:

```bash
ros2 run araco_system_tests araco_gate5_evidence log/gate_5_local
```

Run the complete three-repetition Gate 6 baseline:

```bash
ros2 run araco_system_tests araco_gate6_evidence \
  --build-base build --install-base install log/gate_6_local
```

The destination must not already exist. A complete result contains the resolved
runtime configuration, logs, metrics, JUnit, process outcomes, and validation
report. Development launch is available with
`ros2 launch araco_bringup gazebo.launch.py`.

That launch opens Gazebo and the development-only **Araco Keyboard Control**
window. Wait for its safety line to show `HOLDING` and `ready`, then:

1. Click **Enable Motion**.
2. Keep the keyboard-control window focused.
3. Hold `Space` together with `W/S` (forward/reverse), `A/D`
   (left/right), or `Q/E` (yaw). Multiple direction keys may be held together.
   Releasing or changing direction keys while `Space` remains held keeps motion
   authorized; with no direction key, the robot commands an active stand.
4. Release `Space`, press `Esc`, change windows, or click
   **Controlled Hold** to stop through the normal safety path.

The terminal running `ros2 launch` is not the keyboard receiver. Closing or
unfocusing the control window releases the command fail-closed.

Generated `build/`, `install/`, and `log/` directories are local output and
must not be committed.

## License

Project-authored repository content is licensed under the MIT License; see
`LICENSE`. Third-party dependencies and assets retain their own licenses.
Redistribution boundaries and attribution are recorded in the Phase 0 license
audit.
