# Araco Hexapod

<!-- SPDX-License-Identifier: MIT -->

Simulator-first ROS 2 software for the Araco 25-joint hexapod. The project
targets Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic.

## Current status

Phase 2 / Gate 1 is complete. The repository contains the strict configuration
substrate, canonical 26-link/25-joint model, provisional simulator dynamics,
Gazebo Harmonic integration, exact 24-leg + 1-gimbal `ros2_control` ownership,
ordered lifecycle supervision, a hold-only locomotion component, and atomic
headless scoring evidence. Gate 1 reaches `HOLDING` without ever entering
`MOTION_ENABLED`; it does not implement computed IK, gait, or physical hardware.

The accepted architecture, interface contracts, safety model, configuration
rules, and phased delivery plan are maintained under `docs/agent/`.

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

The destination must not already exist. A complete result contains the resolved
runtime configuration, logs, metrics, JUnit, process outcomes, and validation
report. Development launch is available with
`ros2 launch araco_bringup gazebo.launch.py`.

Generated `build/`, `install/`, and `log/` directories are local output and
must not be committed.

## License

Project-authored repository content is licensed under the MIT License; see
`LICENSE`. Third-party dependencies and assets retain their own licenses.
Redistribution boundaries and attribution are recorded in the Phase 0 license
audit.
