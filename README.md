# Araco Hexapod

<!-- SPDX-License-Identifier: MIT -->

Simulator-first ROS 2 software for the Araco 25-joint hexapod. The project
targets Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic.

## Current status

Phase 0 is complete: the nine accepted ROS packages build, install, and test,
and the seven messages plus one action generate and introspect successfully.
The repository does not yet contain a robot model, runnable control nodes, a
Gazebo world, or a physical-hardware backend. No Gazebo validation gate is
claimed.

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

Generated `build/`, `install/`, and `log/` directories are local output and
must not be committed.

## License

Project-authored repository content is licensed under the MIT License; see
`LICENSE`. Third-party dependencies and assets retain their own licenses.
Redistribution boundaries and attribution are recorded in the Phase 0 license
audit.
