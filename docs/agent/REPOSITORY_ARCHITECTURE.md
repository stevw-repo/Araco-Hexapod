# Araco Hexapod — Repository and Package Architecture

Status: **ACCEPTED**
Decision date: 2026-08-15
Scope: ROS 2 package boundaries, node ownership, dependency direction, and
simulator-first deployment. Acceptance does not authorize scaffolding.

## Architectural rules

1. Package boundaries follow cohesive responsibilities, not individual classes
   or arbitrary directory size.
2. Runtime processes communicate through explicit ROS interfaces. One process
   must not reach into another package's internal implementation.
3. Gait phase, body-motion generation, foot trajectories, and IK remain one
   deterministic locomotion process. Kinematics is a pure library, not a node.
4. Core control packages must not depend on Gazebo, Isaac Sim, or the physical
   servo driver. Backends depend inward on stable control/model contracts.
5. Canonical geometry, joint names, frames, and model limits have one source of
   truth in the description package. Kinematics receives typed model data; it
   does not embed a second set of robot dimensions.
6. Configuration lives in the package that owns it and is installed through
   that package. Do not accumulate loose runtime YAML or launch files at the
   repository root.
7. Do not create a generic `araco_utils` package. Shared code moves to a named
   domain library only after at least two real consumers require it.
8. Simulator ground truth is restricted to scoring and diagnostics. It is not
   a hidden input to a control or state-estimation result presented as realistic.

## Accepted initial packages

These nine packages cover the first Gazebo standing, body-control, and tripod-
walking milestones.

| Package | Type | Owns | Does not own |
|---|---|---|---|
| `araco_interfaces` | `ament_cmake` IDL | Project-specific messages, services, and actions after their contracts are approved | Algorithms, nodes, configuration |
| `araco_description` | `ament_cmake` data | Canonical Xacro/URDF, meshes, simplified collision geometry, joint/frame names, model geometry and limits, nominal simulator pose, RViz model assets | Gazebo worlds, gait logic, servo calibration |
| `araco_kinematics` | C++ library | Typed leg/body geometry, FK/IK calculations, reachability/error results, unit tests | ROS node, gait state, parameters, actuator commands |
| `araco_locomotion` | C++ library plus lifecycle node | Body-motion transform, tripod-gait phase, foot trajectories, calls to kinematics, 24-joint leg command generation | Command arbitration, global safety state, Gazebo or UART APIs, gimbal policy |
| `araco_supervision` | C++ libraries plus two separate lifecycle nodes | Command-source arbitration, command freshness, operating/safety state machine, command gating, health/fault diagnostics | Gait math, controller implementation, power-cut claims |
| `araco_teleop` | Adapter node and mappings | Joystick/keyboard input conversion into a canonical command candidate | Arbitration priority, safety decisions, locomotion |
| `araco_gazebo` | Simulator adapter/data | Gazebo worlds, spawn integration, physics/contact/sensor settings, `ros_gz` bridge configuration, `gz_ros2_control` backend overlay | Canonical robot geometry, locomotion logic, generic controller policy |
| `araco_bringup` | Launch/configuration | Top-level simulation profiles, common controller-manager configuration, lifecycle/startup ordering, parameter composition | Algorithm implementations or model assets |
| `araco_system_tests` | Test-only package | Cross-package launch tests, deterministic simulation fixtures, standing/walking acceptance tests, scoring and regression reports | Production runtime behavior |

Package names are interface commitments once implemented. They should be
changed now if the responsibility split is rejected, not casually renamed after
downstream packages and documentation depend on them.

## Node and process ownership

### Project-owned nodes in the first simulator phase

| Node | Package | Process responsibility |
|---|---|---|
| `teleop_adapter` | `araco_teleop` | Converts an operator device into a timestamped command candidate |
| `command_arbiter` | `araco_supervision` | Selects one valid source according to explicit priority and freshness rules |
| `safety_supervisor` | `araco_supervision` | Owns operating state, validates/gates the selected command, and reports faults |
| `locomotion` | `araco_locomotion` | Atomically advances body target, tripod phase, feet, IK, and 24 leg-joint commands |

`command_arbiter` and `safety_supervisor` share a package because they implement
one supervisory domain, but they remain separate executables and processes so a
command-source failure cannot silently replace safety supervision.

### Upstream/runtime processes

- `robot_state_publisher` publishes the canonical TF tree.
- `controller_manager` hosts the selected standard `ros2_control` controllers.
- `joint_state_broadcaster` publishes simulated joint state.
- One controller path owns all 24 leg joints.
- A separate controller path owns `gimbal_yaw_joint`.
- Gazebo Harmonic and `gz_ros2_control` implement the initial plant/backend.
- `ros_gz_bridge` exposes the required IMU, contact, clock, and later RGB-D
  sensor data.

The accepted controller contract uses `joint_state_broadcaster` and two
`joint_trajectory_controller/JointTrajectoryController` instances: one for all
24 leg joints and one for `gimbal_yaw_joint`. Locomotion streams complete
named, positions-only, one-point trajectories to the leg controller topic.

The first simulator milestone holds the gimbal at zero through its separate
controller. Active gimbal teleoperation/tracking does not justify another
project package until its command and behavior requirements are designed.

## Runtime path

```text
joy/keyboard/test source
  -> teleop_adapter or test adapter
  -> command_arbiter
  -> safety_supervisor
      -> locomotion
          -> 24-joint leg controller
  -> controller_manager
      -> 1-joint gimbal controller holding its initialized zero state
  -> gz_ros2_control
  -> Gazebo robot

Gazebo joint/IMU/contact state
  -> joint_state_broadcaster / ros_gz_bridge
  -> safety_supervisor, locomotion, robot_state_publisher,
     diagnostics, and system tests as explicitly required
```

Safety is layered: the supervisor gates high-level motion and lifecycle state;
the locomotion layer rejects unreachable/non-finite results; controllers and
the backend enforce final joint/command constraints. No single layer is allowed
to imply that the unresolved physical power-off/collapse problem is solved.

## Dependency direction

```text
# A -> B means package A depends on package B.

araco_teleop       -> araco_interfaces
araco_supervision  -> araco_interfaces
araco_locomotion   -> araco_interfaces
araco_locomotion   -> araco_kinematics
araco_locomotion   -> araco_description       # canonical model data only
araco_gazebo       -> araco_description
araco_bringup      -> runtime packages selected by a launch profile
araco_system_tests -> araco_bringup and the system under test
```

More precisely:

- `araco_interfaces`, `araco_description`, and `araco_kinematics` are inward
  foundation packages.
- `araco_locomotion` depends on interfaces, kinematics, and canonical model data
  supplied by the description package.
- `araco_supervision` and `araco_teleop` depend on interfaces, not locomotion
  internals.
- `araco_gazebo` depends on the description and upstream Gazebo integration.
- `araco_bringup` composes packages; production code never depends on bringup.
- `araco_system_tests` may depend on the assembled system; production packages
  never depend on tests.
- No core package depends on `araco_gazebo` or the future physical backend.

## Repository layout

```text
araco-hexapod-temp/
├── src/                         # ROS 2 packages only
│   ├── araco_interfaces/
│   ├── araco_description/
│   ├── araco_kinematics/
│   ├── araco_locomotion/
│   ├── araco_supervision/
│   ├── araco_teleop/
│   ├── araco_gazebo/
│   ├── araco_bringup/
│   └── araco_system_tests/
├── docs/
│   └── agent/                   # Durable context, decisions, and handoff state
├── tools/
│   └── fusion/                  # Offline CAD evidence/export tooling
├── README.md
├── LICENSE                      # GNU GPL v3.0 only; pre-Phase-0 checkpoint
└── .gitignore                   # Added only when scaffolding is authorized
```

Generated `build/`, `install/`, and `log/` directories are never source and must
remain untracked once the workspace exists.

## License and redistributable-asset policy

Project-authored code, configuration, documentation, tests, and original assets
use the GNU General Public License version 3 only (`GPL-3.0-only`). The
separately authorized pre-Phase-0 checkpoint adds the unmodified full GPLv3
text at the repository root. Phase 0 adds an exact package-local copy in each
ROS package, `<license file="LICENSE">GPL-3.0-only</license>` in every initial
`package.xml`, and `SPDX-License-Identifier: GPL-3.0-only` in project-authored
source files where the format supports comments.

This is a strong-copyleft selection. Distribution of covered modified or
combined object-code works must preserve GPLv3 terms and the corresponding-
source obligations. Phase 0 therefore audits direct linked/bundled dependency
licenses for GPLv3 compatibility before declaring the package foundation
complete.

For covered generated forms—such as compiled binaries, containers, generated
STL/DAE meshes, or other non-preferred editable forms—a distribution must be
paired with the applicable preferred modification source and generation tools
as required by GPLv3. The description workflow therefore cannot rely on a
project-authored derived mesh whose editable source cannot be published.
Proprietary API integrations, including the existing Fusion add-in and the
future Isaac adapter, receive a specific compatibility review before GPL-
covered distribution; compatibility is not inferred merely because the SDK is
installed separately.

The repository license does not relicense third-party dependencies, vendor CAD,
logos, data, or other imported assets. Every bundled non-project asset must have
recorded creator, source, license, required attribution, modification status,
and redistribution permission. An asset with unknown or incompatible
redistribution terms is excluded; a project-authored simplified geometry may be
created instead. Package manifests and third-party notices must enumerate any
additional license actually bundled in a package. A `NOTICE` file is added only
when an included work or attribution requires that exact form; otherwise
required third-party attributions are recorded in `THIRD_PARTY_NOTICES.md`.

## Phased packages, not part of the initial scaffold

| Package | Add only when | Expected responsibility |
|---|---|---|
| `araco_hardware` | Physical integration is authorized and safety prerequisites exist | `ros2_control` hardware plugin, Hiwonder protocol/transport, calibrated command conversion, device health; never gait logic |
| `araco_perception` | RGB-D/IMU simulation begins | Gemini-like sensor configuration, RTAB-Map composition, perception-specific filters and diagnostics |
| `araco_navigation` | Stable localization and planar command contract exist | Nav2 parameters, maps, launch composition, Nav2-to-command-candidate adapter |
| `araco_isaac` | Gazebo baseline passes and an Isaac release pair is selected | Isaac-specific robot/sensor adapter and validation configuration; no duplicate robot-control algorithms |

RL packages are deliberately unnamed and deferred until deterministic baselines
exist. A speculative learning package would prematurely freeze action and
observation boundaries.

## Language policy

- C++: kinematics, locomotion, supervision, and the later hardware interface.
- Python is acceptable for operator adapters, launch/tests, data conversion,
  analysis, and offline tooling where deterministic control timing is not the
  responsibility.
- Xacro/YAML: description and configuration data, with schemas or validation
  tests where silent configuration mistakes would be dangerous.

This is the accepted default, not a claim that every C++ component is real-time.
Loop rates, initial executor behavior, clocks, and timing thresholds are frozen
separately in `RUNTIME_TIMING_AND_SIMULATION_CONTRACT.md`. Later physical
real-time scheduling and allocation requirements remain separate decisions.

## Accepted scope and remaining gate

The 2026-08-15 approval freezes:

- the nine initial package responsibilities;
- one locomotion node and two separate supervision nodes;
- the inward dependency direction and simulator/backend isolation;
- physical, perception/navigation, and Isaac packages as later phases.

Later decisions have frozen the command/feedback message contracts, controller
types, software safety states, handover rules, simulator lifecycle order,
concrete topic names, QoS, loop rates, and provisional simulator values.
Configuration schemas/composition are frozen in the accepted
`PARAMETER_AND_CONFIGURATION_COMPOSITION.md` contract; physical safety behavior
remains open. The repository implementation order is frozen in the accepted
`PHASED_DELIVERY_PLAN.md`. The final review and GPL-3.0-only selection are recorded
in `FINAL_ARCHITECTURE_REVIEW.md`. No architecture approval yet permits
scaffolding; Phase 0 still requires an explicit user authorization.
