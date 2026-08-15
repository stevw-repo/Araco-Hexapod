# Araco Hexapod — ROS Interface Contracts

Status: **COMMAND AND FEEDBACK/CONTROLLER CONTRACTS ACCEPTED**
Command decision date: 2026-08-15
Feedback/controller decision date: 2026-08-15
Scope: command source → arbitration → safety → locomotion → controller, plus
feedback provenance and diagnostics. Safety-state and simulator lifecycle
contracts and the simulator runtime/timing/QoS values are accepted separately.
The configuration-schema/composition contract is also accepted; physical
calibration procedures remain a later design stage. Acceptance does not
authorize creation of `.msg`, `.srv`, or `.action` files.

## Goals

- Represent one complete motion request atomically; velocity, body posture, and
  gait request must never arrive as independently timed topics.
- Preserve source and safety provenance through the pipeline.
- Make stale-command handling independent of a source's own claims.
- Use SI units and standard ROS geometry types where their semantics fit.
- Keep navigation, teleoperation, tests, and later autonomy behind the same
  command-candidate boundary.
- Prevent source nodes from commanding joints, selecting safety state, or
  publishing directly to a controller.

## Authority rules

1. Each command source publishes on its own configured candidate input. A
   shared topic containing a self-declared source name is not accepted.
2. The package-owned `araco_supervision` source-registry artifact assigns each
   input a numeric source ID, priority, and freshness timeout;
   `araco_bringup` selects and composes that artifact without redefining it.
   These are not fields controlled by the source publisher.
3. Source ID `0` is reserved for “no selected source.” Human-readable source
   names belong in configuration and diagnostics, not the high-rate command.
4. A candidate timestamp is provenance, not a lease. The arbiter measures
   freshness from local receipt time using a monotonic/steady clock.
5. The safety supervisor independently watches the selected-command stream with
   its own steady-clock timeout. DDS deadline events may aid diagnostics but are
   not the only watchdog.
6. ROS timestamps remain in messages for simulation time, replay, and logs.
   Paused or jumping ROS time must not extend motion authority.
7. Only the safety supervisor publishes the safe-command interface, and only
   locomotion consumes it. Locomotion never consumes raw candidates.

## Command frames and units

- All linear quantities use metres or metres per second.
- All angular quantities use radians or radians per second.
- Initial candidate `header.frame_id` must be `base_link`. An adapter must
  transform any future command expressed in `map` or `odom` before publishing a
  candidate.
- `planar_velocity.linear.x` is forward and `.linear.y` is left.
- `planar_velocity.angular.z` is positive counter-clockwise yaw.
- `linear.z`, `angular.x`, and `angular.y` are reserved and must be zero in the
  first contract.
- `body_pose_offset` is an absolute offset from the configured nominal standing
  pose, expressed in canonical REP-103 body axes. It is not integrated or
  accumulated from one message to the next.
- A neutral body offset has position `[0, 0, 0]` and quaternion `[0, 0, 0, 1]`.
- Body-pose yaw is a posture change relative to the feet. Planar angular
  velocity is a walking/turning rate; the two are deliberately distinct.

## Accepted command message set

The following are contract sketches, not implementation files. Comments shown
here are normative requirements that must accompany any later IDL.

### `MotionIntent.msg`

```text
uint8 GAIT_STAND=0
uint8 GAIT_TRIPOD=1

uint8 gait
geometry_msgs/Twist planar_velocity
geometry_msgs/Pose body_pose_offset
```

Semantics:

- `GAIT_STAND` requires zero planar velocity but may carry a body-pose offset.
- `GAIT_TRIPOD` may carry planar velocity; zero velocity remains valid.
- Only tripod gait is in initial scope. A numeric enum is used instead of a
  string so unsupported values are unambiguous and inexpensive to validate.
- All values are desired targets, not measured state and not actuator output.
- The pose quaternion must be finite, non-zero, and normalized within a
  documented tolerance. Invalid quaternions are rejected, not silently fixed.

### `CommandCandidate.msg`

```text
std_msgs/Header header
uint64 sequence
bool active
MotionIntent intent
```

Semantics:

- `header.stamp` is when the source generated this candidate.
- `header.frame_id` must be `base_link` in the initial system.
- `sequence` increases per publisher session. It supports observability and
  duplicate/out-of-order detection but never replaces the receipt watchdog.
- `active=true` means the source is requesting eligibility for arbitration. It
  does not mean motion is safe or authorized.
- A teleoperation deadman maps to `active`; a navigation adapter sets it only
  while it owns an active navigation request.
- `active=false` explicitly releases the source. Handover behavior after a
  release belongs to the arbitration/safety-state design, not this message.

### `SelectedCommand.msg`

```text
std_msgs/Header header
uint64 selection_epoch
bool has_selection
uint32 source_id
builtin_interfaces/Time source_stamp
uint64 source_sequence
MotionIntent intent
```

Semantics:

- The arbiter writes `header.stamp` at selection time and copies the validated
  command frame to `header.frame_id`.
- `selection_epoch` increments whenever a source is acquired, lost, or changed.
- `source_id`, priority, and timeout originate from trusted arbiter
  configuration, never from `CommandCandidate`.
- `source_stamp` and `source_sequence` preserve candidate provenance.
- If no candidate is active and fresh, `has_selection=false`, `source_id=0`,
  and `intent` is neutral. The arbiter must not indefinitely republish an old
  intent as though it were fresh.
- Selection is still not authorization to move; it is the sole input command
  considered by the safety supervisor.

### `SafeCommand.msg`

```text
uint8 DISPOSITION_HOLD=0
uint8 DISPOSITION_EXECUTE=1
uint8 DISPOSITION_LIMITED=2
uint8 DISPOSITION_CONTROLLED_STOP=3

std_msgs/Header header
uint64 safety_epoch
uint64 selection_epoch
uint8 disposition
uint16 reason_code
uint32 source_id
builtin_interfaces/Time source_stamp
uint64 source_sequence
MotionIntent intent
```

Semantics:

- The safety supervisor is the only publisher.
- `safety_epoch` increments on each safety-state transition.
- `EXECUTE` permits the provided intent unchanged.
- `LIMITED` permits the provided intent after an observable safety limit was
  applied. The original selected command remains available on its own topic for
  comparison and diagnostics.
- `HOLD` tells locomotion to maintain the defined stable hold behavior and
  ignore the intent fields.
- `CONTROLLED_STOP` tells locomotion to execute the defined transition to hold
  and ignore new motion intent during that transition.
- Hold/controlled-stop mechanics and `reason_code` values are frozen in
  `SAFETY_ARCHITECTURE.md`; their numerical timing is frozen in
  `RUNTIME_TIMING_AND_SIMULATION_CONTRACT.md`.
- Provenance fields identify which selected source led to the decision. Source
  ID `0` is used when no source is selected.

Separate stage types are intentional. Reusing one generic command-envelope type
would allow an ordinary source to populate fields that appear to claim trusted
selection or safety authority.

## Pipeline behavior

```text
source-specific candidate topic
  CommandCandidate
    -> command_arbiter
       structural validation + configured priority/freshness
    -> SelectedCommand
    -> safety_supervisor
       operating-state, range, health, and watchdog decision
    -> SafeCommand
    -> locomotion
       body + tripod phase + feet + IK
    -> leg-controller contract (designed next)
```

The arbiter selects; it does not clamp motion. The source adapter is responsible
for mapping device inputs into canonical SI units and its configured normal
range. The safety supervisor owns system limits and produces `LIMITED`, `HOLD`,
or `CONTROLLED_STOP` according to the accepted safety policy.

## Required validation

Every boundary rejects or safely handles:

- NaN or infinity in any numeric field;
- unknown gait enum values;
- a missing or wrong command frame;
- non-zero reserved twist components;
- a zero or materially non-unit quaternion;
- impossible/out-of-policy velocity or body offsets;
- stale streams, duplicate/out-of-order sequences, and ROS time moving
  backwards;
- source acquisition, release, publisher restart, and source handover.

Invalid data must be observable through diagnostics. “Last good command
forever” is forbidden. The accepted safety-state table assigns the response:
ordinary invalid source data is quarantined and stopped without globally
latching a healthy control path, while invalid trusted-path or locomotion data
is a latched fault.

## Deliberate exclusions

- No joint positions, PWM values, servo IDs, or controller topic names appear
  in the high-level command contract.
- Gimbal commands are excluded from the first contract; the separate gimbal
  controller holds zero until active-gimbal behavior is designed.
- Sources cannot request `ARMED`, clear faults, select safety policy, or choose
  their own timeout/priority.
- No source-supplied validity duration exists.
- No incremental body-pose commands exist; incremental commands would drift on
  message loss or replay.
- QoS profiles, nominal publication rates, and concrete topic names are frozen
  separately in `RUNTIME_TIMING_AND_SIMULATION_CONTRACT.md`. Queues do not
  permit stale command backlogs, and watchdogs remain mandatory.

## Accepted scope and remaining gate

The 2026-08-15 approval freezes:

- four project messages: `MotionIntent`, `CommandCandidate`,
  `SelectedCommand`, and `SafeCommand`;
- per-source candidate channels with configured identity, priority, and timeout;
- steady-clock receipt freshness plus ROS timestamp provenance;
- atomic planar velocity, absolute body-pose offset, and stand/tripod intent;
- the authority separation between source, arbiter, safety, and locomotion;
- exclusion of direct joint, gimbal, and safety-state authority from sources.

It does not authorize IDL creation. Feedback, diagnostics, and controller
output are resolved in the following accepted section. Safety-state meanings,
rates, topic names, and QoS were subsequently accepted in their dedicated
architecture documents; this still does not authorize scaffolding.

## Accepted feedback and controller contract

Status: **ACCEPTED**
Decision date: 2026-08-15

### State truthfulness rules

1. `/joint_states` uses the standard `sensor_msgs/JointState` interface and
   canonical joint names for all 25 joints.
2. Position is required. Velocity and effort arrays are empty when their values
   are unavailable; zeros must not be published to imply a measurement.
3. In Gazebo, joint state comes from simulated physics and is labeled
   `SIMULATED_PHYSICS`.
4. On the current open-loop physical robot, joint position can only be the last
   command accepted for transmission by the hardware interface. It is labeled
   `COMMAND_DERIVED`, not measured. Initial physical velocity and effort remain
   unavailable unless real sensors are added.
5. `robot_state_publisher` may use command-derived physical joint position to
   maintain an estimated TF tree, but the TF must not be described as measured
   robot configuration.
6. Controller `feedback` and `error` are interpreted according to the same
   provenance. A near-zero physical controller error based on command-derived
   state is not evidence that a servo reached its target.
7. Simulator base-pose ground truth is published only under an explicitly
   simulation/ground-truth namespace. It never replaces estimated `odom`, IMU,
   or SLAM inputs in an acceptance test.
8. Simulator contact data remains a Gazebo/test interface initially. Core
   locomotion must not depend on contacts that the physical robot cannot sense.

### `JointStateProvenance.msg`

This low-rate/event-driven status accompanies `/joint_states` without replacing
the standard interface.

```text
uint8 SOURCE_UNAVAILABLE=0
uint8 SOURCE_SIMULATED_PHYSICS=1
uint8 SOURCE_HARDWARE_SENSOR=2
uint8 SOURCE_COMMAND_DERIVED=3
uint8 SOURCE_ESTIMATOR=4

std_msgs/Header header
uint64 provenance_epoch
string[] joint_names
uint8[] position_source
uint8[] velocity_source
uint8[] effort_source
```

Requirements:

- All source arrays have exactly the same length and ordering as `joint_names`.
- `provenance_epoch` increments whenever any source classification changes.
- `header.stamp` is when the classification was asserted; `frame_id` is empty.
- The safety supervisor publishes the aggregate provenance from the selected
  backend profile and observed interfaces. The Gazebo and later hardware
  adapters provide the underlying backend facts.
- Publication occurs on activation, on change, and periodically at a low rate
  so a late or restarted observer does not silently assume provenance.

### `LocomotionStatus.msg`

This status reports deterministic locomotion state. It is not an actuator
command and does not duplicate the 24 commanded joint positions.

```text
uint8 MODE_INACTIVE=0
uint8 MODE_HOLDING=1
uint8 MODE_STANDING=2
uint8 MODE_STARTING=3
uint8 MODE_WALKING=4
uint8 MODE_STOPPING=5
uint8 MODE_FAULT=6

uint8 LEG_VALID=0
uint8 LEG_NEAR_LIMIT=1
uint8 LEG_UNREACHABLE=2
uint8 LEG_INVALID=3

std_msgs/Header header
uint64 status_sequence
uint64 processed_safety_epoch
uint64 processed_selection_epoch
uint8 mode
uint8 gait
float64 gait_phase
uint64 gait_cycle
uint8[6] leg_kinematic_status
bool trajectory_valid
uint16 reason_code
```

Requirements:

- `header.frame_id` is `base_link`; `header.stamp` is the calculation time.
- `gait` reuses the accepted `MotionIntent` stand/tripod values.
- `gait_phase` is in `[0, 1)` and is meaningful only while starting, walking,
  or stopping. `gait_cycle` increments once per completed gait cycle.
- The fixed leg order is left front, left middle, left rear, right front, right
  middle, right rear.
- `trajectory_valid=true` means all 24 position targets passed local finite,
  reachability, and configured-limit validation. It does not mean the
  controller accepted them or the robot reached them.
- Any unacceptable leg result prevents publication of a partial 24-joint
  trajectory and moves locomotion according to the accepted safety/fault table.
- Exact `reason_code` values are frozen with the safety and fault contract.

For visualization and regression debugging, locomotion may also publish a
non-authoritative `geometry_msgs/PoseArray` containing the six calculated foot
targets in the same canonical leg order and `base_link` frame. The trajectory
controller input and `LocomotionStatus` remain the authoritative outputs.

### Standard controller selection

Use three upstream `ros2_control` components:

| Controller | Type | Joints | Purpose |
|---|---|---:|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | 25 | Publish standard joint state from the active backend |
| `leg_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | 24 | Execute named, time-interpolated leg-position trajectories |
| `gimbal_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | 1 | Own `gimbal_yaw_joint` independently; hold zero in the first simulator milestone |

Both trajectory controllers use:

- `command_interfaces: [position]`;
- at least `state_interfaces: [position]`;
- `allow_partial_joints_goal: false`;
- `interpolate_from_desired_state: true` for continuously replaced reference
  trajectories;
- `open_loop_control: false` because that Jazzy parameter is deprecated;
- `allow_integration_in_goal_trajectories: false` and
  `allow_nonzero_velocity_at_trajectory_end: false`;
- spline interpolation;
- a non-zero `cmd_timeout`, selected with the rates and horizon, so stale
  input reaches the controller's hold behavior; it must be greater than the
  configured goal-time tolerance or Jazzy ignores it;
- zero feedback gains for the position-command configuration.

Before activation, each backend must initialize its position command interfaces
to NaN or to a validated current/initial state as required by the controller's
desired-state interpolation semantics. Gazebo can use its known initialized
state. The equivalent physical startup state remains blocked on the later
hardware startup/safety procedure.

The joint-state broadcaster may additionally publish backend velocity or effort
when genuinely available even though each trajectory controller only requires
position state for the common simulator/physical contract.

### Locomotion-to-leg-controller contract

Locomotion publishes `trajectory_msgs/JointTrajectory` through the controller's
topic interface, not `FollowJointTrajectory` actions, because the gait is a
continuously replaced reference rather than a finite operator goal.

Each leg command has:

- all 24 canonical leg joint names; names are authoritative even though the
  canonical order is stable;
- exactly one trajectory point;
- 24 finite position values in radians;
- empty velocity, acceleration, and effort arrays initially;
- `header.stamp = 0`, which has the controller-defined meaning “start now”;
- an empty `header.frame_id` because revolute joint positions are not expressed
  in a Cartesian frame;
- a positive `time_from_start` horizon longer than one planned locomotion
  publication period.

Every new one-point trajectory replaces the remaining reference. The
controller interpolates from its current desired state, not directly from
possibly lagging or command-derived feedback. Exact locomotion/controller rates,
horizon, and timeout are selected together in the accepted timing design.

An invalid IK result, missing joint, duplicate joint name, non-finite value, or
configured-limit violation prohibits publication of that trajectory. Partial
leg trajectories are forbidden.

### Gimbal contract in the first milestone

- `gimbal_yaw_joint` is never included in the 24-joint leg trajectory.
- Its separate trajectory controller owns the joint from startup.
- Gazebo initializes it at zero and the controller holds that state.
- No general source command reaches it yet. Active gimbal movement, limits,
  rate policy, and tracking behavior require a later explicit interface review.
- These simulator rules do not authorize a physical gimbal startup command.

### Controller and state monitoring

- Each trajectory controller publishes the standard
  `control_msgs/JointTrajectoryControllerState` interface. System tests compare
  its reference and simulated-physics feedback.
- The safety supervisor watches controller-manager/controller lifecycle state
  through typed controller APIs, not by parsing log text.
- All project nodes publish `diagnostic_msgs/DiagnosticArray` for human and
  operational observability. Stable diagnostic names identify command
  arbitration, safety, locomotion, controllers, backend, and state provenance.
- Diagnostics are not a machine-control protocol. Safety decisions use typed
  state, lifecycle, and watchdog inputs; they never parse diagnostic strings or
  key/value text.
- Diagnostics report stale input, source selection, validation failures, mode,
  loop timing, controller availability, backend identity, and state provenance.

### Layered command-loss behavior

The accepted safety-state table sets the exact transitions, and the controller
boundary requires these independent layers:

1. The arbiter expires stale source candidates.
2. The safety supervisor expires stale selected commands.
3. Locomotion expires a stale `SafeCommand` and stops producing walking
   trajectories according to the controlled-stop/hold policy.
4. The leg trajectory controller has a non-zero command timeout and holds
   position if its stream expires. The gimbal controller uses the same policy
   only after a periodic gimbal command stream exists; in v0 it holds its
   validated initialized state without such a publisher.
5. The future physical hardware interface has its own local output/serial
   watchdog; its exact hold behavior remains a physical safety decision.

No layer is allowed to treat removal of servo power as a generic safe response,
because the current robot collapses when servo power is removed.

### Accepted feedback/controller scope and remaining gate

The 2026-08-15 approval freezes:

- standard `/joint_states` plus explicit per-interface provenance;
- truthful distinction between simulated physics, hardware measurement,
  command-derived state, estimator output, and unavailable data;
- `JointStateProvenance` and `LocomotionStatus` as two additional project
  messages;
- two separate named-joint `JointTrajectoryController` instances using position
  command interfaces, plus `joint_state_broadcaster`;
- one-point, positions-only, start-now, short-horizon leg trajectories through
  the controller topic interface;
- no partial leg trajectory, no gimbal in the leg command, and no controller
  action interface for the continuous gait loop;
- typed controller monitoring, standard diagnostics for observability, and
  layered command-expiry responsibilities.

Approval does not authorize IDL/controller configuration creation. Exact rates,
horizon, timeouts, QoS, topic names, and provisional simulation limits were
subsequently accepted in `RUNTIME_TIMING_AND_SIMULATION_CONTRACT.md`; physical
startup behavior and implementation remain gated.
