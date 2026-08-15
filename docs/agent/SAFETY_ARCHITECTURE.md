# Araco Hexapod — Safety, Handover, and Lifecycle Architecture

Status: **ACCEPTED by the user**
Decision date: 2026-08-15
Scope: software operating states, command-source handover, lifecycle ordering,
watchdogs, fault classification, `SafeCommand` dispositions/reasons, and
simulator behavior. This is not a certified safety system and does not authorize
physical actuation, power switching, IDL creation, or implementation.

## Non-negotiable safety facts

- The present physical robot has open-loop servos with no measured joint or
  contact feedback.
- Servo power removal while standing causes collapse. “Turn power off” is not a
  generic safe software response.
- The controller's physical switch has not been validated as an emergency stop.
- Software hold, controller hold, serial watchdogs, and a future physical stop
  device are different layers; none may be mislabeled as another.
- Gazebo can validate state transitions, command gating, and deterministic stop
  behavior. It cannot prove that the physical robot will remain supported or
  that a servo reached a target.

## Safety state is not lifecycle state

ROS lifecycle states describe whether a process is configured and executing.
They do not grant robot motion authority.

Examples:

- A lifecycle `active` locomotion node may still be in software safety
  `HOLDING` and prohibited from walking.
- A lifecycle `inactive` controller does not imply the physical robot is safe;
  deactivation could remove the only active position hold.
- Restarting an `active` node never restores the previous motion permission.

The safety supervisor owns the software safety state. `araco_bringup` owns
lifecycle ordering. Neither source command candidates nor diagnostics can
directly change safety state.

## Accepted software safety states

```text
uint8 STATE_INITIALIZING=0
uint8 STATE_INACTIVE=1
uint8 STATE_HOLDING=2
uint8 STATE_ENABLING=3
uint8 STATE_MOTION_ENABLED=4
uint8 STATE_STOPPING=5
uint8 STATE_FAULT_HOLD=6
uint8 STATE_SHUTTING_DOWN=7
```

| State | Meaning | `SafeCommand` disposition | New motion |
|---|---|---|---|
| `INITIALIZING` | Supervisor is starting and has not established readiness | `HOLD` when publication is available | Forbidden |
| `INACTIVE` | Runtime exists but one or more required control components are not ready | `HOLD` | Forbidden |
| `HOLDING` | Required components are ready and locomotion reports a stable hold | `HOLD` | Forbidden |
| `ENABLING` | A trusted enable request is waiting for a fresh source activation edge | `HOLD` | Forbidden until the gate completes |
| `MOTION_ENABLED` | A fresh selected source may be executed within limits | `EXECUTE` or `LIMITED`; `HOLD` if temporarily no executable intent | Permitted only through `SafeCommand` |
| `STOPPING` | New motion is blocked while locomotion transitions to stable hold | `CONTROLLED_STOP` | Forbidden |
| `FAULT_HOLD` | A fault is latched; best available hold is requested | `HOLD` | Forbidden; reset required |
| `SHUTTING_DOWN` | A validated stop completed and orderly shutdown is in progress | `HOLD` | Forbidden |

`FAULT_HOLD` is a request for the best available software/controller hold, not a
guarantee that six feet are planted or the physical robot cannot fall. Readiness
and fault status must reveal when the control path needed to hold is unavailable.

## Readiness gate

Motion cannot enter `ENABLING` until every bit required by the active deployment
profile is present:

```text
uint64 READY_MODEL=1
uint64 READY_BACKEND=2
uint64 READY_JOINT_STATE=4
uint64 READY_CONTROLLERS=8
uint64 READY_LOCOMOTION=16
uint64 READY_PROVENANCE=32
uint64 READY_TIME=64
uint64 READY_HARDWARE_CALIBRATION=128  # physical profile; simulator does not require
uint64 READY_POWER=256             # future physical profile
uint64 READY_LOCAL_STOP_INPUT=512  # future physical profile
```

`SafetyStatus` publishes both `readiness_mask` and
`required_readiness_mask`. Readiness is true only when:

```text
(readiness_mask & required_readiness_mask) == required_readiness_mask
```

The first Gazebo profile requires at least model/configuration consistency, an
active Gazebo backend, fresh finite joint position, active state/leg/gimbal
controllers, fresh holding locomotion status, known simulated provenance, and a
valid simulation clock.

`READY_BACKEND` does not require a new project health topic. It is derived from
the configured `ros2_control` hardware-component identity/state obtained through
typed controller-manager APIs and corroborated by fresh clock, joint-state, and
controller-state streams. Service polling runs outside the deterministic safety
loop and updates a validated latest-value mailbox; the safety loop never blocks
on it. Loss is classified by the first concrete failed evidence and may also set
`FAULT_BACKEND` when the composite backend identity/state is no longer valid.

The simulator profile must not assert physical calibration, power, or stop-input
readiness. Those become required only in a future physical profile after the
corresponding mechanisms exist and are verified.

## Operator transition interface

Long-running state changes use one project action rather than unstructured
booleans or lifecycle calls.

### `SafetyTransition.action`

```text
uint8 REQUEST_HOLD=1
uint8 REQUEST_ENABLE_MOTION=2
uint8 REQUEST_RESET_FAULT=3
uint8 REQUEST_SHUTDOWN=4
uint8 REQUEST_LATCHED_HOLD=5

uint8 request
---
bool accepted
uint8 final_state
uint16 reason_code
---
uint8 state
uint16 reason_code
```

Semantics:

- `HOLD` performs a controlled stop and completes only in `HOLDING`.
- `ENABLE_MOTION` is accepted only when all readiness gates pass and no latched
  fault exists. It waits in `ENABLING` for a fresh eligible source activation
  edge; it does not reuse a button or candidate already active at startup.
- `RESET_FAULT` succeeds only after every latched fault condition has cleared,
  all sources have released eligibility, and stable hold can be established. It
  returns to `HOLDING`, never directly to motion.
- `SHUTDOWN` first reaches stable hold. Simulator shutdown may then deactivate
  controllers and stop Gazebo. Physical controller deactivation remains
  prohibited while it could cause an unsupported standing robot to collapse.
- `LATCHED_HOLD` requests the strongest available software stop/hold, enters or
  targets `FAULT_HOLD`, and requires a later reset.
- Ordinary teleop, navigation, and test command sources cannot send this action.
  Only the trusted operator/bringup/test-control role may use it.

In the simulator baseline, “trusted” is enforced by closed launch composition
and test policy, not by cryptographic ROS graph identity. It is not a network
security boundary. A physical or untrusted-network deployment must add and
validate SROS 2 or an equivalent authentication/authorization policy before
this action can be treated as access controlled.

This action is not an emergency stop. It depends on the safety process, ROS 2
communications, locomotion, controllers, backend, and available power. A future
physical emergency-stop design must be independent and physically verified.

## `SafetyStatus.msg`

```text
# The STATE_* and READY_* constants defined above are included in the actual
# message.

uint64 FAULT_SELECTED_COMMAND_STREAM=1
uint64 FAULT_LOCOMOTION=2
uint64 FAULT_CONTROLLER=4
uint64 FAULT_JOINT_STATE=8
uint64 FAULT_KINEMATICS=16
uint64 FAULT_BACKEND=32
uint64 FAULT_TIME=64
uint64 FAULT_INTERNAL=128
uint64 FAULT_SOFTWARE_LATCH=256
uint64 FAULT_POWER=512       # reserved for physical phase
uint64 FAULT_SERIAL=1024     # reserved for physical phase

std_msgs/Header header
uint64 safety_epoch
uint8 state
uint8 disposition
uint16 reason_code
uint32 selected_source_id
uint64 readiness_mask
uint64 required_readiness_mask
uint64 fault_mask
bool reset_required
```

Requirements:

- `safety_epoch` increments for every software safety-state transition.
- `disposition` uses the accepted `SafeCommand` disposition values.
- `reason_code` is the highest-priority current reason. `fault_mask` preserves
  simultaneous latched categories rather than hiding all but one.
- Source ID `0` means no source currently owns selection.
- `reset_required=true` whenever a latched fault is present.
- Publication occurs on transition/change and at a periodic status rate. It is
  not the high-rate command path.
- Detailed text and individual fault context go to standard diagnostics; safety
  decisions use typed fields rather than parsing diagnostic strings.

## Accepted common reason-code allocation

The same numeric contract is used by `SafeCommand`, `SafetyStatus`, transition
results, and relevant `LocomotionStatus` reasons.

| Code | Name | Meaning |
|---:|---|---|
| 0 | `REASON_NONE` | No limiting, stop, or fault reason |
| 1 | `REASON_STARTUP` | Startup/readiness evaluation in progress |
| 2 | `REASON_INACTIVE` | One or more required components are inactive/unready |
| 3 | `REASON_HOLDING` | Stable hold; motion not enabled |
| 4 | `REASON_WAITING_FOR_SOURCE` | Enable accepted; waiting for a fresh source edge |
| 5 | `REASON_MANUAL_HOLD` | Trusted operator requested hold |
| 6 | `REASON_NO_SOURCE` | No eligible source is selected |
| 7 | `REASON_SOURCE_RELEASED` | Selected source relinquished eligibility |
| 8 | `REASON_SOURCE_STALE` | Selected candidate exceeded its configured receipt timeout |
| 9 | `REASON_SOURCE_HANDOVER` | Controlled-stop barrier for source change/preemption |
| 10 | `REASON_SOURCE_INVALID` | Candidate frame, enum, sequence, quaternion, or value is invalid |
| 11 | `REASON_COMMAND_LIMITED` | Finite otherwise-valid intent was limited to configured bounds |
| 12 | `REASON_SELECTED_COMMAND_STALE` | Arbiter output stream is stale/unavailable |
| 13 | `REASON_SAFE_COMMAND_STALE` | Locomotion detected stale supervisor output |
| 14 | `REASON_LOCOMOTION_NOT_READY` | Locomotion lifecycle/status is not ready for motion |
| 15 | `REASON_LOCOMOTION_STALE` | Locomotion status is stale/unavailable |
| 16 | `REASON_KINEMATICS_INVALID` | IK/trajectory calculation is unreachable or non-finite |
| 17 | `REASON_JOINT_LIMIT` | A generated joint target violates configured limits |
| 18 | `REASON_JOINT_STATE_STALE` | Required joint state exceeded its timeout |
| 19 | `REASON_JOINT_STATE_INVALID` | Joint state is missing, duplicate, non-finite, or inconsistent |
| 20 | `REASON_CONTROLLER_NOT_READY` | Required controller is not active/claiming expected interfaces |
| 21 | `REASON_CONTROLLER_FAULT` | Controller error, rejection, or unexpected state transition |
| 22 | `REASON_BACKEND_FAULT` | Gazebo or later hardware backend is unavailable/faulted |
| 23 | `REASON_TIME_DISCONTINUITY` | ROS time reset/backward jump invalidated state; pause-related watchdog expiry uses the relevant stale-stream reason |
| 24 | `REASON_SHUTDOWN_REQUESTED` | Trusted orderly shutdown requested |
| 25 | `REASON_SOFTWARE_LATCHED_HOLD` | Trusted software latched-hold request received |
| 26 | `REASON_RESET_REQUIRED` | Fault condition cleared but explicit reset has not completed |
| 27 | `REASON_INTERNAL_ERROR` | Invariant violation or unexpected internal exception |
| 28 | `REASON_POWER_WARNING` | Reserved for verified physical power monitoring |
| 29 | `REASON_POWER_CRITICAL` | Reserved for verified physical power monitoring |
| 30 | `REASON_SERIAL_FAULT` | Reserved for the physical servo transport |

Unknown reason codes are treated as an internal error at a receiving safety
boundary, not silently ignored.

## Command arbitration and handover

### Eligibility and selection

- Each source has a distinct candidate subscription configured with unique
  non-zero source ID, unique priority, timeout, and enabled/disabled status.
- Duplicate IDs or equal priorities are configuration errors that prevent
  readiness.
- A candidate is eligible only when structurally valid, fresh by steady receipt
  time, `active=true`, and not quarantined after an invalid/stale session.
- The arbiter reports the highest-priority eligible source as selected. Priority
  and timeout never come from the candidate message. Selection alone still does
  not grant execution; safety applies the enable/edge/handover gates below.
- A source quarantined by stale/invalid input must publish a valid
  `active=false` release followed by a fresh `active=true` edge before it may be
  selected again.
- Sequence values assist duplicate/reorder/restart detection. Receipt freshness
  remains authoritative.

### No surprise resumption

- Startup, reset, source loss, Wi-Fi loss, publisher restart, and process restart
  invalidate prior motion permission.
- A candidate already `active=true` when `ENABLE_MOTION` begins cannot start the
  robot. A release followed by a fresh activation edge is required.
- Loss, staleness, or release of the selected source enters `STOPPING` and then
  `HOLDING`. The arbiter may report a lower-priority eligible candidate, but
  safety never executes it automatically.
- After loss to `HOLDING`, both a new enable request and a fresh source edge are
  required.

### Deliberate higher-priority preemption

A higher-priority source may preempt only when its fresh `active=true` edge was
observed after the current selection epoch:

1. The arbiter announces the new selection epoch/source.
2. Safety enters `STOPPING` with `REASON_SOURCE_HANDOVER` and blocks the new
   intent.
3. Locomotion reaches stable six-foot hold and reports `MODE_HOLDING` or
   `MODE_STANDING` with a valid trajectory.
4. A configured hold dwell completes and the pending source remains fresh.
5. Safety returns through `ENABLING` and permits the new source without another
   operator enable action.

If any guard fails, the system ends in `HOLDING`; it does not fall back to an
older or lower-priority moving source.

Exact source IDs, numeric priorities, timeouts, acquisition window, and dwell
duration are selected in the accepted runtime/timing contract.

## Controlled stop and hold semantics

### Transactional locomotion state

Locomotion calculates a complete next gait/IK state before committing it. The
internal phase, foot targets, and command state advance only if all six legs and
all 24 joint targets validate. An invalid calculation is discarded as a whole;
there is no partially committed gait state or partial trajectory.

### `CONTROLLED_STOP`

When the control path is healthy, locomotion:

1. Rejects new motion intent.
2. Reduces planar translation/yaw toward zero using configured limits.
3. Completes the current swing/support transition from the last committed valid
   state.
4. Places all six feet into a stable planned stance.
5. Holds the last safe body-pose offset rather than abruptly resetting it.
6. Reports `MODE_HOLDING`/`MODE_STANDING` and a valid hold trajectory.

Only then may safety enter `HOLDING`, switch source ownership, reset a fault, or
continue shutdown. Exact deceleration, stop phase, stance geometry, timeout, and
hold dwell are timing/gait parameters to be validated in Gazebo.

### `HOLD`

In normal `HOLDING`, locomotion continuously maintains the last validated stable
six-foot stance through the leg controller. It does not merely stop publishing.
The controller timeout is a fallback, not the normal hold mechanism.

If locomotion, controller, or backend health is lost, the system may be unable
to complete a controlled stop. It enters or targets `FAULT_HOLD`; remaining
layers hold their last valid positions where possible and clearly report that a
stable stance is not guaranteed.

## Fault classes and responses

| Condition class | Examples | First response | Latch/reset |
|---|---|---|---|
| Limit | Finite velocity/body offset outside normal policy but inside validated hard bounds | `LIMITED`; remain `MOTION_ENABLED` | Not latched; always diagnosed |
| Recoverable source event | Manual hold, release, candidate stale, invalid candidate, no source | `CONTROLLED_STOP` → `HOLDING` | Not globally latched; source release/reactivation and enable rules apply |
| Deliberate higher-priority handover | Fresh high-priority activation edge | Controlled-stop barrier then guarded new selection | Not latched; failure ends in `HOLDING` |
| Kinematic/output fault | Non-finite/unreachable IK, generated joint-limit violation, partial/malformed trajectory | Discard transaction; stop from last valid state if possible | Latched `FAULT_KINEMATICS`; explicit reset |
| Control-component loss | Stale selected stream, stale locomotion, controller unexpected/inactive, stale/invalid joint state | Controlled stop if remaining path works; otherwise fallback holds | Latched component fault; explicit reset/restart |
| Backend/time/internal fault | Gazebo/hardware loss, backward/reset time jump, invariant/exception | Stop if possible, otherwise best available hold | Latched; reset only after reinitialization/guards pass |
| Software latched hold | Trusted `REQUEST_LATCHED_HOLD` | Strongest available software stop/hold | Latched; explicit reset |
| Future physical fault | Serial loss, verified critical power state, verified local stop input | Policy defined during physical safety phase; never assume power cut | Latched; physical verification required |

An invalid ordinary source is quarantined rather than automatically latching the
entire robot when the trusted arbiter and safety path remain healthy. Invalid
data appearing after trusted arbitration or inside locomotion is an internal
control-path fault and is latched.

## Lifecycle ordering

### Gazebo startup

1. Start Gazebo, simulation clock, canonical robot description, backend, and
   `controller_manager`; spawn at the validated simulator home pose.
2. Configure arbiter, safety supervisor, locomotion, state broadcaster, and both
   trajectory controllers. Configuration validates model/joint sets and
   parameters but grants no motion.
3. Activate `joint_state_broadcaster`; verify complete finite joint position,
   simulated provenance, and time behavior.
4. Activate leg and gimbal controllers using strict controller-manager
   switching. They claim exactly their 24+1 interfaces and hold initialized
   state.
5. Activate locomotion in hold-only mode; establish and report a valid stable
   standing trajectory. This is a startup bootstrap, not motion authority:
   locomotion may continuously command only the validated nominal hold, cannot
   advance gait, and arms its safe-command watchdog after the first accepted
   `SafeCommand` (or after bringup declares the active safety stream expected).
6. Activate the safety supervisor. It begins in `INITIALIZING`/`INACTIVE`,
   verifies all readiness gates, then enters `HOLDING`.
7. Activate the command arbiter and source adapters. Every source must first be
   observed released (`active=false`).
8. A trusted `ENABLE_MOTION` action plus a subsequent fresh source activation
   edge is required before any walking command reaches locomotion.

If any step fails, later steps do not activate and the system does not
automatically retry into motion. Restarted components return to the gated startup
path.

Watchdogs are armed against lifecycle/process expectations, not merely wall
time since process construction. In particular, the absent selected-command
stream before step 7 is an expected startup condition, not a latched arbiter
fault. Once bringup has activated an expected producer or the consumer has
accepted its first valid sample, subsequent expiry follows the accepted fault
table. This removes the startup circularity without allowing motion before a
trusted stream exists.

### Gazebo shutdown

1. Request `SHUTDOWN`; if moving, enter `STOPPING`.
2. Require stable locomotion hold and controller state confirmation.
3. Deactivate source adapters and arbiter so no new candidates enter.
4. Keep safety active while deactivating locomotion and trajectory controllers
   in strict order.
5. Deactivate state broadcaster/backend and stop Gazebo.
6. Finalize nodes.

Expected readiness loss caused by this commanded deactivation sequence remains
`SHUTTING_DOWN`; it is reported but does not create a new fault transition.
Unexpected loss before the shutdown sequence reaches that component still
follows the normal fault table.

### Future physical startup/shutdown

The same logical gate applies, but activation/deactivation ordering is not yet
authorized. It must include verified calibration, serial-controller state,
support condition, power state, and local stop behavior. In particular, physical
shutdown may not deactivate holding controllers or remove servo power while the
robot is standing unless a separately verified support/lowering procedure makes
that safe.

## Watchdog ownership

| Watchdog | Owner | Watches | Expiry response |
|---|---|---|---|
| Candidate freshness | Arbiter | Each source subscription | Mark source ineligible; selection epoch changes |
| Selected-command freshness | Safety supervisor | Arbiter output | `STOPPING` and latched control-stream fault |
| Joint-state/status freshness | Safety supervisor | State broadcaster/provenance/locomotion/controller state | Stop if possible; latch relevant fault |
| Safe-command freshness | Locomotion | Safety output | Locally enter stop/hold behavior; report stale-safety reason |
| Trajectory command timeout | Leg JTC; future active gimbal stream | Locomotion or future gimbal input | Controller holds according to configured behavior; v0 gimbal holds initialized state without a periodic publisher |
| Backend output/serial watchdog | Future hardware interface | Valid controller write/update stream | Physical policy pending; best available hold, never generic power cut |

All motion-authority watchdogs use local steady time. ROS timestamps remain for
provenance and simulation sequencing. A paused simulator will eventually revoke
motion through steady-time expiry; resumption requires fresh gated commands.

## Accepted scope and remaining gates

The user's 2026-08-15 approval freezes:

- eight software safety states and their `SafeCommand` disposition mapping;
- `SafetyTransition.action` and `SafetyStatus.msg` as project interfaces;
- readiness/fault bit categories and reason codes `0–30`;
- explicit enable, source activation-edge, quarantine, and no-surprise-resume
  rules;
- controlled-stop barriers for every source change, with guarded automatic
  higher-priority preemption but no automatic lower-priority fallback;
- transactional locomotion updates and stable six-foot hold semantics;
- fault classes, reset behavior, layered watchdog ownership, and simulator
  lifecycle ordering;
- the rule that software hold is not an emergency stop and physical power-off
  is not assumed safe.

The approval does not authorize IDL or implementation. Exact rates/timeouts,
numeric source priorities, stop profiles, hold dwell, provisional simulator
limits, and QoS/topic names were subsequently accepted in
`RUNTIME_TIMING_AND_SIMULATION_CONTRACT.md`. Physical safety behavior and the
hardware lifecycle remain later gates.
