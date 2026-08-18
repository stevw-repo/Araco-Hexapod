# Araco Hexapod — Runtime Timing, QoS, Topics, and Simulator Values

Status: **ACCEPTED by the user**
Decision date: 2026-08-15
Scope: concrete names, clocks, rates, watchdogs, command envelopes,
provisional joint/dynamics values, and numerical Gazebo acceptance thresholds.
This document does not authorize IDL, configuration files, package scaffolding,
runtime code, or physical actuation.

## Decision boundary

This decision fills the numerical and naming gap left by the accepted package,
interface, safety, and configuration/validation architectures. It intentionally
did not define the configuration-file schemas or launch composition mechanism;
those were subsequently accepted in
`PARAMETER_AND_CONFIGURATION_COMPOSITION.md`.

Every number here is one of:

- a functional runtime contract, such as a publication rate or timeout;
- an operational policy limit for initial slow simulator development;
- a provisional simulator estimate that has no physical-safety standing; or
- a test threshold that may be revised only through an explicit reviewed
  decision with recorded evidence, not merely to make a failing test pass.

## Runtime namespace and naming rules

- Project-owned nodes and topics use the `araco` namespace. Names shown with a
  leading `/` are the effective single-robot names; source code uses relative
  names so a future deployment may remap the robot namespace.
- Standard global ROS topics remain `/joint_states`, `/tf`, `/tf_static`,
  `/clock`, and `/diagnostics` for compatibility with upstream tools.
- Controller names remain at the root because they are upstream
  `ros2_control` interfaces: `joint_state_broadcaster`,
  `leg_trajectory_controller`, and `gimbal_trajectory_controller`.
- Tests may subscribe to production topics but may publish production command
  types only through the configured test candidate input. Tests never publish
  directly to selected, safe, or controller command topics.

### Project nodes

| Effective node name | Owner | Initial execution model |
|---|---|---|
| `/araco/keyboard_teleop_ui` | `araco_teleop` | Development-only Tk window; wall-time 50 Hz full-key-state heartbeat plus safety action client |
| `/araco/teleop_adapter` | `araco_teleop` | One process; input callbacks plus 50 Hz steady timer |
| `/araco/command_arbiter` | `araco_supervision` | One process; deterministic 100 Hz steady loop |
| `/araco/safety_supervisor` | `araco_supervision` | Separate process; deterministic 100 Hz steady loop |
| `/araco/locomotion` | `araco_locomotion` | Separate process; 100 Hz ROS-time motion loop plus steady watchdog |
| `/robot_state_publisher` | upstream | Standard joint-state-driven TF publication |
| `/controller_manager` | upstream | Synchronous 250 Hz `ros2_control` loop |

The three control-domain processes use single-threaded executors initially.
Subscription callbacks validate and update a latest-value mailbox; only the
owning periodic loop commits arbitration, safety, or locomotion state. Callback
arrival order therefore cannot partially advance a control transaction. No
control callback performs blocking I/O. `SafetyTransition` action callbacks
enqueue or reject goals without waiting; the 100 Hz safety loop advances the
transition and completes action feedback/result asynchronously.

## Concrete interface names

### Command, status, and controller interfaces

| Effective name | Type | Publisher | Consumer |
|---|---|---|---|
| `/araco/teleop/key_state` | `std_msgs/msg/String` carrying `araco.keyboard-state.v1` JSON | focused development keyboard UI | teleop adapter only |
| `/araco/command/candidates/teleop` | `araco_interfaces/msg/CommandCandidate` | teleop adapter | command arbiter |
| `/araco/command/candidates/navigation` | `araco_interfaces/msg/CommandCandidate` | future navigation adapter; disabled initially | command arbiter |
| `/araco/command/candidates/system_test` | `araco_interfaces/msg/CommandCandidate` | test fixture only | command arbiter in test profile only |
| `/araco/command/selected` | `araco_interfaces/msg/SelectedCommand` | command arbiter | safety supervisor |
| `/araco/command/safe` | `araco_interfaces/msg/SafeCommand` | safety supervisor | locomotion |
| `/araco/safety/status` | `araco_interfaces/msg/SafetyStatus` | safety supervisor | operator, bringup, tests |
| `/araco/safety/transition` | `araco_interfaces/action/SafetyTransition` | safety supervisor server | trusted operator/bringup/test client |
| `/araco/locomotion/status` | `araco_interfaces/msg/LocomotionStatus` | locomotion | safety supervisor, tests |
| `/araco/locomotion/foot_targets` | `geometry_msgs/msg/PoseArray` | locomotion | RViz/tests; non-authoritative |
| `/araco/state/joint_state_provenance` | `araco_interfaces/msg/JointStateProvenance` | safety supervisor | diagnostics, tests, UI |
| `/joint_states` | `sensor_msgs/msg/JointState` | joint-state broadcaster | TF, safety, locomotion, tests |
| `/leg_trajectory_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | locomotion | leg JTC |
| `/leg_trajectory_controller/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` | leg JTC | safety/tests |
| `/gimbal_trajectory_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | locomotion at `100 Hz` | gimbal JTC |
| `/gimbal_trajectory_controller/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` | gimbal JTC | safety/tests |

The gimbal controller has separate ownership from the 24-leg controller.
`MotionIntent.gimbal_yaw_rad` passes through arbitration and safety. Responsive
joystick mapping `0.13.0` filters normalized axis 4 once and derives both the
posture-yaw and gimbal-yaw targets from that shared state. Locomotion publishes
those already-filtered ordinary targets without a second feel limiter. It
retains bounded return shaping when motion is no longer executable, and the
gimbal controller's physical limits remain authoritative.

### Simulation-only observation interfaces

| Effective name | Type | Use restriction |
|---|---|---|
| `/araco/simulation/ground_truth/odom` | `nav_msgs/msg/Odometry` | Scoring and diagnostics only; never a control or estimation input |
| `/araco/simulation/contacts/left_front` | `ros_gz_interfaces/msg/Contacts` | Gazebo tests/diagnostics only |
| `/araco/simulation/contacts/left_middle` | `ros_gz_interfaces/msg/Contacts` | Gazebo tests/diagnostics only |
| `/araco/simulation/contacts/left_rear` | `ros_gz_interfaces/msg/Contacts` | Gazebo tests/diagnostics only |
| `/araco/simulation/contacts/right_front` | `ros_gz_interfaces/msg/Contacts` | Gazebo tests/diagnostics only |
| `/araco/simulation/contacts/right_middle` | `ros_gz_interfaces/msg/Contacts` | Gazebo tests/diagnostics only |
| `/araco/simulation/contacts/right_rear` | `ros_gz_interfaces/msg/Contacts` | Gazebo tests/diagnostics only |
| `/araco/camera/color/image_raw` | `sensor_msgs/msg/Image` | Provisional simulated RGB stream; frame `camera_color_optical_frame` |
| `/araco/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Simulated pinhole calibration paired with color image |
| `/araco/camera/depth/image_raw` | `sensor_msgs/msg/Image` | Provisional simulated `32FC1` depth stream; frame `camera_depth_optical_frame` |
| `/araco/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Simulated pinhole calibration paired with depth image |
| `/araco/camera/depth/points` | `sensor_msgs/msg/PointCloud2` | Organized 424 x 240 simulated point cloud for perception/RViz |
| `/araco/camera/imu/data` | `sensor_msgs/msg/Imu` | Simulated camera IMU in `camera_link`; later estimator input |

Contact topics contain only contacts involving the named foot collision. Test
code rejects any unexpected non-foot robot-to-ground collision by inspecting
the complete Gazebo contact stream or a test-only filtered report.

## Clock model

All simulator ROS nodes set `use_sim_time=true`, but not every responsibility
uses the ROS clock.

| Responsibility | Clock | Reason |
|---|---|---|
| Gait phase, trajectory timestamps, simulator test duration | ROS/simulation time | Motion must pause with physics |
| Candidate receipt freshness | steady time | A paused or reset ROS clock cannot extend source authority |
| Selected- and safe-command watchdogs | steady time | Trusted control-stream loss must remain detectable |
| Joint/status/controller freshness | steady time | Readiness cannot be retained by a paused stream |
| Periodic arbiter and safety evaluation | steady time | Safety evaluation continues during a Gazebo pause |
| Locomotion motion loop | ROS/simulation time | Prevents gait advancement while physics is paused |
| Locomotion safe-command watchdog | independent steady time | Latches stale authority even while its motion loop is paused |

Simulation clock progress is considered stale after `0.250 s` of steady time
without advancement. Any backward jump greater than `0.001 s`, or a forward
jump greater than `0.100 s` while motion is enabled, invalidates motion state
with `REASON_TIME_DISCONTINUITY`. Startup and deliberate world reset perform the
accepted gated reinitialization rather than being treated as recoverable motion.

During lifecycle startup only, complete readiness must remain continuously true
for `1.0 s` before safety leaves `INACTIVE` for `HOLDING`. A renderer warm-up
that briefly stalls `/clock` before that transition resets the qualification
timer; it cannot create `RESET_REQUIRED` because motion authority has never
existed. Once `HOLDING` is reached, this startup exception ends and the normal
clock/component fault behavior below applies unchanged.

Pausing Gazebo therefore freezes physics immediately and revokes readiness no
later than `0.260 s` after the last clock advancement (timeout plus one 100 Hz
safety tick). Unpausing cannot resume prior gait motion; a fresh enable and
source activation edge are required.

## Nominal rates and horizons

| Function | Rate/period | Clock/domain | Notes |
|---|---:|---|---|
| Gazebo physics | `1000 Hz` / `0.001 s` step | simulation | Same in `gazebo_dev_v0` and `gazebo_ci_v0` |
| Controller manager | `250 Hz` / `0.004 s` | simulation-triggered | Synchronous read/update/write |
| Leg and gimbal JTC update | `250 Hz` | controller manager | No asynchronous controller threads |
| Joint-state broadcaster | `125 Hz` | controller manager divisor | `update_rate: 125`; dynamic topic disabled initially |
| Teleop candidate publication | `50 Hz` | steady timer | Publishes active and explicit release states |
| Future navigation candidate | `20 Hz` | source-specific | Disabled in initial profiles |
| System-test candidate | `100 Hz` | test fixture | Enabled only by individual test composition |
| Arbiter evaluation and selected command | `100 Hz` | steady timer | Periodic latest valid selection |
| Safety evaluation and safe command | `100 Hz` | steady timer | Periodic disposition even while holding |
| Locomotion motion/trajectory loop | `100 Hz` | ROS time | One atomic six-leg transaction per tick |
| Locomotion steady watchdog | `100 Hz` | steady timer | Does not advance gait |
| Locomotion status | `50 Hz` plus change | steady time | Health heartbeat is independent of paused/slow simulation time; motion state still comes from the ROS-time loop |
| Safety status | `10 Hz` plus change/transition | steady time | Not the motion command path |
| Joint-state provenance | `1 Hz` plus change/activation | steady time | Transient-local latest sample |
| Foot-target visualization | `20 Hz` | ROS time | Best-effort debug output |
| Project diagnostics | `1 Hz` plus fault/change | steady time | Standard `/diagnostics` |

The leg trajectory horizon is `0.040 s`: four locomotion publication periods
and ten controller-manager periods. Each new complete 24-joint point replaces
the old remaining trajectory. The JTC `cmd_timeout` is `0.100 s` with
`constraints.goal_time: 0.0`; Jazzy counts `cmd_timeout` from the trajectory's
last point, so the independent controller fallback begins approximately
`0.140–0.144 s` after the last valid trajectory was received.

This is intentionally slower than the legacy nominal 200 Hz algorithm timer.
The legacy timer was not evidence of a measured control requirement. The 100 Hz
locomotion / 250 Hz controller split is sufficient for the initial slow gait,
reduces scheduling load, and retains four-sample command coverage. Rates may be
raised later only with measured timing and simulator evidence.

The accepted position-only trajectory points use the Jazzy spline controller's
linear interpolation. Upstream documentation notes that this guarantees
position but not velocity continuity. Gate 4 therefore records desired and
actual velocity spikes; adding velocity fields later requires an explicit
revision of the already accepted controller contract.

During scored Gate 1–6 windows, the 100 Hz arbiter, safety, and locomotion loops
must each measure `95–105 Hz` in their owning clock domain, have a 99th-
percentile callback execution time below `5 ms`, and never execute longer than
their `10 ms` period. The controller manager must report `247.5–252.5 Hz` and
zero missed controller cycles during scored windows. A loop-health failure is a
test failure even when the robot pose happens to remain acceptable.

## Initial source registry

Higher numeric priority wins. IDs and priorities are configuration authority,
not values supplied by publishers.

| Source | ID | Priority | Candidate rate | Steady freshness timeout | Initial availability |
|---|---:|---:|---:|---:|---|
| Teleop | `10` | `100` | `50 Hz` | `0.150 s` | Registered/enabled in both profiles; adapter and focused keyboard UI launched in development and absent from scored CI runs |
| Navigation | `20` | `50` | `20 Hz` | `0.300 s` | Disabled until navigation phase |
| System test | `250` | `200` | `100 Hz` | `0.100 s` | Test fixture only; forbidden in normal bringup |

An enable request waits at most `5.0 s` for a fresh eligible activation edge.
If none arrives, the action returns unsuccessful and safety returns to
`HOLDING`. A deliberate higher-priority handover uses a `0.250 s` stable-hold
dwell before the pending source may execute. The pending source must remain
fresh throughout the stop and dwell.

The interactive headed joystick profile uses a separate simulator-only source
registry: its joystick-report and teleop-candidate freshness timeouts are both
`0.500 s`. This accommodates the measured desktop renderer/RViz scheduling
jitter without weakening the `0.150 s` CI/development source contract. It is
not a physical joystick-disconnect policy.

## QoS profiles

QoS never replaces the accepted steady-clock receipt watchdogs. Initial v0
profiles deliberately leave DDS deadline, lifespan, and manual liveliness
unset: those policies vary across RMW implementations and can create an
incompatible endpoint. Automatic liveliness remains the middleware default.

| Profile | History/depth | Reliability | Durability | Used by |
|---|---|---|---|---|
| `candidate_latest` | keep last `1` | best effort | volatile | Source candidate topics |
| `trusted_command_latest` | keep last `1` | reliable | volatile | Selected and safe command topics |
| `controller_command` | keep last `1` | reliable | volatile | Both JTC `joint_trajectory` inputs; must match upstream endpoint |
| `operational_status` | keep last `1` | reliable | volatile | Safety and locomotion status |
| `latched_classification` | keep last `1` | reliable | transient local | Joint-state provenance |
| `state_sample` | keep last `5` | best effort | volatile | Joint/controller state subscribers and simulation sensors |
| `debug_latest` | keep last `1` | best effort | volatile | Foot targets and ground truth |
| `diagnostics` | keep last `10` | reliable | volatile | `/diagnostics` |

Best-effort source candidates are deliberate: they are latest-value streams
that may cross Wi-Fi later. Retransmitting an old teleop value is less useful
than receiving the next sample; loss expires into the accepted stop behavior.
Selected and safe commands remain reliable because they are small trusted
streams intended to be local to the robot control computer.

The standard QoS of `/clock`, `/tf`, `/tf_static`, ROS actions, parameters,
services, and controller-manager APIs is not overridden. Startup validation
checks actual publisher/subscriber compatibility. A best-effort state
subscription can accept either a reliable or best-effort upstream publisher.

## Watchdogs and maximum detection latency

All timeout values below are measured from the last locally accepted sample.
The maximum detection budget adds one owning-loop period.

| Watched input | Owner | Timeout | Maximum detection | Expiry result |
|---|---|---:|---:|---|
| Teleop candidate | arbiter | `0.150 s` | `0.160 s` | Source ineligible/quarantined; selection epoch changes |
| Test candidate | arbiter | `0.100 s` | `0.110 s` | Same |
| Future navigation candidate | arbiter | `0.300 s` | `0.310 s` | Same |
| Selected command | safety | `0.050 s` | `0.060 s` | `STOPPING`; latch selected-stream fault |
| Safe command | locomotion | `0.050 s` | `0.060 s` | Local controlled stop from last committed state |
| Joint state | safety | `0.100 s` | `0.110 s` | Stop if possible; latch joint-state fault |
| Locomotion status | safety | `0.100 s` | `0.110 s` | Stop if possible; latch locomotion fault |
| Either controller state | safety | `0.100 s` | `0.110 s` | Stop if possible; latch controller fault |
| Provenance classification | safety | `1.500 s` | `1.510 s` | Readiness false; latch if motion was enabled |
| Simulation clock progress | safety | `0.250 s` | `0.260 s` | Readiness false; time fault during motion |
| Backend composite evidence | safety | dependent typed hardware-component/controller/joint/clock evidence | `0.110 s` for controller/joint loss; `0.260 s` for clock-only process loss | Readiness false; classify concrete component/time fault and backend fault when identity/state is invalid |
| Leg JTC command stream | leg JTC | endpoint + `0.100 s` | about `0.144 s` after last receipt | Controller holds final point; v0 gimbal has no periodic command stream and holds initialized state |

Candidate loss is normally a recoverable source event. Loss of the trusted
selected stream, safe stream, state/control components, backend, or time base is
a control-path fault according to the accepted safety table.

Typed controller-manager identity/state validation is deliberately separate
from those high-rate liveness watchdogs. The simulator policy staggers
`list_controllers` and `list_hardware_components`; each is requested at `1 Hz`.
This avoids loading controller-manager service locks at control-loop frequency.
The latest validated reply remains a mailbox value while its service is
available; controller, joint, and clock streams continue to provide the bounded
ongoing liveness evidence above.

### Interactive headed-simulator timing policy

`gazebo_joystick_v0` selects `headed_simulator_v0` because exact visual meshes,
Gazebo GUI, RGB-D rendering, RViz point-cloud display, and the control stack
share one non-real-time desktop. A live 2026-08-18 trace measured a worst
whole-executor scheduling gap of `327.409 ms`; the previous `100 ms` component
timeouts therefore classified host rendering jitter as a robot fault.

For this one simulator-only profile, selected command, safe command, joint
state, locomotion status, controller state, and clock progress use `0.500 s`
timeouts with `0.510 s` maximum detection. Provenance remains `1.500/1.510 s`.
The safety supervisor receives each value as a generated read-only parameter;
none of those component timeouts remain hard-coded. Headless CI, Gate profiles,
and `gazebo_dev_v0` retain the stricter table above, preserving their behavior
fingerprint and regression sensitivity. A future physical policy must be
measured and approved independently.

## Initial command and gait envelope

The normal envelope is the value delivered to locomotion. A finite value
outside the normal envelope but inside the hard-reject envelope is clamped with
`LIMITED` and `REASON_COMMAND_LIMITED`. A value beyond the hard envelope is
invalid and triggers source quarantine/stop rather than an extreme clamp.

| Quantity | Normal envelope | Hard-reject envelope |
|---|---:|---:|
| Planar translation norm `sqrt(vx²+vy²)` | `≤ 0.050 m/s` | `> 0.080 m/s` |
| Absolute yaw rate `abs(wz)` | `≤ 0.300 rad/s` | `> 0.500 rad/s` |
| Body X or Y offset | `≤ 0.020 m` | `> 0.035 m` |
| Body Z offset | `[-0.030, +0.020] m` | outside `[-0.045, +0.035] m` |
| Absolute body roll or pitch | `≤ 0.150 rad` | `> 0.250 rad` |
| Absolute body yaw offset | `≤ 0.200 rad` | `> 0.350 rad` |

Validation additionally requires:

- `|quaternion_norm - 1| ≤ 1e-6` without silent normalization;
- reserved twist fields absolute value `≤ 1e-12`;
- all numeric fields finite;
- `GAIT_STAND` planar velocity components absolute value `≤ 1e-9`;
- command frame exactly `base_link`.

Initial locomotion shaping values are:

| Parameter | Value |
|---|---:|
| Translation acceleration | `0.100 m/s²` |
| Translation controlled-stop deceleration | `0.150 m/s²` |
| Yaw acceleration | `0.600 rad/s²` |
| Yaw controlled-stop deceleration | `0.900 rad/s²` |
| Body-offset translation rate | `0.030 m/s` |
| Body-offset angular rate | `0.300 rad/s` |
| Tripod baseline cadence | `1.000 Hz` |
| Tripod maximum cadence | `1.500 Hz` |
| Tripod cadence slew limit | `1.000 Hz/s` |
| Preferred maximum stride scale | `0.50` |
| Local-foot-speed admission deadband | `0.005 m/s` |
| Nominal duty factor | `0.50` |
| Maximum planned stride | `0.060 m` |
| Nominal swing clearance | `0.030 m` |
| Stable-hold dwell | `0.250 s` |
| Maximum controlled-stop duration | `1.500 s` |

The `gazebo_joystick_v0` presentation profile instead selects a versioned
Responsive simulator contract:

| Parameter | Responsive value |
|---|---:|
| Translation limit | `0.240 m/s` |
| Hard planar envelope | `0.288 m/s` |
| Walking-yaw limit | `1.200 rad/s` |
| Translation acceleration / stop deceleration | `0.400 / 0.600 m/s²` |
| Yaw acceleration / stop deceleration | `2.400 / 3.600 rad/s²` |
| Tripod baseline / maximum cadence | `1.500 / 2.500 Hz` |
| Tripod cadence slew limit | `2.000 Hz/s` |
| Preferred / absolute maximum stride | `0.072 / 0.120 m` |

Maximum stride is `0.120 m`; maximum clearance is `0.060 m` and is scaled
by the same per-leg normalized factor as stride. Planted-body pose rates are
unchanged. CI, Gate, and keyboard profiles continue to use the initial values
above, so the established acceptance baseline is not retuned.

The stop completes the current swing transition; it does not snap the gait
phase or instantly plant a moving foot. If the healthy control path cannot
reach a six-foot hold within `1.500 s`, safety latches the relevant locomotion
or control-path fault.

## Provisional simulator joint limits

These ranges are `provisional_sim_v0`. They are selected around the approximate
standing reference and are not derived from the reported 270-degree servo
travel. A future physical profile is forbidden from loading them.

| Joint class | Canonical model range (rad) | Initial operational range (rad) | Max model velocity | Initial command-rate cap | Simulator effort cap |
|---|---:|---:|---:|---:|---:|
| Six coxa yaw joints | `[-0.700, +0.700]` | `[-0.450, +0.450]` | `2.0 rad/s` | `1.2 rad/s` | `3.0 N·m` |
| Six femur pitch joints | `[+0.150, +1.350]` | `[+0.350, +1.100]` | `2.0 rad/s` | `1.2 rad/s` | `5.0 N·m` |
| Six tibia pitch joints | `[-2.650, -0.750]` | `[-2.350, -1.150]` | `2.0 rad/s` | `1.2 rad/s` | `3.0 N·m` |
| Six foot pitch joints | `[-1.250, +0.350]` | `[-0.850, +0.100]` | `2.0 rad/s` | `1.2 rad/s` | `3.0 N·m` |
| Gimbal yaw | `[-1.571, +1.571]` | `[-0.314159, +0.314159]` | `1.5 rad/s` | `1.5 rad/s` | `3.0 N·m` |

The table above remains the `0.1.0` baseline selected by CI, Gate, development,
and keyboard profiles. Joystick profile `0.15.0` instead selects the isolated
`0.3.0` wide simulator artifact and uses the same values as its Responsive
operational range:

| Joint class | Joystick wide model/operational range (rad) | Span | Command-rate cap |
|---|---:|---:|---:|
| Six coxa yaw joints | `[-2.356194, +2.356194]` | `270 deg` | `5.5 rad/s` |
| Six femur pitch joints | `[-1.636194, +3.076194]` | `270 deg` | `10.0 rad/s` |
| Six tibia pitch joints | `[-4.256194, +0.456194]` | `270 deg` | `12.5 rad/s` |
| Six foot pitch joints | `[-2.766194, +1.946194]` | `270 deg` | `9.0 rad/s` |

These are permissive simulator presentation limits centered near nominal
standing coordinates. They are not calibrated servo endpoints, collision-free
certification, or hardware safety limits. The gimbal remains
`[-1.571,+1.571] rad`. The knee-down IK branch can only produce its negative
knee solution and continues to reject straight/folded singularities and
geometrically unreachable foot targets.

The Responsive rate caps are presentation-simulator policy derived from a
100 Hz exact-geometry gait/IK sweep. At the configured hard command envelope,
the measured steady peaks were `4.95`, `8.30`, `10.97`, and `7.42 rad/s` for
coxa, femur, tibia, and foot respectively. The selected caps include margin,
and a regression models the real 50 Hz joystick response before asserting that
every 10 ms transaction is accepted at full phase time. These values do not
predict safe or achievable loaded speed on the physical servos.

If a Responsive gait reaches a workspace boundary, locomotion freezes the
current gait phase and decelerates its curve amplitude toward the nominal
six-foot stance. After the retreat completes, planar gait retries automatically
at neutral body posture; planar-stick centering is not required. If a combined
walking-posture request caused the rejection, those posture offsets remain
suppressed and reason 11 stays visible until the posture controls are centered.
This avoids an indefinite operator lock while retaining geometric IK checks.

All 24 standing targets are at least `0.10 rad` inside their initial
operational range. Generated trajectories are rejected before publication if a
target exceeds the operational range or if the implied per-tick rate exceeds
the command-rate cap.

The effort values are functional simulator caps, not continuous servo ratings,
thermal deratings, or physical safety limits. Their presence must not be used
to claim that the real servo can sustain the corresponding load.

## Provisional Gazebo dynamics and contact values

| Item | Initial value | Fidelity status |
|---|---:|---|
| Physics engine | DART | Gazebo's default primary engine; functional baseline |
| Gravity | `[0, 0, -9.80665] m/s²` | Standard model value |
| Maximum physics step | `0.001 s` | Functional timing contract |
| Target real-time factor | `1.0` | Required for equivalent steady/ROS-time watchdog behavior |
| Deterministic test seed | `42` | Gate 6 baseline |
| `gz_ros2_control` position proportional gain | `0.150` | Provisional v2 functional response; simulator-only and not a physical-servo model |
| Foot-ground friction coefficients | `mu = 0.90`, `mu2 = 0.90` | Provisional simulator estimate |
| Non-foot collision friction | `mu = 0.40`, `mu2 = 0.40` | Provisional simulator estimate |
| Restitution | `0.0` | Provisional; no intentional bounce |
| RGB image mode | `424 x 240 @ 15 Hz`, `rgb8` | Performance-safe development mode; not physical calibration |
| Depth/point-cloud mode | `424 x 240 @ 15 Hz`, `32FC1`, organized cloud | Performance-safe development mode; `0.15–20 m` clipping |
| Camera IMU rate | `100 Hz` | Simulator rate; no calibrated noise or latency model |

Joint dynamics begin with:

| Joint class | Viscous damping (`N·m·s/rad`) | Coulomb friction (`N·m`) |
|---|---:|---:|
| Coxa | `0.050` | `0.010` |
| Femur | `0.080` | `0.020` |
| Tibia | `0.050` | `0.010` |
| Foot | `0.030` | `0.010` |
| Gimbal | `0.020` | `0.005` |

`rough_estimate_v0` supplies the initial mass budget. Canonical link inertias
derived from it remain explicitly provisional. Simplified convex/primitive
collisions, not visual triangle meshes, are the physics collision authority.
Before Gate 0 can pass, description authoring must assign the three missing
base-electronics proxies documented rough poses and construct a positive-valid
base-link center of mass and inertia. That estimate does not require another
Fusion export, but its assumptions must remain traceable and replaceable.

Both `gazebo_dev_v0` and `gazebo_ci_v0` use seed `42` and the same physics step,
real-time factor, controller rate, dynamics, friction, gait, source registry,
and safety values. CI may disable rendering, GUI plugins, and the live keyboard
adapter, but it may not run physics unbounded/as-fast-as-possible because
steady-time safety behavior would no longer be equivalent. Adapter presence is
part of the input-selection/run identity, not a change to source authority.

## Numerical Gazebo acceptance thresholds

All time/distance thresholds use simulation time unless explicitly marked
steady or wall time. Ground truth is visible only to the test scorer.

### Gate 0 — Model and configuration integrity

- Exactly 26 primary links, 25 revolute joints, and the accepted 24+1
  controller partition.
- Expanded axes have norm error `≤ 1e-9`; all transforms and values are finite.
- Every link mass is `> 1e-4 kg`; every inertia eigenvalue is `> 1e-9 kg·m²`;
  the principal-moment triangle inequality has tolerance `1e-9 kg·m²`.
- Total represented robot mass matches `rough_estimate_v0` at
  `3.924393 ± 0.001 kg`, including the three base proxies.
- The nominal reference lies at least `0.10 rad` inside every selected
  operational leg-joint range.
- Quaternion norm error is `≤ 1e-9`; axis/frame conversion agrees with the
  accepted manifest within `1e-9 m` and `1e-9 rad` where values are generated
  from the same evidence.
- Strict Xacro/URDF parsing emits no errors, all resources resolve, nominal
  collision checking reports zero unexpected self-collision pairs, and two
  expansions from identical inputs have identical normalized output and
  configuration fingerprints.

### Gate 1 — Spawn, controller ownership, and stable hold

- Startup reaches safety `HOLDING` within `30 s` wall time on the reference
  workstation and never enters `MOTION_ENABLED` automatically.
- The robot settles for at most `3.0 s` simulation time.
- Over the final `0.5 s`: maximum absolute leg-joint position error
  `≤ 0.080 rad`, RMS error `≤ 0.030 rad`, base linear speed `≤ 0.010 m/s`, and
  base angular speed `≤ 0.050 rad/s`.
- Base roll and pitch each remain within `0.080 rad` of the standing target;
  base-height error is `≤ 0.020 m`.
- Every foot reports ground contact for at least `90%` of the final `0.5 s`;
  no non-foot robot collision contacts the ground.
- No NaN/Inf, controller rejection, interface overlap, unclassified project
  `ERROR`/`FATAL`, or collision penetration deeper than `0.005 m` is allowed.

### Gate 2 — Kinematics and standing-reference validity

- At least `10,000` seeded reachable samples per canonical leg class are tested
  across both mirror signs, including explicit boundary/singularity fixtures.
- Away from declared singularities, FK-after-IK Cartesian error is
  `≤ 1e-6 m` and joint round-trip error is `≤ 1e-6 rad` modulo the selected
  canonical branch.
- Mirrored cases agree within `1e-6 m`; non-finite and unreachable inputs are
  rejected in `100%` of explicit negative fixtures.
- The standing reference produces six valid legs, exactly 24 finite targets,
  no operational-range breach, no partial commit, and the Gate 1 hold metrics.

### Gate 3 — Static body-pose control

- Test zero, positive, and negative `50%` normal-envelope offsets on each body
  axis separately, followed by one combined offset at `35%` per axis.
- Each command settles within `2.0 s` and is scored over the next `0.5 s`.
- Ground-truth body position error is `≤ 0.015 m` per axis and roll/pitch/yaw
  error is `≤ 0.050 rad` per axis.
- Maximum leg-joint tracking error is `≤ 0.100 rad`; every foot maintains
  contact for at least `85%` of each scored window.
- No invalid transaction, joint-limit breach, non-foot ground collision, or
  uncommanded gait-phase advancement is allowed.

### Gate 4 — Tripod locomotion and controlled stop

The commands are a `0.006 m/s` precision-forward case, `±0.040 m/s` forward,
`±0.030 m/s` lateral, `±0.200 rad/s` yaw, and one combined command
`[0.030, 0.020, 0.150]` in SI units. Each runs for at least five complete gait
cycles after startup.

The accepted baseline gait artifact `0.7.0` uses the legacy function-defined
horizontal, vertical, and rotational foot-path shapes, normalized to the
configured stride and
clearance and phase-aligned to the new state machine. Its horizontal path is
continuous at legacy phase `0.75`: the `+0.5` plateau changes there into a
linear decrease to zero at phase `1.0`, retaining constant supporting-foot
velocity across the cycle boundary. The production joint-rate limiter remains
authoritative when the raw polynomial requests a sharper step. Each new walk
first executes the legacy negative-counter quarter-cycle: tripod A follows
`-50 → -25`, tripod B follows `0 → 25`, and both trajectories start at the
nominal six-foot stance before joining repeating phase zero continuously. The fixed 100 Hz
locomotion loop advances a continuous `1.0–1.5 Hz` phase oscillator. Stride is
the primary speed variable, cadence rises only above the preferred `0.5` stride
scale, and each leg's maximum clearance uses exactly its own normalized stride
scale. Gate 4 observes each case for `7.6 s` to measure at least five completed
cycles; the central score window and all physical/contact/tracking/stop limits
remain unchanged.

Mixed translation/yaw follows the legacy normalized-magnitude rule: relative
translation and rotation magnitudes become blend weights, while the larger
magnitude sets the overall request. Consequently, a mixed command may remain
at base cadence where the superseded additive-twist scheduler increased it;
all generic cadence, stride, clearance, and velocity-scale bounds still apply.

- Over the central three cycles, planar velocity-vector mean error is
  `≤ 0.020 m/s` and yaw-rate mean error is `≤ 0.100 rad/s`.
- Base roll/pitch RMS is `≤ 0.120 rad` and height RMS error is `≤ 0.025 m`.
- At least two intended support feet remain in contact at every scored sample;
  the intended support tripod has at least two contacts for `≥ 95%` of its
  support interval. Swing-foot contact occupies `≤ 20%` of its swing interval,
  excluding the first `0.050 s`, including its exact boundary, at each
  transition.
- There are no non-foot ground contacts, self-collision events outside the
  declared adjacent-link exclusion set, joint-limit breaches, invalid or
  partial locomotion transactions, or backwards gait-phase jumps.
- Manual hold and selected-source loss produce `CONTROLLED_STOP` no later than
  their watchdog detection budget and reach stable six-foot hold within
  `1.500 s` after detection.
- Post-detection drift is `≤ 0.075 m` translation and `≤ 0.400 rad` yaw.
- An active valid `GAIT_STAND` zero command reaches the same physical hold
  metrics within `1.500 s` but may remain safety `MOTION_ENABLED`, because it
  has not relinquished authority.
- Stable hold requires the final `0.250 s` dwell, all six planned stance feet,
  joint speed `≤ 0.050 rad/s`, base speed `≤ 0.010 m/s`, and valid holding or
  standing locomotion status.

### Gate 5 — Supervision and fault injection

- Every candidate, selected, safe, joint-state, locomotion, controller, and
  clock-loss scenario changes state/disposition within its watchdog's maximum
  detection budget above.
- No new trajectory may advance gait phase more than one `0.010 s` locomotion
  tick after locomotion receives or locally determines a stop disposition.
- Each expected acquisition/loss/change increments `selection_epoch` exactly
  once; each actual safety-state transition increments `safety_epoch` exactly
  once. Duplicate/reordered samples never increment an epoch by themselves.
- Startup-active, restart, reset, lower-priority fallback, and source-loss cases
  produce zero unexpected `EXECUTE` samples and zero surprise gait advancement.
- Higher-priority preemption always crosses controlled stop plus the `0.250 s`
  hold dwell. If any pending-source guard fails, the final state is `HOLDING`.
- Latched faults cannot return to motion without cleared conditions, explicit
  reset, a new enable, and the required fresh source edge.
- Each scenario is repeated three times with identical state, reason, fault,
  epoch, and reset outcomes.

### Gate 6 — Reproducible simulator baseline

- Run the complete headless suite three consecutive times from clean processes
  with seed `42`; all three runs must pass with no retry.
- Exact interface, lifecycle, state-machine, reason/fault, and epoch outcomes
  must match across runs.
- Across repeats, final displacement differs by `≤ 0.020 m`, final yaw by
  `≤ 0.050 rad`, controlled-stop distance by `≤ 0.015 m`, and body roll/pitch
  RMS by `≤ 0.030 rad`.
- After startup, median real-time factor on the reference workstation is
  `≥ 0.80`; the complete suite wall time is no more than twice its planned
  simulated duration plus `60 s` startup/artifact allowance.
- Zero unclassified project `ERROR`/`FATAL`, sanitizer failure, process crash,
  lifecycle deadlock, or missing required artifact is allowed.
- Each run records source/dependency identities, expanded-model and effective-
  configuration fingerprints, seed, physics settings, measured loop rates,
  missed controller cycles, JUnit output, structured metrics, and focused logs
  or a ROS bag for every failure.

A slower future CI host may receive a separately reviewed wall-time allowance,
but may not change simulation-time behavior, watchdogs, physics step, gait,
limits, or pass tolerances.

## Upstream behavior this decision depends on

- ROS 2 QoS uses request/offered compatibility; reliable/volatile/keep-last is
  the default family, while sensor-style latest data commonly uses best effort:
  `https://design.ros2.org/articles/qos`
- Jazzy JTC accepts one-point trajectories, exposes
  `<controller>/joint_trajectory`, publishes controller state at the controller
  manager's rate, and supports desired-state interpolation:
  `https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html`
- JTC `cmd_timeout` is measured after the last trajectory point and is ignored
  unless greater than `constraints.goal_time`:
  `https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/parameters.html`
- A zero trajectory header stamp means “start now”; position-only spline input
  is linearly interpolated and guarantees position, not velocity, continuity:
  `https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html`
- Controller-manager `update_rate` is its read/update/write loop frequency, and
  simulation uses ROS time to trigger that loop:
  `https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html`
- Jazzy controllers/broadcasters may use an integer `update_rate` below the
  controller-manager rate:
  `https://control.ros.org/jazzy/doc/ros2_controllers/doc/controllers_index.html`
- The Jazzy joint-state broadcaster publishes `/joint_states` at the root by
  default and can omit `/dynamic_joint_states`:
  `https://control.ros.org/jazzy/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html`
- `gz_ros2_control` position commands use a discrete first-order response whose
  time constant is the inverse of proportional gain times controller-manager
  rate:
  `https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html`
- Gazebo uses DART by default and supports a configured maximum simulation step:
  `https://gazebosim.org/api/sim/10/physics.html`
- The ROS–Gazebo bridge supports `nav_msgs/Odometry`,
  `ros_gz_interfaces/Contacts`, `sensor_msgs/Imu`, and `/clock` message pairs:
  `https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md`

## Accepted scope and remaining gate

The user's 2026-08-15 approval freezes:

- topic/node names and the single-robot namespace policy;
- dual steady/ROS-time behavior, including pause revocation;
- 50/100/250/1000 Hz rate hierarchy and the 40 ms trajectory horizon;
- best-effort source candidates versus reliable trusted internal commands;
- source IDs, priorities, timeouts, enable window, and handover dwell;
- initial command/body/gait envelope and controlled-stop profile;
- provisional joint limits, effort/velocity caps, dynamics, friction, and
  `gz_ros2_control` position gain;
- every Gate 0–6 numerical threshold.

The accepted values remain versioned initial simulator values, not evidence of
physical fidelity or safety. Acceptance still does not authorize creating IDL,
ROS packages, YAML, Xacro, launch code, tests, or any physical command. Concrete
configuration schemas, artifact paths, and composition mechanics are specified
in the accepted `PARAMETER_AND_CONFIGURATION_COMPOSITION.md` contract. The
phased implementation order is specified in the accepted
`PHASED_DELIVERY_PLAN.md`.
