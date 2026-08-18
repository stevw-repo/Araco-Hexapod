# Araco Hexapod — Configuration, Calibration, and Simulator Validation Architecture

Status: **ACCEPTED by the user**
Decision date: 2026-08-15
Scope: ownership and composition of model/configuration/calibration data,
provisional-versus-verified evidence, configuration mutation rules, and the
ordered Gazebo acceptance gates. This decision did not itself select numeric
values, create schemas or configuration files, scaffold packages, implement
tests, or authorize physical actuation.

## Why this decision comes now

The package, command, feedback/controller, and software-safety boundaries are
accepted. Before selecting rates or implementing nodes, every parameter must
have one owner and every simulator milestone must have a blocking definition of
"passed." Otherwise the project would acquire duplicated joint lists, hidden
launch overrides, simulator estimates presented as physical calibration, and
tests that only show that Gazebo started.

## Terminology and evidence classes

The project uses the following terms precisely:

| Term | Meaning | May establish physical safety? |
|---|---|---|
| Design fact | Topology, frame, joint axis, or geometry supported by the accepted CAD/export reconciliation | Only after physical verification where assembly tolerance matters |
| Model parameter | A value used by URDF/Xacro, kinematics, collision, or control policy | No, merely because it is in the model |
| Simulator estimate | A provisional mass, inertia, friction, limit, noise, or controller value chosen for functional Gazebo work | No |
| Simulator identification | A value tuned against simulator behavior | No; it describes that simulation setup |
| Physical calibration | A measured value tied to the assembled robot and a documented procedure, unit, date, and hardware identity | Only within the scope actually verified |
| Operational policy | A deliberately narrower command, body-pose, gait, or speed envelope | It constrains software but is not an emergency-stop guarantee |

The current Fusion standing pose is
`nominal_standing_reference_v0`: an approximate simulator reference, not a
physical calibration. Fusion's all-Steel mass properties are rejected evidence
and must not enter the simulator as if measured.

Gazebo may be used to find collision-free geometry, IK reachability, functional
controller behavior, and provisional operational envelopes. It cannot discover
the installed servo zeros, directions, mechanical stops, load capacity, torque,
backlash, actual 270-degree usable travel, or a physically safe joint range.

## Configuration ownership rules

1. Every source value has exactly one owning package. Other packages reference,
   compose, or validate it; they do not maintain independent copies.
2. Canonical runtime values use SI units and REP-103 conventions. File keys
   include a unit suffix where ambiguity would otherwise be likely.
3. Every configuration artifact has a schema/version identifier. Calibration
   artifacts additionally identify provenance, applicable hardware, method, and
   date. Secrets never belong in these files.
4. Missing, unknown, duplicate, non-finite, contradictory, or out-of-range
   motion-critical configuration prevents lifecycle configuration or
   `READY_MODEL`; it is not silently defaulted.
5. A named deployment profile selects owned artifacts. It may only override
   fields explicitly declared profile-overridable by the owner.
6. Motion-affecting configuration is immutable while its node is active. A
   change requires controlled hold, lifecycle deactivation, reconfiguration,
   readiness revalidation, and a new explicit motion enable.
7. The effective configuration identity/fingerprint is recorded in diagnostics
   and every system-test result so a pass can be reproduced. This does not yet
   require a new ROS interface field.
8. Generated or derived values are labeled as such and regenerated from their
   owner. A derived file never becomes a competing hand-edited authority.

## Package ownership matrix

| Data | Owner | Required consumers/validators | Boundary |
|---|---|---|---|
| Fusion exports, reconciliation reports, offline extraction tools | `tools/fusion` and durable evidence docs | Description-authoring workflow | Evidence only; never loaded by a running robot |
| Link/joint names, topology, parent-child transforms, axes, frames, visual/collision geometry | `araco_description` | Kinematics, locomotion, Gazebo, bringup, tests | One canonical model registry |
| Bundled model/mesh creator, preferred editable source, generator, license, attribution, modification, and redistribution status | `araco_description` | Preflight, package metadata, public-release audit | Unknown-rights vendor assets and covered generated forms without required source are excluded |
| Model joint-limit set and its evidence/fidelity status | `araco_description` | Kinematics, controllers, supervision, hardware, tests | Provisional simulation limits cannot be selected by a physical profile |
| Nominal standing reference and model geometry constants | `araco_description` | Kinematics, locomotion, spawn, tests | Versioned reference; current v0 is simulator-only and approximate |
| Link mass, center of mass, inertia, and fidelity/provenance | `araco_description` | Gazebo and model validators | Raw all-Steel Fusion values are forbidden |
| IK/FK solver tolerances and numerical policy | `araco_kinematics` | Locomotion and unit tests | No duplicate robot dimensions |
| Tripod phase, foot path, stance/body envelope, acceleration/deceleration, trajectory horizon | `araco_locomotion` | Supervision limits and system tests validate compatibility | Algorithm policy, not model geometry |
| Source registry, IDs, priorities, freshness rules, command bounds, readiness and fault policy | `araco_supervision` | Bringup and fault-injection tests | Publishers cannot supply or override authority values |
| Device mappings and input shaping | `araco_teleop` | Teleop tests | Cannot widen supervision bounds |
| Physics engine, world, gravity, contact/friction, sensor noise, spawn integration, and Gazebo backend settings | `araco_gazebo` | Bringup profile and simulator tests | No canonical geometry or gait parameters |
| Controller-manager composition and lifecycle profile selection | `araco_bringup` | System tests | Composes owner data; does not redefine it |
| Test seeds, fixtures, scenarios, metrics, tolerances, and expected outcomes | `araco_system_tests` | CI/reporting | Ground truth is a scorer only |
| Future servo IDs, bus mapping, sign, zero/PWM conversion, per-unit calibration evidence | future `araco_hardware` | Hardware startup validator | Never used by a Gazebo profile; no gait policy |

Later perception, navigation, and Isaac packages own only their domain-specific
settings and adapters. They must not duplicate the canonical robot model or
core command/safety policy.

## Joint-limit hierarchy

One unqualified `joint_limit` value is insufficient. The effective limit is a
nested intersection:

```text
effective command range
  = canonical model range
  intersection verified actuator range (physical profile only)
  intersection operational locomotion/controller range
```

- The canonical model range belongs to `araco_description`. During the initial
  simulator phase it may be a clearly named provisional simulator set supported
  by collision and reachability testing.
- A provisional simulator set is never eligible for a physical deployment
  profile. Before hardware actuation, physical measurements must replace or
  validate it through an accepted description update.
- The future hardware calibration maps canonical joint radians to a particular
  actuator and may further restrict the range. It cannot widen the canonical
  model range.
- Locomotion and controller policy may further tighten the range for stance,
  trajectory continuity, or margin. It cannot widen either upstream range.
- Startup validation requires a non-empty intersection for every joint and
  requires the nominal standing target inside all selected ranges.
- Every generated 24-joint trajectory is checked against the effective range
  before the locomotion transaction commits.

This preserves the accepted rule that geometry and model limits have one
canonical description source while keeping per-robot actuator conversion in
the future hardware package.

## Named configuration profiles

The initial architecture defines two versioned profile identities:

- `gazebo_dev_v0`: local GUI/RViz-friendly composition with developer
  diagnostics.
- `gazebo_ci_v0`: headless, fixed-seed, artifact-producing composition.

They share the same canonical model, behavior limits, locomotion settings,
safety policy, physics, real-time target, and deterministic seed `42`.
Differences are restricted to presentation, logging, recording, rendering, and
reporting controls that cannot change robot or physics behavior. The closed
input-selection policy may launch the live keyboard adapter in development and
omit it from a scored CI run, but both profiles select the same source registry;
that adapter presence is recorded in the input-selection and run fingerprints.
A test must not pass by giving CI an easier gait, different seed or physics, or
wider safety policy than development.

Fault-injection overrides belong to individual `araco_system_tests` fixtures,
are visibly test-only, and cannot be selected by normal bringup.

A future physical profile is not defined yet. It must select a verified model
limit set and a hardware calibration artifact tied to the assembled robot. The
presence of a future profile name must never bypass those readiness checks.

## Static configuration validation

Before Gazebo starts, automated validation must establish:

- exactly one root link, 26 primary links, and 25 revolute joints with the
  accepted names and parent-child pairs;
- exactly 24 leg joints plus `gimbal_yaw_joint`, with no missing, duplicate, or
  multiply owned controller joints;
- normalized finite axes, finite transforms, resolved mesh resources, and
  REP-103-consistent frames;
- complete redistributable-asset metadata, with no unknown-license or
  unknown-redistribution resource and no project-generated form missing its
  required preferred editable source in the installed description;
- positive finite masses and physically valid inertia tensors for every dynamic
  link, with fidelity/provenance labels and no rejected all-Steel import;
- collision geometry for the moving robot and no unexpected self-collision in
  the nominal reference;
- finite ordered limit ranges, valid velocity/effort policy where required, and
  the nominal target inside every effective range;
- one unique non-zero ID and one unique priority per enabled command source;
- complete typed parameters with no unknown keys or silent defaults;
- compatible model, controller, kinematics, locomotion, supervision, and test
  configuration identities.

Any failure blocks startup readiness. A warning is permitted only for an
explicitly declared fidelity limitation that does not invalidate the functional
test being run.

## Ordered Gazebo acceptance gates

Each gate depends on all earlier gates. A later success cannot waive an earlier
failure.

### Gate 0 — Model and configuration integrity

Runs without Gazebo where possible.

Pass requires all static checks above, successful strict Xacro/URDF parsing,
kinematic-tree agreement with the accepted manifest, exact controller joint
partitioning, and reproducible configuration fingerprints.

This gate catches unit errors, stale joint lists, bad inertias, invalid limits,
and profile drift before physics obscures the cause.

### Gate 1 — Spawn, controller ownership, and stable hold

Starts Gazebo headless in the nominal simulator pose but does not enable motion.

Pass requires:

- clean spawn without NaNs, joint explosion, unresolved resources, or severe
  penetration;
- valid simulation clock and simulated joint-state provenance for all 25
  joints;
- `joint_state_broadcaster`, the 24-joint leg JTC, and the one-joint gimbal JTC
  active with exact, non-overlapping interface claims;
- the gimbal held at zero and the leg controller holding a complete named
  standing target;
- the accepted startup lifecycle sequence reaching safety `HOLDING`, never
  `MOTION_ENABLED`;
- bounded settling/contact/body-pose metrics with no unexpected motion.

### Gate 2 — Kinematics and standing-reference validity

Pass requires deterministic FK/IK unit and property tests across all six legs,
including symmetry, round trips, reachable boundary samples, unreachable and
non-finite rejection, continuity near ordinary operating points, and joint-limit
enforcement.

The nominal standing reference must produce six valid leg solutions, a complete
24-joint transaction, collision-free geometry, and a stable simulated hold. Its
status remains provisional rather than physically calibrated.

### Gate 3 — Static body-pose control

Exercises height, planar translation, roll, pitch, and yaw offsets without
walking.

Pass requires complete finite trajectories, no limit violations, correct
direction/sign conventions, expected six-foot support, and ground-truth-scored
body-pose error within reviewed tolerances. Ground truth is restricted to the
test scorer and is not fed back into locomotion.

### Gate 4 — Tripod locomotion and controlled stop

Exercises forward, reverse, lateral, and yaw motion, then combined bounded
commands.

Pass requires the accepted tripod phase/contact ordering, continuous valid
whole-robot transactions, bounded body/velocity tracking metrics, no unexpected
self/ground collision, no limit breach, and repeatable transition to a planned
six-foot stable hold.

A manual hold, zero command, and selected-source loss must each stop within
reviewed time/distance/pose bounds while preserving the controlled-stop
semantics. Exact thresholds are set in the following timing/limits decision.

### Gate 5 — Supervision and fault injection

Pass requires automated scenarios for:

- startup with a source already active;
- explicit enable plus fresh source edge;
- release, stale, malformed, duplicate, reordered, and restarted source input;
- lower-priority source availability after selected-source loss;
- deliberate fresh higher-priority preemption;
- stale/missing selected command and stale/missing safe command;
- unreachable IK, generated joint-limit violation, malformed joint state,
  locomotion loss, controller loss, backend loss, and time reset/jump;
- latched hold, reset guards, component restart, and orderly shutdown.

Every scenario asserts the exact safety state, disposition, reason/fault class,
selection/safety epoch behavior, controller outcome, and whether explicit reset
or re-enable is required. No test may accept surprise resumption or automatic
lower-priority motion fallback.

### Gate 6 — Reproducible simulator baseline

Runs the complete functional suite headlessly from a clean ROS 2 Jazzy/Gazebo
Harmonic environment with fixed seeds and bounded runtime.

Pass requires repeated runs within reviewed invariant/metric tolerances, no
intermittent lifecycle or discovery failure, and machine-readable results. Each
run records the source revision, dependency versions, configuration
fingerprints, seed, physics settings, JUnit-style pass/fail output, structured
metrics, and logs or a focused ROS bag on failure.

Physics samples need not be bit-identical. Reproducibility is judged by exact
interface/state-machine outcomes and bounded physical metrics.

## Gate ownership and progression

| Gate | Primary owner | Blocks |
|---|---|---|
| 0 | Owning package unit/static tests plus `araco_system_tests` cross-checks | Any simulator launch treated as valid |
| 1 | `araco_gazebo`, `araco_bringup`, `araco_system_tests` | Kinematics/locomotion simulator milestones |
| 2 | `araco_kinematics`, `araco_locomotion`, `araco_system_tests` | Body control and walking |
| 3 | `araco_locomotion`, `araco_system_tests` | Walking |
| 4 | `araco_locomotion`, `araco_system_tests` | Safety release baseline and later perception |
| 5 | `araco_supervision`, `araco_system_tests` | Declaring the simulator control baseline complete |
| 6 | `araco_system_tests` and CI composition | Isaac, perception/navigation, or physical-backend progression |

Core failures cannot be marked expected or waived merely because the robot is
simulated. Explicit fidelity limitations—such as provisional mass/inertia or an
approximate standing reference—are tracked separately and constrain what a pass
proves. In particular, Gates 0–6 establish a reproducible functional simulator,
not physical safety or sim-to-real fidelity.

## Accepted scope and remaining gates

The user's 2026-08-15 approval freezes:

- the terminology separating design facts, simulator estimates, physical
  calibration, and operational policy;
- the package ownership matrix and no-duplicate-authority rule;
- configuration validation, named profile, immutability, and fail-closed rules;
- the nested joint-limit hierarchy and the prohibition on physical use of
  provisional simulator limits;
- the six ordered Gazebo functional gates plus Gate 0 static validation;
- machine-readable evidence and configuration fingerprints for every accepted
  test run;
- the statement that passing Gazebo gates proves functional software behavior,
  not physical safety or fidelity.

The approval did not itself select exact runtime values; rates, timeouts, QoS,
topic names, test tolerances, and provisional simulator values were subsequently
accepted in `RUNTIME_TIMING_AND_SIMULATION_CONTRACT.md`. Configuration schemas,
parameter names, artifact paths, and composition mechanics are now specified in
the accepted `PARAMETER_AND_CONFIGURATION_COMPOSITION.md` contract. Physical
calibration procedures, implementation tools, and package scaffolding remain
later decisions. The implementation order is specified in the accepted
`PHASED_DELIVERY_PLAN.md`.
