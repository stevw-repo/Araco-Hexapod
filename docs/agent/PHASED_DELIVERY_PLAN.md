# Araco Hexapod — Phased Simulator Delivery Plan

Status: **ACCEPTED by the user**

Decision date: 2026-08-15

Scope: implementation order for the accepted nine-package ROS 2 Jazzy
repository and Gazebo Gates 0–6. This plan defines deliverables, test layers,
evidence, regression rules, and phase boundaries. It does not authorize
scaffolding, implementation, package creation, physical actuation, or external
CI/publishing.

## Delivery outcome

Implementation will use one repository-foundation phase followed by one
blocking phase for each accepted Gazebo gate:

```text
Architecture closeout and explicit scaffolding authorization
  -> Phase 0: repository foundation (no Gazebo gate claimed)
  -> Phase 1: Gate 0 model/configuration integrity
  -> Phase 2: Gate 1 spawn/controllers/stable hold
  -> Phase 3: Gate 2 kinematics/standing validity
  -> Phase 4: Gate 3 static body-pose control
  -> Phase 5: Gate 4 tripod locomotion/controlled stop
  -> Phase 6: Gate 5 supervision/fault injection
  -> Phase 7: Gate 6 reproducible headless baseline
  -> complete simulator control baseline
```

No phase may claim a later gate while an earlier gate is failing, skipped,
disabled, marked expected-failure, or passing only with an easier configuration.
Every gate run uses the accepted thresholds from
`RUNTIME_TIMING_AND_SIMULATION_CONTRACT.md`.

## Global delivery rules

1. **One vertical increment at a time.** Implement only the production behavior
   and test support required for the current gate plus already accepted common
   foundations.
2. **No fake readiness.** A placeholder node, static status publisher, or test
   process cannot assert a production readiness bit merely to advance launch.
3. **Real production path for scored behavior.** Gate 3 onward commands enter
   through the configured system-test candidate input, arbitration, safety,
   locomotion, standard controllers, and Gazebo backend. Tests never publish
   directly to selected, safe, or controller command topics.
4. **Tests are written with the behavior.** A production increment is not
   complete until its local tests and current gate evidence exist.
5. **Prior gates remain green.** A phase exit reruns every gate through the
   current one in order.
6. **No silent tuning.** Accepted rates, timeouts, limits, gains, physics,
   commands, and thresholds cannot be weakened to obtain a pass. A required
   change is a versioned architecture/configuration revision with rationale,
   followed by all affected regression gates.
7. **Installed-space execution.** Gate validation resolves package-share
   resources from `install/`; a test that works only from source-tree paths does
   not pass.
8. **Simulator-only evidence.** No result from these phases promotes a
   provisional limit, mass, inertia, gain, or stop behavior to physical safety
   evidence.
9. **No automatic commits or publishing.** Source-control checkpoints are
   presented to the user, but commits, pushes, releases, CI service changes,
   and public publication require their own authorization.
10. **Handoff after every gate.** Durable working state records the completed
    scope, files, validation commands/results, evidence location, remaining
    limitations, blockers, and exact next phase.

## Architecture-closeout boundary

Acceptance of this plan completes the currently required simulator architecture
sequence. Before Phase 0 starts, a separate review and explicit
authorization must establish:

- the accepted architecture documents are internally consistent;
- the repository/package license value is selected for `package.xml`; the root
  `LICENSE` was added in the separately authorized pre-Phase-0 checkpoint;
- the user explicitly authorizes creation of `src/`, the nine package
  skeletons, and normal workspace metadata;
- no physical profile, hardware transport, servo command, or Pi deployment is
  included;
- existing uncommitted evidence and documentation are preserved.

The Pi OS decision does not block simulator implementation. The PXN-2113 Pro
mapping also does not block it; keyboard teleoperation and the test source are
sufficient for the baseline.

## Phase summary

| Phase | Blocking result | Primary packages maturing | Motion scope at exit |
|---|---|---|---|
| 0 — Repository foundation | Clean workspace build/test substrate; no gate claimed | All package manifests; `araco_interfaces` first | None; no runnable robot |
| 1 — Static model/configuration | Gate 0 pass | Description, bringup composer, package schemas/config, static system tests | None; Gazebo is not required |
| 2 — Spawn and hold | Gates 0–1 pass | Gazebo, bringup lifecycle, hold-only locomotion, minimum readiness supervision | Stable six-foot standing hold only |
| 3 — Kinematics | Gates 0–2 pass | Kinematics and locomotion standing transaction | Computed standing hold only |
| 4 — Static body pose | Gates 0–3 pass | Locomotion body transform; happy-path arbitration/safety | Stationary height/translation/roll/pitch/yaw |
| 5 — Tripod and stop | Gates 0–4 pass | Locomotion gait/stop, source-loss handling, keyboard teleop | Slow accepted tripod envelope |
| 6 — Full supervision | Gates 0–5 pass | Arbiter/safety fault, reset, handover, restart behavior | Same motion envelope under complete software supervision |
| 7 — Reproducible baseline | Gates 0–6 pass three clean runs | System tests, evidence/reporting, CI composition | Simulator control baseline declared complete |

## Phase 0 — Repository foundation

### Purpose

Create a buildable, testable ROS workspace structure after explicit scaffolding
authorization. This phase establishes packaging and interfaces but makes no
claim that the robot model or runtime is valid.

### Deliverables

- Create the accepted nine packages under `src/` with correct package types,
  maintainers, selected license, build tools, and dependency direction.
- Apply the selected MIT License at the repository root and as an exact copy in
  every package; set each initial package manifest to
  `<license file="LICENSE">MIT</license>` and apply project source SPDX headers.
- Audit direct linked and bundled dependency licenses and record every
  third-party license/attribution obligation without
  relicensing dependencies or imported assets.
- Review the existing Fusion add-in's proprietary Autodesk API boundary before
  treating that tool as a supported distributable integration; MIT does not
  replace Autodesk entitlement or developer terms.
- Add root workspace hygiene appropriate to a public repository: `.gitignore`,
  replace the existing minimal `README.md` stub with a concise build-focused
  overview, and track no `build/`, `install/`, or `log/` output.
- Implement the already accepted project IDL in `araco_interfaces`, including
  messages/actions, constants, package dependencies, and interface-level tests.
- Establish warning/lint/test conventions without creating a generic
  `araco_utils` package.
- Declare package installation rules so later `config/`, `schema/`, meshes,
  URDF, worlds, and launch resources resolve from installed package shares.
- Add package-local test directories but no empty test that reports a false
  pass and no placeholder executable that reports readiness.

### Required checks

- Dependency graph matches `REPOSITORY_ARCHITECTURE.md` and contains no cycle.
- Direct linked/bundled dependencies have no unresolved license incompatibility
  or missing required attribution/source-distribution plan.
- A clean `rosdep` resolution and `colcon build` succeeds.
- Interface generation and introspection succeed with exact accepted fields,
  constants, and dependency types.
- `colcon test` reports no failed tests and no package with a fabricated gate
  result.
- A clean shell can source `install/setup.bash` and locate all nine packages.

### Exit boundary

Phase 0 proves only repository integrity. It does not pass Gate 0, render a
robot, start Gazebo, or authorize a motion-capable node. The phase checkpoint
must explicitly say “no Gazebo gate claimed.”

## Phase 1 — Gate 0: model and configuration integrity

### Work packet 1A — Configuration substrate

- Implement the strict YAML/JSON-compatible loader, common artifact envelope,
  package-owned JSON Schemas, semantic validators, and generated parameter
  declarations accepted in
  `PARAMETER_AND_CONFIGURATION_COMPOSITION.md`.
- Implement the `araco_bringup` preflight/composition tool and its atomic
  runtime-bundle writer.
- Create the package-owned v0 artifacts and the `gazebo_dev_v0` and
  `gazebo_ci_v0` profile-selection graphs with equal behavior fingerprints.
- Reject duplicate/unknown/non-finite data, unresolved package resources,
  version/dependency mismatches, forbidden deployment scopes, and arbitrary
  overlays.
- Unit-test canonical normalization, hashes, non-circular fingerprint fields,
  exact package-share resolution, and failure-before-launch behavior.

### Work packet 1B — Canonical description

- Ingest the accepted Fusion/export evidence into the single canonical model
  registry with 26 primary links, 25 revolute joints, accepted names,
  parent/child relationships, axes, frames, roles, and ordering.
- Add reviewed visual meshes and deliberately simplified collision geometry;
  visual meshes do not automatically become collision meshes.
- Record creator, source, exact license/attribution, modification, and
  redistribution status for every bundled mesh/model resource. Exclude vendor
  CAD with unknown redistribution terms and replace it with project-authored
  simplified geometry when necessary.
- For every project-generated mesh/model form, retain the preferred editable
  source and reproducible generation tooling; do not depend on an unpublished
  mixed-rights Fusion archive as its source.
- Encode the simulator-only nominal standing reference and provisional joint
  limits with evidence/fidelity labels.
- Convert `rough_estimate_v0` into the package-owned runtime dynamics artifact
  while preserving its generation provenance.
- Assign the missing PiSugar, main-battery, and LSC-32 proxies documented rough
  poses, then construct a positive-valid base-link center of mass and inertia.
  This is an estimate and does not require another Fusion export.
- Make Xacro/templates rendering mechanisms rather than a second numeric
  topology. Produce one normalized expanded URDF and hash it.

### Work packet 1C — Generated integration forms

- Derive the 24-leg, one-gimbal, and 25-state joint sets from canonical roles.
- Generate separate controller-manager and per-controller parameter files;
  do not hand-maintain joint arrays.
- Generate the Gazebo description/backend mapping and validate all resource
  references, but do not require a live physics process for Gate 0.
- Add the flat-world descriptor, controller policy, source/safety/gait/QoS
  artifacts, and executable Gate 0 threshold artifact under their owners.

### Gate 0 test layers

- Owner schema/unit tests for every artifact and generated parameter schema.
- Description tests for tree topology, axes, transforms, resources, model
  limits, mass/inertia mathematics, total mass, nominal pose, collision, and
  redistributable-asset metadata.
- Composer tests for profile compatibility, exact artifact versions, generated
  controller partitions, fingerprints, and deterministic repeated expansion.
- Cross-package `araco_system_tests` validation that consumes only installed
  resources and the emitted runtime bundle.
- Explicit negative fixtures for every fail-closed class; corrupt artifacts
  must fail before any Gazebo process starts.

### Exit boundary

Gate 0 passes only when every numerical and structural Gate 0 threshold passes,
including the `3.924393 ± 0.001 kg` total and valid proxy-inclusive base
inertia. The evidence contains both profile manifests and proves their behavior
fingerprints match. No live robot motion is attempted.

## Phase 2 — Gate 1: spawn, controller ownership, and stable hold

### Purpose

Prove the complete simulator plant, controller ownership, startup ordering, and
non-moving safety hold before trusting computed kinematics or walking.

### Deliverables

- Implement the authoritative flat Gazebo world, accepted DART/step/friction
  settings, contact sensors, ground-truth scorer outputs, `ros_gz` bridges, and
  `gz_ros2_control` backend overlay.
- Deliver the single expanded robot description to `robot_state_publisher` and
  `controller_manager` through the accepted Jazzy topic path.
- Start `joint_state_broadcaster`, the 24-joint leg JTC, and the one-joint
  gimbal JTC with their generated files passed to each spawner using
  `--param-file`.
- Implement the locomotion lifecycle shell as a real hold-only component. At
  this gate it reads the one accepted nominal standing joint reference and
  continuously publishes a complete 24-joint hold trajectory; it does not
  claim computed IK yet.
- Implement the minimum real supervisor behavior needed for
  `INITIALIZING -> INACTIVE -> HOLDING`, readiness masks, controller/backend/
  joint/time/status watchdogs, and simulated `JointStateProvenance`.
- Implement the real keyboard adapter and its initial mapping/input tests, but
  activate it only in released/no-command state for this gate.
- Implement bringup's exact startup/shutdown sequence. The arbiter and keyboard
  adapter activate only after safety reaches `HOLDING`; no enable request is
  issued in Gate 1.
- Hold the gimbal at zero with no periodic project gimbal publisher.

The direct nominal joint hold is a deliberate transitional implementation, not
a second standing authority. Phase 3 replaces target production with computed
FK/IK and retains the same artifact as a validation oracle.

### Gate 1 test layers

- Launch tests for preflight-before-Gazebo, ordered lifecycle transitions,
  process failure propagation, and orderly shutdown.
- Controller-manager assertions for exact active controllers and exclusive
  24+1 interface claims.
- Topic/provenance assertions for `/clock`, complete finite 25-joint state,
  correct simulated classification, TF, contacts, controller state, and absent
  unauthorized motion.
- Headless physics scoring for settling, tracking error, base velocity/pose,
  foot contacts, penetration, and forbidden contacts.
- Negative startup cases for missing model, missing controller file, incorrect
  partition, unresolved mesh, absent clock, and invalid initial state.

### Exit boundary

Gates 0 and 1 pass consecutively. Safety reaches `HOLDING` within the accepted
wall-time budget, never enters `MOTION_ENABLED`, and the final scored hold meets
every accepted metric. No body-pose or gait command is enabled.

## Phase 3 — Gate 2: kinematics and standing-reference validity

### Deliverables

- Implement `araco_kinematics` as a pure C++ library with typed SI/REP-103
  inputs, deterministic FK/IK, explicit branch policy, reachability result,
  singularity classification, and finite/limit checks.
- Give the pure library a typed geometry input. The locomotion-side model
  adapter derives that input from the canonical description artifact; the
  kinematics production library does not depend on the description package or
  add hard-coded legacy dimensions.
- Add seeded property tests with at least 10,000 reachable samples per
  canonical leg class across both mirror signs, explicit boundaries,
  singularities, unreachable cases, and non-finite inputs.
- Implement locomotion's six-leg standing calculation and transactional
  24-joint commit. A failed leg discards the whole candidate state.
- Replace Gate 1's direct joint-target production with computed standing IK,
  while comparing the result to the one nominal standing reference and its
  accepted branch.
- Report real `LocomotionStatus` validity, reason, per-leg status, and complete
  trajectory status; do not claim controller tracking.

### Gate 2 test layers

- Pure library unit/property tests for exact accepted FK/IK error, symmetry,
  branch, limit, finite, and rejection criteria.
- Component tests for whole-body transform construction, all-six-leg
  transaction behavior, no partial internal commit, and complete ordered
  trajectory generation.
- Gazebo standing test that reuses the Gate 1 scorer with the IK-generated hold.
- Regression execution of Gates 0 and 1 before Gate 2 scoring.

### Exit boundary

Gates 0–2 pass. The simulator still supports standing only. A correct-looking
pose cannot compensate for a property-test failure or an invalid partial
transaction.

## Phase 4 — Gate 3: static body-pose control

Status: implemented and validated on 2026-08-16. Gates 0–3 pass. Phase 5
remains separately authorized work.

### Deliverables

- Implement locomotion's absolute body-pose offset handling for height,
  planar translation, roll, pitch, and posture yaw while all six nominal feet
  remain fixed in the ground/body transform calculation.
- Implement the happy-path command pipeline required for scoring: system-test
  candidate validation, arbiter selection, safety readiness, trusted
  `ENABLE_MOTION`, mandatory released state, fresh activation edge, and safe
  command execution.
- Enforce configured body/command envelopes before IK and final effective joint
  limits before transaction commit.
- Preserve zero gait-phase advancement for every `GAIT_STAND` body command.
- Publish foot targets for visualization as non-authoritative debug data.
- Use only system-test ground truth in the scorer; locomotion and supervision
  must not subscribe to it.

### Gate 3 test layers

- Unit/component tests for body transforms, direction/sign conventions,
  quaternion handling, envelope limits, absolute-not-integrated pose semantics,
  and fixed-foot targets.
- Command-pipeline integration tests for release, enable, fresh edge, command
  limiting, and hold without implementing the later complete fault matrix by
  shortcut.
- Gazebo cases for zero, positive/negative 50% single-axis offsets, and the
  combined 35% case, using the accepted settle/scoring windows.
- Regression execution of Gates 0–2 before Gate 3 scoring.

### Exit boundary

Gates 0–3 pass with no gait-phase movement, limit breach, partial transaction,
non-foot contact, or ground-truth leakage into production control.

## Phase 5 — Gate 4: tripod locomotion and controlled stop

Status: implemented and validated on 2026-08-16. Gates 0–4 pass. Phase 6
remains unauthorized.

### Deliverables

- Implement the deterministic tripod phase machine, support/swing grouping,
  foot path, stance placement, and body/turn blending at the fixed 100 Hz
  locomotion rate. The subsequently accepted scheduler advances continuous
  `1.0–1.5 Hz` phase while using stride as the primary speed variable.
- Generate complete named positions-only one-point trajectories with the
  accepted 40 ms horizon and transactional phase/foot/IK commit.
- Implement bounded forward, reverse, lateral, yaw, and combined commands in
  the accepted slow simulator envelope.
- Implement `GAIT_STAND`, manual hold, source release/staleness, controlled
  deceleration, completion of the current swing transition, planned six-foot
  stance, 0.250-second stable-hold dwell, and no-surprise-resume behavior.
- Complete the arbiter/safety portions required for ordinary selection loss and
  manual hold. Full restart, handover, and component-fault matrices remain
  Phase 6 work and cannot be falsely reported as complete.
- Exercise the focused keyboard window and full-state heartbeat adapter through
  the normal developer command path as a non-scored human-input smoke test.
  Gate 4 scoring still uses the deterministic system-test source. PXN-2113 Pro
  mapping remains deferred until the actual device is observed.

### Gate 4 test layers

- Pure gait tests for phase monotonicity/wrap, tripod membership, foot-path
  continuity, support/swing transitions, stand transition, and deterministic
  seeded results.
- Locomotion transaction tests for invalid single-leg results, complete
  trajectories, limit checks, controlled-stop state, and hold dwell.
- End-to-end system-test-source cases for all accepted baseline directions,
  yaw, combined command, manual hold, active zero/`GAIT_STAND`, and selected
  source loss.
- Gazebo scoring for contact order, tracking, body stability, collisions,
  phase, stop timing, drift, and final six-foot hold.
- Regression execution of Gates 0–3 before Gate 4 scoring.

### Exit boundary

Gates 0–4 pass. The robot can perform the accepted slow simulator gait and
controlled stop, but the complete fault/restart/handover contract is not yet a
released baseline.

## Phase 6 — Gate 5: supervision and fault injection

### Deliverables

- Complete the command arbiter's eligibility, source quarantine, duplicate/
  reorder/restart handling, unique IDs/priorities, steady freshness, selection
  epochs, and explicit no-lower-priority-fallback behavior.
- Complete all eight safety states, dispositions, reason codes, readiness and
  fault masks, latching, reset guards, enable acquisition window, safety epochs,
  deliberate higher-priority controlled-stop handover, and orderly shutdown.
- Complete every accepted watchdog and fault response across candidate,
  selected, safe, joint-state, locomotion, controller, backend, and clock
  boundaries.
- Ensure locomotion independently stops on stale safe command and never
  advances more than one 10 ms tick after a received/locally detected stop.
- Implement only typed, explicit `test_only` scenario composition. Production
  launch cannot select fault fixtures.

### Fault-injection test architecture

Gate 5 uses the narrowest truthful test seam for each failure:

- Candidate malformed/duplicate/reordered/restart cases publish only through
  the allowed system-test candidate input.
- Candidate, selected, safe, joint-state, and status loss use test-only relays,
  lifecycle transitions, or process termination to withhold real output; tests
  do not forge trusted command messages.
- Controller loss uses controller-manager lifecycle/switch operations; backend
  loss uses the test-owned Gazebo process lifecycle.
- Clock pause/reset uses Gazebo control. It does not add a competing production
  `/clock` publisher.
- Impossible internal IK/limit/invariant branches use dependency-injected
  harnesses around the real locomotion/supervision libraries. Where controller
  outcome is part of the accepted scenario, the full test composition launches
  the real node wrapper with a test-only injected provider, so Gazebo and the
  real controller remain in the path. This injection is compiled only into a
  test target and does not add a runtime “force fault” service or parameter to
  production nodes.
- Malformed joint-state integration may use a test-only relay selected by the
  test profile; normal profiles connect directly to the broadcaster.

This split is intentional: a system test should use the real process boundary
where the failure can occur naturally, while an impossible internal branch is
tested by deterministic library injection rather than a production backdoor.

### Gate 5 test layers

- Exhaustive deterministic state-machine table tests for every accepted event,
  guard, state, disposition, reason, fault, reset, and epoch result.
- Timer/watchdog tests using controlled steady and ROS clocks rather than wall
  sleeps.
- ROS component tests for lifecycle restarts, topic loss, QoS compatibility,
  and no forged trusted-command path.
- Gazebo process/controller/time failure scenarios where plant behavior is
  relevant.
- Three repetitions of every required scenario with exact discrete outcomes.
- Regression execution of Gates 0–4 before Gate 5 completion.

### Exit boundary

Gates 0–5 pass. There are zero unexpected execute samples, surprise resumes,
automatic lower-priority motion fallbacks, unguarded resets, or epoch mismatches.
The result remains software supervision in simulation, not a physical emergency
stop or certified safety system.

## Phase 7 — Gate 6: reproducible headless baseline

### Deliverables

- Finalize the `araco_system_tests` gate runner, scenario registry, metrics,
  JUnit output, structured result schema, log classification, and focused ROS
  bag-on-failure policy.
- Run all package unit/component/static tests and Gates 0–5 from installed
  resources before the repeated baseline.
- Prove `gazebo_dev_v0` and `gazebo_ci_v0` have identical production behavior
  fingerprints; CI may disable presentation only.
- Run the complete headless suite three consecutive times from clean processes
  with seed 42 and no retry.
- Add project-owned C++ sanitizer execution and reject any project crash,
  sanitizer failure, lifecycle deadlock, missing artifact, or unclassified
  project `ERROR`/`FATAL`.
- Record dependency and environment versions, measured loop/controller rates,
  missed cycles, physics settings, expanded-model hash, all effective
  configuration identities, and the accepted performance/repeatability metrics.
- Write a simulator developer runbook and troubleshooting guide only after the
  commands have been verified from a clean shell.
- Define an external CI workflow as a separate mutation requiring user
  authorization; local headless Gate 6 can pass before credentials or a hosted
  CI service are configured.

### Exit boundary

Gate 6 passes only when all three no-retry runs meet exact discrete outcomes,
cross-run physical tolerances, real-time-factor/runtime budgets, and evidence
requirements. A slower future CI host may receive only the separately reviewed
wall-time allowance already permitted by the runtime contract.

Passing Gate 6 declares the Gazebo functional control baseline complete. It
unblocks later perception/navigation and Isaac adapter work. It does not
authorize physical hardware, establish sim-to-real fidelity, or unblock a
physical profile without the separate physical-safety decisions.

## Package maturation matrix

`F` means foundation, `P` means first production implementation, `E` means an
extension, and `V` means validation/evidence ownership.

| Package | Phase 0 | Gate 0 | Gate 1 | Gate 2 | Gate 3 | Gate 4 | Gate 5 | Gate 6 |
|---|---|---|---|---|---|---|---|---|
| `araco_interfaces` | F/P IDL | V contract | — | — | — | — | — | V compatibility |
| `araco_description` | F | P/V model | E Gazebo-facing model fixes only | V standing oracle | V pose resources | V limits/collision | — | V fingerprints |
| `araco_kinematics` | F | Config/schema only | — | P/V typed geometry | E body cases | V gait cases | V injected faults | V regression |
| `araco_locomotion` | F | Config/schema only | P hold shell | E standing IK | E static body | E gait/stop | E watchdog/fault behavior | V timing/regression |
| `araco_supervision` | F | Config/schema only | P readiness/hold | V status | E happy path | E ordinary stop/loss | E/V full contract | V timing/regression |
| `araco_teleop` | F | Mapping schema | P keyboard adapter, released only | — | — | V dev smoke | V restart/input cases | V build/regression |
| `araco_gazebo` | F | World/backend descriptors | P/V plant/bridge | V hold | V pose scoring | V gait scoring | E/V fault control | V headless baseline |
| `araco_bringup` | F | P/V composer/profiles | E/V lifecycle/controllers | V | V command composition | V | E test-only compositions | E/V evidence orchestration |
| `araco_system_tests` | F | P/V Gate 0 | E/V Gate 1 | E/V Gate 2 | E/V Gate 3 | E/V Gate 4 | E/V Gate 5 | E/V Gate 6 |

A dash means no planned production expansion, not “skip all regression tests.”

## Test and evidence hierarchy

Every phase uses the cheapest layer capable of proving the property:

1. **Schema/static tests** prove configuration and model structure without ROS
   processes or physics.
2. **Pure unit/property tests** prove algorithms, state machines, and numerical
   invariants deterministically.
3. **Component tests** prove node-local parameters, lifecycle, clocks,
   callbacks, transactions, and test-injected dependency failures.
4. **ROS integration tests** prove interfaces, QoS, process boundaries,
   controllers, and launch failure behavior.
5. **Gazebo system tests** prove plant/contact/controller behavior and score
   ground truth without feeding it into control.
6. **Repeated headless suites** prove bounded reproducibility and operational
   evidence production.

Using Gazebo for a property that a pure test can prove is slower and less
diagnostic. Using a pure mock for a property that depends on controller claims,
contacts, physics time, or process loss is insufficient. Required coverage is
therefore split rather than duplicated blindly at every layer.

## Evidence bundle contract

Every gate attempt, including a failure, receives a unique non-source output
directory. At minimum it contains:

```text
gate_<n>_<run_id>/
├── gate_result.json
├── junit.xml
├── metrics.json
├── environment.json
├── process_outcomes.json
├── input_selection.json
├── validation_report.json
├── effective_config/          # only after successful preflight
├── logs/
└── failure/                   # created when needed
    ├── focused.bag/
    └── failure_summary.json
```

`gate_result.json` records:

- gate/phase identity, result, start/end time, seed, and repetition index;
- source revision or explicit “dirty working tree” identity;
- behavior/input-selection/run fingerprints and generated resource hashes;
- artifact/schema/profile/threshold/scenario IDs, versions, and hashes;
- ROS, Gazebo, RMW, compiler, dependency, OS, and host identities;
- exact discrete expectations and metric thresholds/results;
- loop rates, controller cycles, process exits, classified warnings/errors;
- fidelity limitations and a reason for every failure.

An expected fail-closed fixture that stops during preflight records the input
selection and validation report but does not create a partial
`effective_config/` directory. The containing Gate 0 suite passes only when the
fixture is rejected for the expected typed reason.

Artifacts remain under the run/log output and are not committed by default.
Failure evidence is retained even when the phase is repaired; the final handoff
links the latest passing run and any unresolved failure.

## Failure and tuning policy

When a gate fails, classify it before changing values:

| Failure class | Required response |
|---|---|
| Implementation defect | Fix within the current phase; rerun current and affected prior tests |
| Invalid/missing source evidence | Stop the phase, improve the owning artifact/evidence, version it, rerun from Gate 0 |
| Provisional simulator model mismatch | Change only the owning simulator artifact with rationale/version; rerun Gate 0 through current |
| Accepted architecture contradiction | Stop implementation and propose an architecture revision before code continues |
| Threshold appears unrealistic | Present measured evidence; never relax it silently or only in CI |
| Environment/dependency defect | Record versions and repair environment; do not classify it as a robot pass |
| Nondeterministic/flaky outcome | Treat as failure; no automatic retry or “known flaky” waiver for required gates |

Tuning uses fixed documented scenarios and a separate exploratory run identity.
Exploratory results never count as gate evidence. Once a proposed change is
accepted, the owner artifact is versioned and the scored gate starts from a
fresh process with no hidden runtime mutation.

## Regression and invalidation rules

Every source change reruns static build/lint/unit tests plus the earliest gate
it can affect and all later gates already reached:

| Changed area | Earliest mandatory gate |
|---|---:|
| Interface fields/constants or common configuration semantics | 0 |
| Model topology, transforms, axes, meshes, collision, dynamics, limits, pose | 0 |
| Profile, composer, controller policy, world, backend, bridge, timing, QoS | 0 |
| Lifecycle/startup/controller-hold behavior | 1 |
| Kinematics, model-to-solver mapping, standing transaction | 2, plus Gate 1 hold regression |
| Body transform or static-pose policy | 3 |
| Gait, trajectory generation, controlled stop | 4 |
| Arbitration, safety, watchdog, fault, reset, handover | 5, plus Gate 1 readiness and Gate 4 stop regressions |
| Test scorer or accepted threshold artifact | Owning gate, after explicit review |
| Presentation-only GUI/RViz/logging choice | Static behavior-fingerprint equality plus relevant launch smoke test |

If ownership is ambiguous, choose the earlier gate. A later pass is invalid if
an earlier required rerun is missing.

## Phase checkpoint and handoff format

At each phase boundary, `WORKING_STATE.md` is rewritten to include:

- current phase and exact goal;
- completed deliverables and files changed;
- validation commands and concise outcomes;
- gate evidence path and configuration fingerprints;
- accepted fidelity limitations;
- failures still under investigation;
- explicit blockers and whether user input is actually required;
- the exact next work packet and its entry criteria.

The checkpoint presented to the user distinguishes:

- **implemented** from merely designed;
- **test passed** from visually inspected;
- **simulator estimate** from measured physical evidence;
- **gate passed** from partial progress;
- **ready for next phase** from **authorized to begin next phase**.

## Accepted scope and authorization boundary

The user's 2026-08-15 approval freezes:

- one foundation phase followed by one blocking phase per Gate 0–6;
- the package maturation and real-production-path order;
- the transitional Gate 1 nominal hold followed by Gate 2 computed IK;
- per-phase test layers, evidence, failure classification, and no-retry policy;
- regression invalidation and durable handoff requirements;
- Gate 6 as the boundary that unlocks later simulator phases but not hardware.

Approval of this plan did not itself authorize implementation. The user
subsequently authorized and completed Phase 0, then separately authorized
Phase 1 / static Gate 0, Phase 2 / live Gate 1, Phase 3 / computed-standing
Gate 2, and Phase 4 / static body-pose Gate 3; all passed on 2026-08-16.
Phase 5 / Gate 4 was separately authorized, implemented, and validated on
2026-08-16. Gates 0–4 pass. Phase 6 and later work still require separate
authorization.
