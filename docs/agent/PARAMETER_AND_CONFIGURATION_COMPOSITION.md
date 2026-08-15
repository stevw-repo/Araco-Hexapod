# Araco Hexapod — Parameter, Artifact, and Runtime Composition Contract

Status: **ACCEPTED by the user**

Decision date: 2026-08-15

Scope: exact configuration layers, artifact envelopes and locations, ROS
parameter policy, deployment profiles, deterministic runtime composition,
fingerprints, overrides, and Jazzy controller integration. This decision did
not create schemas, YAML, Xacro, launch files, ROS packages, or tests and does
not authorize physical actuation.

## Decision outcome

The initial system will use three deliberately different configuration layers:

1. **Package-owned source artifacts** are the human-reviewed authority for
   model, algorithm, simulator, integration, and test values.
2. **Generated ROS parameter declarations** give every project node typed,
   validated, documented parameters at compile time.
3. **A resolved runtime bundle** is produced by a fail-closed preflight step
   for one named profile and one run. Launch and upstream ROS/Gazebo processes
   consume only this bundle or the exact installed artifacts it records.

These layers must not be collapsed into one large bringup YAML. ROS parameters
cannot faithfully represent every structured robot artifact, and a large
bringup file would duplicate values owned by the description, locomotion,
supervision, Gazebo, and test packages.

## Non-negotiable composition rules

1. A source value appears in one owned artifact only. Profiles select artifacts;
   they do not copy their data.
2. Joint names and controller partitions are resolved from the canonical model
   registry. They are never separately hand-maintained in controller YAML.
3. There is no generic YAML deep merge, anchor/alias inheritance, or arbitrary
   command-line parameter overlay in an accepted launch path.
4. Every selected artifact is resolved from an installed package share through
   the ament resource index. Source-tree absolute paths and “find the newest
   file” behavior are forbidden.
5. A profile pins an exact package, relative path, artifact ID, and artifact
   version. The preflight compiler rejects any mismatch.
6. Missing, duplicate, unknown, non-finite, out-of-range, incompatible, or
   simulator-ineligible motion data is a startup error.
7. Motion-affecting values have no silent code defaults. The selected profile
   must supply them.
8. Generated files are written outside the source tree, visibly labeled, and
   never edited as authorities.
9. `gazebo_dev_v0` and `gazebo_ci_v0` must have the same behavior fingerprint.
   Their permitted differences cannot change robot behavior.
10. A physical profile cannot be created by renaming or overriding a simulator
    profile. It requires separately approved artifacts and validators.

## Authoring format and validation model

### Strict YAML subset

Project-owned structured artifacts use UTF-8 YAML files whose parsed data is
also valid JSON data. The loader rejects:

- duplicate mapping keys;
- anchors, aliases, merge keys, custom tags, and implicit timestamps;
- non-string mapping keys;
- non-finite numbers, including NaN and positive/negative infinity;
- multiple YAML documents in one file;
- unknown envelope or data fields;
- values whose type depends on YAML 1.1 implicit conversion.

This restriction keeps the files readable while making validation and hashing
unambiguous. Unit-bearing values use SI/REP-103 and a unit suffix where the
quantity would otherwise be unclear, such as `_rad`, `_m`, `_s`, `_hz`, or
`_kg`.

### Artifact envelope

Every structured source artifact has this logical envelope:

```yaml
schema_id: araco://schemas/<owner>/<domain>/v1
schema_version: 1
artifact_id: araco.<owner-domain>.<descriptive-name>
artifact_version: 0.1.0
owner_package: araco_<package>
deployment_scope: simulator_only
evidence:
  class: simulator_estimate
  sources: []
dependencies: []
data: {}
```

The exact allowed fields are:

| Field | Rule |
|---|---|
| `schema_id` | Stable local schema identity; never fetched from the network at runtime |
| `schema_version` | Positive integer version of the artifact shape |
| `artifact_id` | Stable semantic identity; changing a filename does not change the identity |
| `artifact_version` | Semantic version; any changed behavior/value requires a version change |
| `owner_package` | Must equal the installed package containing the artifact |
| `deployment_scope` | Initially `simulator_only`, `test_only`, or `deployment_eligible`; the last is not valid without accepted evidence rules |
| `evidence` | Evidence class plus source/provenance references; never credentials or secrets |
| `dependencies` | Exact artifact IDs and versions required to interpret this artifact |
| `generated_from` | Optional source artifact IDs/hashes and generator identity for a derived snapshot |
| `data` | Owner-defined, schema-validated payload |

The common envelope is enforced by the composition compiler. Each owning
package installs a self-contained JSON Schema Draft 2020-12 document for its
`data` payload under `schema/`. Schemas close their objects with
`additionalProperties: false` or the equivalent composed-schema rule. Runtime
validation is offline; a schema `$ref` cannot trigger a network request.

Native resources—meshes, Xacro templates, URDF, and SDF—retain their native
formats and validators. A structured resource descriptor gives them an artifact
identity, version, path, dependencies, evidence classification, creator,
source, SPDX or other exact license expression, required attribution,
modification status, and redistribution status; preflight computes the resource
hash. A project-generated form additionally identifies its preferred editable
source and generator/tool identity. A resource with unknown
redistribution permission or unavailable required source is rejected from the
installed/public bundle. Numeric robot topology is not copied into the
descriptor.

## Package layout convention

When scaffolding is eventually authorized, every package uses these locations:

```text
<package>/
├── config/                     # owned source values and resource descriptors
├── schema/                     # JSON schemas for owned source artifacts
├── parameters/                 # generate_parameter_library declaration input
├── launch/                     # only where the package genuinely owns launch
└── ...                         # source, include, meshes, urdf, worlds, tests
```

All runtime artifacts and schemas are installed below
`share/<package>/...`. `parameters/` files are compile-time declarations, not
deployment values and not a second configuration authority.

No loose runtime configuration is placed at the repository root.

## Initial artifact registry

The filenames below are the accepted first artifact identities and planned
paths. They remain design contracts until Phase 1 creates them. Version suffix
`v0` means the values are provisional or initial; schema version remains
independent of that fidelity label.

| Owner | Planned source artifact | Authority |
|---|---|---|
| `araco_description` | `config/model/canonical_model_v1.yaml` | 26-link/25-joint names, topology, order, roles, transforms, axes, frames, and geometry resource references |
| `araco_description` | `config/limits/provisional_sim_v0.yaml` | Simulator-only canonical joint range, velocity, and effort set |
| `araco_description` | `config/poses/nominal_standing_reference_v0.yaml` | Approximate simulator standing joint/body reference |
| `araco_description` | `config/dynamics/rough_estimate_v0.yaml` | Derived mass/center/inertia snapshot generated from accepted rough Fusion evidence |
| `araco_description` | `config/resources/robot_description_v1.yaml` | Xacro/template and mesh resource identities; no duplicate transforms |
| `araco_kinematics` | `config/solver/ik_v0.yaml` | Solver tolerances, iteration/numerical policy, and reachability margins |
| `araco_locomotion` | `config/gait/tripod_slow_sim_v0.yaml` | Tripod phasing, foot path, cycle duration, stance/body envelope, and trajectory horizon |
| `araco_locomotion` | `config/policy/operational_sim_v0.yaml` | Operational joint/body/velocity/acceleration restrictions nested inside model limits |
| `araco_supervision` | `config/sources/simulator_v0.yaml` | Source IDs, priorities, enablement, rates, and freshness timeouts |
| `araco_supervision` | `config/policy/simulator_v0.yaml` | Readiness, handover, stop, fault, reset, command-bound, and watchdog policy |
| `araco_supervision` | `config/qos/control_v0.yaml` | Project-owned command/status endpoint QoS profiles |
| `araco_teleop` | `config/mappings/keyboard_sim_v0.yaml` | Initial keyboard mapping and shaping |
| `araco_gazebo` | `config/world/flat_ground_v0.yaml` | Descriptor for the authoritative SDF world and accepted physics/seed integration |
| `araco_gazebo` | `config/backend/gz_ros2_control_v0.yaml` | Gazebo control plugin, simulated interfaces, gains, damping/friction overlay, and spawn behavior |
| `araco_gazebo` | `config/bridge/simulator_v0.yaml` | ROS–Gazebo bridge endpoints and simulation-only topic mapping |
| `araco_bringup` | `config/wiring/single_robot_v0.yaml` | Node names, namespace-relative topic wiring, remaps, and process composition |
| `araco_bringup` | `config/controllers/simulator_v0.yaml` | Controller types, interfaces, rates, JTC policy, and lifecycle order—but not hand-written joint lists |
| `araco_bringup` | `config/profiles/gazebo_dev_v0.yaml` | Exact development artifact selection and allowed presentation choices |
| `araco_bringup` | `config/profiles/gazebo_ci_v0.yaml` | Exact CI artifact selection and allowed headless/reporting choices |
| `araco_system_tests` | `config/thresholds/gazebo_baseline_v0.yaml` | One executable authority for accepted Gate 0–6 thresholds |
| `araco_system_tests` | `config/scenarios/<gate>_<case>_v0.yaml` | Scenario inputs, duration, required threshold IDs, and expected state/fault outcomes |

`tools/fusion/araco_rough_dynamics_v0.json` remains offline evidence. The later
description artifact is a generated, labeled runtime snapshot with that file
and generator recorded in `generated_from`; runtime launch never reads
`tools/fusion`.

No placeholder gamepad artifact is created. The keyboard source is sufficient
for initial simulator development. A future
`config/mappings/pxn_2113_pro_v0.yaml` can be accepted after the installed
device's axes, buttons, dead zones, and disconnect behavior are observed.

## Canonical model registry contract

`canonical_model_v1` is the only source for ordered joint membership. Each
joint has a stable role, including:

- `leg_command` for the 24 leg joints;
- `gimbal_command` for `gimbal_yaw_joint`;
- `state` for every joint that must appear in `/joint_states`;
- leg identity and segment role for kinematics and validation.

The composition compiler derives and checks:

- the exact 24-joint leg-controller list in canonical order;
- the one-joint gimbal-controller list;
- the expected 25-joint state set;
- every per-joint initial target, limit, and dynamics lookup;
- controller/resource ownership with no gaps or overlaps.

Xacro templates may contain reusable rendering logic, but they cannot contain a
second numeric topology, transform, axis, limit, or standing-pose table. They
consume compiler-normalized model data. Expanded URDF is a generated runtime
artifact whose hash is recorded.

## ROS node parameter contract

Every project-owned node uses the appropriate C++ or Python target from
`generate_parameter_library` for its ROS parameter declaration schema. The
installed workstation already provides both `generate_parameter_library` and
`generate_parameter_library_py` (`0.7.3` at the time of this decision).

The declaration schema supplies:

- generated typed parameter access rather than string lookups;
- type, range, non-empty, and owner-specific validators;
- `read_only` descriptors for immutable parameters;
- generated parameter documentation;
- required parameters without motion-critical defaults.

Every project node receives these immutable identity parameters:

```text
config.profile_id
config.profile_version
config.behavior_fingerprint
config.input_selection_fingerprint
config.node_config_fingerprint
config.selected_artifact_ids
```

The final `run_fingerprint` and whole-file hashes are not embedded in node
parameter files because doing so would make the final manifest hash
self-referential. `config.node_config_fingerprint` is instead calculated over
the normalized node parameters with that one field excluded; the node can
recalculate and compare it during configuration. The manifest separately
records the SHA-256 of the emitted parameter file.

Small node-owned values are emitted directly into that node's generated ROS
parameter YAML. Large canonical artifacts are supplied as an exact installed
path plus expected SHA-256 to a package-owned typed loader. A node never accepts
an unresolved “latest” artifact name.

For the initial simulator:

- motion-affecting parameters are read-only for the process lifetime;
- undeclared parameters and automatic declaration from overrides are disabled;
- an unknown ROS parameter is fatal during configuration;
- `use_sim_time: true` is explicitly supplied to every simulator node;
- production launch exposes no generic `-p` or extra `--params-file` escape
  hatch.

A changed motion value therefore requires controlled hold, lifecycle
deactivation, process replacement with a newly compiled bundle, complete
readiness validation, and a fresh explicit enable/source edge. Process
replacement is the v0 implementation of accepted reconfiguration; in-place
parameter mutation is intentionally stricter than the minimum architecture.

## Deployment profile contract

A profile is a selection graph, not a bag of copied values. Each artifact
reference contains:

```yaml
package: araco_description
path: config/model/canonical_model_v1.yaml
artifact_id: araco.description.canonical-model
artifact_version: 1.0.0
```

The profile also declares its own ID/version, deployment class, required node
set, namespace policy, and permitted presentation/reporting behavior.

### `gazebo_dev_v0`

- selects the common simulator model, limits, pose, dynamics, algorithms,
  safety policy, controllers, world, and bridge artifacts;
- enables Gazebo GUI, RViz, developer diagnostics, and teleop keyboard by
  profile policy;
- uses the accepted deterministic seed and physics values.

### `gazebo_ci_v0`

- selects the exact same behavior-affecting artifacts;
- is headless and produces the required machine-readable evidence bundle;
- does not launch the live keyboard adapter during scored runs, while retaining
  the same enabled teleop entry in the source registry;
- enables test-only sources only when an individual validated system-test
  fixture composes them;
- uses the same deterministic seed and physics values.

The compiler calculates a behavior fingerprint from the resolved production
behavior artifacts, excluding profile identity and presentation choices, and
refuses normal validation if the development and CI values differ.

## Override policy

Normal bringup accepts only a closed launch-argument set:

- profile ID from the installed allow-list;
- robot namespace;
- GUI on/off;
- RViz on/off;
- log level;
- record-on-failure/report destination behavior.

Namespace and presentation choices affect the full run identity but not the
behavior identity. They cannot change rates, timeouts, source authority,
limits, gait, controllers, physics, safety rules, or `use_sim_time`.

There is no general-purpose profile inheritance or deep merge. A new behavior
combination is a new versioned profile selecting new versioned artifacts.

System-test fault injection uses a separate `test_only` scenario schema with
explicit fields such as source loss, clock pause, delayed status, invalid IK
request, or controller-state interruption. It cannot express an arbitrary path
replacement, select a physical backend, or publish directly to trusted command
topics. The resulting test run has its own run fingerprint while retaining the
base behavior fingerprint and recording the fixture hash.

## Deterministic preflight and runtime bundle

`araco_bringup` will own a Python preflight/composition tool because this is
launch-time orchestration, not real-time control. Its ordered transaction is:

1. Resolve the requested installed profile through the ament index.
2. Parse the strict YAML subset and validate the profile envelope/schema.
3. Resolve every artifact by exact package and relative installed path.
4. Validate artifact envelopes, owner packages, schemas, versions, deployment
   scopes, and exact dependencies.
5. Validate cross-artifact invariants, including joint completeness,
   non-overlapping controller ownership, nested limits, standing-pose validity,
   rates/horizons/watchdogs, source uniqueness, QoS compatibility, and profile
   eligibility.
6. Normalize artifacts and calculate their content hashes.
7. Derive the controller joint lists and all other duplicated upstream forms
   from the canonical model registry.
8. Render and validate the expanded robot description and Gazebo resources.
9. Emit a complete runtime bundle into a unique run-log directory using
   sibling temporary files followed by atomic rename.
10. Re-read and hash the emitted files, then return one immutable manifest to
    launch. Gazebo has not started before this point.

If any step fails, no partial bundle is launched and `READY_MODEL` is
unreachable.

The generated directory contains at least:

```text
effective_config/
├── manifest.json
├── validation_report.json
├── normalized_artifacts/
├── node_params/
│   ├── teleop_adapter.yaml
│   ├── command_arbiter.yaml
│   ├── safety_supervisor.yaml
│   └── locomotion.yaml
├── ros2_control/
│   ├── controller_manager.yaml
│   ├── joint_state_broadcaster.yaml
│   ├── leg_trajectory_controller.yaml
│   └── gimbal_trajectory_controller.yaml
├── gazebo/
│   ├── bridge.yaml
│   └── resolved_world.sdf
└── description/
    └── robot.urdf
```

The runtime bundle is evidence attached to the run, not source configuration.
CI retains it with test results. Local runs retain it under the corresponding
ROS log/run directory according to an explicit retention policy; no code reads
an unrelated previous run's bundle.

## Fingerprints and reproducibility

The loader normalizes parsed data to canonical JSON: keys are recursively
sorted, arrays retain order, strings are UTF-8, insignificant whitespace is
removed, and finite numbers use one specified serialization implemented and
tested by the compiler. SHA-256 is calculated over the normalized bytes.

The manifest records:

- profile ID/version and source hash;
- every artifact ID/version, package, installed path, content hash, schema
  identity, deployment scope, and evidence class;
- generator/compiler identity and source revision when available;
- expanded URDF, world, bridge, node-parameter, and controller-file hashes;
- ROS distribution and package versions, Gazebo version, RMW implementation,
  namespace, seed, and accepted launch arguments;
- behavior, input-selection, and final-run fingerprints below.

`behavior_fingerprint` covers the resolved production artifacts capable of
changing model, robot, controller, command, timing, physics, sensor, or safety
behavior. It excludes profile identity, the closed presentation/logging fields,
and test-only fixtures. A test result records its threshold and fixture hashes
separately, so their exclusion cannot hide which test was run.

`input_selection_fingerprint` covers the exact selected source artifacts,
source-adapter presence, and accepted run arguments before any files are
emitted. It therefore distinguishes a development run with the keyboard
adapter from a scored CI run without it, without misclassifying that closed
input choice as a production-policy difference. It is available to nodes
without creating a hash cycle.

`run_fingerprint` covers the complete resolved manifest, including namespace,
presentation, logging/reporting choices, any validated test fixture, and every
generated-file hash. It is recorded by launch and test evidence, not inserted
back into those generated files.

Each fingerprint field is excluded from its own hash input. Per-node
configuration fingerprints let each node verify exactly what it received;
whole-file hashes verify emitted bytes. Startup logs and diagnostics publish
the non-circular input/behavior/node identities, but diagnostics do not become
an authority in the safety command path.

## Jazzy launch and controller integration

Top-level launch is Python and remains thin: select profile, invoke preflight,
start processes in the already accepted lifecycle order, and stop on any
failure. Domain numbers and joint arrays do not live in launch code.

Jazzy-specific rules are frozen into the composer:

- package resources are resolved from installed share directories, not the
  working tree;
- `robot_state_publisher` publishes the single expanded robot description, and
  `controller_manager` receives the Jazzy `robot_description` topic with an
  explicit remap if namespacing requires it;
- every controller's generated parameter file is passed to its controller
  spawner with `--param-file`; loading controller values only into
  `controller_manager` is not valid in Jazzy;
- controller-manager parameters and individual controller parameters are
  separate generated files;
- no wildcard `/**` parameter block is used for motion-critical nodes;
- each generated node file targets one exact fully qualified node name.

The generated simulator controller composition contains the accepted
`joint_state_broadcaster`, 24-joint leg JTC, and one-joint gimbal JTC. Controller
types, update rates, command/state interfaces, interpolation, timeout, and
constraint policy come from the bringup controller artifact; only membership
and order come from the canonical model registry.

## Validation ownership

Validation is layered rather than entrusted to one script:

| Layer | Responsibility |
|---|---|
| Strict loader | Syntax, duplicate keys, finite JSON-compatible values |
| Owner JSON Schema | Data shape, types, enumerations, local ranges, unknown fields |
| Owner semantic validator | Domain relations that JSON Schema cannot express clearly |
| Bringup composer | Cross-package dependencies, exact selections, wiring, fingerprints, derived outputs |
| Node configure callback | Typed local parameters, artifact hash, runtime assumptions, local resources |
| `ros2_control`/URDF/SDF validators | Upstream-native resource and interface validity |
| System tests | End-to-end Gate 0–6 behavior and retained evidence |

Warnings are permitted only for explicit evidence/fidelity limitations already
allowed by the chosen simulator profile. A malformed or incompatible value is
never downgraded to a warning.

## Accepted scope and implementation boundary

The user's 2026-08-15 approval freezes:

- strict source-artifact format and common envelope;
- exact package ownership and accepted first artifact paths;
- generated typed ROS parameters and read-only motion configuration;
- profile selection instead of copy/merge inheritance;
- closed override policy and separate typed test fixtures;
- deterministic preflight, runtime-bundle contents, and the non-circular
  fingerprint scheme;
- canonical generation of controller joint lists;
- Jazzy spawner and robot-description delivery rules.

Approval does not authorize creating the files. The implementation sequence is
specified in the accepted `PHASED_DELIVERY_PLAN.md`, and the complete
architecture review is recorded in `FINAL_ARCHITECTURE_REVIEW.md`;
implementation still requires a separate explicit Phase 0 authorization.

## Upstream mechanisms this decision depends on

- ROS 2 Jazzy `generate_parameter_library` generates typed parameter libraries
  from declarative schemas and supports validation/read-only declarations:
  `https://github.com/PickNikRobotics/generate_parameter_library`
- ROS 2 launch provides scoped parameter-file actions and Python launch
  composition:
  `https://docs.ros.org/en/ros2_packages/jazzy/api/launch_ros/launch_ros.actions.html`
- Package share locations are resolved through the ament index:
  `https://docs.ros.org/en/ros2_packages/jazzy/api/ament_index_cpp/generated/function_get__package__share__directory_8hpp_1a3680c19cb3223de4a536289d4656ed8e.html`
- Jazzy controllers no longer inherit a parameter file supplied only to
  `controller_manager`; the spawner must receive `--param-file`. Jazzy also
  receives the description on the `robot_description` topic:
  `https://control.ros.org/jazzy/doc/ros2_control/doc/migration.html`
- JSON Schema Draft 2020-12 defines the structured-data schema vocabulary used
  for source artifacts:
  `https://json-schema.org/draft/2020-12`
