# Araco Hexapod — Final Architecture Consistency Review

Status: **COMPLETE — PASS FOR EXPLICIT PHASE 0 AUTHORIZATION**

Review date: 2026-08-15

License revision date: 2026-08-16

Scope: the accepted simulator-first repository, package, interface, safety,
configuration, runtime, robot-description, testing, and delivery contracts.
This review selects the repository/package license and closes architecture
design before scaffolding. It does not itself authorize Phase 0, create ROS
packages, publish the repository, configure hosted CI, or command hardware.

## Verdict

After the corrections recorded below, the accepted contracts form one
implementable, acyclic, fail-closed simulator architecture. There is one
canonical 26-primary-link/25-revolute-joint model, one 24+1 controller
partition, one high-level command authority pipeline, one configuration owner
per behavior value, seven ordered Gazebo gates, and one phase before those
gates for repository foundations.

No unresolved architecture contradiction blocks Phase 0. Remaining uncertainty
is evidence- or implementation-scoped and is assigned to an explicit gate.
Physical deployment remains blocked independently of simulator progress.

## Authorities reviewed

| Concern | Normative authority | Result |
|---|---|---|
| Package/node ownership and dependency direction | `REPOSITORY_ARCHITECTURE.md` | Consistent; nine initial packages and no inward backend dependency |
| CAD/model evidence and canonical topology | `ROBOT_DESCRIPTION_MANIFEST.md` | Accepted for simulator authoring; physical zero/limit/calibration claims remain blocked |
| Command, feedback, and controller interfaces | `INTERFACE_CONTRACTS.md` | Consistent four-stage authority path and truthful 25-joint provenance |
| Safety, handover, lifecycle, and watchdog behavior | `SAFETY_ARCHITECTURE.md` | Consistent after startup-bootstrap and watchdog-arming clarification |
| Configuration ownership and gate semantics | `CONFIGURATION_AND_VALIDATION_ARCHITECTURE.md` | Consistent; no duplicate source of truth |
| Topics, rates, QoS, simulator values, and thresholds | `RUNTIME_TIMING_AND_SIMULATION_CONTRACT.md` | Consistent 50/100/250/1000 Hz hierarchy and Gates 0–6 |
| Artifact paths, profiles, preflight, and fingerprints | `PARAMETER_AND_CONFIGURATION_COMPOSITION.md` | Consistent after profile/input-selection clarification |
| Implementation order, evidence, and regression | `PHASED_DELIVERY_PLAN.md` | Consistent Phase 0 plus one phase per Gate 0–6 |

## Reconciliations made by this review

1. **Configuration authority.** Source IDs, priorities, and freshness timeouts
   are owned by the `araco_supervision` source-registry artifact. Bringup only
   selects and composes it.
2. **Profile identity and equivalence.** All documents now use
   `gazebo_dev_v0` and `gazebo_ci_v0`. Both use seed `42`, the same production
   behavior artifacts, physics, and source registry. The development keyboard
   adapter may be absent in scored CI, but that closed input choice is captured
   by the input-selection/run fingerprints rather than hidden as behavior.
3. **Startup circularity.** Locomotion may bootstrap only the validated nominal
   hold before the first safe command; it cannot advance gait. Trusted-stream
   watchdogs arm when their producer becomes expected or after the first valid
   sample, so the deliberately inactive arbiter is not misclassified as failed
   during ordered startup.
4. **Orderly shutdown.** Expected readiness loss caused by commanded
   deactivation remains in `SHUTTING_DOWN`; unexpected early loss still faults.
5. **Gimbal ownership.** The v0 gimbal JTC holds its validated initialized zero
   state. Safety and locomotion do not publish an undocumented periodic gimbal
   command, and the leg JTC timeout remains independent.
6. **Backend readiness.** Backend readiness is a composite of the configured
   `ros2_control` hardware-component identity/state and fresh typed
   clock/joint/controller evidence. No circular project health topic is added.
7. **Robot-description status.** The Fusion reconciliation is accepted as
   simulator-authoring evidence. Physical sign/zero/limit/calibration and
   high-fidelity inertia questions are physical/fidelity deferrals, not Phase 0
   blockers.
8. **Asset redistribution.** Detailed vendor CAD in the Fusion archive is not
   assumed redistributable. Bundled assets require explicit provenance and
   redistribution metadata; unknown-rights geometry is excluded or replaced
   by project-authored simplified proxies.
9. **Trust boundary.** The simulator's trusted transition client is a closed
   composition assumption, not cryptographic authorization. A physical or
   untrusted-network profile must add validated ROS security or equivalent
   access control.

## License selection

At the user's request on 2026-08-16, the selected license for project-authored
repository and package content is the **GNU General Public License version 3
only**, SPDX identifier **`GPL-3.0-only`**. This supersedes the 2026-08-15
Apache-2.0 selection before either license was applied through a root `LICENSE`
or ROS package manifest.

`GPL-3.0-only` means version 3 specifically; it does not automatically grant
the option to use a future GPL version. It is a strong-copyleft license:
distribution of covered modified or combined works must preserve GPLv3 terms,
and object-code distribution carries the applicable Corresponding Source and
installation-information obligations in the license. Private modification and
use without distribution do not trigger a source-publication requirement.

The separately authorized pre-Phase-0 checkpoint applies the unmodified
official GPLv3 text at root `LICENSE`. Phase 0 completes application as follows:

- place an exact full-text `LICENSE` copy in each of the nine ROS packages so a
  separately distributed package remains self-contained;
- use `<license file="LICENSE">GPL-3.0-only</license>` in each initial
  `package.xml`;
- add `SPDX-License-Identifier: GPL-3.0-only` to project-authored source files
  whose format supports comments;
- audit linked and bundled dependency licenses for GPLv3 compatibility;
- identify the preferred editable source and generation tooling for covered
  generated binaries/meshes that may be distributed;
- review the existing Fusion add-in's Autodesk API boundary, and later the
  Isaac adapter's proprietary SDK boundary, before distributing those
  integrations as GPL-covered combined works;
- create `THIRD_PARTY_NOTICES.md` only when bundled attribution requires it,
  preserving any exact third-party notice-file requirement.

GPL-3.0-only covers only material the copyright holder has the right to license.
It does not relicense ROS/Gazebo dependencies, vendor CAD, product logos, or
other third-party assets. Any bundled work under another license retains that
license and required notices, and every package containing it must declare the
additional license as required. This is an engineering license selection, not
legal advice.

Primary references checked on 2026-08-16:

- `https://www.gnu.org/licenses/gpl-3.0.txt`
- `https://www.gnu.org/licenses/gpl.en.html`
- `https://spdx.org/licenses/GPL-3.0-only.html`
- `https://github.com/ros-infrastructure/rep/blob/master/rep-0149.rst`
- `https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/customizing-your-repository/licensing-a-repository`

## Gate-assigned uncertainties

| Uncertainty | Earliest resolution | Phase 0 blocker? | Hardware blocker? |
|---|---|---:|---:|
| Expanded mount rotations and canonical simulator sign checks | Gate 0 model validation | No | Physical sign validation still yes |
| Provisional simulator ranges/caps | Gate 0 artifact and nominal-margin validation | No | Yes; cannot enter a physical profile |
| Proxy poses and positive-valid aggregate base inertia | Gate 0 description authoring | No | Physical fidelity still yes |
| Mesh/model rights and GPL Corresponding Source | Before a resource is bundled in Phase 1/Gate 0 | No | Public distribution blocker for that asset |
| Proprietary Fusion/Isaac SDK compatibility | Phase 0 for the existing exporter; future Isaac phase for its adapter | License audit only | Distribution blocker if incompatible |
| Gait/IK/controller behavior | Gates 1–4 as assigned | No | Simulator evidence alone never clears hardware |
| Full fault/restart/handover behavior | Gate 5 | No | Physical safety design remains separate |
| Repeatability and operational evidence | Gate 6 | No | Does not establish sim-to-real fidelity |
| Pi OS, servo transport, power-off/collapse, local stop, measured state | Later physical phase | No | Yes |

## Exact boundary of a future Phase 0 authorization

An explicit Phase 0 authorization permits only:

- creating `src/` and the nine accepted package skeletons;
- adding package manifests, build metadata, install/export rules, package-local
  test structure, and the already accepted IDL;
- preserving root `LICENSE`, adding package-local GPL-3.0-only license copies,
  and adding project source headers;
- adding `.gitignore` and replacing the existing minimal `README.md` stub with
  build-focused repository information;
- configuring warnings/lint and proving clean rosdep, build, test, install-space
  discovery, IDL generation, and acyclic dependencies;
- preserving all current documentation and Fusion evidence.

It does **not** permit canonical model/Xacro/mesh authoring, runtime YAML or
schemas, Gazebo launch/world implementation, control/safety/locomotion
executables, physical profiles, servo/UART work, hardware commands, commits,
pushes, releases, hosted CI changes, or publication. Phase 0 claims no Gazebo
gate and creates no runnable robot.

Before writing public `package.xml` metadata, the maintainer name and email must
be confirmed as values the user is willing to publish. This is a metadata input,
not an architecture decision.

## Next authorization point

The architecture-closeout condition is satisfied. Work remains stopped at the
Phase 0 boundary until the user explicitly authorizes the exact scope above.
