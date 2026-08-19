# Araco Hexapod — Working State

Updated: 2026-08-18
Machine: `stevw-s14-Stealth-14Studio-A13VF` (Ubuntu 24.04.4 LTS)
Location: these continuity files moved from `docs/agent/` to `.agent/` on
2026-08-18, committed as `6b23132`.

## Current goal and result

The immediate SLAM-drift correction is implemented. Repeated operator routes
04–08 were suspended and replaced with short synchronized trials. Those trials
isolated the simulator camera-IMU timestamp path as defective: over 90% of
timestamp-specific `camera_link -> base_link` lookups were unavailable, while
RGB-D-only tracking handled both body motion and gimbal motion without tracking
loss.

`gazebo_perception_v0` now uses six-DoF RGB-D visual odometry without IMU
fusion (`araco.navigation.rtabmap-rgbd-sim` `0.4.0`). The simulated IMU remains
published and recorded; it is excluded only from the operational estimator.
Exact dynamic-IMU, fixed-gimbal-IMU, and visual-only variants remain available
as diagnostic profiles. Ground truth remains observer-only.

The acceptance protocol is also corrected. It requires the first four route
waypoints, then final position and starting yaw held for two simulator seconds,
followed by ten seconds of tracking-healthy stable corrected pose. The arena
has a visible +X heading arrow. The scorer reports strict JSON and cannot finish
on position alone before heading or graph convergence.

This is a defensible correction, not yet a full-route acceptance pass. The next
operator trial is route 09 using a fresh database and score directory.

## Implemented ownership

- `araco_gazebo` owns `rgbd_validation_v0` `0.3.0`, including the visible +X
  heading arrow and the established landmark route.
- `araco_perception` owns registered 424 x 240, 15 Hz simulated RGB-D streams,
  the 100 Hz simulated camera IMU, and RTAB-Map RViz layout.
- `araco_navigation` owns the operational visual-only estimator `0.4.0` and
  three test-only estimator variants:
  - `rtabmap_rgbd_visual_only_sim_v0.yaml`
  - `rtabmap_rgbd_dynamic_gimbal_imu_sim_v0.yaml`
  - `rtabmap_rgbd_fixed_gimbal_imu_sim_v0.yaml`
- `araco_bringup` resolves the exact selected navigation artifact. Operational
  profile `gazebo_perception_v0` selects visual-only estimation. Three
  `gazebo_perception_diagnostic_*` profiles select the exact variants above.
- `araco_system_tests` owns `araco.system-tests.slam-acceptance` `0.4.0`, the
  corrected scorer, and `araco_slam_diagnose`. The diagnostic recorder
  publishes commands on a dedicated 50 Hz wall-time thread, records safety
  state/reason, and rejects stale-command or no-motion trials.

## Controlled diagnostic evidence

All error values compare the estimator's relative motion with simulator truth.
No ground-truth signal entered RTAB-Map.

- Visual-only stationary:
  `/tmp/araco_diag_visual_stationary_20260818_01/summary.json` — corrected
  translation `0.0000505 m`, yaw `0.0000882 rad`, zero tracking loss.
- Fixed-IMU stationary:
  `/tmp/araco_diag_fixed_stationary_20260818_01/summary.json` — corrected
  translation `0.0000418 m`, yaw `0.0000579 rad`, zero tracking loss.
- Visual-only translation:
  `/tmp/araco_diag_visual_translation_20260818_02/summary.json` — `0.58626 m`
  true travel, corrected translation error `0.08229 m`, yaw error
  `0.002286 rad`, zero tracking loss or source staleness.
- Fixed-IMU translation:
  `/tmp/araco_diag_fixed_translation_20260818_03/summary.json` — `0.58452 m`
  true travel, corrected translation error `0.10386 m`, yaw error
  `0.004548 rad`, zero tracking loss or source staleness.
- Visual-only body yaw:
  `/tmp/araco_diag_visual_body_yaw_20260818_01/summary.json` — `3.134 rad`
  maximum true yaw, corrected translation error `0.0640 m`, yaw error
  `0.02211 rad`, zero tracking loss or source staleness.
- Fixed-IMU body yaw:
  `/tmp/araco_diag_fixed_body_yaw_20260818_02/summary.json` — `3.141 rad`
  maximum true yaw, corrected translation error `0.05005 m`, yaw error
  `0.02184 rad`, zero tracking loss or source staleness.
- Visual-only gimbal yaw with stationary body:
  `/tmp/araco_diag_visual_gimbal_yaw_20260818_01/summary.json` — gimbal reached
  `0.28 rad`; estimator invented only `0.0000470 m` translation and
  `0.000399 rad` yaw, with zero tracking loss.
- Timestamped camera-IMU transform checks failed for about 90–94% of samples in
  every controlled trial (for example visual stationary `135/1475` successes,
  fixed stationary `122/1481`). The fixed variant masks this by accepting the
  latest transform; it does not repair timing or prove valid inertial fusion.

The earlier fixed translation trial
`/tmp/araco_diag_fixed_translation_20260818_02` is excluded from comparison
because it reused a graph after a large yaw trial. Early diagnostic runs that
triggered `SOURCE_STALE` are also invalid and are not acceptance evidence.

## Static validation

- All operational and diagnostic profiles composed successfully from installed
  artifacts. Operational behavior fingerprint:
  `32dd967509420327c135167abefa9b2dfc2f5ef0754c5727d2f37db12f7a7aa2`.
  This replaces
  `d7d55a9774692baf62ae4f57c1272f782f0b26e59fc612b97c16c5eeb668b03c`, which was
  reproduced exactly from the unmodified tree immediately before the
  2026-08-18 evidence-source repoint and remains the correct value for commit
  `f1e41af`. Fingerprints are recomputable from source and were not lost with
  the deleted `/tmp` evidence.
- `gz sdf -k src/araco_gazebo/worlds/rgbd_validation_v0.sdf`: valid.
- Focused navigation, profile-composition, and scorer tests pass.
- The earlier `386 tests, 0 errors, 0 failures, 23 skipped` result covered only
  `araco_navigation`, `araco_gazebo`, `araco_bringup`, and `araco_system_tests`.
  It never exercised `araco_description` or `araco_perception`.
- 2026-08-18 full-workspace run at commit `3bd9dc9`: `colcon build` succeeded;
  `colcon test` reported `424 tests, 0 errors, 4 failures, 26 skipped`. The
  four reported failures are two distinct tests, each counted twice across
  `Test.xml` and the xunit report. Both are described below.
- All Gazebo, RViz, bridge, RTAB-Map, and control sessions were closed after
  live diagnostics.

## Two open test failures — both fixed 2026-08-18

Both are resolved. `colcon build` succeeded and `colcon test` reports
`424 tests, 0 errors, 0 failures, 26 skipped`. Gate 6's independent
installed-space package-test phase reproduced the same totals.

1. `araco_description` —
   `test_gate0_description.py::test_resources_are_redistributable_hashed_and_reproducible`.
   Fixed by regenerating `meshes/presentation_exact/normalization_manifest.json`
   with `normalize_fusion_exact_visuals.py` using the arguments the test uses.
   The regenerated tree was diffed against the committed one before it was
   installed: all 49 STL outputs are byte-identical and only the manifest
   changed, by exactly two lines. **Two hashes were stale, not one.** The
   earlier note recorded only `canonical_model_sha256`; `nominal_pose_sha256`
   was stale for the same reason, because the 2026-08-18 repoint edited the
   evidence-source path in both `config/model/canonical_model_v1.yaml` and
   `config/poses/nominal_standing_reference_v0.yaml`.
   - `canonical_model_sha256`: `2286773314ded079...` -> `71dba5050f1f402b...`
   - `nominal_pose_sha256`: `76ba6ad316237ae1...` -> `2715b641788da5c8...`

2. `araco_perception` —
   `test_sensor_contract.py::test_rtabmap_rviz_layout_covers_2d_and_colored_3d_maps`.
   Fixed in the test by comparing `display['Topic']['Value']`, matching the Map
   assertion above it. The RViz layout was not changed and is correct.

Working tree contains only these two edits. Not committed; no commit was
authorized.

Note: `meshes/detailed/normalization_manifest.json` still embeds the stale
`nominal_pose_sha256`. No test covers that file, so it was left alone. It
should be regenerated or retired deliberately.

## Gate 0-6 rerun at 2026-08-18 (post-fix)

Run from installed space into `log/`, not `/tmp`. The mandated rerun found two
real scoring failures and one real teardown defect. One of the three, the Gate
5 failure, has since been root-caused and fixed. The Gate 0-5 precondition is
still not met and route 09 remains blocked. Gate 6 halted in preflight at
Gate 2 on the teardown defect.

| Gate | Result | Evidence directory | Run fingerprint |
| --- | --- | --- | --- |
| 0 | PASS | `log/gate_0_20260818_slam_regression` | ci `a7df64b2fdc9c476...` dev `7925dd73d7eaf0e1...` |
| 1 | FAIL (1 PASS of 6) | `log/gate_1_20260818_slam_regression`(PASS), `_02`,`_03`,`_04` | `a7df64b2fdc9c476...` |
| 2 | FAIL (0 of 2) | `log/gate_2_20260818_slam_regression`, `_02` | `a7df64b2fdc9c476...` |
| 3 | PASS | `log/gate_3_20260818_slam_regression` | `67d86b7cc029fa50...` |
| 4 | scoring PASS after GPU fix | `log/gate_4_20260819_nvidia_full_02` (8/8 cases); `_slam_regression`,`_02` pre-fix | `133e92276ea5fd07...` |
| 5 | FAIL (0 of 3) | `_01`,`_02` pre-fix; `log/gate_5_20260818_slam_regression_03` post-fix, 28/29 | `812d652eb82b56dd...` |
| 6 | FAIL | `log/gate_6_20260818_slam_regression` | scenario `c442bc2b70947fdd...` |

Gate 0 behavior fingerprint: `228c8ca49d0f146bf9e2d86e6c0f8b5e3fa62d9dd151a733960c929dffacc3bb`.
This replaces `4f5d37e91c937543fae18dc76793b57eb58adabacba3c72eba91fd1677f14dc8`
from the 2026-08-16 `gate_0_20260816_phase5_final` run. Gates 1-5 all report the
same behavior fingerprint as Gate 0. Input-selection fingerprints for Gate 0:
ci `85e8fba289fb4364...`, development `5d92d70b107db8ca...`.

The Gate 0 `source_revision` is `unreported-dirty-or-installed-tree` because the
two test fixes are uncommitted.

### Gate 6 outcome

Gate 6 ran to completion with `--build-base build --install-base install` and
FAILED. Its own stages that passed are worth keeping:

- `package_tests` and `package_test_results`: PASS. Gate 6 independently
  reproduced `424 tests, 0 errors, 0 failures, 26 skipped` from installed space,
  confirming the two test fixes.
- `sanitizers` and `no_sanitizer_diagnostic`: PASS. The sanitizer build and
  suite produced no diagnostic.
- `no_lifecycle_deadlock`: PASS.

Failed checks: `gates_0_through_5_preflight`,
`three_complete_no_retry_repetitions`, `suite_wall_budget`,
`no_unclassified_error_or_fatal`.

**Gate 6 halted in preflight at Gate 2, so it never reached Gates 3-5 and never
exercised Regressions A or B.** Preflight Gate 0 PASS, Gate 1 PASS, Gate 2 FAIL
on `launch_exit` and `launch_log_clean` only. The proximate cause of the Gate 6
failure is therefore Defect C, the intermittent teardown deadlock, not the two
scoring regressions. Both still have to be fixed.

`automatic_retry_performed` is `false`; the runner does not retry a required
attempt. Environment captured: Gazebo `8.11.0`, ROS 2 Jazzy,
`rmw_fastrtps_cpp`, git revision `0098cbb` with the three worktree changes
listed.

### Regression A — Gate 4 yaw JOINT_STATE_STALE — FIXED 2026-08-19 (environment)

**Root cause was the graphics stack, not the gait and not the watchdog.**
Gazebo was rendering the robot's cameras in **software**. This machine has an
Intel Iris Xe iGPU and an NVIDIA RTX 4060 (driver 595.84, working), with both
`10_nvidia.json` and `50_mesa.json` EGL vendors installed. Headless gz picked
Mesa, failed with `libEGL warning: egl: failed to create dri2 screen`, and fell
back to llvmpipe. The NVIDIA GPU sat at 0% and 15 MiB during gate runs.

Software rendering of the 15 Hz `gemini_rgbd` and `gemini_color` cameras pushed
the real-time factor down to ~0.48 and produced wall-clock scheduling gaps of
115-138 ms in `/joint_states`. The safety supervisor's wall-clock
`joint_state_timeout_s` of 0.1 s then tripped `REASON_JOINT_STATE_STALE` (18),
latching `FAULT_HOLD`, aborting `yaw_left`, and leaving `yaw_right` and
`combined` unable to acquire authority.

Fix — per-process environment only, **no source or config change**:

```bash
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```

Measured effect on Gate 4:

| | before | after |
| --- | --- | --- |
| NVIDIA GPU use | 0%, 15 MiB | up to 55%, 736 MiB |
| `dri2 screen` errors | 2 | 0 |
| `reason=18` faults | 2 | 0 |
| RTF (yaw case alone) | 0.484 | 0.887 |
| RTF (full 8-case matrix) | 0.693 | 0.731 |
| scorer | FAIL | **PASS, all 8 cases** |

`log/gate_4_20260819_nvidia_full_02` — `precision_forward`, `forward`,
`reverse`, `left`, `right`, `yaw_left`, `yaw_right`, `combined` all PASS.

**Evidence integrity: the run fingerprint is unchanged at
`133e92276ea5fd07...` and the behavior fingerprint at `228c8ca49d0f146b...`,
identical to the failing 2026-08-18 runs.** Same configuration, different
runtime environment. The earlier gate results were therefore valid evidence of
a real environmental defect, not of a configuration difference.

Gate 4 still reports FAIL overall, but now only on `launch_log_clean`, caused by
the Defect C shutdown `SIGSEGV` (exit code 139). Every scored check passes.

Withdrawn along the way: the earlier "tripod yaw loses support / suspect the
`legacy_rotation_scale` blend" theory, and the follow-on theory that the
watchdog itself needed relaxing. **No safety threshold was changed and no gait
code was touched.** The offline gait replay and the isolated-case run that
disproved the gait theory remain valid.

One `REASON_CONTROLLER_NOT_READY` (20) startup abort was seen in the first full
GPU run and did not reproduce on rerun. Treat as a startup flake unless it
recurs.

**Made durable 2026-08-19 in `~/.bashrc`** (operator's choice), as a guarded
block that no-ops when `/usr/share/glvnd/egl_vendor.d/10_nvidia.json` is absent
and never overrides an already-set value. Repository source was deliberately
left unchanged, because hardcoding NVIDIA selection into
`araco_bringup/launch/gazebo.launch.py` would be wrong on machines without an
NVIDIA EGL vendor.

Verified end to end with no manual exports, relying only on `~/.bashrc`:
`log/gate_4_20260819_bashrc_verify` — `yaw_left` PASS, 7 cycles, RTF 0.836,
zero `reason=18`, zero `dri2 screen` errors.

Caveat: `~/.bashrc` only applies to interactive shells. Gate runs started from a
desktop launcher, a cron job, or CI will **not** inherit these variables and will
silently fall back to software rendering. If gates ever need to run
non-interactively, this must be solved again at that layer.

### Gate 6 outcome

Gate 6 ran to completion with `--build-base build --install-base install` and
FAILED. Its own stages that passed are worth keeping:

- `package_tests` and `package_test_results`: PASS. Gate 6 independently
  reproduced `424 tests, 0 errors, 0 failures, 26 skipped` from installed space,
  confirming the two test fixes.
- `sanitizers` and `no_sanitizer_diagnostic`: PASS. The sanitizer build and
  suite produced no diagnostic.
- `no_lifecycle_deadlock`: PASS.

Failed checks: `gates_0_through_5_preflight`,
`three_complete_no_retry_repetitions`, `suite_wall_budget`,
`no_unclassified_error_or_fatal`.

**Gate 6 halted in preflight at Gate 2, so it never reached Gates 3-5 and never
exercised Regressions A or B.** Preflight Gate 0 PASS, Gate 1 PASS, Gate 2 FAIL
on `launch_exit` and `launch_log_clean` only. The proximate cause of the Gate 6
failure is therefore Defect C, the intermittent teardown deadlock, not the two
scoring regressions. Both still have to be fixed.

`automatic_retry_performed` is `false`; the runner does not retry a required
attempt. Environment captured: Gazebo `8.11.0`, ROS 2 Jazzy,
`rmw_fastrtps_cpp`, git revision `0098cbb` with the three worktree changes
listed.

### Regression A — Gate 4 yaw case aborted by JOINT_STATE_STALE (blocking)

**Corrected characterization, 2026-08-19.** This was first recorded as "tripod
yaw loses support", suspected in the `legacy_rotation_scale` /
`rotation_warm_start` blend added by `f1e41af`. **The gait is not defective and
that lead is withdrawn.** Two independent checks:

- An offline driver linking the real `tripod_gait.cpp` with the real
  `tripod_slow_sim_v0` values (`base_cadence 1.0`, `maximum_stride_m 0.06`,
  `swing_clearance_m 0.03`, `planar_command_scale 0.05`,
  `yaw_command_scale 0.3`) replayed a pure 0.2 rad/s yaw for 400 steps:
  **0 non-monotonic phase events, 0 lift-without-swing-flag events.** The
  startup handover and the `legacy_curve_phase` swing/lift alignment are
  self-consistent.
- Running `yaw_left` **alone** (`--case-name yaw_left`) reports
  `minimum_support_contacts: 3`. The support loss reported in the full 8-case
  run did not reproduce, so it is a downstream artifact, not the cause.

What actually happens: the `yaw_left` case is aborted by a safety fault
`REASON_JOINT_STATE_STALE` (18), seen twice in every Gate 4 run including the
isolated one. The supervisor latches `FAULT_HOLD`, which is why `yaw_right` and
`combined` then report `authority acquisition failed`. `gait_cycles_completed`
is 4 instead of 7 because the case is cut short.

Mechanism: `joint_state_timeout_s` is `0.1` and is evaluated by
`fresh(joint_receipt_, joint_timeout)` in `safety_supervisor_node.cpp`, where
`joint_receipt_` is a `SteadyClock` (wall) stamp. The `/joint_states` stream is
paced by **simulation** time, so its wall-clock rate scales with the real-time
factor. Measured during the yaw case on a pinned domain (1605 samples):

| metric | value |
| --- | --- |
| mean gap | 17.5 ms |
| p99 gap | 109 ms |
| max gap | 137.6 ms |
| gaps over the 100 ms budget | 38 |

These are marginal overruns of roughly 8x nominal jitter, **not** a stall. RTF
measured 0.484 (yaw alone) to 0.697 (full matrix), and headless runs log
`libEGL warning: egl: failed to create dri2 screen`, indicating a **software**
GL fallback.

Attribution is partly open. The watchdog values were **not** changed by
`f1e41af` (`watchdogs_s.joint_state: 0.1` predates it; that commit added only
`startup_readiness_stable_s`). `f1e41af` did add an always-on 15 Hz
`gemini_rgbd` camera, a 15 Hz `gemini_color` camera and a 100 Hz IMU to the
robot, and added the `araco.perception.gemini-335-sim` artifact to the **Gate 4
profile** — a locomotion gate that does not need rendering. `gemini_rgbd` first
appears in `f1e41af`.

**That added-cost hypothesis is plausible but UNPROVEN.** An attempt to A/B it
by removing the sensor from `gazebo_gate4_v0.yaml` failed to test anything: the
composer requires the artifact (`expected exactly one selected
simulated_rgbd_imu artifact`), so the runtime bundle was never emitted and the
run produced no usable timing data. The profile was restored and verified by
recomposition; the run fingerprint is back to `133e92276ea5fd07...`.

No fix applied. Every available option is a design or safety decision:

1. Make the simulator-facing staleness check sim-clock based, or scale it by
   RTF. Note the current use of steady clock looks deliberate: it lets a
   stalled sim clock be detected, and a separate `clock_progress` watchdog
   exists. On real hardware wall time is the correct basis.
2. Relax `watchdogs_s.joint_state` in the simulator-only safety policy
   (`araco_supervision/config/policy/simulator_v0.yaml`, `simulator_only`
   scope) with a version bump. This weakens a safety threshold and must be an
   explicit owner decision.
3. Restore hardware GL so rendering stops falling back to software and RTF
   recovers. Safety-neutral and environmental; the EGL dri2 failure is the
   concrete lead. Try this first.
4. Add a sensor-artifact variant with the camera disabled for the locomotion
   gates, plus composer support for it. This is the only option that removes
   the cost rather than accommodating it, and it is the largest change.

### Gate 6 outcome

Gate 6 ran to completion with `--build-base build --install-base install` and
FAILED. Its own stages that passed are worth keeping:

- `package_tests` and `package_test_results`: PASS. Gate 6 independently
  reproduced `424 tests, 0 errors, 0 failures, 26 skipped` from installed space,
  confirming the two test fixes.
- `sanitizers` and `no_sanitizer_diagnostic`: PASS. The sanitizer build and
  suite produced no diagnostic.
- `no_lifecycle_deadlock`: PASS.

Failed checks: `gates_0_through_5_preflight`,
`three_complete_no_retry_repetitions`, `suite_wall_budget`,
`no_unclassified_error_or_fatal`.

**Gate 6 halted in preflight at Gate 2, so it never reached Gates 3-5 and never
exercised Regressions A or B.** Preflight Gate 0 PASS, Gate 1 PASS, Gate 2 FAIL
on `launch_exit` and `launch_log_clean` only. The proximate cause of the Gate 6
failure is therefore Defect C, the intermittent teardown deadlock, not the two
scoring regressions. Both still have to be fixed.

`automatic_retry_performed` is `false`; the runner does not retry a required
attempt. Environment captured: Gazebo `8.11.0`, ROS 2 Jazzy,
`rmw_fastrtps_cpp`, git revision `0098cbb` with the three worktree changes
listed.

### Regression A — Gate 4 tripod yaw loses support (blocking)

Reproduced identically twice. The five linear cases
(`precision_forward`, `forward`, `reverse`, `left`, `right`) all PASS. Then:

- `yaw_left` FAIL: `minimum_support_contacts` reaches `0` while three swing legs
  are touching, `phase_monotonic` is `false`, `controlled_stop_seen` is `false`,
  and the scheduler reports `policy_valid: false` with `cadence_hz_min 0.0`,
  `stride_scale_min 0.0`, `velocity_scale_min 0.0`.
- `yaw_right` and `combined` then FAIL with `authority acquisition failed`,
  which looks like a cascade from the `yaw_left` fault rather than an
  independent defect.
- Failed scorer checks: `active_stand`, `all_direction_cases`, `manual_hold`.

Suspect `f1e41af`, which rewrote `tripod_gait.cpp` to the
`tripod_legacy_translation_rotation_blend_responsive_scheduler` with a separate
`legacy_rotation_scale` curve and a `rotation_warm_start` special case. The
gait config itself changed only `gait_id`, version, and evidence, so the
behavior change is in the C++ blend, not the artifact.

### Regression B — Gate 5 never reached enabled motion — FIXED 2026-08-18

**Corrected attribution.** This was first recorded as a suspected `f1e41af`
supervision regression (`gimbal_yaw_rad` intent contract,
`startup_readiness_stable_s` dwell). **That was wrong.** Investigating the
`gimbal_yaw_hard_rad` lead disproved it: the arbiter deliberately passes hard
limits into both the normal and hard envelope slots for every field, so the
apparent duplicate `gimbal_yaw_hard_rad, gimbal_yaw_hard_rad` pair in
`command_arbiter_node.cpp` is the intended pattern, not a defect.

Actual root cause: `src/araco_system_tests/scripts/araco_joint_state_relay`
lost its execute bit. Git recorded it as `100644` while all nine sibling
scripts are `100755`. Gate 5 is the only gate that starts the relay, and the
workspace uses `--symlink-install`, so the installed path is a symlink to the
non-executable source and `ros2 run` returned `No executable found`. The relay
never ran, joint-state readiness never set, readiness stalled at `91/127` with
`joints=0`, and the supervisor entered `FAULT_HOLD reason=19 fault_mask=8`
before ever reaching HOLDING.

Introduced by `9e0284b` (2026-08-17), **not** `f1e41af`. Gate 5 last passed
2026-08-16, the bit was dropped 2026-08-17, and gates were not rerun until
2026-08-18, so this sat undetected. The evidence that would have caught it went
to `/tmp` and was lost.

Fix applied: `chmod +x` plus `git update-index --chmod=+x` on that script. No
rebuild was needed because of symlink-install.

Result after the fix, `log/gate_5_20260818_slam_regression_03`: 28 of 29 scorer
checks pass, `fault_matrix_complete` true, `orderly_safety_shutdown` true,
`startup_failure` null. Gate 5's whole fault matrix now runs.

The single remaining Gate 5 scorer failure is
`backend_process_loss_quiesces_runtime`, and it **is Defect C, not a separate
defect**. That final scenario issues the same
`gz service -s /server_control --req 'stop: true'` and then requires the
runtime to quiesce within 2 s. While the server refuses to exit, the safe and
locomotion streams keep publishing and the check cannot pass.

Lesson worth keeping: a lost file mode is invisible to `colcon test` and to
code review of diffs. Consider a metadata test asserting every file in
`scripts/` is mode `100755`.

### Defect C — Gazebo shutdown race, upstream in gz-sim 8.11.0 (top blocker)

**Corrected characterization, 2026-08-18.** An earlier entry in this file
claimed the orderly safety shutdown triggers this, "isolated with a controlled
pair of runs". **That claim is withdrawn.** It rested on one run per arm, and
the failure is intermittent, so it was a coincidence. A 3-vs-3 repetition on
`gazebo_ci_v0` shows no effect from the safety shutdown:

| | exited | wedged |
| --- | --- | --- |
| with orderly safety shutdown | 2 | 1 |
| without | 1 | 2 |

Gate 3 also sends the orderly shutdown and tears down cleanly, which is
consistent with the corrected reading. Do not look for this in the
safety/arbitration path.

What actually happens: on `gz service -s /server_control --req 'stop: true'`
the server reaches one of three outcomes, non-deterministically — clean exit,
`SIGSEGV`, or hang.

- **Hang:** all 56 threads sleeping, 40 in `futex_do_wait`. No progress, not a
  busy loop. The gz log stops right after `Successfully switched controllers!`
  and never reaches the `controller_manager.pal_statistics` teardown lines that
  a clean shutdown emits.
- **SIGSEGV:** null dereference (`Address not mapped to object (nil)`) on a
  gz-sim worker thread. Post-mortem backtrace from the apport core of the Gate
  5 run (`gdb` on a core needs no `ptrace_scope` change):
  `clone3` -> `start_thread` -> libstdc++ thread trampoline -> two frames inside
  `libgz-sim8.so.8`. So it is a gz-sim internal thread faulting during teardown.
  Exact function not nameable: `libgz-sim8` is stripped and the frames are local
  symbols, the nearest exported symbols being 5864 and 273 bytes away and
  therefore meaningless. Naming it needs gz-sim debug symbols.

Scope, established by elimination — each arm run three times:

| Configuration | Result |
| --- | --- |
| stock Gazebo world (`shapes.sdf`), no Araco plugins | clean 3/3 |
| our `resolved_world.sdf` alone, no robot, no ROS nodes | clean 3/3 |
| full launch: robot spawned with `gz_ros2_control` + controllers | fails ~50% |

So the world's own plugin set (contact, imu, physics, scene-broadcaster,
sensors, user-commands) is **not** sufficient. The trigger needs the robot model
with `gz_ros2_control`, which hosts the `controller_manager` inside the server
process.

**Conclusion: this is an upstream shutdown race in gz-sim 8.11.0 exercised via
`gz_ros2_control` 1.2.19, not a defect in Araco code.** No Araco source change
will fix it directly. Both failure modes occur strictly after all scored
behavior has completed and after `metrics.json` is written, which is why Gates
1 and 2 pass every scored check and fail only `launch_exit` and
`launch_log_clean`.

Options, none of which should be chosen silently:

1. Install gz-sim debug symbols, symbolize the core, and file upstream.
2. Upgrade or patch Gazebo.
3. Change the gate contract so "server failed to exit cleanly" is recorded as a
   distinct, explicitly tracked upstream condition instead of being conflated
   with `launch_log_clean`. This is now defensible because the defect is
   upstream and post-scoring, but it is a contract change and is the operator's
   decision, not the agent's. It must not be done by quietly enlarging the
   runner's 5 s wait, which would hide a real crash.

Operational note: the wrapper matched by `pgrep -f resolved_world.sdf` is
`/bin/sh`, not the server; the real server is its child. Do not use `pkill -f`
with a pattern that also matches your own shell command line.

## Remaining risks

- A complete route has not yet passed with the corrected estimator and scorer.
- Visual odometry still accumulated roughly 8.2 cm translation error over a
  controlled 58.6 cm translation and roughly 6.4 cm apparent translation
  during a large body-yaw trial. Loop closure must correct this within the
  acceptance bounds on route 09.
- The camera IMU is not qualified for operational fusion until a timestamped
  transform/preprocessing path is implemented and retested. The physical
  Gemini driver, calibration, latency, and gimbal-angle feedback also remain
  unimplemented.
- Saved-database relocalization and Nav2 remain blocked on a clean route pass.
- No commit or push is authorized by the current request.

## Exact next step

Route 09 remains blocked. The mandated Gate 0-6 rerun has been performed. It
found three real defects, one of which (Gate 5) is now fixed. **Two remain:
Regression A and Defect C.** Regression A is now fixed by using the discrete
GPU (environment only, no code change), so **Defect C is the only remaining
blocker**. It is an upstream Gazebo shutdown crash/hang and is now the sole
cause of the Gate 1, 2, 4, 5, and 6 failures. It needs an operator decision.

Required order:

1. Done 2026-08-18. Both test failures fixed; `colcon build` clean and
   `colcon test` reports `424 tests, 0 errors, 0 failures, 26 skipped`,
   independently reproduced by Gate 6's package-test stage.
2. Done 2026-08-18. Gates 0-6 run into `log/`. Results, fingerprints, and the
   three defects are recorded above. Gates 0 and 3 pass. Gates 1 and 2 pass
   every scored check but fail teardown. Gates 4 and 6 fail; Gate 5 now fails
   only on Defect C.
3. Done 2026-08-19. Regression A was root-caused to software GL rendering and
   fixed by selecting the NVIDIA EGL vendor. Gate 4 scoring now passes all
   eight cases. Decide how to make those environment variables durable.
4. Done 2026-08-18. Regression B was root-caused to a lost execute bit on
   `araco_joint_state_relay` and fixed. Gate 5 now passes 28 of 29 scorer
   checks; its last failure is Defect C. The supervision-change hypothesis was
   disproved, not merely superseded.
5. **Decide how to handle Defect C — the top blocker.** It alone accounts for
   the Gate 1, Gate 2, and Gate 6 failures and for Gate 5's single remaining
   scorer failure. It is now established as an upstream gz-sim 8.11.0 shutdown
   race requiring `gz_ros2_control`, not an Araco defect and not related to the
   safety shutdown, so there is no Araco fix to write. Choose between the three
   options recorded in the Defect C section. This needs an operator decision
   before Gates 0-6 can go green.
6. Rerun Gates 0-6 clean, then record fingerprints again.
7. Only then run route 09.

Reproduction for Defect C, which does not need the gate harness:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch araco_bringup gazebo.launch.py profile:=gazebo_ci_v0
```

Once holding, send the orderly shutdown and then the server stop. With the
shutdown first, the server never exits; without it, the server exits in under
a second.

```bash
ros2 action send_goal /araco/safety/transition \
  araco_interfaces/action/SafetyTransition "{request: 4}"
gz service -s /server_control --reqtype gz.msgs.ServerControl \
  --reptype gz.msgs.Boolean --timeout 3000 --req 'stop: true'
```

Route 09 procedure, once the gates above pass:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch araco_bringup gazebo.launch.py \
  profile:=gazebo_perception_v0 \
  database_path:=/tmp/araco_rgbd_acceptance_09.db
```

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run araco_system_tests araco_slam_score \
  /tmp/araco_slam_acceptance_score_09
```

Drive east/red, north/blue, west/green, south/yellow, then origin/white. At the
origin, align with the +X floor arrow and keep the robot stationary until the
scorer reports convergence. Do not move the gimbal during this first acceptance
route, even though the isolated visual-only gimbal trial was clean.
