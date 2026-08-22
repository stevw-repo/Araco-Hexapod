# Araco Hexapod — Working State

Updated: 2026-08-22
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
from the 2026-08-16 `gate_0_20260816_phase5_final` run. Gates 1-4 report the
same behavior fingerprint as Gate 0. **Gate 5 does not, and never did:** it runs
`gazebo_gate5_v0` and reports `92fc5afca01d5783...`. The 2026-08-18 and
2026-08-22 Gate 5 runs agree exactly on that value, so it is stable and
correct. The earlier blanket "Gates 1-5" claim here was wrong. Input-selection fingerprints for Gate 0:
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

### Defect C — Gazebo shutdown race, upstream in gz-sim 8.11.0

**Status 2026-08-20: option 3 taken, committed as `8865e3c`. The defect is
now recorded as a named non-blocking upstream condition rather than counted
as a gate failure. It is not fixed, and options 1 and 2 remain open.** The
full decision, guards, and accepted limitations are in `DECISIONS.md` under
"Record the gz-sim teardown failure as a named non-blocking upstream
defect". The characterization below stands unchanged and is what justified
that choice.

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
   **Still open.**
2. Upgrade or patch Gazebo. **Still open.**
3. Change the gate contract so "server failed to exit cleanly" is recorded as a
   distinct, explicitly tracked upstream condition instead of being conflated
   with `launch_log_clean`. This is now defensible because the defect is
   upstream and post-scoring, but it is a contract change and is the operator's
   decision, not the agent's. It must not be done by quietly enlarging the
   runner's 5 s wait, which would hide a real crash.
   **Taken 2026-08-20 as `8865e3c`.** The 5 s wait was left alone, as
   required. Gates 1-5 share `araco_gate1_evidence`, so one change covers
   all five; Gate 6 classifies the same signatures in its log scan.
   Attribution requires that scoring completed and that a stop was
   requested, only `[gazebo-1]` may be excused for a crash signal, and
   group-signal deaths count only when the runner actually escalated.
   Every attributed line is written to evidence under
   `upstream_defects.gz_sim_shutdown`, so a PASS now means "scored clean,
   any unclean exit matched the recorded signature" rather than "the
   simulator exited cleanly". Covered by six unit tests; **not yet
   exercised by a gate run.**

Operational note: the wrapper matched by `pgrep -f resolved_world.sdf` is
`/bin/sh`, not the server; the real server is its child. Do not use `pkill -f`
with a pattern that also matches your own shell command line.

## Gate 0-5 rerun at 2026-08-22 — all six PASS

First all-green Gate 0-5 result, and the first live exercise of the `8865e3c`
shutdown-defect classification. Run from installed space into `log/`, tag
`20260822_defectc_class`.

| Gate | Result | Run fingerprint | Behavior fingerprint | Defect C variant seen |
| --- | --- | --- | --- | --- |
| 0 | PASS | ci `a7df64b2fdc9c476...` dev `7925dd73d7eaf0e1...` | `228c8ca49d0f146b...` | n/a, no simulator |
| 1 | PASS | `a7df64b2fdc9c476...` | `228c8ca49d0f146b...` | SIGSEGV + escalation |
| 2 | PASS | `a7df64b2fdc9c476...` | `228c8ca49d0f146b...` | SIGSEGV, clean launch exit |
| 3 | PASS | `67d86b7cc029fa50...` | `228c8ca49d0f146b...` | hang |
| 4 | PASS, 8/8 cases | `133e92276ea5fd07...` | `228c8ca49d0f146b...` | hang |
| 5 | PASS, 29/29 checks | `812d652eb82b56dd...` | `92fc5afca01d5783...` | SIGSEGV, clean launch exit |

**Every run fingerprint reproduces its previously recorded value exactly.** Gate
0's input-selection fingerprints also reproduce: ci `85e8fba289fb4364...`,
development `5d92d70b107db8ca...`. Gate 5 reported 29/29 and Gate 4 all eight
cases; nothing else moved.

**Caveat added later the same day: Gate 5's 29/29 was variant-dependent.** It
passed because that run drew the crash form of Defect C. Later runs of the same
gate returned 28/29 whenever the hang form occurred. See "Gate 5's last scorer
check is decided by which Defect C variant occurs". This table records what the
run produced; it is not a claim that Gate 5 passes reliably.

`source_revision` is again `unreported-dirty-or-installed-tree`, because the
2026-08-22 documentation edits are uncommitted. The identical fingerprints
confirm documentation content is not part of artifact identity.

### The classification behaved correctly on all three variants

- **SIGSEGV with escalation** (Gate 1): `[gazebo-1] Segmentation fault`, launch
  return code `-15`, `escalated: true`, six lines attributed — the gazebo crash
  plus five nodes killed by the group signal.
- **SIGSEGV with a clean launch exit** (Gates 2 and 5): return code `0`, no
  escalation, three lines, all `[gazebo-1]`.
- **Hang** (Gates 3 and 4): return code `-15`, `escalated: true`, five lines,
  **no `[gazebo-1]` line at all**. A deadlocked server logs nothing, so
  attribution rests entirely on the escalation rule. This is the weakest of the
  three: any other cause of launch failing to exit within 5 s would present
  identically. It is consistent with the design but should be remembered when
  reading a Gate result that reports the hang variant.

No gate reported an unclassified error line, and no gate was excused a failure
it should have kept: Gates 4 and 5 failed loudly on scored checks in the
contaminated attempt described below, exactly as the guards intend.

### Defect C leaves an orphaned server that poisons the next gate

**Found 2026-08-22 and not previously recorded.** When the hang variant fires,
the runner signals the process group, but the real `gz sim` server is a *child*
of the `/bin/sh` wrapper and survives. It keeps running at roughly 70% CPU on
the same DDS domain.

The first attempt at this rerun ran Gates 2-5 back to back with no cleanup
between them. Gate 3's server orphaned at 19:38:29 and was still alive through
both Gate 4 and Gate 5:

- **Gate 4 failed `gate4_score`** after 43 s with zero cases run. Its safety
  supervisor went silent for 4.2 s, resumed with `joints=0 controllers=0`, and
  latched `state=6 reason=20` (`CONTROLLER_NOT_READY`) 105 microseconds later,
  recovering to `127/127` just 240 ms after that. A starvation blip, not a
  controller defect. Zero `dri2 screen` errors, so this was **not** Regression A
  returning.
- **Gate 5 failed** `gate5_score` and `backend_process_loss_proven`.

Rerun in isolation with the orphan reaped first, Gate 4 passed all eight cases
over 77.8 simulated seconds and Gate 5 passed 29/29. Preserved for comparison:
`log/gate_4_20260822_orphancontam`, `log/gate_5_20260822_orphancontam`. **Those
two directories are invalid evidence.**

The orphan **ignores SIGTERM** and needs SIGKILL — expected, since the process
is deadlocked in `futex_do_wait`. Reap by PID from `pgrep -x "gz sim"`. Do not
`pkill -f` on the world path: the wrapper is `/bin/sh` and the pattern also
matches your own command line.

**Implication for Gate 6, which has not been verified:** Gate 6 runs preflight
Gates 0-5 and then three full repetitions sequentially in one process tree. If a
sub-gate orphans a server, every later sub-gate runs against a CPU competitor,
and Gate 6 does no reaping between them. The 2026-08-18 Gate 6 failure was
attributed to preflight halting at Gate 2 on Defect C; that reading is still
correct for that run, since preflight stopped before anything could cascade. But
any Gate 6 run that gets past preflight is exposed. Adding a reap between
sub-gates is a runner change and is the operator's call.

Operational rule for any sequential gate run:

```bash
pgrep -x "gz sim" && kill -KILL $(pgrep -x "gz sim")
```

## `8865e3c` shipped unlinted code and failed Gate 6 (fixed 2026-08-22)

The first Gate 6 run on 2026-08-22 failed in four minutes with
`432 tests, 0 errors, 8 failures, 26 skipped`. **Every failure was a linter
failure on `araco_system_tests`, all of it in the code `8865e3c` added.** That
commit was never linted before it was committed and pushed.

- flake8, 4: import order in `gate6.py` — the configured style sorts
  case-insensitively, so `launch_exit_code` belongs between
  `GAZEBO_CRASH_EXIT_CODES` and `TEARDOWN_SIGNAL_EXIT_CODES` — plus three
  double-quoted string fragments in `test_gate1_scoring.py`. Only the fragments
  containing no single quote of their own were converted; the ones holding
  `cmd '...'` correctly stay double-quoted.
- pep257, 2: `D213` on the two new multi-line docstrings, in
  `gate1.classify_launch_log` and `gate6.is_gz_shutdown_defect`. The enforced
  style puts the summary on the second line. There was no precedent to copy:
  every other docstring in the package is a single line.

The remaining four Gate 6 failures — `gates_0_through_5_preflight`,
`three_complete_no_retry_repetitions`, `suite_wall_budget`, and
`no_unclassified_error_or_fatal` — were pure cascade. The run aborted at package
tests, so no simulator ever started. The evidence for that is the **absence of a
`preflight/` directory** in `log/gate_6_20260822_defectc_class`, not the orphan
sampler that ran alongside it: that sampler used `pgrep -x "gz sim"` and was
blind, for the reason recorded in the next section.

Fixed and verified with the authoritative ament linters, not a hand-rolled
flake8 invocation — a local `python3 -m flake8` with guessed options reports
`W504` on pre-existing lines and is not the configured check. After the fix,
`colcon test --packages-select araco_system_tests` reports
`67 tests, 0 errors, 0 failures, 0 skipped`.

Lesson worth keeping: this defect had nothing to do with Defect C and would have
failed Gate 6 on its own. Run `colcon test --packages-select araco_system_tests`
after editing anything in that package, before running the gates.

## Gate 6 at 2026-08-22 — seven of nine checks pass, orphan cascade confirmed

Second run, after the lint fix: `log/gate_6_20260822_defectc_class_02`, 773 s.

| Check | Result |
| --- | --- |
| `package_tests` | PASS |
| `package_test_results` | PASS |
| `sanitizers` | PASS |
| `no_sanitizer_diagnostic` | PASS |
| `no_lifecycle_deadlock` | PASS |
| `no_unclassified_error_or_fatal` | PASS |
| `gates_0_through_5_preflight` | **PASS — first time** |
| `three_complete_no_retry_repetitions` | FAIL |
| `suite_wall_budget` | FAIL |

`no_unclassified_error_or_fatal` passing is the live confirmation that the Gate 6
half of the `8865e3c` classification works across a whole log tree.

### Both remaining failures are one event

`suite_wall_budget` is not a timing problem. It reads:

```python
checks['suite_wall_budget'] = (
    len(repetition_wall) == registry['repetitions'] and
    all(value <= suite_wall_limit for value in repetition_wall))
```

Only one repetition ran, so `len(...) == 1 != 3` and the check fails on arity,
not on duration. The one repetition that did run took 121.65 s against a 260 s
limit. Both failures therefore reduce to: **repetition 1 stopped early**, and
the loop has `if not passed: break`.

Repetition 1 ran gates 0, 1, 2 PASS and then **gate 3 FAIL**, on
`gate3_score`: `motion_enabled_observed`, `trusted_enable_accepted`,
`release_returns_to_hold`, and `all_pose_cases` all false. The same gate 3
passed standalone an hour earlier.

### Cause: orphaned servers starve the next sub-gate of CPU

The failing repetition gate 3 latched `state=6 reason=23`,
**`REASON_TIME_DISCONTINUITY`**, with readiness collapsing to `3/127`. Zero
`dri2` errors, so rendering was fine.

Four orphaned `gz sim` servers were left running by this Gate 6 run, started
20:02:38, 20:03:17, 20:05:23 and 20:09:16, still alive after it exited, one at
75% CPU.

**Corrected 2026-08-22.** An earlier version of this section blamed `/clock`
crosstalk on a shared DDS domain. **That was wrong, and the claim is
withdrawn.** `araco_gate1_evidence` already isolates every run: it sets
`ROS_DOMAIN_ID` to `100 + os.getpid() % 100` when `--domain-id` is absent, and a
unique `GZ_PARTITION` alongside it. The domains actually recorded in this run
were preflight `106, 138, 178, 163, 180` and repetition `100, 130, 125` — all
distinct, and the orphans held domains of their own. No orphan could publish
into the failing gate's domain.

The mechanism is contention, not crosstalk. Three to four deadlocked servers
running full physics leave too little CPU for the new sub-gate, its own `/clock`
publishing stutters, and the supervisor faults on the gap. This is the same
class of failure as the standalone Gate 4 contamination earlier the same day,
which showed a 4.2 s supervisor stall under one orphan.

Chain, end to end: Defect C hang -> the runner signals the process group but the
real server is a child of the `/bin/sh` wrapper and survives -> the orphan
ignores SIGTERM because it is deadlocked in `futex_do_wait` -> it keeps running
full physics and burning CPU -> a later sub-gate is starved and faults on time
discontinuity -> the repetition breaks -> `three_complete_no_retry_repetitions`
and `suite_wall_budget` both fail.

This supersedes the 2026-08-18 reading that Gate 6 fails because preflight halts
at Gate 2. That was true then. Preflight now passes and the failure has moved.

### Correction: the orphan sampler used in both runs was invalid

It counted with `pgrep -x "gz sim"`, which matches the process **name**, and the
server's name is `ruby` — the string `gz sim -r -s ...` is only its argument
list. The sampler reported a peak of zero throughout a run that in fact leaked
four servers, and the postflight "orphans: none" line was equally wrong. The
reap helper in the Gate 4-5 rerun script happened to work only because it had a
`pgrep -f '^gz sim -r -s'` fallback after the `-x` attempt.

**Use `pgrep -f '^gz sim'`.** Never `pgrep -x "gz sim"`, and never `pkill -f` on
the world path.

```bash
pgrep -f '^gz sim' && kill -KILL $(pgrep -f '^gz sim')
```

### Decision needed before Gate 6 can pass

No Araco behavior is wrong here; the gate harness does not isolate its sub-gates
from a known upstream leak. Options, for the operator:

1. Reap orphaned servers between sub-gates in `araco_gate6_evidence`. Smallest
   change, directly targets the observed cause.
2. Give each sub-gate its own `ROS_DOMAIN_ID`. `araco_gate1_evidence` already
   takes `--domain-id`, so this is wiring, and it isolates `/clock` even from an
   orphan that is still alive. Does not reclaim the CPU an orphan burns.
3. Both. Recommended: 2 prevents the interference, 1 stops the machine filling
   with deadlocked servers over a long suite.

All three are runner changes, so none was made. Note this is the *harness*
reacting to the upstream defect, a different question from the Defect C gate
contract already settled on 2026-08-20.

## Gate 5's last scorer check is decided by which Defect C variant occurs

**Established 2026-08-22 by direct experiment.** This is now the top blocker,
and it is not covered by the 2026-08-20 contract decision.

`backend_process_loss_quiesces_runtime` is the 29th Gate 5 scorer check. Its
outcome tracks the Defect C variant exactly, with no exceptions in five
observations:

| Run | Variant | `quiesce` |
| --- | --- | --- |
| `gate_5_20260822_defectc_class` | crash, rc `0`, no escalation | **PASS** |
| `gate_5_20260822_probe_explicitdomain` | crash, rc `0`, no escalation | **PASS** |
| `gate_5_20260822_probe_default` | hang, rc `-15`, escalated | FAIL |
| `gate_6_20260822_reap_domains` preflight gate 5 | hang, rc `-15`, escalated | FAIL |
| `gate_5_20260822_orphancontam` | hang, rc `-15`, escalated | FAIL |

The mechanism was already recorded correctly in the Regression B section: the
final Gate 5 scenario issues the same `/server_control stop` and then requires
the runtime to quiesce within 2 s. When the server crashes it dies immediately
and the runtime quiesces. When it hangs, the backend is still alive and still
publishing, so the check cannot pass. **The check is not flaky.** It is a
deterministic function of a non-deterministic upstream outcome, which the
2026-08-18 3-vs-3 repetition put at roughly 50/50.

### This is why Gate 6 cannot pass yet

Gate 6 runs Gate 5 four times: once in preflight and once per repetition. At
roughly even odds per run, all four landing on the crash variant is about a 6%
outcome. Reaping does not help — the orphan is a *consequence* of the hang, and
by the time it is reaped the scored check has already failed.

**The 2026-08-20 Option 3 decision does not cover this.** That classification
was deliberately scoped to `launch_exit` and `launch_log_clean`, and it
deliberately never excuses a scored check. This is a scored check. So Defect C
still blocks Gate 6, through a path the contract change was never meant to
address.

### The 2026-08-22 all-green Gate 0-5 result was variant-dependent

Recorded honestly: that run's Gate 5 passed because it drew the crash variant.
It was a real pass of every check, reproduced fingerprints and all, but it is
**not** evidence that Gate 5 passes reliably. Re-running Gate 5 today produced
29/29 twice and 28/29 three times.

### Ruled out by experiment, not assumption

The explicit `--domain-id` added on 2026-08-22 was suspected when preflight
gate 5 failed. Two standalone probes settled it: **arm A with the default
pid-derived domain FAILED 28/29, arm B with an explicit `--domain-id 145`
PASSED 29/29** — the opposite of what a domain-id regression would produce.
The check also failed on 2026-08-18 in
`gate_5_20260818_slam_regression_03`, before any of this existed.

### Options, for the operator

1. Fix the upstream hang — upgrade or patch Gazebo, or file it. The only option
   that removes the cause rather than working around it.
2. Change the Gate 5 scenario so it forces the backend dead — SIGKILL once the
   graceful stop is seen to fail — and only then asserts quiescence within 2 s.
   **Recommended.** The property under test is "the runtime quiesces when the
   backend is lost". When the server hangs the backend is not lost, so the
   scenario's own premise is unmet and the current result is not a true
   negative. Forcing the kill tests the real property and stops an upstream
   hang presenting as an Araco safety failure.
3. Extend the shutdown-defect classification to this scored check. **Not
   recommended.** It would excuse a genuine safety assertion on the strength of
   a log signature, which is exactly the line the 2026-08-20 decision drew and
   should keep.

## Gates 2-5 run stale code unless the workspace is rebuilt (found 2026-08-22)

`araco_gate1_evidence` is the shared runner for Gates 1-5. `CMakeLists.txt`
installs it four more times with `RENAME araco_gate2_evidence` and so on.
**`colcon build --symlink-install` symlinks the plain `PROGRAMS` installs but
copies the renamed ones.** Gate 0, Gate 1, and Gate 6 are symlinks and pick up
source edits immediately; Gates 2, 3, 4, and 5 are copies and keep running the
code from the last build.

This was found the hard way on 2026-08-22. The first rerun after `8865e3c` was
started without rebuilding. Gate 1 passed with the new classification while
Gates 2 and 3 failed on `launch_log_clean` and emitted no `upstream_defects`
block at all, because their installed copies still held the `9e0284b` runner.
Those two runs are preserved as `log/gate_2_20260822_staleinstall` and
`log/gate_3_20260822_staleinstall`. **They are not valid evidence for anything
except this finding.**

The earlier campaign is unaffected. The stale copies hashed identically to
`9e0284b`, which was the current runner throughout the 2026-08-18 and
2026-08-19 runs, so those results stand.

Rule: **after any edit to `scripts/araco_gate1_evidence`, run
`colcon build --packages-select araco_system_tests --symlink-install` before
running Gates 2-5.** Verify with a hash comparison, which takes a second and is
the only reliable check:

```bash
sha256sum src/araco_system_tests/scripts/araco_gate1_evidence \
  install/araco_system_tests/lib/araco_system_tests/araco_gate{1,2,3,4,5}_evidence
```

Worth fixing properly: the four `RENAME` installs could be replaced with
symlinks created at install time, which would remove the trap. Not done,
because it changes package install behavior and was out of scope for the run
that found it.

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
- The Gate 1-6 shutdown-defect classification added by `8865e3c` has unit
  tests but has never run against a live gate. The first rerun must confirm
  that real logs match the intended signatures, that
  `upstream_defects.gz_sim_shutdown.observed` is reported truthfully, and
  that a clean-exit run still reports `observed: false` rather than
  silently passing everything.
- Code work through `8865e3c` is committed on
  `fix/gate0-tests-and-relay-exec-bit` and pushed. The 2026-08-22 lint fixes to
  `gate1.py`, `gate6.py`, and `test_gate1_scoring.py`, and the updates to
  this file and `DECISIONS.md`, are **uncommitted**; no commit was authorized
  by the request that made them. Gate runs from this tree therefore report
  `source_revision: unreported-dirty-or-installed-tree`, as the 2026-08-18
  run did. Fingerprints are unaffected: documentation content is not part of
  artifact identity, which the 2026-08-22 Gate 0 run confirms empirically by
  reproducing every recorded fingerprint exactly.

## Exact next step

Route 09 remains blocked on Gate 6. **Gates 0-5 all pass** as of 2026-08-22,
and Gate 6 passes seven of its nine checks including preflight Gates 0-5 for the
first time. The two that remain, `three_complete_no_retry_repetitions` and
`suite_wall_budget`, are a single event: an orphaned simulator left by the
Defect C hang keeps publishing `/clock` on the shared DDS domain and faults a
later sub-gate with `REASON_TIME_DISCONTINUITY`.

**One decision is outstanding: how to isolate Gate 6's sub-gates from orphaned
servers.** Three options are recorded in the Gate 6 section; the recommendation
is per-sub-gate `ROS_DOMAIN_ID` plus reaping between sub-gates. It is a runner
change, so it is the operator's call.

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
5. Done 2026-08-20 as `8865e3c`. Defect C was handled by option 3: the
   upstream gz-sim teardown crash/hang is recorded as a named non-blocking
   condition in Gates 1-6 instead of failing `launch_exit` and
   `launch_log_clean`. It is not fixed and filing upstream remains open.
   See the Defect C section and the `DECISIONS.md` entry of the same date.
6. Done 2026-08-22 for Gates 0-5: all six PASS, every fingerprint reproduced,
   and the classification exercised live on all three variants of the defect.
   Three defects were found and fixed along the way — stale renamed installs,
   unlinted code in `8865e3c`, and a contaminated first attempt caused by
   orphaned servers. Gate 6 improved from one passing check to seven.
7. **Current step. Decide how to isolate Gate 6's sub-gates**, then rerun Gate
   6 alone. Gates 0-5 do not need rerunning unless their inputs change. Before
   any sequential run, and after it, check `pgrep -f '^gz sim'` and SIGKILL what
   it finds.
8. Then run route 09.

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
