# Araco Hexapod — Working State

Updated: 2026-08-18
Machine: `stevw-s14-Stealth-14Studio-A13VF` (Ubuntu 24.04.4 LTS)
Location: these continuity files moved from `docs/agent/` to `.agent/` on
2026-08-18. The move is staged and not committed.

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
- `colcon test` for `araco_navigation`, `araco_gazebo`, `araco_bringup`, and
  `araco_system_tests`: `386 tests, 0 errors, 0 failures, 23 skipped`.
- All Gazebo, RViz, bridge, RTAB-Map, and control sessions were closed after
  live diagnostics.

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

Route 09 is blocked. Commit `f1e41af` changed interface fields, model limits
and poses, profiles, the composer, gait and controlled stop, and the
arbitration/safety path. Under the regression rules in
`PHASED_DELIVERY_PLAN.md` this mandates a rerun of Gate 0 through Gate 5, plus
Gate 6 because it was already reached. A later pass is invalid if an earlier
required rerun is missing, so route 09 evidence taken now would be void.

Required order:

1. Rebuild and run the full suite:
   `colcon build --symlink-install && colcon test && colcon test-result --verbose`.
2. Run Gates 0 through 6 into `log/`, not `/tmp`. Evidence written to `/tmp`
   has already been lost once to a reboot. `log/` is git-ignored but durable.
3. Record the resulting fingerprints and evidence paths here.
4. Only then run route 09.

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
