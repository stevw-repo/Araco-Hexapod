# Araco Hexapod — Working State

Updated: 2026-08-17

## Current result

- Simulator Phases 0–7 are implemented and verified. Formal Gate 6 passed at
  `/tmp/araco_gate6_final_20260816_06`.
- The simplified PXN-2113 Pro posture layout is implemented and validated in
  source, schema, focused tests, and Gate 0 configuration composition.
- This remains simulator-only work. Physical actuation and Raspberry Pi
  deployment are absent and unauthorized.
- No Gazebo or isolated ROS integration session is currently running. The
  former headed sessions `11834` and `55560` and the boundary-recovery harness
  were stopped cleanly. The user explicitly requires every Gazebo session to
  remain closed at handoff; a final process-table check on 2026-08-17 found no
  Gazebo or associated demo node.
- The user explicitly authorized committing and pushing the complete current
  simulator checkpoint on 2026-08-17. This working-state document is part of
  that checkpoint; use Git history for its resulting commit identifier. The
  checkpoint is committed locally on `main`. Publication is pending explicit
  confirmation that `https://github.com/stevw-repo/araco-hexapod-temp.git` is
  the intended trusted destination for the complete 109-file payload.

## Implemented simulator path

- Commands traverse adapter → arbiter → safety → locomotion → named
  24-joint trajectory controller → Gazebo. Tests do not bypass this path.
- `araco_kinematics` provides checked four-DOF FK/IK. Locomotion commits only a
  complete valid six-leg/24-joint transaction.
- Static standing supports bounded body X/Y/Z, roll, pitch, and posture yaw
  while all nominal feet stay planted. World-down projection supplies the
  per-leg foot-pitch target during body roll/pitch.
- Tripod gait uses the approved legacy function-defined foot curves, including
  the phase-0.75 horizontal continuity correction.
- The locomotion callback is fixed at 100 Hz. Stride is the primary speed
  variable and clearance uses exactly the same normalized factor as stride,
  up to 30 mm. Standard non-joystick profiles retain the original 60 mm stride
  and 1.0–1.5 Hz scheduler; Responsive joystick profile `0.9.0` uses the
  separately validated 120 mm stride and 1.5–2.5 Hz scheduler.
- Stand, manual hold, source loss, and deadman expiry use controlled
  deceleration, a safe tripod boundary, six-foot placement, and a 0.250 s
  stable-hold dwell.
- Phase 6 supervision provides source arbitration, freshness and restart
  quarantine, deterministic safety states, no-surprise resume, independent
  locomotion command guarding, lifecycle bringup, and controlled shutdown.
- Phase 7 provides repeatable Gate 6 execution, evidence capture, cross-run
  comparison, timing statistics, and log classification.

## Accepted system evidence

- Final scheduler root: `/tmp/araco_scheduler_final_20260816_02`; 255 tests,
  zero failures, 15 expected skips. Gate 0–4 pass with behavior fingerprint
  `6f9678fd5aae3b832b2afede9710d187c3252a326a46a382d5cc74b937b58bda`.
- Keyboard UI root: `/tmp/araco_keyboard_ui_final_20260816_03`; 259 tests,
  zero failures, 15 expected skips. Gate 0–4 pass with behavior fingerprint
  `35574c357af798bc014d5de8fdf8909ba02af07331e002fd1fd8ae2052c452db`.
- Final Gate 6 root: `/tmp/araco_gate6_final_20260816_06`; all 21 formal checks
  pass. Its three repetitions share behavior fingerprint
  `866f756334259bd34e2d3960948f69af92ccaadb1b5719c4ff567ca6c048e829`.
  Package results were 328 checks, zero errors/failures, and 24 expected skips.

## Current joystick mapping

- Device: `LiteStar PXN-2113 Pro`, USB `11ff:0837`, SDL GUID
  `0300b14bff1100003708000010010000`; ROS observes six axes and 12 buttons.
- Mapping/profile version: `0.9.0`.
- Trigger / physical button 1 (live-verified ROS index 0): intentionally unused.
- Physical button 2 (live-verified ROS index 1): intentionally unused.
- Inverted axis 5: direct planted-body pitch.
- Axis 4: planted-body posture yaw.
- Physical button 3 (expected ROS index 2): roll left, `-0.15 rad`.
- Physical button 4 (expected ROS index 3): roll right, `+0.15 rad`.
- Pressing both roll buttons commands zero roll. There is no roll/pitch modifier
  or mode-dependent HAT behavior.
- There is no joystick deadman. Any fresh complete joystick report retains
  neutral stand authority. There is no separate operator Enable Motion step.
  The profile auto-enables once after readiness and a fresh valid neutral
  standing selection. Translation, walking yaw, roll, pitch, and posture yaw
  must be neutral; body-height trim may remain at its current position. The
  adapter emits one internal inactive startup sample to establish a new arbiter
  session. Timeout, disconnect, malformed input, controlled hold, and faults
  stop motion without automatic resume during the same launch.
- The user live-verified in Gazebo that forward/reverse was initially the only
  correct direction. Version `0.6.0` reversed every other control, after which
  the user found height and roll had been correct before that broad reversal.
  Version `0.7.0` therefore preserves axis 1 forward/reverse, keeps the corrected
  lateral (axis 0), walking yaw (axis 3), pitch (axis 5), and posture yaw (axis
  4), and restores the prior body-height and dedicated roll-button polarities.
- The user selected the Responsive simulator option, then clarified that
  doubling stride is intended to double robot speed. Joystick profile `0.9.0`
  selects `0.200 m/s` translation, `1.200 rad/s` walking yaw, `1.5–2.5 Hz`
  cadence, `2.0 Hz/s` cadence slew, a `0.120 m` stride cap, and shaping rates
  scaled to preserve response time. Clearance remains proportional to stride
  up to `0.030 m`.
  CI, Gate, and keyboard profiles retain their existing slow artifacts.
- A headed `0.4.0` run verified no-trigger candidate authority, then correctly
  exposed that the former manual Enable Motion edge policy could not arm a
  continuously active source. That interim result is not final passing evidence.

## Superseded no-deadman mapping validation

- Clean root: `/tmp/araco_joystick_nodeadman_20260816_01`.
- The affected package dependency graph builds successfully.
- `araco_teleop` and `araco_bringup` report 56 checks, zero errors, zero
  failures, and zero skips. Tests prove fresh neutral authority without a
  trigger, unused buttons 1/2, direct pitch,
  roll-left/right signs, simultaneous cancellation, invalid control overlap,
  malformed Joy reports, deadzone, timeout, and fail-closed behavior.
- Composed profile root: `/tmp/araco_joystick_nodeadman_profile_v040_01`.
- `gazebo_joystick_v0` Gate 0 status: `PASS`.
- Behavior fingerprint:
  `76be396ace4297e1560f60277326cf1b978b090ee2e4d200a91859a3598eab07`.
- Run fingerprint:
  `313f92897806bb599b850ecc655481dad986843dc146218b8236722bab7acb52`.

This evidence applies to `0.4.0`, before one-shot automatic enable, and is
superseded by the final evidence below.

## Final no-deadman automatic-enable validation

- Clean affected-package root:
  `/tmp/araco_joystick_auto_final_20260816_01`.
- `araco_supervision`, `araco_teleop`, and `araco_bringup` report 132 checks,
  zero errors, zero failures, and 11 expected skips.
- Joystick Gate 0 composition:
  `/tmp/araco_joystick_auto_final_profile_v050_03`, status `PASS`, behavior
  fingerprint
  `683ad02b5cbc0112f3e1b674795b5f015eb89fc87c869fc07fe79594b99b8adf`.
- Keyboard scope check:
  `/tmp/araco_keyboard_auto_scope_check_20260816_03`, status `PASS`; generated
  automatic-enable parameter is false, while the joystick profile's is true.
- A headed launch with the connected controller observed source 10 selected and
  the automatic safety transition from `HOLDING` to `MOTION_ENABLED` with no
  button or action input.
- The apparent Gazebo freeze was reproduced in the live launch log as a
  one-tick locomotion reason-16 event followed by the correct safety FaultHold;
  the GUI renderer itself did not hang. Combined joystick posture/walking
  requests can reach the IK workspace boundary. Locomotion now preserves and
  republishes its last complete valid 24-joint target, reports reason 11
  (`COMMAND_LIMITED`), and retries while the request remains saturated. Only
  loss of the last committed trajectory invariant remains reason 16 and
  latched.
- Validation on 2026-08-17: the complete eight-package dependency set built in
  `/tmp/araco_joystick_polarity_full_20260817_01`; the focused affected-package
  result contains 129 checks, zero errors/failures, and 10 expected static-
  analysis skips. Profile `0.6.0` composed at
  `/tmp/araco_joystick_polarity_profile_v060_20260817_02` with behavior
  fingerprint `854549de4a7a7fcf7a7d65aa85c17c8b2306aa0b54ea29eb8c1efc2e7f21c931`.
- The headed joystick launch reached `MOTION_ENABLED`. Live operator input hit
  one gait/posture and two body-pose IK boundaries; every event held the last
  valid trajectory without a fault. A post-event typed safety sample remained
  state 4, reason 0, readiness `127/127`, fault mask 0, source 10. Final visual
  confirmation of every corrected direction remained with the user at that
  evidence point; that launch was later stopped.
- User correction on 2026-08-17 supersedes the last sentence above for height
  and roll. Mapping/profile `0.7.0` passes 57 focused checks with zero failures
  and composes at `/tmp/araco_joystick_polarity_profile_v070_20260817_01`
  with behavior fingerprint
  `cbd3bcf19d976fa6269bca75d8dd1194fb62eb93bdba13b7f3f35f2a0ed4c0e8`.
  The old `0.6.0` headed launch was stopped so its loaded mapping cannot be
  mistaken for the correction. The subsequent Responsive `0.8.0` source
  changes are implemented and configuration-validated but not yet operator-
  validated in a headed simulator.

## Responsive joystick validation

- Clean build root:
  `/tmp/araco_joystick_responsive_build_20260817_01`; all eight dependency
  packages build successfully.
- `araco_teleop`, `araco_locomotion`, `araco_supervision`, and
  `araco_bringup` report 205 checks, zero errors/failures, and 21 expected
  static-analysis skips.
- Composed bundle:
  `/tmp/araco_joystick_responsive_profile_v080_20260817_01`.
- Behavior fingerprint:
  `07d0049377d65cd61a2a4d1784bdc59c5c280fe2d9bea4009d88754173f1d564`.
- Generated parameters verify `0.100/0.120 m/s` normal/hard translation,
  `0.600/0.750 rad/s` normal/hard yaw, `1.5–2.5 Hz` cadence, `2.0 Hz/s`
  cadence slew, and the selected acceleration/deceleration values.
- All five non-joystick profiles compose with their previous behavior
  fingerprints, so the Responsive change is isolated from CI/Gate/keyboard.
- The restarted headed launch detected the PXN-2113 Pro, loaded profile `0.8.0`, and
  reached `MOTION_ENABLED`. A live safety sample reported state 4, reason 0,
  source 10, readiness `127/127`, fault mask 0, and no reset requirement.
- Operator input can still reach the known IK workspace boundary; observed
  events held the last complete valid trajectory and retried, as designed,
  without latching safety. Final subjective speed and direction approval is
  still pending from the user.

The `0.8.0` evidence above is superseded for speed and workspace recovery by
joystick profile `0.9.0`:

- Clean build root:
  `/tmp/araco_joystick_stride_recovery_build_20260817_02`; 208 affected checks,
  zero errors/failures, and 21 expected static-analysis skips.
- Composed bundle:
  `/tmp/araco_joystick_double_speed_profile_v090_20260817_01`; behavior
  fingerprint
  `15277c3a7fbc20e7315dc665bfab482bf0d8f9380059c5cdf55a694bdf9ab1bd`.
- Generated values are `0.200/0.240 m/s` normal/hard translation,
  `1.200/1.500 rad/s` normal/hard yaw, `1.5–2.5 Hz` cadence, `0.120 m` maximum
  stride, full provisional simulator leg-joint ranges, and `2.0 rad/s`
  command-rate caps. Non-joystick behavior fingerprints are unchanged.
- Exact-geometry regression sweeps all primary planar directions and maximum
  diagonal translation plus maximum yaw across multiple cycles at neutral
  pose; all complete six-leg IK transactions pass.
- The former indefinite boundary lock is corrected by a phase-frozen
  retreat-to-nominal path. An isolated ROS test forced reason 11, observed a
  complete valid retreat, centered the command, and observed reason 0 with all
  six legs valid. A held saturated command remains limited until centered.

## Fidelity facts and limitations

- The presentation model retains 49 exact Fusion/vendor meshes: 26 primary,
  13 servo-case, seven servo-horn, and three Gemini 335 role meshes.
- Each foot collision uses a separate orientation-independent 4 mm point-contact
  sphere at the kinematic tip.
- The known Fusion tibia occurrence/canonical-frame offset remains intentionally
  unforced per user direction.
- Masses, inertias, servo zeros/directions/limits, camera extrinsics, contact
  material, and controller response remain provisional or unresolved.
- Gazebo controller gains are simulator-only and are not physical-servo gains
  or hardware-safety claims.

## Exact next step

- Keep Gazebo closed at this handoff. When the user next requests a headed
  test, launch `gazebo_joystick_v0` from the `0.9.0` build and validate
  subjective speed and repeated boundary recovery. No trigger or Enable Motion
  action is required. If reason 11 appears, center the planar controls briefly;
  gait should re-arm automatically rather than lock.
