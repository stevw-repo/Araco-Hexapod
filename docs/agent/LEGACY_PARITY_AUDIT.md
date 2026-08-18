# Araco Legacy-to-Current Parity Audit

Updated: 2026-08-17

## Verdict

The repository remains a simulator-first system rather than a complete port of
the physical legacy robot. For locomotion and joystick behavior, however, the
specific gait-path gaps identified in the previous audit are now corrected:
radial translation, the rotation curve, translation/yaw mixing, repeated
smooth startup, and legacy-equivalent input smoothing are implemented and
tested. Axis 4 is filtered once and drives body posture and gimbal outputs in
normalized synchronization. Their current approved ranges intentionally differ
from the legacy body-yaw range.

The current architecture intentionally retains checked IK, atomic six-leg
transactions, source arbitration, lifecycle, supervision, `ros2_control`, and
contact-aware Gazebo validation instead of copying the legacy system's
unchecked hardware behavior.

## Capability comparison

| Capability | Legacy system | Current system | Assessment |
|---|---|---|---|
| Joystick translation | Left-stick direction plus magnitude capped at `1.0` | Radially normalized left stick, then configured planar scale | Restored |
| Translation curve | Piecewise horizontal function | Same function with approved phase-0.75 continuous linear return | Restored with intentional fix |
| Vertical curve | Piecewise lift including negative startup | Same repeating and startup branches | Restored |
| Walking yaw | Separate piecewise `rotation()` path | Same normalized curve applied as exact base-origin foot arcs | Restored |
| Translation/yaw mix | Relative-magnitude weighting; larger magnitude sets overall command | Same normalized weighting and overall-magnitude rule | Restored |
| First step | Tripod A begins at counter `-50`, tripod B at `0` | Per-walk warm marker reproduces negative-counter startup | Restored |
| Speed control | Counter increment varies | Responsive stride/cadence scheduler | Intentional redesign retained |
| Clearance | Fixed high legacy multiplier | Scales exactly with stride; 60 mm Responsive maximum | User-approved redesign |
| Stop | Counter-specific slow return | Controlled deceleration, safe boundary, nominal retreat, dwell | Same objective, safer redesign |
| Body height/pitch/yaw | Trigger and axes transform foot targets | Checked planted-foot body transform | Present, rewritten |
| Body roll | Not present | Dedicated buttons 3/4 | New capability |
| Input smoothing | 200 Hz P-only update (`Kp=0.02`, `Ki=Kd=0`); height uses `Kp=0.01` | Time-step-independent equivalent on translation, walking yaw, height, roll, pitch, and shared axis 4 | Restored for all current joystick controls |
| Leg geometry | `43/120/120/50 mm` | `43/120/120/50 mm` | Preserved |
| IK failures | Unchecked trigonometry and no atomic rejection | Checked reach/singularity/limits/rates and atomic six-leg commit | Safer redesign |
| Gimbal | Joint 25 and body yaw derive from the same smoothed axis state; ranges `±pi/10` and `±pi/8` | Axis 4 is filtered once and scaled into both targets; ranges remain approved `±pi/10` gimbal and `±0.2 rad` body | Shared response restored; range difference intentional |
| Source failures | No arbitration/session model | Freshness, quarantine, centered release/reacquisition, visible reasons | New capability |
| Servo output | Calibrated PWM/Hiwonder serial transport | No physical backend | Missing by simulator-first scope |
| RGB-D processing | Legacy point-cloud executable, not active by default | Simulated RGB-D topics and camera frames; no full perception stack | Partial |
| Foot debugging | Six target topics and helper vectors | Typed status and `/araco/debug/foot_targets_body` | Functionally replaced |
| Gazebo | Loose absolute-path assets | Composed Harmonic + `gz_ros2_control` + contacts | Current system stronger |
| Isaac Sim | Legacy USD assets | Deferred package | Missing/deferred |

## Corrected lock defect

The former independent `0.20 m/s` X/Y mapping produced a `0.2828427 m/s`
diagonal above the `0.24 m/s` hard envelope and quarantined the source. A fresh
centered no-deadman report remained active, so it could not clear quarantine.

The current adapter normalizes X/Y as one radial vector. Full diagonal input is
exactly the current `0.24 m/s` normal boundary. If a source is quarantined for
another reason, centering all
motion/posture/gimbal controls causes one deliberate inactive release; the next
fresh report establishes a new active session. Unit regressions cover the
mapping and invalid-session recovery sequence.

## Visible failure reasons

The joystick adapter subscribes to arbitration, safety, and locomotion status.
It logs source readiness/quarantine and typed reasons such as
`SOURCE_INVALID`, `SOURCE_RELEASED`, `NO_SOURCE`, and `COMMAND_LIMITED` when
the state changes. This distinguishes command-source failures from checked IK
or command-envelope limits instead of presenting both as a Gazebo freeze.

## Evidence boundary

- Gate 4 at `/tmp/araco_legacy_port_gate4_20260817_07` validates the full
  conservative motion/contact matrix through production arbitration, safety,
  locomotion, controller, and Gazebo paths.
- The isolated Responsive live run at
  `/tmp/araco_legacy_port_joystick_live_20260817_03` validates actual joystick
  mapping and exact gimbal controller feedback.
- Unit regressions validate exact curve samples, pure translation, pure yaw,
  mixed weighting, restart warm-up, radial diagonals, quarantine recovery,
  time-step equivalence, every filtered control, and shared axis-4 response.
- These results do not authorize physical hardware and do not yet prove every
  simultaneous extreme Responsive posture/gait combination reachable.

## Remaining parity work

1. Add a reachable-command projector for extreme combined gait and posture.
2. Add an exhaustive Responsive joystick/contact matrix after operator tuning.
3. Restore only features still wanted: perception, Isaac integration, and
   eventually a separately reviewed physical servo backend.
