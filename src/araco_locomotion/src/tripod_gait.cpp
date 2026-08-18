// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include "araco_locomotion/tripod_gait.hpp"

#include <algorithm>
#include <cmath>

namespace araco_locomotion
{
namespace
{

bool finite_velocity(const PlanarVelocity & value)
{
  return std::isfinite(value.x_m_s) && std::isfinite(value.y_m_s) &&
         std::isfinite(value.yaw_rad_s);
}

double approach(double value, double target, double maximum_step)
{
  return value + std::clamp(target - value, -maximum_step, maximum_step);
}

double wrap_phase(double phase)
{
  return phase - std::floor(phase);
}

double legacy_curve_phase(double local_phase)
{
  // The legacy curve's swing occupies phase [0.25, 0.75).  Shift it onto the
  // current state machine's [0.5, 1.0) swing interval without changing the
  // established tripod handover phases.
  return wrap_phase(local_phase + 0.75);
}

double legacy_horizontal_scale(double phase)
{
  // Functional behavior is derived from the project's Apache-2.0 legacy
  // algo.py trajectory.  This C++ expression is independently rewritten and
  // includes the user-approved phase-0.75 continuity correction below.
  // Re-express the legacy function-defined trajectory on a normalized
  // [-0.5, +0.5] stride.  The final branch intentionally differs from the
  // legacy implementation: it starts at +0.5 at phase 0.75 and decreases
  // linearly to zero at phase 1.0, removing the position discontinuity while
  // preserving constant supporting-foot velocity across the cycle boundary.
  const double counter = phase * 100.0;
  double value = 0.0;
  if (counter >= -50.0 && counter < -25.0) {
    value = (-1.0 / 625.0) * std::pow(counter, 3) -
      (6.0 / 25.0) * std::pow(counter, 2) - 9.0 * counter - 50.0;
  } else if (counter >= -25.0 && counter < 25.0) {
    value = -2.0 * counter;
  } else if (counter < 50.0) {
    value = (2.0 / 25.0) * std::pow(counter - 25.0, 2) - 50.0;
  } else if (counter < 60.0) {
    value = (-3.0 / 50.0) * std::pow(counter - 50.0, 3) +
      (7.0 / 10.0) * std::pow(counter - 50.0, 2) +
      4.0 * (counter - 50.0);
  } else if (counter < 75.0) {
    value = 50.0;
  } else {
    value = 2.0 * (100.0 - counter);
  }
  return value / 100.0;
}

double legacy_lift_scale(double phase)
{
  const double counter = phase * 100.0;
  double value = 0.0;
  if (counter >= -50.0 && counter < -40.0) {
    value = -0.3 * std::pow(counter, 2) - 24.0 * counter - 450.0;
  } else if (counter >= -40.0 && counter < -35.0) {
    value = 30.0;
  } else if (counter >= -35.0 && counter < -25.0) {
    value = (3.0 / 50.0) * std::pow(counter, 3) +
      (27.0 / 5.0) * std::pow(counter, 2) +
      (315.0 / 2.0) * counter + 1500.0;
  } else if (counter >= 25.0 && counter < 50.0) {
    value = 0.00208 * std::pow(counter, 3) -
      0.308 * std::pow(counter, 2) + 15.2 * counter - 220.0;
  } else if (counter >= 50.0 && counter < 60.0) {
    value = 30.0;
  } else if (counter >= 60.0 && counter < 75.0) {
    value = (4.0 / 225.0) * std::pow(counter - 60.0, 3) -
      (2.0 / 5.0) * std::pow(counter - 60.0, 2) + 30.0;
  }
  return std::clamp(value / 30.0, 0.0, 1.0);
}

double legacy_rotation_scale(double phase)
{
  // This is the separate legacy rotation() curve, normalized from its
  // original [-50, +50] angular command range to [-0.5, +0.5].
  const double counter = phase * 100.0;
  double value = 0.0;
  if (counter >= 0.0 && counter < 25.0) {
    value = -2.0 * counter;
  } else if (counter >= 25.0 && counter < 75.0) {
    value = (-1.0 / 625.0) * std::pow(counter, 3) +
      (150.0 / 625.0) * std::pow(counter, 2) - 9.0 * counter + 50.0;
  } else if (counter >= 75.0 && counter < 100.0) {
    value = -2.0 * counter + 200.0;
  }
  return value / 100.0;
}

bool finite_feet(const std::array<araco_kinematics::Point3, kLegCount> & feet)
{
  return std::all_of(feet.begin(), feet.end(), [](const auto & foot) {
             return std::isfinite(foot.x) && std::isfinite(foot.y) && std::isfinite(foot.z);
    });
}

double maximum_local_speed(
  const PlanarVelocity & velocity,
  const std::array<araco_kinematics::Point3, kLegCount> & feet)
{
  double maximum = 0.0;
  for (const auto & foot : feet) {
    maximum = std::max(maximum, std::hypot(
        velocity.x_m_s - velocity.yaw_rad_s * foot.y,
        velocity.y_m_s + velocity.yaw_rad_s * foot.x));
  }
  return maximum;
}

struct LegacyBlend
{
  double translation_weight{1.0};
  double rotation_weight{0.0};
  double translation_x_m_s{0.0};
  double translation_y_m_s{0.0};
  double yaw_rad_s{0.0};
  double maximum_local_speed_m_s{0.0};
};

LegacyBlend legacy_blend(
  const TripodConfig & config,
  const PlanarVelocity & velocity,
  const std::array<araco_kinematics::Point3, kLegCount> & feet)
{
  LegacyBlend result;
  const double translation_magnitude = std::hypot(velocity.x_m_s, velocity.y_m_s);
  const double translation_normalized =
    translation_magnitude / config.planar_command_scale_m_s;
  const double rotation_normalized =
    std::abs(velocity.yaw_rad_s) / config.yaw_command_scale_rad_s;
  const double combined = translation_normalized + rotation_normalized;
  if (combined > 1.0e-12) {
    result.translation_weight = translation_normalized / combined;
    result.rotation_weight = rotation_normalized / combined;
  }
  const double overall = std::max(translation_normalized, rotation_normalized);
  if (translation_magnitude > 1.0e-12) {
    const double translation_speed = config.planar_command_scale_m_s * overall;
    result.translation_x_m_s =
      translation_speed * velocity.x_m_s / translation_magnitude;
    result.translation_y_m_s =
      translation_speed * velocity.y_m_s / translation_magnitude;
  }
  if (std::abs(velocity.yaw_rad_s) > 1.0e-12) {
    result.yaw_rad_s = std::copysign(
      config.yaw_command_scale_rad_s * overall, velocity.yaw_rad_s);
  }
  for (const auto & foot : feet) {
    const double local_x = result.translation_weight * result.translation_x_m_s -
      result.rotation_weight * result.yaw_rad_s * foot.y;
    const double local_y = result.translation_weight * result.translation_y_m_s +
      result.rotation_weight * result.yaw_rad_s * foot.x;
    result.maximum_local_speed_m_s = std::max(
      result.maximum_local_speed_m_s, std::hypot(local_x, local_y));
  }
  return result;
}

bool crossed_half_boundary(double before, double after)
{
  if (after < before) {
    return true;
  }
  return before < 0.5 && after >= 0.5;
}

struct LegacyFootSample
{
  double horizontal_scale{0.0};
  double rotation_scale{0.0};
  double lift_scale{0.0};
  bool swing{false};
};

LegacyFootSample legacy_foot_sample(
  double repeating_phase, double startup_phase, bool starting,
  bool rotation_warm_start, bool group_a)
{
  if (starting) {
    const double counter_phase = startup_phase + (group_a ? -0.5 : 0.0);
    return {
      legacy_horizontal_scale(counter_phase),
      legacy_rotation_scale(counter_phase),
      legacy_lift_scale(counter_phase),
      group_a,
    };
  }

  double local_phase = repeating_phase + (group_a ? 0.0 : 0.5);
  local_phase = wrap_phase(local_phase);
  const double curve_phase = legacy_curve_phase(local_phase);
  // In the legacy negative-counter first step, tripod A remains at zero yaw
  // offset from counter -25 through 0. The repeating rotation curve would
  // otherwise introduce a discontinuous +half-yaw offset at the warm-start
  // handover.
  const double rotation_scale = rotation_warm_start && group_a && repeating_phase < 0.25 ?
    0.0 : legacy_rotation_scale(curve_phase);
  return {
    legacy_horizontal_scale(curve_phase),
    rotation_scale,
    legacy_lift_scale(curve_phase),
    local_phase >= 0.5,
  };
}

}  // namespace

TripodStep advance_tripod(
  const TripodConfig & config,
  const TripodState & previous,
  const std::array<araco_kinematics::Point3, kLegCount> & nominal_feet,
  const PlanarVelocity & requested_velocity,
  bool request_walk,
  bool request_stop,
  double dt_s)
{
  TripodStep output;
  output.state = previous;
  output.foot_targets_base_m = nominal_feet;
  const bool config_valid = std::isfinite(dt_s) && dt_s > 0.0 &&
    std::isfinite(config.base_cadence_hz) && config.base_cadence_hz > 0.0 &&
    std::isfinite(config.maximum_cadence_hz) &&
    config.maximum_cadence_hz >= config.base_cadence_hz &&
    std::isfinite(config.cadence_rate_hz_s) && config.cadence_rate_hz_s > 0.0 &&
    std::isfinite(config.preferred_maximum_stride_scale) &&
    config.preferred_maximum_stride_scale > 0.0 &&
    config.preferred_maximum_stride_scale <= 1.0 &&
    std::isfinite(config.motion_deadband_m_s) && config.motion_deadband_m_s >= 0.0 &&
    std::isfinite(config.duty_factor) &&
    std::abs(config.duty_factor - 0.5) < 1.0e-12 &&
    std::isfinite(config.maximum_stride_m) && config.maximum_stride_m > 0.0 &&
    std::isfinite(config.swing_clearance_m) && config.swing_clearance_m > 0.0 &&
    std::isfinite(config.planar_command_scale_m_s) &&
    config.planar_command_scale_m_s > 0.0 &&
    std::isfinite(config.yaw_command_scale_rad_s) &&
    config.yaw_command_scale_rad_s > 0.0 &&
    std::isfinite(config.stable_hold_dwell_s) && config.stable_hold_dwell_s >= 0.0 &&
    finite_velocity(requested_velocity) && finite_velocity(previous.velocity) &&
    finite_feet(nominal_feet);
  if (!config_valid) {
    return output;
  }

  const bool command_above_deadband =
    maximum_local_speed(requested_velocity, nominal_feet) > config.motion_deadband_m_s;
  const bool effective_walk_request =
    request_walk && !request_stop && command_above_deadband;
  const bool stopping = !effective_walk_request;
  const PlanarVelocity target = stopping ? PlanarVelocity{} : requested_velocity;
  const double linear_acceleration = stopping ?
    config.translation_stop_deceleration_m_s2 : config.translation_acceleration_m_s2;
  const double yaw_acceleration = stopping ?
    config.yaw_stop_deceleration_rad_s2 : config.yaw_acceleration_rad_s2;
  if (config.operator_input_pre_filtered && effective_walk_request) {
    // The joystick adapter already applies the legacy response once to every
    // operator control. Do not distort it with a second acceleration filter.
    output.state.velocity = target;
  } else {
    output.state.velocity.x_m_s = approach(
      previous.velocity.x_m_s, target.x_m_s, linear_acceleration * dt_s);
    output.state.velocity.y_m_s = approach(
      previous.velocity.y_m_s, target.y_m_s, linear_acceleration * dt_s);
    output.state.velocity.yaw_rad_s = approach(
      previous.velocity.yaw_rad_s, target.yaw_rad_s, yaw_acceleration * dt_s);
  }

  const auto shaped_blend = legacy_blend(config, output.state.velocity, nominal_feet);
  const double shaped_local_speed = shaped_blend.maximum_local_speed_m_s;
  const bool shaped_moving = shaped_local_speed > 1.0e-12;
  if (effective_walk_request) {
    if (!previous.walking && !previous.stopping) {
      output.state.starting = true;
      output.state.startup_phase = 0.0;
      output.state.phase = 0.0;
    }
    output.state.walking = true;
    output.state.stopping = false;
    output.state.hold_dwell_s = 0.0;
  } else if (previous.walking || previous.stopping) {
    output.state.walking = true;
    output.state.stopping = true;
  }

  if (output.state.walking) {
    const double preferred_stride =
      config.preferred_maximum_stride_scale * config.maximum_stride_m;
    const double target_cadence = std::clamp(
      config.duty_factor * shaped_local_speed / preferred_stride,
      config.base_cadence_hz, config.maximum_cadence_hz);
    output.state.cadence_hz = previous.cadence_hz > 0.0 ? approach(
      previous.cadence_hz, target_cadence, config.cadence_rate_hz_s * dt_s) :
      config.base_cadence_hz;
    const double unconstrained_maximum_stride = shaped_moving ?
      config.duty_factor * shaped_local_speed / output.state.cadence_hz : 0.0;
    output.state.applied_velocity_scale = unconstrained_maximum_stride > 0.0 ?
      std::min(1.0, config.maximum_stride_m / unconstrained_maximum_stride) : 0.0;
    output.state.maximum_stride_scale = std::min(
      1.0, unconstrained_maximum_stride * output.state.applied_velocity_scale /
      config.maximum_stride_m);
    output.state.maximum_clearance_m =
      config.swing_clearance_m * output.state.maximum_stride_scale;

    const double old_phase = previous.phase;
    const double phase_advance = dt_s * output.state.cadence_hz;
    if (output.state.starting) {
      output.state.startup_phase += phase_advance;
      if (output.state.startup_phase >= 0.25 - 1.0e-12) {
        output.state.phase = std::max(0.0, output.state.startup_phase - 0.25);
        output.state.startup_phase = 0.25;
        output.state.starting = false;
      }
    } else {
      output.state.phase += phase_advance;
      if (output.state.startup_phase >= 0.25 - 1.0e-12 &&
        output.state.phase >= 0.25 - 1.0e-12)
      {
        output.state.startup_phase = 0.0;
      }
    }
    if (output.state.phase >= 1.0 - 1.0e-12) {
      const auto completed_cycles = static_cast<std::uint64_t>(
        std::floor(output.state.phase + 1.0e-12));
      output.state.phase -= static_cast<double>(completed_cycles);
      output.state.phase = std::max(0.0, output.state.phase);
      output.state.cycle += completed_cycles;
    }
    if (output.state.stopping && !shaped_moving &&
      crossed_half_boundary(old_phase, output.state.phase))
    {
      output.state.walking = false;
      output.state.stopping = false;
      output.state.starting = false;
      output.state.startup_phase = 0.0;
      output.state.phase = 0.0;
      output.state.hold_dwell_s = dt_s;
      output.state.cadence_hz = 0.0;
      output.state.maximum_stride_scale = 0.0;
      output.state.maximum_clearance_m = 0.0;
      output.state.applied_velocity_scale = 0.0;
      output.state.velocity = {};
    }
  } else {
    output.state.cadence_hz = 0.0;
    output.state.maximum_stride_scale = 0.0;
    output.state.maximum_clearance_m = 0.0;
    output.state.applied_velocity_scale = 0.0;
    if (output.state.hold_dwell_s > 0.0 &&
      output.state.hold_dwell_s < config.stable_hold_dwell_s)
    {
      output.state.hold_dwell_s += dt_s;
    }
  }

  if (!output.state.walking) {
    output.mode = output.state.hold_dwell_s > 0.0 &&
      output.state.hold_dwell_s < config.stable_hold_dwell_s ?
      GaitMode::kStopping : GaitMode::kHolding;
    output.valid = true;
    return output;
  }

  output.mode = output.state.stopping ? GaitMode::kStopping :
    (output.state.starting ? GaitMode::kStarting : GaitMode::kWalking);
  constexpr std::array<bool, kLegCount> kGroupA{true, false, true, false, true, false};
  for (std::size_t leg = 0; leg < kLegCount; ++leg) {
    const auto foot_sample = legacy_foot_sample(
      output.state.phase, output.state.startup_phase,
      output.state.starting, output.state.startup_phase > 0.0, kGroupA[leg]);
    output.swing[leg] = foot_sample.swing;
    const double stance_time = config.duty_factor / output.state.cadence_hz;
    const double stride_time = stance_time * output.state.applied_velocity_scale;
    const double stride_x = shaped_blend.translation_x_m_s * stride_time;
    const double stride_y = shaped_blend.translation_y_m_s * stride_time;
    const double rotation_angle =
      foot_sample.rotation_scale * shaped_blend.yaw_rad_s * stride_time;
    const double cosine = std::cos(rotation_angle);
    const double sine = std::sin(rotation_angle);
    const double rotated_x = cosine * nominal_feet[leg].x -
      sine * nominal_feet[leg].y - nominal_feet[leg].x;
    const double rotated_y = sine * nominal_feet[leg].x +
      cosine * nominal_feet[leg].y - nominal_feet[leg].y;
    const double lift = output.state.maximum_clearance_m * foot_sample.lift_scale;
    output.foot_targets_base_m[leg].x +=
      shaped_blend.translation_weight * foot_sample.horizontal_scale * stride_x +
      shaped_blend.rotation_weight * rotated_x;
    output.foot_targets_base_m[leg].y +=
      shaped_blend.translation_weight * foot_sample.horizontal_scale * stride_y +
      shaped_blend.rotation_weight * rotated_y;
    output.foot_targets_base_m[leg].z += lift;
  }
  output.valid = true;
  return output;
}

TripodStep retreat_tripod_to_nominal(
  const TripodConfig & config,
  const TripodState & previous,
  const std::array<araco_kinematics::Point3, kLegCount> & nominal_feet,
  double dt_s)
{
  TripodStep output;
  output.state = previous;
  output.foot_targets_base_m = nominal_feet;
  const bool valid = std::isfinite(dt_s) && dt_s > 0.0 &&
    std::isfinite(config.translation_stop_deceleration_m_s2) &&
    config.translation_stop_deceleration_m_s2 > 0.0 &&
    std::isfinite(config.yaw_stop_deceleration_rad_s2) &&
    config.yaw_stop_deceleration_rad_s2 > 0.0 &&
    std::isfinite(config.maximum_stride_m) && config.maximum_stride_m > 0.0 &&
    std::isfinite(config.swing_clearance_m) && config.swing_clearance_m > 0.0 &&
    std::isfinite(config.stable_hold_dwell_s) && config.stable_hold_dwell_s >= 0.0 &&
    finite_velocity(previous.velocity) && finite_feet(nominal_feet);
  if (!valid) {
    return output;
  }

  output.state.velocity.x_m_s = approach(
    previous.velocity.x_m_s, 0.0,
    config.translation_stop_deceleration_m_s2 * dt_s);
  output.state.velocity.y_m_s = approach(
    previous.velocity.y_m_s, 0.0,
    config.translation_stop_deceleration_m_s2 * dt_s);
  output.state.velocity.yaw_rad_s = approach(
    previous.velocity.yaw_rad_s, 0.0,
    config.yaw_stop_deceleration_rad_s2 * dt_s);
  const auto shaped_blend = legacy_blend(config, output.state.velocity, nominal_feet);
  const double local_speed = shaped_blend.maximum_local_speed_m_s;
  if (local_speed <= 1.0e-12) {
    output.state.velocity = {};
    output.state.walking = false;
    output.state.stopping = false;
    output.state.starting = false;
    output.state.startup_phase = 0.0;
    output.state.phase = 0.0;
    output.state.cadence_hz = 0.0;
    output.state.maximum_stride_scale = 0.0;
    output.state.maximum_clearance_m = 0.0;
    output.state.applied_velocity_scale = 0.0;
    output.state.hold_dwell_s = previous.walking || previous.stopping ?
      dt_s : std::min(
      config.stable_hold_dwell_s, previous.hold_dwell_s + dt_s);
    output.mode = output.state.hold_dwell_s < config.stable_hold_dwell_s ?
      GaitMode::kStopping : GaitMode::kHolding;
    output.valid = true;
    return output;
  }

  output.state.walking = true;
  output.state.stopping = true;
  output.state.hold_dwell_s = 0.0;
  output.state.cadence_hz = previous.cadence_hz > 0.0 ?
    previous.cadence_hz : config.base_cadence_hz;
  const double unconstrained_stride =
    config.duty_factor * local_speed / output.state.cadence_hz;
  output.state.applied_velocity_scale = std::min(
    1.0, config.maximum_stride_m / unconstrained_stride);
  output.state.maximum_stride_scale = std::min(
    1.0, unconstrained_stride * output.state.applied_velocity_scale /
    config.maximum_stride_m);
  output.state.maximum_clearance_m =
    config.swing_clearance_m * output.state.maximum_stride_scale;
  output.mode = GaitMode::kStopping;

  constexpr std::array<bool, kLegCount> kGroupA{true, false, true, false, true, false};
  for (std::size_t leg = 0; leg < kLegCount; ++leg) {
    const auto foot_sample = legacy_foot_sample(
      output.state.phase, output.state.startup_phase,
      output.state.starting, output.state.startup_phase > 0.0, kGroupA[leg]);
    output.swing[leg] = foot_sample.swing;
    const double stance_time = config.duty_factor / output.state.cadence_hz;
    const double stride_time = stance_time * output.state.applied_velocity_scale;
    const double stride_x = shaped_blend.translation_x_m_s * stride_time;
    const double stride_y = shaped_blend.translation_y_m_s * stride_time;
    const double rotation_angle =
      foot_sample.rotation_scale * shaped_blend.yaw_rad_s * stride_time;
    const double cosine = std::cos(rotation_angle);
    const double sine = std::sin(rotation_angle);
    const double rotated_x = cosine * nominal_feet[leg].x -
      sine * nominal_feet[leg].y - nominal_feet[leg].x;
    const double rotated_y = sine * nominal_feet[leg].x +
      cosine * nominal_feet[leg].y - nominal_feet[leg].y;
    const double lift = output.state.maximum_clearance_m * foot_sample.lift_scale;
    output.foot_targets_base_m[leg].x +=
      shaped_blend.translation_weight * foot_sample.horizontal_scale * stride_x +
      shaped_blend.rotation_weight * rotated_x;
    output.foot_targets_base_m[leg].y +=
      shaped_blend.translation_weight * foot_sample.horizontal_scale * stride_y +
      shaped_blend.rotation_weight * rotated_y;
    output.foot_targets_base_m[leg].z += lift;
  }
  output.valid = true;
  return output;
}

}  // namespace araco_locomotion
