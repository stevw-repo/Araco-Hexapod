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
  if (counter < 25.0) {
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
  if (counter >= 25.0 && counter < 50.0) {
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

bool crossed_half_boundary(double before, double after)
{
  if (after < before) {
    return true;
  }
  return before < 0.5 && after >= 0.5;
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
  output.state.velocity.x_m_s = approach(
    previous.velocity.x_m_s, target.x_m_s, linear_acceleration * dt_s);
  output.state.velocity.y_m_s = approach(
    previous.velocity.y_m_s, target.y_m_s, linear_acceleration * dt_s);
  output.state.velocity.yaw_rad_s = approach(
    previous.velocity.yaw_rad_s, target.yaw_rad_s, yaw_acceleration * dt_s);

  const double shaped_local_speed = maximum_local_speed(output.state.velocity, nominal_feet);
  const bool shaped_moving = shaped_local_speed > 1.0e-12;
  if (effective_walk_request) {
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
    output.state.phase += dt_s * output.state.cadence_hz;
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
    (output.state.cycle == 0 ? GaitMode::kStarting : GaitMode::kWalking);
  constexpr std::array<bool, kLegCount> kGroupA{true, false, true, false, true, false};
  for (std::size_t leg = 0; leg < kLegCount; ++leg) {
    double local_phase = output.state.phase + (kGroupA[leg] ? 0.0 : 0.5);
    local_phase -= std::floor(local_phase);
    const bool swing = local_phase >= config.duty_factor;
    output.swing[leg] = swing;
    const double local_x = output.state.velocity.x_m_s -
      output.state.velocity.yaw_rad_s * nominal_feet[leg].y;
    const double local_y = output.state.velocity.y_m_s +
      output.state.velocity.yaw_rad_s * nominal_feet[leg].x;
    const double stance_time = config.duty_factor / output.state.cadence_hz;
    const double stride_x = local_x * stance_time * output.state.applied_velocity_scale;
    const double stride_y = local_y * stance_time * output.state.applied_velocity_scale;
    const double stride_scale = std::min(
      1.0, std::hypot(stride_x, stride_y) / config.maximum_stride_m);
    const double curve_phase = legacy_curve_phase(local_phase);
    const double offset_scale = legacy_horizontal_scale(curve_phase);
    const double lift = config.swing_clearance_m * stride_scale *
      legacy_lift_scale(curve_phase);
    output.foot_targets_base_m[leg].x += offset_scale * stride_x;
    output.foot_targets_base_m[leg].y += offset_scale * stride_y;
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
  const double local_speed = maximum_local_speed(output.state.velocity, nominal_feet);
  if (local_speed <= 1.0e-12) {
    output.state.velocity = {};
    output.state.walking = false;
    output.state.stopping = false;
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
    double local_phase = output.state.phase + (kGroupA[leg] ? 0.0 : 0.5);
    local_phase -= std::floor(local_phase);
    output.swing[leg] = local_phase >= config.duty_factor;
    const double local_x = output.state.velocity.x_m_s -
      output.state.velocity.yaw_rad_s * nominal_feet[leg].y;
    const double local_y = output.state.velocity.y_m_s +
      output.state.velocity.yaw_rad_s * nominal_feet[leg].x;
    const double stance_time = config.duty_factor / output.state.cadence_hz;
    const double stride_x = local_x * stance_time * output.state.applied_velocity_scale;
    const double stride_y = local_y * stance_time * output.state.applied_velocity_scale;
    const double stride_scale = std::min(
      1.0, std::hypot(stride_x, stride_y) / config.maximum_stride_m);
    const double curve_phase = legacy_curve_phase(local_phase);
    const double offset_scale = legacy_horizontal_scale(curve_phase);
    const double lift = config.swing_clearance_m * stride_scale *
      legacy_lift_scale(curve_phase);
    output.foot_targets_base_m[leg].x += offset_scale * stride_x;
    output.foot_targets_base_m[leg].y += offset_scale * stride_y;
    output.foot_targets_base_m[leg].z += lift;
  }
  output.valid = true;
  return output;
}

}  // namespace araco_locomotion
