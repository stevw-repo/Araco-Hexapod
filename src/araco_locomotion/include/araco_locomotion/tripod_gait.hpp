// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#ifndef ARACO_LOCOMOTION__TRIPOD_GAIT_HPP_
#define ARACO_LOCOMOTION__TRIPOD_GAIT_HPP_

#include <array>
#include <cstdint>

#include "araco_kinematics/leg_kinematics.hpp"
#include "araco_locomotion/standing_target.hpp"

namespace araco_locomotion
{

struct PlanarVelocity
{
  double x_m_s{0.0};
  double y_m_s{0.0};
  double yaw_rad_s{0.0};
};

struct TripodConfig
{
  double base_cadence_hz{1.0};
  double maximum_cadence_hz{1.5};
  double cadence_rate_hz_s{1.0};
  double preferred_maximum_stride_scale{0.5};
  double motion_deadband_m_s{0.005};
  double duty_factor{0.5};
  double maximum_stride_m{0.06};
  double swing_clearance_m{0.03};
  double translation_acceleration_m_s2{0.1};
  double translation_stop_deceleration_m_s2{0.15};
  double yaw_acceleration_rad_s2{0.6};
  double yaw_stop_deceleration_rad_s2{0.9};
  double stable_hold_dwell_s{0.25};
};

enum class GaitMode : std::uint8_t
{
  kHolding,
  kStarting,
  kWalking,
  kStopping,
};

struct TripodState
{
  double phase{0.0};
  std::uint64_t cycle{0};
  PlanarVelocity velocity{};
  bool walking{false};
  bool stopping{false};
  double hold_dwell_s{0.0};
  double cadence_hz{0.0};
  double maximum_stride_scale{0.0};
  double maximum_clearance_m{0.0};
  double applied_velocity_scale{0.0};
};

struct TripodStep
{
  bool valid{false};
  TripodState state{};
  GaitMode mode{GaitMode::kHolding};
  std::array<araco_kinematics::Point3, kLegCount> foot_targets_base_m{};
  std::array<bool, kLegCount> swing{};
};

[[nodiscard]] TripodStep advance_tripod(
  const TripodConfig & config,
  const TripodState & previous,
  const std::array<araco_kinematics::Point3, kLegCount> & nominal_feet,
  const PlanarVelocity & requested_velocity,
  bool request_walk,
  bool request_stop,
  double dt_s);

// Freeze phase and reduce gait amplitude toward the nominal six-foot stance.
// This is the deterministic inward recovery used when the normal curve cannot
// take another reachable step at a workspace boundary.
[[nodiscard]] TripodStep retreat_tripod_to_nominal(
  const TripodConfig & config,
  const TripodState & previous,
  const std::array<araco_kinematics::Point3, kLegCount> & nominal_feet,
  double dt_s);

}  // namespace araco_locomotion

#endif  // ARACO_LOCOMOTION__TRIPOD_GAIT_HPP_
