// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#ifndef ARACO_SUPERVISION__COMMAND_POLICY_HPP_
#define ARACO_SUPERVISION__COMMAND_POLICY_HPP_

#include <array>
#include <cstdint>
#include <string>

namespace araco_supervision
{

struct Quaternion
{
  double x;
  double y;
  double z;
  double w;
};

struct StaticIntent
{
  std::uint8_t gait;
  std::array<double, 6> twist;
  std::array<double, 3> position_m;
  Quaternion orientation;
};

struct BodyEnvelope
{
  double planar_speed_normal_m_s;
  double planar_speed_hard_m_s;
  double yaw_rate_normal_rad_s;
  double yaw_rate_hard_rad_s;
  double xy_normal_m;
  double z_normal_lower_m;
  double z_normal_upper_m;
  double roll_pitch_normal_rad;
  double yaw_normal_rad;
  double xy_hard_m;
  double z_hard_lower_m;
  double z_hard_upper_m;
  double roll_pitch_hard_rad;
  double yaw_hard_rad;
  double quaternion_norm_tolerance;
  double reserved_twist_tolerance;
  double stand_velocity_tolerance;
};

enum class CommandValidity : std::uint8_t
{
  kValid = 0,
  kLimited = 1,
  kInvalid = 2,
};

struct PolicyResult
{
  CommandValidity validity{CommandValidity::kInvalid};
  StaticIntent intent{};
  std::array<double, 3> rpy_rad{};
};

[[nodiscard]] PolicyResult validate_and_limit_static_intent(
  const StaticIntent & intent,
  const std::string & frame_id,
  const BodyEnvelope & envelope,
  bool apply_normal_limits);

[[nodiscard]] Quaternion quaternion_from_rpy(double roll, double pitch, double yaw);

}  // namespace araco_supervision

#endif  // ARACO_SUPERVISION__COMMAND_POLICY_HPP_
