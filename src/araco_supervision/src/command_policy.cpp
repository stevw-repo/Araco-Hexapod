// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include "araco_supervision/command_policy.hpp"

#include <algorithm>
#include <cmath>

namespace araco_supervision
{
namespace
{

constexpr double kNormalBoundaryTolerance = 1.0e-12;

bool finite_intent(const StaticIntent & intent)
{
  const std::array<double, 14> values{
    intent.twist[0], intent.twist[1], intent.twist[2],
    intent.twist[3], intent.twist[4], intent.twist[5],
    intent.position_m[0], intent.position_m[1], intent.position_m[2],
    intent.orientation.x, intent.orientation.y,
    intent.orientation.z, intent.orientation.w, intent.gimbal_yaw_rad,
  };
  return std::all_of(values.begin(), values.end(), [](double value) {
             return std::isfinite(value);
    });
}

std::array<double, 3> rpy_from_quaternion(const Quaternion & q)
{
  const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  const double pitch = std::abs(sinp) >= 1.0 ?
    std::copysign(1.5707963267948966, sinp) : std::asin(sinp);
  return {
    std::atan2(sinr_cosp, cosr_cosp),
    pitch,
    std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
  };
}

bool outside(double value, double lower, double upper)
{
  return value < lower || value > upper;
}

}  // namespace

Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  return {
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy,
  };
}

PolicyResult validate_and_limit_static_intent(
  const StaticIntent & intent,
  const std::string & frame_id,
  const BodyEnvelope & envelope,
  bool apply_normal_limits)
{
  PolicyResult result;
  result.intent = intent;
  if (frame_id != "base_link" || intent.gait > 1 || !finite_intent(intent)) {
    return result;
  }
  const double norm = std::sqrt(
    intent.orientation.x * intent.orientation.x +
    intent.orientation.y * intent.orientation.y +
    intent.orientation.z * intent.orientation.z +
    intent.orientation.w * intent.orientation.w);
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > envelope.quaternion_norm_tolerance) {
    return result;
  }
  if (std::abs(intent.twist[2]) > envelope.reserved_twist_tolerance ||
    std::abs(intent.twist[3]) > envelope.reserved_twist_tolerance ||
    std::abs(intent.twist[4]) > envelope.reserved_twist_tolerance)
  {
    return result;
  }
  const double planar_speed = std::hypot(intent.twist[0], intent.twist[1]);
  if ((intent.gait == 0 &&
    (planar_speed > envelope.stand_velocity_tolerance ||
    std::abs(intent.twist[5]) > envelope.stand_velocity_tolerance)) ||
    (intent.gait == 1 &&
    (planar_speed > envelope.planar_speed_hard_m_s ||
    std::abs(intent.twist[5]) > envelope.yaw_rate_hard_rad_s)))
  {
    return result;
  }
  result.rpy_rad = rpy_from_quaternion(intent.orientation);
  if (std::abs(intent.position_m[0]) > envelope.xy_hard_m ||
    std::abs(intent.position_m[1]) > envelope.xy_hard_m ||
    outside(intent.position_m[2], envelope.z_hard_lower_m, envelope.z_hard_upper_m) ||
    std::abs(result.rpy_rad[0]) > envelope.roll_pitch_hard_rad ||
    std::abs(result.rpy_rad[1]) > envelope.roll_pitch_hard_rad ||
    std::abs(result.rpy_rad[2]) > envelope.yaw_hard_rad ||
    std::abs(intent.gimbal_yaw_rad) > envelope.gimbal_yaw_hard_rad)
  {
    return result;
  }

  result.validity = CommandValidity::kValid;
  if (!apply_normal_limits) {
    return result;
  }
  auto clamp = [&result](double value, double lower, double upper) {
      const double limited = std::clamp(value, lower, upper);
      if (std::abs(limited - value) > kNormalBoundaryTolerance) {
        result.validity = CommandValidity::kLimited;
      }
      return limited;
    };
  if (intent.gait == 1 && planar_speed > envelope.planar_speed_normal_m_s) {
    const double scale = envelope.planar_speed_normal_m_s / planar_speed;
    result.intent.twist[0] *= scale;
    result.intent.twist[1] *= scale;
    if (planar_speed - envelope.planar_speed_normal_m_s >
      kNormalBoundaryTolerance)
    {
      result.validity = CommandValidity::kLimited;
    }
  }
  result.intent.twist[5] = clamp(
    intent.twist[5], -envelope.yaw_rate_normal_rad_s,
    envelope.yaw_rate_normal_rad_s);
  result.intent.position_m[0] = clamp(
    intent.position_m[0], -envelope.xy_normal_m, envelope.xy_normal_m);
  result.intent.position_m[1] = clamp(
    intent.position_m[1], -envelope.xy_normal_m, envelope.xy_normal_m);
  result.intent.position_m[2] = clamp(
    intent.position_m[2], envelope.z_normal_lower_m, envelope.z_normal_upper_m);
  for (std::size_t index = 0; index < 2; ++index) {
    result.rpy_rad[index] = clamp(
      result.rpy_rad[index], -envelope.roll_pitch_normal_rad,
      envelope.roll_pitch_normal_rad);
  }
  result.rpy_rad[2] = clamp(
    result.rpy_rad[2], -envelope.yaw_normal_rad, envelope.yaw_normal_rad);
  result.intent.gimbal_yaw_rad = clamp(
    result.intent.gimbal_yaw_rad, -envelope.gimbal_yaw_normal_rad,
    envelope.gimbal_yaw_normal_rad);
  result.intent.orientation = quaternion_from_rpy(
    result.rpy_rad[0], result.rpy_rad[1], result.rpy_rad[2]);
  return result;
}

}  // namespace araco_supervision
