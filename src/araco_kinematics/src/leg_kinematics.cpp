// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include "araco_kinematics/leg_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace araco_kinematics
{
namespace
{

bool finite(double value)
{
  return std::isfinite(value);
}

bool finite(const Point3 & point)
{
  return finite(point.x) && finite(point.y) && finite(point.z);
}

bool finite(const JointVector & joints)
{
  return std::all_of(joints.begin(), joints.end(), [](double value) {return finite(value);});
}

double distance(const Point3 & left, const Point3 & right)
{
  return std::hypot(std::hypot(left.x - right.x, left.y - right.y), left.z - right.z);
}

double angle_error(double left, double right)
{
  return std::remainder(left - right, 2.0 * std::acos(-1.0));
}

}  // namespace

LegKinematics::LegKinematics(
  LegGeometry geometry, JointLimits limits, SolverOptions options)
: geometry_(geometry), limits_(limits), options_(options)
{
  const std::array<double, 4> lengths{
    geometry_.coxa_m, geometry_.femur_m, geometry_.tibia_m, geometry_.foot_m};
  valid_configuration_ =
    std::all_of(lengths.begin(), lengths.end(), [](double value) {
        return finite(value) && value > 0.0;
    }) &&
    std::all_of(limits_.begin(), limits_.end(), [](const JointLimit & limit) {
        return finite(limit.lower_rad) && finite(limit.upper_rad) &&
               limit.lower_rad < limit.upper_rad;
    }) &&
    finite(options_.position_tolerance_m) && options_.position_tolerance_m > 0.0 &&
    finite(options_.singularity_threshold) && options_.singularity_threshold >= 0.0 &&
    finite(options_.near_limit_margin_rad) && options_.near_limit_margin_rad >= 0.0;
}

bool LegKinematics::valid_configuration() const
{
  return valid_configuration_;
}

ForwardResult LegKinematics::forward(const JointVector & joints_rad) const
{
  ForwardResult result;
  if (!valid_configuration_ || !finite(joints_rad)) {
    return result;
  }
  for (std::size_t index = 0; index < joints_rad.size(); ++index) {
    if (joints_rad[index] < limits_[index].lower_rad ||
      joints_rad[index] > limits_[index].upper_rad)
    {
      result.status = Status::kJointLimit;
      return result;
    }
  }

  const double pitch_1 = joints_rad[1];
  const double pitch_2 = pitch_1 + joints_rad[2];
  result.foot_pitch_rad = pitch_2 + joints_rad[3];
  const double radial = geometry_.coxa_m +
    geometry_.femur_m * std::cos(pitch_1) +
    geometry_.tibia_m * std::cos(pitch_2) +
    geometry_.foot_m * std::cos(result.foot_pitch_rad);
  result.foot_position_m = {
    radial * std::cos(joints_rad[0]),
    radial * std::sin(joints_rad[0]),
    geometry_.femur_m * std::sin(pitch_1) +
    geometry_.tibia_m * std::sin(pitch_2) +
    geometry_.foot_m * std::sin(result.foot_pitch_rad),
  };
  result.status = Status::kValid;
  return result;
}

InverseResult LegKinematics::inverse(
  const Point3 & foot_position_m,
  double foot_pitch_rad,
  Branch branch) const
{
  InverseResult result;
  if (!valid_configuration_ || !finite(foot_position_m) || !finite(foot_pitch_rad)) {
    return result;
  }

  const double radial = std::hypot(foot_position_m.x, foot_position_m.y);
  if (radial <= std::numeric_limits<double>::epsilon()) {
    result.status = Status::kSingular;
    return result;
  }
  const double wrist_radial = radial - geometry_.coxa_m -
    geometry_.foot_m * std::cos(foot_pitch_rad);
  const double wrist_z = foot_position_m.z -
    geometry_.foot_m * std::sin(foot_pitch_rad);
  const double denominator = 2.0 * geometry_.femur_m * geometry_.tibia_m;
  const double cosine_knee =
    (wrist_radial * wrist_radial + wrist_z * wrist_z -
    geometry_.femur_m * geometry_.femur_m -
    geometry_.tibia_m * geometry_.tibia_m) / denominator;
  if (cosine_knee < -1.0 - options_.position_tolerance_m ||
    cosine_knee > 1.0 + options_.position_tolerance_m)
  {
    result.status = Status::kUnreachable;
    return result;
  }

  const double knee_magnitude = std::acos(std::clamp(cosine_knee, -1.0, 1.0));
  const double knee = branch == Branch::kKneeDown ? -knee_magnitude : knee_magnitude;
  if (std::abs(std::sin(knee)) <= options_.singularity_threshold) {
    result.status = Status::kSingular;
    return result;
  }
  const double femur = std::atan2(wrist_z, wrist_radial) -
    std::atan2(
    geometry_.tibia_m * std::sin(knee),
    geometry_.femur_m + geometry_.tibia_m * std::cos(knee));
  result.joints_rad = {
    std::atan2(foot_position_m.y, foot_position_m.x),
    femur,
    knee,
    foot_pitch_rad - femur - knee,
  };
  if (!finite(result.joints_rad)) {
    result.status = Status::kInvalidInput;
    return result;
  }

  bool near_limit = false;
  for (std::size_t index = 0; index < result.joints_rad.size(); ++index) {
    const auto & limit = limits_[index];
    const double value = result.joints_rad[index];
    if (value < limit.lower_rad || value > limit.upper_rad) {
      result.status = Status::kJointLimit;
      return result;
    }
    near_limit = near_limit || value - limit.lower_rad <= options_.near_limit_margin_rad ||
      limit.upper_rad - value <= options_.near_limit_margin_rad;
  }

  const ForwardResult check = forward(result.joints_rad);
  if (check.status != Status::kValid) {
    result.status = check.status;
    return result;
  }
  result.position_error_m = distance(check.foot_position_m, foot_position_m);
  result.pitch_error_rad = std::abs(angle_error(check.foot_pitch_rad, foot_pitch_rad));
  if (result.position_error_m > options_.position_tolerance_m ||
    result.pitch_error_rad > options_.position_tolerance_m)
  {
    result.status = Status::kInvalidInput;
    return result;
  }
  result.status = near_limit ? Status::kNearLimit : Status::kValid;
  return result;
}

}  // namespace araco_kinematics
