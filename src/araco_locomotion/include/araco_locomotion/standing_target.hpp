// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#ifndef ARACO_LOCOMOTION__STANDING_TARGET_HPP_
#define ARACO_LOCOMOTION__STANDING_TARGET_HPP_

#include <array>
#include <cstdint>

#include "araco_kinematics/leg_kinematics.hpp"

namespace araco_locomotion
{

constexpr std::size_t kLegCount = 6;
constexpr std::size_t kJointsPerLeg = 4;
constexpr std::size_t kLegJointCount = kLegCount * kJointsPerLeg;

enum class LegStatus : std::uint8_t
{
  kValid = 0,
  kNearLimit = 1,
  kUnreachable = 2,
  kInvalid = 3,
};

struct LegMount
{
  araco_kinematics::Point3 position_base_m;
  double yaw_base_rad;
};

struct StandingRequest
{
  araco_kinematics::LegGeometry geometry;
  std::array<araco_kinematics::JointLimits, kLegCount> limits;
  araco_kinematics::SolverOptions solver_options;
  araco_kinematics::Branch branch;
  std::array<LegMount, kLegCount> mounts;
  std::array<araco_kinematics::Point3, kLegCount> foot_targets_base_m;
  std::array<double, kLegCount> foot_pitch_rad;
  std::array<double, kLegJointCount> oracle_joints_rad;
  double oracle_tolerance_rad;
};

struct BodyPoseOffset
{
  araco_kinematics::Point3 translation_m;
  double roll_rad;
  double pitch_rad;
  double yaw_rad;
};

struct FootPitchProjection
{
  bool valid{false};
  double foot_pitch_rad{0.0};
  double unavoidable_residual_rad{0.0};
};

struct StandingResult
{
  bool committed{false};
  std::array<double, kLegJointCount> joints_rad{};
  std::array<LegStatus, kLegCount> leg_status{};
  std::array<araco_kinematics::Point3, kLegCount> foot_targets_body_m{};
  std::array<double, kLegCount> foot_pitch_target_rad{};
  std::array<double, kLegCount> foot_orientation_residual_rad{};
  double maximum_foot_orientation_residual_rad{0.0};
  double maximum_oracle_error_rad{0.0};
};

[[nodiscard]] FootPitchProjection project_ground_vertical_foot_pitch(
  const araco_kinematics::Point3 & foot_target_body_m,
  const BodyPoseOffset & body_pose,
  const LegMount & mount);

[[nodiscard]] StandingResult compute_standing_target(
  const StandingRequest & request,
  const std::array<double, kLegJointCount> & previous_commit);

[[nodiscard]] StandingResult compute_body_pose_target(
  const StandingRequest & request,
  const BodyPoseOffset & body_pose,
  const std::array<double, kLegJointCount> & previous_commit);

[[nodiscard]] StandingResult compute_foot_pose_target(
  const StandingRequest & request,
  const std::array<araco_kinematics::Point3, kLegCount> & foot_targets_base_m,
  const BodyPoseOffset & body_pose,
  const std::array<double, kLegJointCount> & previous_commit);

}  // namespace araco_locomotion

#endif  // ARACO_LOCOMOTION__STANDING_TARGET_HPP_
