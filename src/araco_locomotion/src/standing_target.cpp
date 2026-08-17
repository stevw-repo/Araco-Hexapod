// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include "araco_locomotion/standing_target.hpp"

#include <algorithm>
#include <cmath>

namespace araco_locomotion
{
namespace
{

LegStatus status_for(araco_kinematics::Status status)
{
  switch (status) {
    case araco_kinematics::Status::kValid:
      return LegStatus::kValid;
    case araco_kinematics::Status::kNearLimit:
      return LegStatus::kNearLimit;
    case araco_kinematics::Status::kUnreachable:
      return LegStatus::kUnreachable;
    default:
      return LegStatus::kInvalid;
  }
}

araco_kinematics::Point3 target_in_mount(
  const araco_kinematics::Point3 & target,
  const LegMount & mount)
{
  const double dx = target.x - mount.position_base_m.x;
  const double dy = target.y - mount.position_base_m.y;
  const double cosine = std::cos(mount.yaw_base_rad);
  const double sine = std::sin(mount.yaw_base_rad);
  return {
    cosine * dx + sine * dy,
    -sine * dx + cosine * dy,
    target.z - mount.position_base_m.z,
  };
}

bool finite_pose(const BodyPoseOffset & pose)
{
  return std::isfinite(pose.translation_m.x) &&
         std::isfinite(pose.translation_m.y) &&
         std::isfinite(pose.translation_m.z) &&
         std::isfinite(pose.roll_rad) &&
         std::isfinite(pose.pitch_rad) &&
         std::isfinite(pose.yaw_rad);
}

araco_kinematics::Point3 target_in_moved_body(
  const araco_kinematics::Point3 & nominal_target,
  const BodyPoseOffset & pose)
{
  const double x = nominal_target.x - pose.translation_m.x;
  const double y = nominal_target.y - pose.translation_m.y;
  const double z = nominal_target.z - pose.translation_m.z;
  const double cr = std::cos(pose.roll_rad);
  const double sr = std::sin(pose.roll_rad);
  const double cp = std::cos(pose.pitch_rad);
  const double sp = std::sin(pose.pitch_rad);
  const double cy = std::cos(pose.yaw_rad);
  const double sy = std::sin(pose.yaw_rad);

  // R = Rz(yaw) * Ry(pitch) * Rx(roll). Fixed world feet expressed in the
  // moved body are R^T * (p_world - body_translation).
  return {
    cy * cp * x + sy * cp * y - sp * z,
    (cy * sp * sr - sy * cr) * x +
    (sy * sp * sr + cy * cr) * y + cp * sr * z,
    (cy * sp * cr + sy * sr) * x +
    (sy * sp * cr - cy * sr) * y + cp * cr * z,
  };
}

StandingResult solve_targets(
  const StandingRequest & request,
  const std::array<araco_kinematics::Point3, kLegCount> & targets,
  const std::array<double, kLegCount> & foot_pitches,
  const std::array<double, kLegCount> & orientation_residuals,
  const std::array<double, kLegJointCount> & previous_commit,
  bool enforce_oracle)
{
  StandingResult result;
  result.joints_rad = previous_commit;
  result.foot_targets_body_m = targets;
  result.foot_pitch_target_rad = foot_pitches;
  result.foot_orientation_residual_rad = orientation_residuals;
  std::array<double, kLegJointCount> candidate{};
  bool valid = std::isfinite(request.oracle_tolerance_rad) &&
    request.oracle_tolerance_rad >= 0.0;

  for (std::size_t leg = 0; leg < kLegCount; ++leg) {
    result.maximum_foot_orientation_residual_rad = std::max(
      result.maximum_foot_orientation_residual_rad, orientation_residuals[leg]);
    if (!std::isfinite(foot_pitches[leg]) ||
      !std::isfinite(orientation_residuals[leg]) || orientation_residuals[leg] < 0.0)
    {
      result.leg_status[leg] = LegStatus::kInvalid;
      valid = false;
      continue;
    }
    const araco_kinematics::LegKinematics solver(
      request.geometry, request.limits[leg], request.solver_options);
    const auto solved = solver.inverse(
      target_in_mount(targets[leg], request.mounts[leg]),
      foot_pitches[leg], request.branch);
    result.leg_status[leg] = status_for(solved.status);
    const bool leg_valid = solved.status == araco_kinematics::Status::kValid ||
      solved.status == araco_kinematics::Status::kNearLimit;
    if (!leg_valid) {
      valid = false;
      continue;
    }
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const std::size_t index = leg * kJointsPerLeg + joint;
      candidate[index] = solved.joints_rad[joint];
      if (enforce_oracle) {
        const double oracle_error =
          std::abs(candidate[index] - request.oracle_joints_rad[index]);
        result.maximum_oracle_error_rad = std::max(
          result.maximum_oracle_error_rad, oracle_error);
        if (!std::isfinite(oracle_error) || oracle_error > request.oracle_tolerance_rad) {
          result.leg_status[leg] = LegStatus::kInvalid;
          valid = false;
        }
      }
    }
  }

  if (valid) {
    result.joints_rad = candidate;
    result.committed = true;
  }
  return result;
}

}  // namespace

FootPitchProjection project_ground_vertical_foot_pitch(
  const araco_kinematics::Point3 & foot_target_body_m,
  const BodyPoseOffset & body_pose,
  const LegMount & mount)
{
  FootPitchProjection projection;
  if (!finite_pose(body_pose) ||
    !std::isfinite(foot_target_body_m.x) ||
    !std::isfinite(foot_target_body_m.y) ||
    !std::isfinite(foot_target_body_m.z) ||
    !std::isfinite(mount.position_base_m.x) ||
    !std::isfinite(mount.position_base_m.y) ||
    !std::isfinite(mount.position_base_m.z) ||
    !std::isfinite(mount.yaw_base_rad))
  {
    return projection;
  }

  // R = Rz(yaw) * Ry(pitch) * Rx(roll). World-down expressed in the moved
  // body is R^T * [0, 0, -1]. Body yaw cancels because it rotates around the
  // same world vertical axis.
  const double cr = std::cos(body_pose.roll_rad);
  const double sr = std::sin(body_pose.roll_rad);
  const double cp = std::cos(body_pose.pitch_rad);
  const double sp = std::sin(body_pose.pitch_rad);
  const araco_kinematics::Point3 down_body{sp, -cp * sr, -cp * cr};

  const auto target_mount = target_in_mount(foot_target_body_m, mount);
  const double mount_cosine = std::cos(mount.yaw_base_rad);
  const double mount_sine = std::sin(mount.yaw_base_rad);
  const double down_mount_x =
    mount_cosine * down_body.x + mount_sine * down_body.y;
  const double down_mount_y =
    -mount_sine * down_body.x + mount_cosine * down_body.y;
  const double coxa_yaw = std::atan2(target_mount.y, target_mount.x);
  const double coxa_cosine = std::cos(coxa_yaw);
  const double coxa_sine = std::sin(coxa_yaw);

  // The three pitch joints can orient the foot only in the sagittal plane
  // selected by coxa yaw. radial/z are the realizable projection; lateral is
  // the unavoidable component that this four-DOF chain cannot control.
  const double radial =
    coxa_cosine * down_mount_x + coxa_sine * down_mount_y;
  const double lateral =
    -coxa_sine * down_mount_x + coxa_cosine * down_mount_y;
  const double projection_norm = std::hypot(radial, down_body.z);
  constexpr double kProjectionEpsilon = 1.0e-12;
  if (!std::isfinite(projection_norm) || projection_norm <= kProjectionEpsilon) {
    return projection;
  }

  projection.foot_pitch_rad = std::atan2(down_body.z, radial);
  projection.unavoidable_residual_rad = std::asin(
    std::clamp(std::abs(lateral), 0.0, 1.0));
  projection.valid = std::isfinite(projection.foot_pitch_rad) &&
    std::isfinite(projection.unavoidable_residual_rad);
  return projection;
}

StandingResult compute_standing_target(
  const StandingRequest & request,
  const std::array<double, kLegJointCount> & previous_commit)
{
  std::array<double, kLegCount> residuals{};
  return solve_targets(
    request, request.foot_targets_base_m, request.foot_pitch_rad, residuals,
    previous_commit, true);
}

StandingResult compute_body_pose_target(
  const StandingRequest & request,
  const BodyPoseOffset & body_pose,
  const std::array<double, kLegJointCount> & previous_commit)
{
  if (!finite_pose(body_pose)) {
    StandingResult result;
    result.joints_rad = previous_commit;
    result.leg_status.fill(LegStatus::kInvalid);
    return result;
  }
  std::array<araco_kinematics::Point3, kLegCount> targets{};
  std::array<double, kLegCount> foot_pitches{};
  std::array<double, kLegCount> orientation_residuals{};
  for (std::size_t leg = 0; leg < kLegCount; ++leg) {
    targets[leg] = target_in_moved_body(request.foot_targets_base_m[leg], body_pose);
    const auto projection = project_ground_vertical_foot_pitch(
      targets[leg], body_pose, request.mounts[leg]);
    if (!projection.valid) {
      StandingResult result;
      result.joints_rad = previous_commit;
      result.foot_targets_body_m = targets;
      result.leg_status.fill(LegStatus::kInvalid);
      return result;
    }
    foot_pitches[leg] = projection.foot_pitch_rad;
    orientation_residuals[leg] = projection.unavoidable_residual_rad;
  }
  return solve_targets(
    request, targets, foot_pitches, orientation_residuals, previous_commit, false);
}

StandingResult compute_foot_pose_target(
  const StandingRequest & request,
  const std::array<araco_kinematics::Point3, kLegCount> & foot_targets_base_m,
  const BodyPoseOffset & body_pose,
  const std::array<double, kLegJointCount> & previous_commit)
{
  if (!finite_pose(body_pose)) {
    StandingResult result;
    result.joints_rad = previous_commit;
    result.leg_status.fill(LegStatus::kInvalid);
    return result;
  }
  std::array<araco_kinematics::Point3, kLegCount> targets{};
  std::array<double, kLegCount> foot_pitches{};
  std::array<double, kLegCount> orientation_residuals{};
  for (std::size_t leg = 0; leg < kLegCount; ++leg) {
    targets[leg] = target_in_moved_body(foot_targets_base_m[leg], body_pose);
    const auto projection = project_ground_vertical_foot_pitch(
      targets[leg], body_pose, request.mounts[leg]);
    if (!projection.valid) {
      StandingResult result;
      result.joints_rad = previous_commit;
      result.foot_targets_body_m = targets;
      result.leg_status.fill(LegStatus::kInvalid);
      return result;
    }
    foot_pitches[leg] = projection.foot_pitch_rad;
    orientation_residuals[leg] = projection.unavoidable_residual_rad;
  }
  return solve_targets(
    request, targets, foot_pitches, orientation_residuals, previous_commit, false);
}

}  // namespace araco_locomotion
