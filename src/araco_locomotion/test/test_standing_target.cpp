// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include "araco_kinematics/leg_kinematics.hpp"
#include "araco_locomotion/standing_target.hpp"
#include "araco_locomotion/tripod_gait.hpp"

namespace
{

using araco_locomotion::LegMount;
using araco_locomotion::LegStatus;
using araco_locomotion::BodyPoseOffset;
using araco_locomotion::StandingRequest;
using araco_locomotion::compute_body_pose_target;
using araco_locomotion::compute_foot_pose_target;
using araco_locomotion::compute_standing_target;
using araco_locomotion::project_ground_vertical_foot_pitch;
using araco_locomotion::advance_tripod;
using araco_locomotion::PlanarVelocity;
using araco_locomotion::TripodConfig;
using araco_locomotion::TripodState;
using araco_kinematics::Branch;
using araco_kinematics::JointLimit;
using araco_kinematics::JointLimits;
using araco_kinematics::JointVector;
using araco_kinematics::LegGeometry;
using araco_kinematics::LegKinematics;
using araco_kinematics::Point3;
using araco_kinematics::SolverOptions;

constexpr double kPi = 3.14159265358979323846;

Point3 rotate_body_to_world(const Point3 & vector, const BodyPoseOffset & pose)
{
  const double cr = std::cos(pose.roll_rad);
  const double sr = std::sin(pose.roll_rad);
  const double cp = std::cos(pose.pitch_rad);
  const double sp = std::sin(pose.pitch_rad);
  const double cy = std::cos(pose.yaw_rad);
  const double sy = std::sin(pose.yaw_rad);
  return {
    cy * cp * vector.x + (cy * sp * sr - sy * cr) * vector.y +
    (cy * sp * cr + sy * sr) * vector.z,
    sy * cp * vector.x + (sy * sp * sr + cy * cr) * vector.y +
    (sy * sp * cr - cy * sr) * vector.z,
    -sp * vector.x + cp * sr * vector.y + cp * cr * vector.z,
  };
}

double angle_to_world_down(const Point3 & unit_vector)
{
  return std::acos(std::clamp(-unit_vector.z, -1.0, 1.0));
}

StandingRequest nominal_request()
{
  StandingRequest request;
  request.geometry = LegGeometry{0.043, 0.12, 0.12, 0.05};
  request.solver_options = SolverOptions{1.0e-9, 1.0e-6, 0.05};
  request.branch = Branch::kKneeDown;
  request.oracle_tolerance_rad = 2.0e-6;
  const JointLimits limits{{
    JointLimit{-0.7, 0.7}, JointLimit{0.15, 1.35},
    JointLimit{-2.65, -0.75}, JointLimit{-1.25, 0.35}}};
  request.limits.fill(limits);
  request.mounts = {{
    LegMount{Point3{0.081819805, 0.066819805, -0.02675}, 0.684823427809552},
    LegMount{Point3{0.0, 0.09, -0.02675}, kPi / 2.0},
    LegMount{Point3{-0.081819805, 0.066819805, -0.02675}, 2.456769225780241},
    LegMount{Point3{0.081819805, -0.066819805, -0.02675}, -0.684823427809552},
    LegMount{Point3{0.0, -0.09, -0.02675}, -kPi / 2.0},
    LegMount{Point3{-0.081819805, -0.066819805, -0.02675}, -2.456769225780241},
  }};
  const std::array<JointVector, 6> oracle{{
    {{0.166907, 0.749681, -1.947935, -0.372542}},
    {{0.0, 0.694188, -1.791009, -0.473975}},
    {{-0.166907, 0.749681, -1.947935, -0.372542}},
    {{-0.166907, 0.749681, -1.947935, -0.372542}},
    {{0.0, 0.694188, -1.791009, -0.473975}},
    {{0.166907, 0.749681, -1.947935, -0.372542}},
  }};
  request.foot_pitch_rad.fill(-kPi / 2.0);
  for (std::size_t leg = 0; leg < oracle.size(); ++leg) {
    const LegKinematics solver(request.geometry, limits, request.solver_options);
    auto target_joints = oracle[leg];
    target_joints[3] = request.foot_pitch_rad[leg] - target_joints[1] - target_joints[2];
    const auto local = solver.forward(target_joints);
    const double cosine = std::cos(request.mounts[leg].yaw_base_rad);
    const double sine = std::sin(request.mounts[leg].yaw_base_rad);
    request.foot_targets_base_m[leg] = {
      request.mounts[leg].position_base_m.x +
      cosine * local.foot_position_m.x - sine * local.foot_position_m.y,
      request.mounts[leg].position_base_m.y +
      sine * local.foot_position_m.x + cosine * local.foot_position_m.y,
      request.mounts[leg].position_base_m.z + local.foot_position_m.z,
    };
    for (std::size_t joint = 0; joint < oracle[leg].size(); ++joint) {
      request.oracle_joints_rad[leg * 4 + joint] = oracle[leg][joint];
    }
  }
  return request;
}

TEST(StandingTarget, ConstructsWholeBodyTransformsAndCompleteOrderedTrajectory)
{
  const auto request = nominal_request();
  std::array<double, 24> previous{};
  previous.fill(9.0);
  const auto result = compute_standing_target(request, previous);
  ASSERT_TRUE(result.committed);
  EXPECT_LT(result.maximum_oracle_error_rad, request.oracle_tolerance_rad);
  for (std::size_t index = 0; index < result.joints_rad.size(); ++index) {
    EXPECT_NEAR(
      result.joints_rad[index], request.oracle_joints_rad[index],
      request.oracle_tolerance_rad);
  }
  for (const auto status : result.leg_status) {
    EXPECT_EQ(status, LegStatus::kValid);
  }
}

TEST(StandingTarget, OneFailedLegRejectsTransactionWithoutPartialCommit)
{
  auto request = nominal_request();
  request.foot_targets_base_m[3].x = 10.0;
  std::array<double, 24> previous{};
  for (std::size_t index = 0; index < previous.size(); ++index) {
    previous[index] = 100.0 + static_cast<double>(index);
  }
  const auto result = compute_standing_target(request, previous);
  EXPECT_FALSE(result.committed);
  EXPECT_EQ(result.joints_rad, previous);
  EXPECT_EQ(result.leg_status[3], LegStatus::kUnreachable);
  EXPECT_EQ(result.leg_status[0], LegStatus::kValid);
}

TEST(StandingTarget, OracleMismatchRejectsOtherwiseReachableCandidate)
{
  auto request = nominal_request();
  request.oracle_joints_rad[8] += 0.01;
  const std::array<double, 24> previous{};
  const auto result = compute_standing_target(request, previous);
  EXPECT_FALSE(result.committed);
  EXPECT_EQ(result.joints_rad, previous);
  EXPECT_EQ(result.leg_status[2], LegStatus::kInvalid);
  EXPECT_GT(result.maximum_oracle_error_rad, request.oracle_tolerance_rad);
}

TEST(StandingTarget, AbsoluteBodyTranslationKeepsWorldFeetFixed)
{
  const auto request = nominal_request();
  const auto standing = compute_standing_target(request, {});
  ASSERT_TRUE(standing.committed);
  const BodyPoseOffset pose{{0.01, -0.005, 0.008}, 0.0, 0.0, 0.0};
  const auto moved = compute_body_pose_target(request, pose, standing.joints_rad);
  ASSERT_TRUE(moved.committed);
  for (std::size_t leg = 0; leg < 6; ++leg) {
    EXPECT_NEAR(
      moved.foot_targets_body_m[leg].x,
      request.foot_targets_base_m[leg].x - pose.translation_m.x, 1.0e-12);
    EXPECT_NEAR(
      moved.foot_targets_body_m[leg].y,
      request.foot_targets_base_m[leg].y - pose.translation_m.y, 1.0e-12);
    EXPECT_NEAR(
      moved.foot_targets_body_m[leg].z,
      request.foot_targets_base_m[leg].z - pose.translation_m.z, 1.0e-12);
  }
}

TEST(StandingTarget, RotationUsesInverseBodyTransformWithCorrectSigns)
{
  const auto request = nominal_request();
  const auto standing = compute_standing_target(request, {});
  ASSERT_TRUE(standing.committed);
  const BodyPoseOffset yaw{{0.0, 0.0, 0.0}, 0.0, 0.0, 0.1};
  const auto moved = compute_body_pose_target(request, yaw, standing.joints_rad);
  ASSERT_TRUE(moved.committed);
  const double cosine = std::cos(yaw.yaw_rad);
  const double sine = std::sin(yaw.yaw_rad);
  EXPECT_NEAR(
    moved.foot_targets_body_m[0].x,
    cosine * request.foot_targets_base_m[0].x +
    sine * request.foot_targets_base_m[0].y, 1.0e-12);
  EXPECT_NEAR(
    moved.foot_targets_body_m[0].y,
    -sine * request.foot_targets_base_m[0].x +
    cosine * request.foot_targets_base_m[0].y, 1.0e-12);
}

TEST(StandingTarget, LevelAndYawOnlyKeepNominalGroundVerticalPitch)
{
  const auto request = nominal_request();
  for (const double yaw : {0.0, -0.1, 0.1}) {
    const BodyPoseOffset pose{{0.0, 0.0, 0.0}, 0.0, 0.0, yaw};
    const auto result = compute_body_pose_target(request, pose, {});
    ASSERT_TRUE(result.committed);
    for (std::size_t leg = 0; leg < 6; ++leg) {
      EXPECT_NEAR(result.foot_pitch_target_rad[leg], -kPi / 2.0, 1.0e-12);
      EXPECT_NEAR(result.foot_orientation_residual_rad[leg], 0.0, 1.0e-12);
    }
    EXPECT_NEAR(result.maximum_foot_orientation_residual_rad, 0.0, 1.0e-12);
  }
}

TEST(StandingTarget, TiltUsesClosestPerLegGroundVerticalProjection)
{
  const auto request = nominal_request();
  const BodyPoseOffset pose{{0.0, 0.0, 0.0}, 0.075, -0.06, 0.1};
  const auto result = compute_body_pose_target(request, pose, {});
  ASSERT_TRUE(result.committed);

  const double fixed_pitch_world_error = std::acos(
    std::clamp(std::cos(pose.roll_rad) * std::cos(pose.pitch_rad), -1.0, 1.0));
  bool pitch_varies_by_leg = false;
  bool projection_strictly_improves_one_leg = false;
  double measured_maximum_residual = 0.0;
  for (std::size_t leg = 0; leg < 6; ++leg) {
    const double coxa = result.joints_rad[leg * 4];
    const double pitch = result.joints_rad[leg * 4 + 1] +
      result.joints_rad[leg * 4 + 2] + result.joints_rad[leg * 4 + 3];
    EXPECT_NEAR(pitch, result.foot_pitch_target_rad[leg], 1.0e-9);

    const double azimuth = request.mounts[leg].yaw_base_rad + coxa;
    const Point3 foot_axis_body{
      std::cos(azimuth) * std::cos(pitch),
      std::sin(azimuth) * std::cos(pitch),
      std::sin(pitch),
    };
    const double world_error = angle_to_world_down(
      rotate_body_to_world(foot_axis_body, pose));
    EXPECT_NEAR(
      world_error, result.foot_orientation_residual_rad[leg], 1.0e-9);
    EXPECT_LE(world_error, fixed_pitch_world_error + 1.0e-12);
    measured_maximum_residual = std::max(measured_maximum_residual, world_error);
    pitch_varies_by_leg = pitch_varies_by_leg ||
      std::abs(pitch - result.foot_pitch_target_rad[0]) > 1.0e-4;
    projection_strictly_improves_one_leg = projection_strictly_improves_one_leg ||
      world_error < fixed_pitch_world_error - 1.0e-4;
  }
  EXPECT_TRUE(pitch_varies_by_leg);
  EXPECT_TRUE(projection_strictly_improves_one_leg);
  EXPECT_NEAR(
    result.maximum_foot_orientation_residual_rad, measured_maximum_residual, 1.0e-9);
}

TEST(StandingTarget, DegenerateFootProjectionRejectsCompleteTransaction)
{
  auto request = nominal_request();
  const BodyPoseOffset pose{{0.0, 0.0, 0.0}, kPi / 2.0, 0.0, 0.0};
  const Point3 target_in_body{
    request.mounts[0].position_base_m.x + 0.20,
    request.mounts[0].position_base_m.y,
    request.mounts[0].position_base_m.z,
  };
  request.foot_targets_base_m[0] = rotate_body_to_world(target_in_body, pose);
  const auto direct = project_ground_vertical_foot_pitch(
    target_in_body, pose, request.mounts[0]);
  EXPECT_FALSE(direct.valid);

  std::array<double, 24> previous{};
  previous.fill(0.25);
  const auto result = compute_body_pose_target(request, pose, previous);
  EXPECT_FALSE(result.committed);
  EXPECT_EQ(result.joints_rad, previous);
  for (const auto status : result.leg_status) {
    EXPECT_EQ(status, LegStatus::kInvalid);
  }
}

TEST(StandingTarget, RepeatedAbsolutePoseDoesNotAccumulate)
{
  const auto request = nominal_request();
  const auto standing = compute_standing_target(request, {});
  ASSERT_TRUE(standing.committed);
  const BodyPoseOffset pose{{0.004, 0.003, -0.006}, 0.03, -0.02, 0.025};
  const auto first = compute_body_pose_target(request, pose, standing.joints_rad);
  ASSERT_TRUE(first.committed);
  const auto repeated = compute_body_pose_target(request, pose, first.joints_rad);
  ASSERT_TRUE(repeated.committed);
  EXPECT_EQ(repeated.joints_rad, first.joints_rad);
  for (std::size_t leg = 0; leg < 6; ++leg) {
    EXPECT_EQ(repeated.foot_targets_body_m[leg].x, first.foot_targets_body_m[leg].x);
    EXPECT_EQ(repeated.foot_targets_body_m[leg].y, first.foot_targets_body_m[leg].y);
    EXPECT_EQ(repeated.foot_targets_body_m[leg].z, first.foot_targets_body_m[leg].z);
  }
}

TEST(StandingTarget, NonFinitePoseRejectsWithoutPartialCommit)
{
  const auto request = nominal_request();
  std::array<double, 24> previous{};
  previous.fill(0.25);
  const BodyPoseOffset pose{
    {0.0, std::numeric_limits<double>::quiet_NaN(), 0.0}, 0.0, 0.0, 0.0};
  const auto result = compute_body_pose_target(request, pose, previous);
  EXPECT_FALSE(result.committed);
  EXPECT_EQ(result.joints_rad, previous);
  for (const auto status : result.leg_status) {
    EXPECT_EQ(status, LegStatus::kInvalid);
  }
}

TEST(StandingTarget, LegacyCurveRemainsReachableAndRequiresRuntimeRateShaping)
{
  const auto request = nominal_request();
  const auto standing = compute_standing_target(request, {});
  ASSERT_TRUE(standing.committed);
  const std::array<PlanarVelocity, 7> commands{{
    {0.04, 0.0, 0.0}, {-0.04, 0.0, 0.0},
    {0.0, 0.03, 0.0}, {0.0, -0.03, 0.0},
    {0.0, 0.0, 0.2}, {0.0, 0.0, -0.2},
    {0.03, 0.02, 0.15},
  }};
  bool raw_step_exceeds_rate_cap = false;
  for (const auto & command : commands) {
    TripodState gait_state;
    auto previous = standing.joints_rad;
    for (std::size_t tick = 0; tick < 620; ++tick) {
      const auto gait = advance_tripod(
        TripodConfig{}, gait_state, request.foot_targets_base_m,
        command, true, false, 0.01);
      ASSERT_TRUE(gait.valid);
      const auto candidate = compute_foot_pose_target(
        request, gait.foot_targets_base_m, {}, previous);
      ASSERT_TRUE(candidate.committed);
      for (std::size_t joint = 0; joint < candidate.joints_rad.size(); ++joint) {
        raw_step_exceeds_rate_cap = raw_step_exceeds_rate_cap ||
          std::abs(candidate.joints_rad[joint] - previous[joint]) > 0.012 + 1.0e-12;
      }
      previous = candidate.joints_rad;
      gait_state = gait.state;
    }
    EXPECT_GE(gait_state.cycle, 5U);
  }
  EXPECT_TRUE(raw_step_exceeds_rate_cap);
}

TEST(StandingTarget, DoubleSpeedResponsiveCurveRemainsReachableAtNeutralPose)
{
  const auto request = nominal_request();
  const auto standing = compute_standing_target(request, {});
  ASSERT_TRUE(standing.committed);
  TripodConfig config;
  config.base_cadence_hz = 1.5;
  config.maximum_cadence_hz = 2.5;
  config.cadence_rate_hz_s = 2.0;
  config.maximum_stride_m = 0.12;
  config.translation_acceleration_m_s2 = 0.4;
  config.yaw_acceleration_rad_s2 = 2.4;
  constexpr double kDiagonal = 0.1414213562373095;
  const std::array<PlanarVelocity, 7> commands{{
    {0.2, 0.0, 0.0}, {-0.2, 0.0, 0.0},
    {0.0, 0.2, 0.0}, {0.0, -0.2, 0.0},
    {0.0, 0.0, 1.2}, {0.0, 0.0, -1.2},
    {kDiagonal, kDiagonal, 1.2},
  }};
  for (const auto & command : commands) {
    TripodState gait_state;
    auto previous = standing.joints_rad;
    for (std::size_t tick = 0; tick < 1400; ++tick) {
      const auto gait = advance_tripod(
        config, gait_state, request.foot_targets_base_m,
        command, true, false, 0.001);
      ASSERT_TRUE(gait.valid);
      const auto candidate = compute_foot_pose_target(
        request, gait.foot_targets_base_m, {}, previous);
      ASSERT_TRUE(candidate.committed) <<
        "command=" << command.x_m_s << "," << command.y_m_s << "," <<
        command.yaw_rad_s << " tick=" << tick << " phase=" << gait.state.phase;
      previous = candidate.joints_rad;
      gait_state = gait.state;
    }
    EXPECT_GE(gait_state.cycle, 2U);
  }
}

}  // namespace
