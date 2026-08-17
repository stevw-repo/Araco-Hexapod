// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <random>

#include "araco_kinematics/leg_kinematics.hpp"

namespace
{

using araco_kinematics::Branch;
using araco_kinematics::JointLimits;
using araco_kinematics::JointVector;
using araco_kinematics::LegGeometry;
using araco_kinematics::LegKinematics;
using araco_kinematics::Point3;
using araco_kinematics::SolverOptions;
using araco_kinematics::Status;

constexpr double kTolerance = 1.0e-10;

LegKinematics solver()
{
  return LegKinematics(
    LegGeometry{0.043, 0.12, 0.12, 0.05},
    JointLimits{{{-0.7, 0.7}, {0.15, 1.35}, {-2.65, -0.75}, {-1.25, 0.35}}},
    SolverOptions{1.0e-9, 1.0e-6, 1.0e-4});
}

void expect_same(const JointVector & actual, const JointVector & expected)
{
  for (std::size_t index = 0; index < expected.size(); ++index) {
    EXPECT_NEAR(actual[index], expected[index], kTolerance) << "joint " << index;
  }
}

TEST(LegKinematics, NominalCanonicalClassesAndMirrorSignsRoundTrip)
{
  const auto kinematics = solver();
  const std::array<JointVector, 6> poses{{
    {{0.166907, 0.749681, -1.947935, -0.372542}},
    {{0.0, 0.694188, -1.791009, -0.473975}},
    {{-0.166907, 0.749681, -1.947935, -0.372542}},
    {{-0.166907, 0.749681, -1.947935, -0.372542}},
    {{0.0, 0.694188, -1.791009, -0.473975}},
    {{0.166907, 0.749681, -1.947935, -0.372542}},
  }};
  for (const auto & pose : poses) {
    const auto forward = kinematics.forward(pose);
    ASSERT_EQ(forward.status, Status::kValid);
    const auto inverse = kinematics.inverse(
      forward.foot_position_m, forward.foot_pitch_rad, Branch::kKneeDown);
    ASSERT_EQ(inverse.status, Status::kValid);
    expect_same(inverse.joints_rad, pose);
    EXPECT_LT(inverse.position_error_m, kTolerance);
    EXPECT_LT(inverse.pitch_error_rad, kTolerance);
  }
}

TEST(LegKinematics, SeededReachablePropertiesCoverClassesAndMirrors)
{
  const auto kinematics = solver();
  std::mt19937_64 generator(0x415241434fULL);
  std::uniform_real_distribution<double> yaw_magnitude(0.02, 0.55);
  std::uniform_real_distribution<double> femur(0.3, 1.15);
  std::uniform_real_distribution<double> tibia(-2.35, -1.0);
  std::uniform_real_distribution<double> foot(-1.05, 0.15);

  for (std::size_t leg_class = 0; leg_class < 2; ++leg_class) {
    for (double mirror : {-1.0, 1.0}) {
      for (std::size_t sample = 0; sample < 10000; ++sample) {
        const double class_offset = leg_class == 0 ? 0.0 : 0.02;
        const JointVector expected{{
          mirror * yaw_magnitude(generator),
          femur(generator) + class_offset,
          tibia(generator),
          foot(generator),
        }};
        const auto forward = kinematics.forward(expected);
        ASSERT_EQ(forward.status, Status::kValid);
        ASSERT_GT(std::hypot(forward.foot_position_m.x, forward.foot_position_m.y), 0.02);
        const auto inverse = kinematics.inverse(
          forward.foot_position_m, forward.foot_pitch_rad, Branch::kKneeDown);
        ASSERT_TRUE(inverse.status == Status::kValid || inverse.status == Status::kNearLimit);
        expect_same(inverse.joints_rad, expected);
        EXPECT_LT(inverse.position_error_m, kTolerance);
        EXPECT_LT(inverse.pitch_error_rad, kTolerance);
      }
    }
  }
}

TEST(LegKinematics, ExplicitBranchPolicySelectsKneeSign)
{
  const LegKinematics unrestricted(
    LegGeometry{0.043, 0.12, 0.12, 0.05},
    JointLimits{{{-3.2, 3.2}, {-3.2, 3.2}, {-3.2, 3.2}, {-3.2, 3.2}}},
    SolverOptions{1.0e-9, 1.0e-6, 0.0});
  const JointVector down{{0.2, 0.8, -1.7, -0.5}};
  const auto target = unrestricted.forward(down);
  ASSERT_EQ(target.status, Status::kValid);
  const auto down_result = unrestricted.inverse(
    target.foot_position_m, target.foot_pitch_rad, Branch::kKneeDown);
  const auto up_result = unrestricted.inverse(
    target.foot_position_m, target.foot_pitch_rad, Branch::kKneeUp);
  ASSERT_EQ(down_result.status, Status::kValid);
  ASSERT_EQ(up_result.status, Status::kValid);
  EXPECT_LT(down_result.joints_rad[2], 0.0);
  EXPECT_GT(up_result.joints_rad[2], 0.0);
  EXPECT_LT(down_result.position_error_m, kTolerance);
  EXPECT_LT(up_result.position_error_m, kTolerance);
}

TEST(LegKinematics, RejectsUnreachableSingularNonFiniteAndLimitCases)
{
  const auto kinematics = solver();
  EXPECT_EQ(
    kinematics.inverse(Point3{1.0, 0.0, 0.0}, -1.57, Branch::kKneeDown).status,
    Status::kUnreachable);
  EXPECT_EQ(
    kinematics.inverse(Point3{0.283, 0.0, -0.05}, -1.570796326794897,
      Branch::kKneeDown).status,
    Status::kSingular);
  EXPECT_EQ(
    kinematics.inverse(
      Point3{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
      -1.57, Branch::kKneeDown).status,
    Status::kInvalidInput);
  EXPECT_EQ(
    kinematics.forward(JointVector{{0.0, 0.0, -1.5, -0.2}}).status,
    Status::kJointLimit);
}

TEST(LegKinematics, InvalidConfigurationFailsClosed)
{
  const LegKinematics invalid(
    LegGeometry{0.0, 0.12, 0.12, 0.05},
    JointLimits{{{-0.7, 0.7}, {0.15, 1.35}, {-2.65, -0.75}, {-1.25, 0.35}}},
    SolverOptions{1.0e-9, 1.0e-6, 0.01});
  EXPECT_FALSE(invalid.valid_configuration());
  EXPECT_EQ(invalid.forward(JointVector{{0.0, 0.7, -1.8, -0.4}}).status,
    Status::kInvalidInput);
}

}  // namespace
