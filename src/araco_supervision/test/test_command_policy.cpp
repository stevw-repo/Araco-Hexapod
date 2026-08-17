// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "araco_supervision/command_policy.hpp"

namespace
{

using araco_supervision::BodyEnvelope;
using araco_supervision::CommandValidity;
using araco_supervision::StaticIntent;
using araco_supervision::quaternion_from_rpy;
using araco_supervision::validate_and_limit_static_intent;

BodyEnvelope envelope()
{
  return {0.05, 0.08, 0.3, 0.5, 0.02, -0.03, 0.02, 0.15, 0.2,
    0.035, -0.045, 0.035, 0.25, 0.35, 1.0e-6, 1.0e-12, 1.0e-9};
}

StaticIntent neutral()
{
  StaticIntent intent{};
  intent.orientation.w = 1.0;
  return intent;
}

TEST(CommandPolicy, AcceptsEveryNormalStaticAxisAndPreservesSigns)
{
  auto intent = neutral();
  intent.position_m = {0.01, -0.012, 0.008};
  intent.orientation = quaternion_from_rpy(0.07, -0.08, 0.09);
  const auto result = validate_and_limit_static_intent(
    intent, "base_link", envelope(), true);
  ASSERT_EQ(result.validity, CommandValidity::kValid);
  EXPECT_EQ(result.intent.position_m, intent.position_m);
  EXPECT_NEAR(result.rpy_rad[0], 0.07, 1.0e-12);
  EXPECT_NEAR(result.rpy_rad[1], -0.08, 1.0e-12);
  EXPECT_NEAR(result.rpy_rad[2], 0.09, 1.0e-12);
}

TEST(CommandPolicy, LimitsNormalEnvelopeButRejectsHardEnvelope)
{
  auto intent = neutral();
  intent.position_m = {0.03, -0.025, 0.03};
  intent.orientation = quaternion_from_rpy(0.2, -0.18, 0.3);
  const auto limited = validate_and_limit_static_intent(
    intent, "base_link", envelope(), true);
  ASSERT_EQ(limited.validity, CommandValidity::kLimited);
  EXPECT_EQ(limited.intent.position_m[0], 0.02);
  EXPECT_EQ(limited.intent.position_m[1], -0.02);
  EXPECT_EQ(limited.intent.position_m[2], 0.02);
  EXPECT_NEAR(limited.rpy_rad[0], 0.15, 1.0e-12);
  EXPECT_NEAR(limited.rpy_rad[1], -0.15, 1.0e-12);
  EXPECT_NEAR(limited.rpy_rad[2], 0.2, 1.0e-12);

  intent.position_m[0] = 0.036;
  EXPECT_EQ(
    validate_and_limit_static_intent(intent, "base_link", envelope(), true).validity,
    CommandValidity::kInvalid);
}

TEST(CommandPolicy, RejectsWrongFrameQuaternionTwistGaitAndNonFinite)
{
  auto intent = neutral();
  EXPECT_EQ(
    validate_and_limit_static_intent(intent, "odom", envelope(), false).validity,
    CommandValidity::kInvalid);
  intent = neutral();
  intent.orientation.w = 0.5;
  EXPECT_EQ(
    validate_and_limit_static_intent(intent, "base_link", envelope(), false).validity,
    CommandValidity::kInvalid);
  intent = neutral();
  intent.twist[0] = 1.0e-4;
  EXPECT_EQ(
    validate_and_limit_static_intent(intent, "base_link", envelope(), false).validity,
    CommandValidity::kInvalid);
  intent = neutral();
  intent.gait = 2;
  EXPECT_EQ(
    validate_and_limit_static_intent(intent, "base_link", envelope(), false).validity,
    CommandValidity::kInvalid);
  intent = neutral();
  intent.position_m[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    validate_and_limit_static_intent(intent, "base_link", envelope(), false).validity,
    CommandValidity::kInvalid);
}

TEST(CommandPolicy, AcceptsAndLimitsTripodPlanarAndYawCommands)
{
  auto intent = neutral();
  intent.gait = 1;
  intent.twist[0] = 0.04;
  intent.twist[1] = 0.03;
  intent.twist[5] = -0.2;
  auto result = validate_and_limit_static_intent(
    intent, "base_link", envelope(), true);
  ASSERT_EQ(result.validity, CommandValidity::kValid);
  EXPECT_DOUBLE_EQ(result.intent.twist[0], 0.04);
  EXPECT_DOUBLE_EQ(result.intent.twist[1], 0.03);
  EXPECT_DOUBLE_EQ(result.intent.twist[5], -0.2);

  intent.twist[0] = 0.06;
  intent.twist[1] = 0.04;
  intent.twist[5] = 0.4;
  result = validate_and_limit_static_intent(
    intent, "base_link", envelope(), true);
  ASSERT_EQ(result.validity, CommandValidity::kLimited);
  EXPECT_NEAR(std::hypot(result.intent.twist[0], result.intent.twist[1]), 0.05, 1.0e-12);
  EXPECT_DOUBLE_EQ(result.intent.twist[5], 0.3);
}

}  // namespace
