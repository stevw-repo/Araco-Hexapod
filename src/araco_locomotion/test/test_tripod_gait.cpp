// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "araco_locomotion/tripod_gait.hpp"

namespace
{

using araco_kinematics::Point3;
using araco_locomotion::GaitMode;
using araco_locomotion::PlanarVelocity;
using araco_locomotion::TripodConfig;
using araco_locomotion::TripodState;
using araco_locomotion::TripodStep;
using araco_locomotion::advance_tripod;
using araco_locomotion::retreat_tripod_to_nominal;

const std::array<Point3, 6> kFeet{{
  {0.20, 0.16, -0.08}, {0.24, 0.0, -0.08}, {0.20, -0.16, -0.08},
  {-0.20, 0.16, -0.08}, {-0.24, 0.0, -0.08}, {-0.20, -0.16, -0.08},
}};

TripodStep sample_group_a_at_phase(double phase)
{
  constexpr double kPhaseStep = 0.01;
  TripodState state;
  state.phase = phase == 0.0 ? 1.0 - kPhaseStep : phase - kPhaseStep;
  state.cycle = 1;
  state.velocity = {0.05, 0.0, 0.0};
  state.walking = true;
  const TripodConfig config;
  state.cadence_hz = config.base_cadence_hz;
  return advance_tripod(
    config, state, kFeet, state.velocity, true, false,
    kPhaseStep / config.base_cadence_hz);
}

TEST(TripodGait, AlternatesExactTripodsAndWrapsMonotonically)
{
  TripodState state;
  const TripodConfig config;
  state.velocity = {0.04, 0.0, 0.0};
  state.walking = true;
  state.cadence_hz = config.base_cadence_hz;
  double previous_phase = 0.0;
  std::uint64_t wraps = 0;
  for (int tick = 0; tick < 360; ++tick) {
    const auto step = advance_tripod(
      config, state, kFeet, {0.04, 0.0, 0.0}, true, false, 0.01);
    ASSERT_TRUE(step.valid);
    EXPECT_EQ(step.swing[0], step.swing[2]);
    EXPECT_EQ(step.swing[0], step.swing[4]);
    EXPECT_EQ(step.swing[1], step.swing[3]);
    EXPECT_EQ(step.swing[1], step.swing[5]);
    EXPECT_NE(step.swing[0], step.swing[1]);
    if (step.state.phase < previous_phase) {
      ++wraps;
    } else {
      EXPECT_GT(step.state.phase, previous_phase);
    }
    previous_phase = step.state.phase;
    state = step.state;
  }
  EXPECT_EQ(wraps, 3U);
  EXPECT_EQ(state.cycle, 3U);
}

TEST(TripodGait, FootPathIsContinuousAndSwingClearsGround)
{
  TripodState state;
  const TripodConfig config;
  std::array<Point3, 6> previous = kFeet;
  double maximum_lift = 0.0;
  for (int tick = 0; tick < 240; ++tick) {
    const auto step = advance_tripod(
      config, state, kFeet, {0.04, -0.02, 0.15}, true, false, 0.01);
    ASSERT_TRUE(step.valid);
    for (std::size_t leg = 0; leg < 6; ++leg) {
      EXPECT_LT(std::hypot(
        step.foot_targets_base_m[leg].x - previous[leg].x,
        step.foot_targets_base_m[leg].y - previous[leg].y), 0.004);
      EXPECT_LT(std::abs(
        step.foot_targets_base_m[leg].z - previous[leg].z), 0.004);
      maximum_lift = std::max(
        maximum_lift, step.foot_targets_base_m[leg].z - kFeet[leg].z);
      EXPECT_LE(
        step.foot_targets_base_m[leg].z - kFeet[leg].z,
        step.state.maximum_clearance_m + 1.0e-12);
    }
    previous = step.foot_targets_base_m;
    state = step.state;
  }
  EXPECT_GT(maximum_lift, 0.005);
  EXPECT_LT(maximum_lift, config.swing_clearance_m);
}

TEST(TripodGait, UsesApprovedLegacyFunctionDefinedFootPath)
{
  const TripodConfig config;
  const double stride =
    0.05 * config.duty_factor / config.base_cadence_hz;
  const double clearance = config.swing_clearance_m *
    stride / config.maximum_stride_m;
  const std::array<double, 5> phases{{0.0, 0.25, 0.5, 0.75, 0.85}};
  const std::array<double, 5> horizontal_scales{{0.5, 0.0, -0.5, 0.0, 0.5}};
  const std::array<double, 5> lift_scales{{0.0, 0.0, 0.0, 1.0, 1.0}};
  for (std::size_t index = 0; index < phases.size(); ++index) {
    const auto step = sample_group_a_at_phase(phases[index]);
    ASSERT_TRUE(step.valid);
    EXPECT_NEAR(step.state.phase, phases[index], 1.0e-12);
    EXPECT_EQ(step.swing[0], phases[index] >= 0.5);
    EXPECT_NEAR(
      step.foot_targets_base_m[0].x - kFeet[0].x,
      horizontal_scales[index] * stride, 1.0e-12);
    EXPECT_NEAR(
      step.foot_targets_base_m[0].z - kFeet[0].z,
      lift_scales[index] * clearance, 1.0e-12);
  }
}

TEST(TripodGait, Phase075FixIsContinuousAndKeepsStanceVelocityConstant)
{
  const auto just_before = sample_group_a_at_phase(0.99);
  const auto boundary = sample_group_a_at_phase(0.0);
  ASSERT_TRUE(just_before.valid);
  ASSERT_TRUE(boundary.valid);
  EXPECT_NEAR(
    just_before.foot_targets_base_m[0].x,
    boundary.foot_targets_base_m[0].x, 1.0e-12);

  std::array<double, 6> stance_x{};
  for (std::size_t index = 0; index < stance_x.size(); ++index) {
    const auto step = sample_group_a_at_phase(0.1 * static_cast<double>(index));
    ASSERT_TRUE(step.valid);
    stance_x[index] = step.foot_targets_base_m[0].x;
  }
  const TripodConfig config;
  const double stride =
    0.05 * config.duty_factor / config.base_cadence_hz;
  const double expected_delta = -0.2 * stride;
  for (std::size_t index = 1; index < stance_x.size(); ++index) {
    EXPECT_NEAR(stance_x[index] - stance_x[index - 1], expected_delta, 1.0e-12);
  }
}

TEST(TripodGait, DeadbandHoldsAndPrecisionStrideScalesClearanceExactly)
{
  const TripodConfig config;
  TripodState state;
  for (int tick = 0; tick < 20; ++tick) {
    const auto below = advance_tripod(
      config, state, kFeet, {0.004, 0.0, 0.0}, true, false, 0.01);
    ASSERT_TRUE(below.valid);
    EXPECT_EQ(below.mode, GaitMode::kHolding);
    EXPECT_FALSE(below.state.walking);
    EXPECT_DOUBLE_EQ(below.state.cadence_hz, 0.0);
    state = below.state;
  }

  state = {};
  state.phase = 0.74;
  state.cycle = 1;
  state.velocity = {0.006, 0.0, 0.0};
  state.walking = true;
  state.cadence_hz = config.base_cadence_hz;
  const auto precision = advance_tripod(
    config, state, kFeet, state.velocity, true, false, 0.01);
  ASSERT_TRUE(precision.valid);
  ASSERT_NEAR(precision.state.phase, 0.75, 1.0e-12);
  const double expected_stride =
    state.velocity.x_m_s * config.duty_factor / config.base_cadence_hz;
  const double expected_scale = expected_stride / config.maximum_stride_m;
  const double expected_clearance = config.swing_clearance_m * expected_scale;
  EXPECT_NEAR(precision.state.maximum_stride_scale, expected_scale, 1.0e-12);
  EXPECT_NEAR(precision.state.maximum_clearance_m, expected_clearance, 1.0e-12);
  EXPECT_NEAR(
    precision.foot_targets_base_m[0].z - kFeet[0].z,
    expected_clearance, 1.0e-12);
}

TEST(TripodGait, AdmittedCommandStartsFromZeroStrideWithoutDeadbandJump)
{
  const TripodConfig config;
  TripodState state;
  double previous_stride_scale = 0.0;
  for (int tick = 0; tick < 8; ++tick) {
    const auto starting = advance_tripod(
      config, state, kFeet, {0.03, 0.0, 0.0}, true, false, 0.01);
    ASSERT_TRUE(starting.valid);
    EXPECT_TRUE(starting.state.walking);
    EXPECT_GT(starting.state.maximum_stride_scale, previous_stride_scale);
    EXPECT_LE(
      starting.state.maximum_stride_scale - previous_stride_scale,
      config.translation_acceleration_m_s2 * 0.01 * config.duty_factor /
      config.base_cadence_hz / config.maximum_stride_m + 1.0e-12);
    previous_stride_scale = starting.state.maximum_stride_scale;
    state = starting.state;
  }
}

TEST(TripodGait, DoubleStrideProfileProducesTwiceTheSteadyForwardStride)
{
  TripodConfig config;
  config.base_cadence_hz = 1.5;
  config.maximum_cadence_hz = 2.5;
  config.cadence_rate_hz_s = 2.0;
  config.maximum_stride_m = 0.12;
  TripodState state;
  state.phase = 0.98;
  state.cycle = 1;
  state.velocity = {0.2, 0.0, 0.0};
  state.walking = true;
  state.cadence_hz = 5.0 / 3.0;

  const auto step = advance_tripod(
    config, state, kFeet, state.velocity, true, false, 0.012);
  ASSERT_TRUE(step.valid);
  EXPECT_NEAR(step.state.phase, 0.0, 1.0e-12);
  EXPECT_NEAR(step.state.cadence_hz, 5.0 / 3.0, 1.0e-12);
  EXPECT_NEAR(
    step.foot_targets_base_m[0].x - kFeet[0].x,
    0.5 * 0.06, 1.0e-12);
  EXPECT_NEAR(step.state.maximum_stride_scale, 0.5, 1.0e-12);
  EXPECT_NEAR(step.state.maximum_clearance_m, 0.015, 1.0e-12);
}

TEST(TripodGait, WorkspaceRetreatFreezesPhaseAndConvergesToNominalHold)
{
  TripodConfig config;
  config.base_cadence_hz = 1.5;
  config.maximum_cadence_hz = 2.5;
  config.maximum_stride_m = 0.12;
  config.translation_stop_deceleration_m_s2 = 0.3;
  TripodState state;
  state.phase = 0.37;
  state.cycle = 3;
  state.velocity = {0.06, -0.03, 0.2};
  state.walking = true;
  state.cadence_hz = 1.0;
  double previous_speed = std::numeric_limits<double>::infinity();
  bool reached_hold = false;
  for (int tick = 0; tick < 250; ++tick) {
    const auto step = retreat_tripod_to_nominal(config, state, kFeet, 0.01);
    ASSERT_TRUE(step.valid);
    const double speed = std::hypot(
      std::hypot(step.state.velocity.x_m_s, step.state.velocity.y_m_s),
      step.state.velocity.yaw_rad_s);
    EXPECT_LE(speed, previous_speed + 1.0e-12);
    if (step.state.walking) {
      EXPECT_DOUBLE_EQ(step.state.phase, 0.37);
      EXPECT_EQ(step.mode, GaitMode::kStopping);
    } else {
      for (std::size_t leg = 0; leg < kFeet.size(); ++leg) {
        EXPECT_DOUBLE_EQ(step.foot_targets_base_m[leg].x, kFeet[leg].x);
        EXPECT_DOUBLE_EQ(step.foot_targets_base_m[leg].y, kFeet[leg].y);
        EXPECT_DOUBLE_EQ(step.foot_targets_base_m[leg].z, kFeet[leg].z);
      }
      reached_hold = step.mode == GaitMode::kHolding;
    }
    previous_speed = speed;
    state = step.state;
  }
  EXPECT_TRUE(reached_hold);
  EXPECT_FALSE(state.walking);
  EXPECT_DOUBLE_EQ(state.phase, 0.0);
  EXPECT_DOUBLE_EQ(state.cadence_hz, 0.0);
}

TEST(TripodGait, CadenceRisesOnlyAfterPreferredStrideAndSlewsContinuously)
{
  const TripodConfig config;
  TripodState state;
  state.phase = 0.2;
  state.cycle = 2;
  state.velocity = {0.04, 0.0, 0.0};
  state.walking = true;
  state.cadence_hz = config.base_cadence_hz;

  const auto precision = advance_tripod(
    config, state, kFeet, state.velocity, true, false, 0.01);
  ASSERT_TRUE(precision.valid);
  EXPECT_DOUBLE_EQ(precision.state.cadence_hz, config.base_cadence_hz);
  EXPECT_LT(
    precision.state.maximum_stride_scale,
    config.preferred_maximum_stride_scale);

  state = precision.state;
  double previous_phase = state.phase;
  double previous_cadence = state.cadence_hz;
  bool cadence_increased = false;
  for (int tick = 0; tick < 50; ++tick) {
    const auto faster = advance_tripod(
      config, state, kFeet, {0.08, 0.0, 0.0}, true, false, 0.01);
    ASSERT_TRUE(faster.valid);
    EXPECT_LE(
      std::abs(faster.state.cadence_hz - previous_cadence),
      config.cadence_rate_hz_s * 0.01 + 1.0e-12);
    EXPECT_LE(faster.state.maximum_stride_scale, 1.0);
    const double expected_phase = std::fmod(
      previous_phase + faster.state.cadence_hz * 0.01, 1.0);
    EXPECT_NEAR(faster.state.phase, expected_phase, 1.0e-12);
    cadence_increased = cadence_increased ||
      faster.state.cadence_hz > config.base_cadence_hz;
    previous_phase = faster.state.phase;
    previous_cadence = faster.state.cadence_hz;
    state = faster.state;
  }
  EXPECT_TRUE(cadence_increased);
  EXPECT_GT(state.cadence_hz, config.base_cadence_hz);
  EXPECT_LE(state.cadence_hz, config.maximum_cadence_hz);
}

TEST(TripodGait, MaximumCadenceUniformlySaturatesOverspeedCommand)
{
  const TripodConfig config;
  TripodState state;
  state.phase = 0.2;
  state.cycle = 2;
  state.velocity = {0.5, 0.0, 0.0};
  state.walking = true;
  state.cadence_hz = config.maximum_cadence_hz;
  const auto saturated = advance_tripod(
    config, state, kFeet, state.velocity, true, false, 0.01);
  ASSERT_TRUE(saturated.valid);
  const double unconstrained_stride =
    state.velocity.x_m_s * config.duty_factor / config.maximum_cadence_hz;
  const double expected_scale = config.maximum_stride_m / unconstrained_stride;
  EXPECT_DOUBLE_EQ(saturated.state.cadence_hz, config.maximum_cadence_hz);
  EXPECT_NEAR(saturated.state.applied_velocity_scale, expected_scale, 1.0e-12);
  EXPECT_DOUBLE_EQ(saturated.state.maximum_stride_scale, 1.0);
  EXPECT_DOUBLE_EQ(saturated.state.maximum_clearance_m, config.swing_clearance_m);
  for (std::size_t leg = 0; leg < 6; ++leg) {
    EXPECT_LE(
      std::hypot(
        saturated.foot_targets_base_m[leg].x - kFeet[leg].x,
        saturated.foot_targets_base_m[leg].y - kFeet[leg].y),
      0.5 * config.maximum_stride_m + 1.0e-12);
  }
}

TEST(TripodGait, ControlledStopCompletesAtBoundaryAndDwells)
{
  TripodState state;
  const TripodConfig config;
  for (int tick = 0; tick < 150; ++tick) {
    state = advance_tripod(
      config, state, kFeet, {0.04, 0.02, 0.2}, true, false, 0.01).state;
  }
  bool reached_holding = false;
  double elapsed = 0.0;
  for (int tick = 0; tick < 150; ++tick) {
    const auto step = advance_tripod(
      config, state, kFeet, {}, false, true, 0.01);
    ASSERT_TRUE(step.valid);
    elapsed += 0.01;
    state = step.state;
    if (step.mode == GaitMode::kHolding) {
      reached_holding = true;
      for (std::size_t leg = 0; leg < 6; ++leg) {
        EXPECT_DOUBLE_EQ(step.foot_targets_base_m[leg].x, kFeet[leg].x);
        EXPECT_DOUBLE_EQ(step.foot_targets_base_m[leg].y, kFeet[leg].y);
        EXPECT_DOUBLE_EQ(step.foot_targets_base_m[leg].z, kFeet[leg].z);
      }
      break;
    }
  }
  EXPECT_TRUE(reached_holding);
  EXPECT_LE(elapsed, 1.5);
  EXPECT_DOUBLE_EQ(state.phase, 0.0);
  EXPECT_NEAR(state.hold_dwell_s, config.stable_hold_dwell_s, 0.011);
}

TEST(TripodGait, RejectsNonFiniteInputWithoutMutatingCommit)
{
  TripodState state;
  PlanarVelocity command;
  command.x_m_s = std::numeric_limits<double>::quiet_NaN();
  const auto step = advance_tripod(
    TripodConfig{}, state, kFeet, command, true, false, 0.01);
  EXPECT_FALSE(step.valid);
  EXPECT_DOUBLE_EQ(step.state.phase, state.phase);
  for (std::size_t leg = 0; leg < 6; ++leg) {
    EXPECT_DOUBLE_EQ(step.foot_targets_base_m[leg].x, kFeet[leg].x);
    EXPECT_DOUBLE_EQ(step.foot_targets_base_m[leg].y, kFeet[leg].y);
    EXPECT_DOUBLE_EQ(step.foot_targets_base_m[leg].z, kFeet[leg].z);
  }
}

}  // namespace
