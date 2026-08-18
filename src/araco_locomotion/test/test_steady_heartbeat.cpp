// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <limits>

#include "araco_locomotion/steady_heartbeat.hpp"

namespace
{

using araco_locomotion::SteadyHeartbeat;

TEST(SteadyHeartbeat, PublishesFromSteadyTimeWhenSimulationTimeDoesNotAdvance)
{
  SteadyHeartbeat heartbeat(0.02, 0.05);
  EXPECT_TRUE(heartbeat.update(10.0).publish);
  EXPECT_FALSE(heartbeat.update(10.01).publish);
  EXPECT_TRUE(heartbeat.update(10.02).publish);
  EXPECT_FALSE(heartbeat.update(10.03).publish);
  EXPECT_TRUE(heartbeat.update(10.04).publish);
}

TEST(SteadyHeartbeat, ReportsDelayedExecutorAndDoesNotReplayBursts)
{
  SteadyHeartbeat heartbeat(0.02, 0.05);
  ASSERT_TRUE(heartbeat.update(1.0).publish);
  const auto delayed = heartbeat.update(1.2);
  EXPECT_TRUE(delayed.publish);
  EXPECT_TRUE(delayed.callback_delayed);
  EXPECT_NEAR(delayed.callback_gap_s, 0.2, 1.0e-12);
  EXPECT_FALSE(heartbeat.update(1.201).publish);
}

TEST(SteadyHeartbeat, ResetStartsANewHeartbeatSession)
{
  SteadyHeartbeat heartbeat(0.02, 0.05);
  ASSERT_TRUE(heartbeat.update(5.0).publish);
  heartbeat.reset();
  EXPECT_TRUE(heartbeat.update(2.0).publish);
}

TEST(SteadyHeartbeat, RejectsInvalidConfigurationAndTime)
{
  EXPECT_THROW(SteadyHeartbeat(0.0, 0.05), std::invalid_argument);
  EXPECT_THROW(SteadyHeartbeat(0.02, 0.02), std::invalid_argument);
  SteadyHeartbeat heartbeat(0.02, 0.05);
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(heartbeat.update(nan), std::invalid_argument);
  ASSERT_TRUE(heartbeat.update(2.0).publish);
  EXPECT_THROW(heartbeat.update(1.0), std::invalid_argument);
}

}  // namespace
