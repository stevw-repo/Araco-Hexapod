// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <limits>

#include "araco_locomotion/safe_command_guard.hpp"

namespace
{

using araco_locomotion::SafeCommandGuard;

TEST(SafeCommandGuard, StartsUnarmedAndRequiresReleaseBeforeExecute)
{
  SafeCommandGuard guard(0.05);
  guard.reset();
  EXPECT_FALSE(guard.evaluate(0.0).armed);
  EXPECT_FALSE(guard.accept(1, 1, true, 0.01));
  EXPECT_TRUE(guard.evaluate(0.02).quarantined);
  EXPECT_TRUE(guard.accept(1, 0, true, 0.03));
  EXPECT_FALSE(guard.accept(1, 1, true, 0.04));
  EXPECT_TRUE(guard.accept(1, 0, true, 0.05));
  EXPECT_TRUE(guard.accept(2, 1, true, 0.06));
  EXPECT_TRUE(guard.evaluate(0.07).executable);
}

TEST(SafeCommandGuard, StaleAuthorityQuarantinesUntilHoldAndNewEpoch)
{
  SafeCommandGuard guard(0.05);
  guard.reset();
  ASSERT_TRUE(guard.accept(4, 0, true, 0.0));
  ASSERT_TRUE(guard.accept(5, 1, true, 0.01));
  auto result = guard.evaluate(0.061);
  EXPECT_FALSE(result.executable);
  EXPECT_TRUE(result.quarantined);
  EXPECT_EQ(result.reason, 13);
  EXPECT_FALSE(guard.accept(6, 1, true, 0.07));
  EXPECT_TRUE(guard.accept(6, 3, true, 0.08));
  EXPECT_FALSE(guard.accept(6, 2, true, 0.09));
  EXPECT_TRUE(guard.accept(6, 0, true, 0.10));
  EXPECT_TRUE(guard.accept(7, 2, true, 0.11));
  EXPECT_TRUE(guard.evaluate(0.12).executable);
}

TEST(SafeCommandGuard, InvalidAndReorderedTrustedSamplesFailClosed)
{
  SafeCommandGuard guard(0.05);
  guard.reset();
  EXPECT_FALSE(guard.accept(1, 9, true, 0.0));
  EXPECT_EQ(guard.evaluate(0.01).reason, 27);
  EXPECT_TRUE(guard.accept(2, 0, true, 0.02));
  EXPECT_TRUE(guard.accept(3, 1, true, 0.03));
  EXPECT_FALSE(guard.accept(2, 1, true, 0.04));
  EXPECT_FALSE(guard.accept(4, 1, false, 0.05));
}

TEST(SafeCommandGuard, RejectsInvalidTiming)
{
  EXPECT_THROW(SafeCommandGuard(0.0), std::invalid_argument);
  SafeCommandGuard guard(0.05);
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(guard.accept(1, 0, true, nan), std::invalid_argument);
  EXPECT_THROW(guard.evaluate(nan), std::invalid_argument);
}

}  // namespace
