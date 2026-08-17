// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "araco_supervision/source_arbiter.hpp"

namespace
{

using araco_supervision::SourceArbiter;
using araco_supervision::SourceConfig;

SourceArbiter arbiter()
{
  return SourceArbiter({
      SourceConfig{10, 100, 0.15, true},
      SourceConfig{250, 200, 0.10, true},
  });
}

TEST(SourceArbiter, RejectsInvalidConfiguration)
{
  EXPECT_THROW(SourceArbiter({}), std::invalid_argument);
  EXPECT_THROW(
    SourceArbiter({SourceConfig{10, 1, 0.1, true}, SourceConfig{10, 2, 0.1, true}}),
    std::invalid_argument);
  EXPECT_THROW(
    SourceArbiter({SourceConfig{10, 1, 0.1, true}, SourceConfig{20, 1, 0.1, true}}),
    std::invalid_argument);
}

TEST(SourceArbiter, StartupActiveCannotAcquireWithoutReleaseEdge)
{
  auto policy = arbiter();
  EXPECT_FALSE(policy.accept(10, 1, true, true, 0.0));
  auto decision = policy.evaluate(0.0);
  EXPECT_FALSE(decision.has_selection);
  EXPECT_EQ(decision.selection_epoch, 0U);
  ASSERT_EQ(decision.quarantined_source_ids.size(), 1U);
  EXPECT_EQ(decision.quarantined_source_ids.front(), 10U);

  EXPECT_TRUE(policy.accept(10, 2, false, true, 0.01));
  EXPECT_FALSE(policy.all_sources_released());
  EXPECT_TRUE(policy.accept(250, 1, false, true, 0.01));
  EXPECT_TRUE(policy.all_sources_released());
  EXPECT_TRUE(policy.accept(10, 3, true, true, 0.02));
  decision = policy.evaluate(0.02);
  EXPECT_TRUE(decision.has_selection);
  EXPECT_EQ(decision.source_id, 10U);
  EXPECT_EQ(decision.selection_epoch, 1U);
}

TEST(SourceArbiter, DuplicateAndReorderQuarantineUntilNewReleaseSession)
{
  auto policy = arbiter();
  ASSERT_TRUE(policy.accept(10, 10, false, true, 0.0));
  ASSERT_TRUE(policy.accept(10, 11, true, true, 0.01));
  EXPECT_EQ(policy.evaluate(0.01).source_id, 10U);

  EXPECT_FALSE(policy.accept(10, 11, true, true, 0.02));
  auto lost = policy.evaluate(0.02);
  EXPECT_FALSE(lost.has_selection);
  EXPECT_EQ(lost.selection_epoch, 2U);
  EXPECT_EQ(lost.reason_code, 10U);
  EXPECT_FALSE(policy.accept(10, 12, true, true, 0.03));
  EXPECT_EQ(policy.evaluate(0.03).selection_epoch, 2U);

  // A restarted publisher may reset its counter only with an inactive sample.
  EXPECT_TRUE(policy.accept(10, 1, false, true, 0.04));
  EXPECT_TRUE(policy.accept(10, 2, true, true, 0.05));
  const auto reacquired = policy.evaluate(0.05);
  EXPECT_EQ(reacquired.source_id, 10U);
  EXPECT_EQ(reacquired.selection_epoch, 3U);
}

TEST(SourceArbiter, StaleSourceIsQuarantinedAndCannotSurpriseResume)
{
  auto policy = arbiter();
  ASSERT_TRUE(policy.accept(10, 1, false, true, 0.0));
  ASSERT_TRUE(policy.accept(10, 2, true, true, 0.01));
  ASSERT_EQ(policy.evaluate(0.01).source_id, 10U);
  auto stale = policy.evaluate(0.17);
  EXPECT_FALSE(stale.has_selection);
  EXPECT_EQ(stale.reason_code, 8U);
  EXPECT_EQ(stale.selection_epoch, 2U);
  EXPECT_FALSE(policy.accept(10, 3, true, true, 0.18));
  EXPECT_FALSE(policy.evaluate(0.18).has_selection);
}

TEST(SourceArbiter, FreshHigherPriorityEdgeMarksDeliberatePreemption)
{
  auto policy = arbiter();
  ASSERT_TRUE(policy.accept(10, 1, false, true, 0.0));
  ASSERT_TRUE(policy.accept(250, 1, false, true, 0.0));
  ASSERT_TRUE(policy.accept(10, 2, true, true, 0.01));
  ASSERT_EQ(policy.evaluate(0.01).source_id, 10U);

  ASSERT_TRUE(policy.accept(250, 2, true, true, 0.02));
  const auto preemption = policy.evaluate(0.02);
  EXPECT_EQ(preemption.previous_source_id, 10U);
  EXPECT_EQ(preemption.source_id, 250U);
  EXPECT_EQ(preemption.reason_code, 9U);
  EXPECT_TRUE(preemption.deliberate_higher_priority_preemption);
  EXPECT_EQ(preemption.selection_epoch, 2U);
}

TEST(SourceArbiter, LowerPriorityAvailabilityDoesNotLookLikePreemption)
{
  auto policy = arbiter();
  ASSERT_TRUE(policy.accept(10, 1, false, true, 0.0));
  ASSERT_TRUE(policy.accept(250, 1, false, true, 0.0));
  ASSERT_TRUE(policy.accept(10, 2, true, true, 0.01));
  ASSERT_TRUE(policy.accept(250, 2, true, true, 0.02));
  ASSERT_EQ(policy.evaluate(0.02).source_id, 250U);

  ASSERT_TRUE(policy.accept(250, 3, false, true, 0.03));
  const auto fallback = policy.evaluate(0.03);
  EXPECT_EQ(fallback.source_id, 10U);
  EXPECT_EQ(fallback.reason_code, 7U);
  EXPECT_FALSE(fallback.deliberate_higher_priority_preemption);
  EXPECT_EQ(fallback.selection_epoch, 2U);
}

TEST(SourceArbiter, InvalidSampleQuarantinesOnlyItsSource)
{
  auto policy = arbiter();
  ASSERT_TRUE(policy.accept(10, 1, false, true, 0.0));
  ASSERT_TRUE(policy.accept(10, 2, true, true, 0.01));
  ASSERT_EQ(policy.evaluate(0.01).source_id, 10U);
  EXPECT_FALSE(policy.accept(10, 3, true, false, 0.02));
  const auto lost = policy.evaluate(0.02);
  EXPECT_FALSE(lost.has_selection);
  EXPECT_EQ(lost.reason_code, 10U);
}

}  // namespace
