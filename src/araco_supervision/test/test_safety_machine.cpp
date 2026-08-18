// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <limits>

#include "araco_supervision/safety_machine.hpp"

namespace
{

using araco_supervision::SafetyMachine;
using araco_supervision::SafetyMachineConfig;
using araco_supervision::SafetyMachineInput;
using araco_supervision::SafetyRequest;
using araco_supervision::SafetyState;

constexpr std::uint16_t kReasonSourceReleased = 7;
constexpr std::uint16_t kReasonSourceStale = 8;
constexpr std::uint16_t kReasonSourceHandover = 9;
constexpr std::uint16_t kReasonSourceInvalid = 10;
constexpr std::uint16_t kReasonSelectedStale = 12;
constexpr std::uint16_t kReasonResetRequired = 26;
constexpr std::uint64_t kFaultSelected = 1;
constexpr std::uint64_t kFaultController = 4;
constexpr std::uint64_t kFaultJointState = 8;
constexpr std::uint64_t kFaultTime = 64;

SafetyMachineInput ready_released()
{
  SafetyMachineInput input;
  input.ready = true;
  input.selected_stream_armed = true;
  input.selected_stream_fresh = true;
  input.arbitration_status_fresh = true;
  input.all_sources_released = true;
  input.locomotion_stably_holding = true;
  input.controlled_stop_available = true;
  return input;
}

void enable_source(SafetyMachine & machine, SafetyMachineInput & input, double time)
{
  ASSERT_TRUE(machine.request(SafetyRequest::kEnableMotion, input, time).accepted);
  input.all_sources_released = false;
  input.selected_has_selection = true;
  input.selected_valid = true;
  input.selected_source_id = 10;
  input.selection_epoch = 1;
  EXPECT_EQ(machine.update(input, time + 0.01).state, SafetyState::kMotionEnabled);
}

TEST(SafetyMachine, StartupAndEpochsCoverInactiveHoldingAndMotion)
{
  SafetyMachine machine({5.0, 0.25});
  machine.reset(0.0);
  SafetyMachineInput input;
  EXPECT_EQ(machine.update(input, 0.01).state, SafetyState::kInactive);
  EXPECT_EQ(machine.output().safety_epoch, 1U);
  input = ready_released();
  EXPECT_EQ(machine.update(input, 0.02).state, SafetyState::kHolding);
  EXPECT_EQ(machine.output().safety_epoch, 2U);
  enable_source(machine, input, 0.03);
  EXPECT_EQ(machine.output().safety_epoch, 4U);
}

TEST(SafetyMachine, StartupRequiresContinuousReadinessButFaultsRemainActiveAfterward)
{
  SafetyMachine machine({5.0, 0.25, false, 1.0});
  machine.reset(0.0);
  auto input = ready_released();
  EXPECT_EQ(machine.update(input, 0.01).state, SafetyState::kInactive);
  EXPECT_EQ(machine.update(input, 0.50).state, SafetyState::kInactive);

  input.ready = false;
  EXPECT_EQ(machine.update(input, 0.60).state, SafetyState::kInactive);
  input.ready = true;
  EXPECT_EQ(machine.update(input, 0.70).state, SafetyState::kInactive);
  EXPECT_EQ(machine.update(input, 1.69).state, SafetyState::kInactive);
  EXPECT_EQ(machine.update(input, 1.71).state, SafetyState::kHolding);

  input.ready = false;
  input.condition_fault_mask = kFaultTime;
  input.condition_reason = 23;
  EXPECT_EQ(machine.update(input, 1.72).state, SafetyState::kFaultHold);
}

TEST(SafetyMachine, EnableRequiresReleaseAndFreshPostEnableSelectionEdge)
{
  SafetyMachine machine({0.5, 0.25});
  machine.reset(0.0);
  auto input = ready_released();
  machine.update(input, 0.01);
  input.all_sources_released = false;
  EXPECT_FALSE(machine.request(SafetyRequest::kEnableMotion, input, 0.02).accepted);
  input.all_sources_released = true;
  input.selection_epoch = 4;
  EXPECT_TRUE(machine.request(SafetyRequest::kEnableMotion, input, 0.03).accepted);
  input.selected_has_selection = true;
  input.selected_valid = true;
  EXPECT_EQ(machine.update(input, 0.04).state, SafetyState::kEnabling);
  EXPECT_EQ(machine.update(input, 0.54).state, SafetyState::kHolding);
}

TEST(SafetyMachine, ConfiguredStandingSourceAutomaticallyEnablesOnce)
{
  SafetyMachine machine({0.5, 0.25, true});
  machine.reset(0.0);
  auto input = ready_released();
  machine.update(input, 0.01);
  input.all_sources_released = false;
  input.selected_has_selection = true;
  input.selected_valid = true;
  input.selected_neutral_standing = true;
  input.selected_source_id = 10;
  input.selection_epoch = 4;
  EXPECT_EQ(machine.update(input, 0.02).state, SafetyState::kMotionEnabled);

  EXPECT_TRUE(machine.request(SafetyRequest::kHold, input, 0.03).accepted);
  EXPECT_EQ(machine.output().state, SafetyState::kStopping);
  machine.update(input, 0.04);
  EXPECT_EQ(machine.update(input, 0.30).state, SafetyState::kHolding);
  EXPECT_EQ(machine.update(input, 0.31).state, SafetyState::kHolding);
}

TEST(SafetyMachine, AutomaticEnableWaitsForStandingValidFreshSource)
{
  SafetyMachine machine({0.5, 0.25, true});
  machine.reset(0.0);
  auto input = ready_released();
  machine.update(input, 0.01);
  input.all_sources_released = false;
  input.selected_has_selection = true;
  input.selected_valid = true;
  input.selected_neutral_standing = false;
  EXPECT_EQ(machine.update(input, 0.02).state, SafetyState::kHolding);
  input.selected_neutral_standing = true;
  input.selected_valid = false;
  EXPECT_EQ(machine.update(input, 0.03).state, SafetyState::kHolding);
  input.selected_valid = true;
  EXPECT_EQ(machine.update(input, 0.04).state, SafetyState::kMotionEnabled);

  SafetyMachine stale_machine({0.5, 0.25, true});
  stale_machine.reset(0.0);
  input = ready_released();
  stale_machine.update(input, 0.01);
  input.all_sources_released = false;
  input.selected_has_selection = true;
  input.selected_valid = true;
  input.selected_neutral_standing = true;
  input.selected_stream_fresh = false;
  EXPECT_EQ(stale_machine.update(input, 0.02).state, SafetyState::kFaultHold);
}

TEST(SafetyMachine, SourceLossStopsDwellsAndNeverFallsBackIntoMotion)
{
  SafetyMachine machine({5.0, 0.25});
  machine.reset(0.0);
  auto input = ready_released();
  machine.update(input, 0.01);
  enable_source(machine, input, 0.02);
  input.selected_source_id = 20;
  input.selection_epoch = 2;
  input.arbitration_reason = kReasonSourceReleased;
  input.deliberate_higher_priority_preemption = false;
  EXPECT_EQ(machine.update(input, 0.04).state, SafetyState::kMotionEnabled);
  input.selected_has_selection = false;
  EXPECT_EQ(machine.update(input, 0.05).state, SafetyState::kStopping);
  machine.update(input, 0.06);
  EXPECT_EQ(machine.update(input, 0.32).state, SafetyState::kHolding);
  EXPECT_EQ(machine.output().reason, kReasonSourceReleased);
}

TEST(SafetyMachine, SourceLossWaitsForMatchingArbitrationEpochReason)
{
  SafetyMachine machine({5.0, 0.25});
  machine.reset(0.0);
  auto input = ready_released();
  machine.update(input, 0.01);
  enable_source(machine, input, 0.02);

  input.selected_has_selection = false;
  input.selection_epoch = 2;
  input.arbitration_status_fresh = false;
  input.arbitration_reason = 0;
  EXPECT_EQ(machine.update(input, 0.04).state, SafetyState::kMotionEnabled);
  EXPECT_EQ(machine.output().reason, 0U);

  input.arbitration_status_fresh = true;
  input.arbitration_reason = kReasonSourceInvalid;
  EXPECT_EQ(machine.update(input, 0.05).state, SafetyState::kStopping);
  EXPECT_EQ(machine.output().reason, kReasonSourceInvalid);
}

TEST(SafetyMachine, DeliberatePreemptionCrossesStopAndEnablingBarrier)
{
  SafetyMachine machine({5.0, 0.25});
  machine.reset(0.0);
  auto input = ready_released();
  machine.update(input, 0.01);
  enable_source(machine, input, 0.02);
  input.selected_source_id = 250;
  input.selection_epoch = 2;
  input.arbitration_reason = kReasonSourceHandover;
  input.deliberate_higher_priority_preemption = true;
  EXPECT_EQ(machine.update(input, 0.04).state, SafetyState::kStopping);
  machine.update(input, 0.05);
  EXPECT_EQ(machine.update(input, 0.31).state, SafetyState::kEnabling);
  EXPECT_EQ(machine.update(input, 0.32).state, SafetyState::kMotionEnabled);
}

TEST(SafetyMachine, FailedHandoverGuardEndsHolding)
{
  SafetyMachine machine({5.0, 0.25});
  machine.reset(0.0);
  auto input = ready_released();
  machine.update(input, 0.01);
  enable_source(machine, input, 0.02);
  input.selected_source_id = 250;
  input.selection_epoch = 2;
  input.arbitration_reason = kReasonSourceHandover;
  input.deliberate_higher_priority_preemption = true;
  machine.update(input, 0.04);
  input.selected_has_selection = false;
  machine.update(input, 0.05);
  EXPECT_EQ(machine.update(input, 0.31).state, SafetyState::kHolding);
}

TEST(SafetyMachine, TrustedSelectedStreamLossLatchesAfterControlledStop)
{
  SafetyMachine machine({5.0, 0.25});
  machine.reset(0.0);
  auto input = ready_released();
  machine.update(input, 0.01);
  enable_source(machine, input, 0.02);
  input.selected_stream_fresh = false;
  EXPECT_EQ(machine.update(input, 0.04).state, SafetyState::kStopping);
  EXPECT_EQ(machine.output().fault_mask, kFaultSelected);
  EXPECT_EQ(machine.output().reason, kReasonSelectedStale);
  machine.update(input, 0.05);
  EXPECT_EQ(machine.update(input, 0.31).state, SafetyState::kFaultHold);
  input.selected_stream_fresh = true;
  input.selected_has_selection = false;
  input.all_sources_released = true;
  machine.update(input, 0.32);
  EXPECT_EQ(machine.output().reason, kReasonResetRequired);
  EXPECT_TRUE(machine.request(SafetyRequest::kResetFault, input, 0.33).accepted);
  EXPECT_EQ(machine.output().state, SafetyState::kHolding);
}

TEST(SafetyMachine, ComponentFaultCannotResetUntilConditionAndSourcesClear)
{
  SafetyMachine machine({5.0, 0.25});
  machine.reset(0.0);
  auto input = ready_released();
  machine.update(input, 0.01);
  input.ready = false;
  input.condition_fault_mask = kFaultController;
  input.condition_reason = 20;
  EXPECT_EQ(machine.update(input, 0.02).state, SafetyState::kFaultHold);
  EXPECT_FALSE(machine.request(SafetyRequest::kResetFault, input, 0.03).accepted);
  input = ready_released();
  input.all_sources_released = false;
  EXPECT_FALSE(machine.request(SafetyRequest::kResetFault, input, 0.04).accepted);
  input.all_sources_released = true;
  EXPECT_TRUE(machine.request(SafetyRequest::kResetFault, input, 0.05).accepted);
}

TEST(SafetyMachine, ComponentFaultUsesControlledStopOnlyWhenPathIsAvailable)
{
  SafetyMachine machine({5.0, 0.25});
  machine.reset(0.0);
  auto input = ready_released();
  machine.update(input, 0.01);
  enable_source(machine, input, 0.02);
  input.ready = false;
  input.condition_fault_mask = kFaultController;
  input.condition_reason = 20;
  input.controlled_stop_available = true;
  EXPECT_EQ(machine.update(input, 0.04).state, SafetyState::kStopping);

  SafetyMachine unavailable({5.0, 0.25});
  unavailable.reset(0.0);
  input = ready_released();
  unavailable.update(input, 0.01);
  enable_source(unavailable, input, 0.02);
  input.ready = false;
  input.condition_fault_mask = kFaultController;
  input.condition_reason = 20;
  input.controlled_stop_available = false;
  EXPECT_EQ(unavailable.update(input, 0.04).state, SafetyState::kFaultHold);
}

TEST(SafetyMachine, FaultHoldAccumulatesConcurrentFaultCategories)
{
  SafetyMachine machine({5.0, 0.25});
  machine.reset(0.0);
  auto input = ready_released();
  machine.update(input, 0.01);
  input.ready = false;
  input.condition_fault_mask = kFaultController;
  input.condition_reason = 20;
  EXPECT_EQ(machine.update(input, 0.02).state, SafetyState::kFaultHold);
  input.condition_fault_mask = kFaultController | kFaultJointState | kFaultTime;
  input.condition_reason = 23;
  machine.update(input, 0.03);
  EXPECT_EQ(
    machine.output().fault_mask,
    kFaultController | kFaultJointState | kFaultTime);
  EXPECT_EQ(machine.output().state, SafetyState::kFaultHold);
}

TEST(SafetyMachine, LatchedHoldAndShutdownUseAcceptedTargets)
{
  SafetyMachine machine({5.0, 0.25});
  machine.reset(0.0);
  auto input = ready_released();
  machine.update(input, 0.01);
  enable_source(machine, input, 0.02);
  EXPECT_TRUE(machine.request(SafetyRequest::kLatchedHold, input, 0.04).accepted);
  EXPECT_EQ(machine.output().state, SafetyState::kStopping);
  machine.update(input, 0.05);
  EXPECT_EQ(machine.update(input, 0.31).state, SafetyState::kFaultHold);
  input.selected_has_selection = false;
  input.all_sources_released = true;
  EXPECT_TRUE(machine.request(SafetyRequest::kResetFault, input, 0.32).accepted);
  EXPECT_TRUE(machine.request(SafetyRequest::kShutdown, input, 0.33).accepted);
  EXPECT_EQ(machine.output().state, SafetyState::kShuttingDown);
}

TEST(SafetyMachine, InvalidTimingAndTimeAreRejected)
{
  EXPECT_THROW(SafetyMachine(SafetyMachineConfig{0.0, 0.25}), std::invalid_argument);
  EXPECT_THROW(
    SafetyMachine(SafetyMachineConfig{5.0, 0.25, false, -0.01}),
    std::invalid_argument);
  SafetyMachine machine({5.0, 0.25});
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(machine.reset(nan), std::invalid_argument);
  SafetyMachineInput input;
  EXPECT_THROW(machine.update(input, nan), std::invalid_argument);
}

TEST(SafetyMachine, StaleCandidateReasonRemainsRecoverable)
{
  SafetyMachine machine({5.0, 0.0});
  machine.reset(0.0);
  auto input = ready_released();
  machine.update(input, 0.01);
  enable_source(machine, input, 0.02);
  input.selected_has_selection = false;
  input.arbitration_reason = kReasonSourceStale;
  EXPECT_EQ(machine.update(input, 0.04).state, SafetyState::kStopping);
  machine.update(input, 0.05);
  EXPECT_EQ(machine.update(input, 0.06).state, SafetyState::kHolding);
  EXPECT_EQ(machine.output().fault_mask, 0U);
}

}  // namespace
