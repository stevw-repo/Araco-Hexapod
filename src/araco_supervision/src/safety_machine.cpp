// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include "araco_supervision/safety_machine.hpp"

#include <cmath>
#include <stdexcept>

namespace araco_supervision
{
namespace
{

constexpr std::uint16_t kReasonNone = 0;
constexpr std::uint16_t kReasonStartup = 1;
constexpr std::uint16_t kReasonInactive = 2;
constexpr std::uint16_t kReasonHolding = 3;
constexpr std::uint16_t kReasonWaitingForSource = 4;
constexpr std::uint16_t kReasonManualHold = 5;
constexpr std::uint16_t kReasonNoSource = 6;
constexpr std::uint16_t kReasonSourceReleased = 7;
constexpr std::uint16_t kReasonSourceHandover = 9;
constexpr std::uint16_t kReasonSelectedCommandStale = 12;
constexpr std::uint16_t kReasonShutdownRequested = 24;
constexpr std::uint16_t kReasonSoftwareLatchedHold = 25;
constexpr std::uint16_t kReasonResetRequired = 26;

constexpr std::uint64_t kFaultSelectedCommandStream = 1;
constexpr std::uint64_t kFaultSoftwareLatch = 256;

}  // namespace

SafetyMachine::SafetyMachine(SafetyMachineConfig config)
: config_(config)
{
  if (!std::isfinite(config.enable_wait_timeout_s) || config.enable_wait_timeout_s <= 0.0 ||
    !std::isfinite(config.stable_hold_dwell_s) || config.stable_hold_dwell_s < 0.0 ||
    !std::isfinite(config.startup_readiness_stable_s) ||
    config.startup_readiness_stable_s < 0.0)
  {
    throw std::invalid_argument("safety machine timing must be finite and non-negative");
  }
}

void SafetyMachine::reset(double steady_now_s)
{
  if (!std::isfinite(steady_now_s)) {
    throw std::invalid_argument("safety steady time must be finite");
  }
  state_ = SafetyState::kInitializing;
  stop_target_ = StopTarget::kHolding;
  reason_ = kReasonStartup;
  safety_epoch_ = 0;
  fault_mask_ = 0;
  enable_selection_epoch_floor_ = 0;
  pending_selection_epoch_ = 0;
  pending_source_id_ = 0;
  enable_deadline_s_ = steady_now_s;
  stable_hold_since_s_ = -1.0;
  startup_ready_since_s_ = -1.0;
  ever_ready_ = false;
  automatic_handover_enable_ = false;
  automatic_enable_consumed_ = false;
}

void SafetyMachine::transition(SafetyState next, std::uint16_t reason)
{
  if (state_ != next) {
    state_ = next;
    ++safety_epoch_;
  }
  reason_ = reason;
}

void SafetyMachine::begin_stop(StopTarget target, std::uint16_t reason)
{
  stop_target_ = target;
  stable_hold_since_s_ = -1.0;
  transition(SafetyState::kStopping, reason);
}

SafetyRequestResult SafetyMachine::request(
  SafetyRequest request_value, const SafetyMachineInput & input, double steady_now_s)
{
  if (!std::isfinite(steady_now_s)) {
    throw std::invalid_argument("safety steady time must be finite");
  }
  SafetyRequestResult result;
  if (state_ == SafetyState::kShuttingDown) {
    return result;
  }

  switch (request_value) {
    case SafetyRequest::kHold:
      result.accepted = true;
      if (state_ == SafetyState::kMotionEnabled || state_ == SafetyState::kEnabling ||
        state_ == SafetyState::kStopping)
      {
        begin_stop(StopTarget::kHolding, kReasonManualHold);
      } else if (state_ == SafetyState::kHolding) {
        reason_ = kReasonManualHold;
        result.completed = true;
      }
      break;
    case SafetyRequest::kEnableMotion:
      result.accepted = state_ == SafetyState::kHolding && input.ready && fault_mask_ == 0 &&
        input.arbitration_status_fresh && input.all_sources_released &&
        !input.selected_has_selection;
      if (result.accepted) {
        enable_selection_epoch_floor_ = input.selection_epoch;
        enable_deadline_s_ = steady_now_s + config_.enable_wait_timeout_s;
        automatic_handover_enable_ = false;
        transition(SafetyState::kEnabling, kReasonWaitingForSource);
      }
      break;
    case SafetyRequest::kResetFault:
      result.accepted = state_ == SafetyState::kFaultHold && input.ready &&
        input.condition_fault_mask == 0 && input.all_sources_released &&
        input.locomotion_stably_holding;
      if (result.accepted) {
        fault_mask_ = 0;
        transition(SafetyState::kHolding, kReasonHolding);
        result.completed = true;
      }
      break;
    case SafetyRequest::kShutdown:
      result.accepted = true;
      if (state_ == SafetyState::kHolding && input.locomotion_stably_holding) {
        transition(SafetyState::kShuttingDown, kReasonShutdownRequested);
        result.completed = true;
      } else if (state_ == SafetyState::kFaultHold) {
        transition(SafetyState::kShuttingDown, kReasonShutdownRequested);
        result.completed = true;
      } else {
        begin_stop(StopTarget::kShutdown, kReasonShutdownRequested);
      }
      break;
    case SafetyRequest::kLatchedHold:
      result.accepted = true;
      fault_mask_ |= kFaultSoftwareLatch;
      if ((state_ == SafetyState::kMotionEnabled || state_ == SafetyState::kEnabling) &&
        input.ready)
      {
        begin_stop(StopTarget::kFaultHold, kReasonSoftwareLatchedHold);
      } else {
        transition(SafetyState::kFaultHold, kReasonSoftwareLatchedHold);
        result.completed = true;
      }
      break;
  }
  return result;
}

void SafetyMachine::evaluate_faults(const SafetyMachineInput & input)
{
  std::uint64_t current_faults = input.condition_fault_mask;
  std::uint16_t current_reason = input.condition_reason;
  if (input.selected_stream_armed && !input.selected_stream_fresh) {
    current_faults |= kFaultSelectedCommandStream;
    current_reason = kReasonSelectedCommandStale;
  }
  if (current_faults == 0 || state_ == SafetyState::kShuttingDown) {
    return;
  }
  if (state_ == SafetyState::kFaultHold) {
    fault_mask_ |= current_faults;
    return;
  }

  fault_mask_ |= current_faults;
  if ((state_ == SafetyState::kMotionEnabled || state_ == SafetyState::kEnabling) &&
    input.controlled_stop_available)
  {
    begin_stop(StopTarget::kFaultHold, current_reason);
  } else {
    transition(SafetyState::kFaultHold, current_reason);
  }
}

bool SafetyMachine::pending_handover_valid(const SafetyMachineInput & input) const
{
  return input.selected_stream_fresh && input.arbitration_status_fresh &&
         input.selected_has_selection && input.selected_valid &&
         input.selected_source_id == pending_source_id_ &&
         input.selection_epoch == pending_selection_epoch_;
}

SafetyMachineOutput SafetyMachine::update(
  const SafetyMachineInput & input, double steady_now_s)
{
  if (!std::isfinite(steady_now_s)) {
    throw std::invalid_argument("safety steady time must be finite");
  }

  if (state_ == SafetyState::kInitializing) {
    transition(SafetyState::kInactive, kReasonInactive);
    startup_ready_since_s_ = input.ready ? steady_now_s : -1.0;
    if (input.ready && config_.startup_readiness_stable_s == 0.0) {
      ever_ready_ = true;
      transition(SafetyState::kHolding, kReasonHolding);
    }
    return output();
  }
  if (state_ == SafetyState::kInactive) {
    if (!input.ready) {
      startup_ready_since_s_ = -1.0;
      return output();
    }
    if (startup_ready_since_s_ < 0.0) {
      startup_ready_since_s_ = steady_now_s;
    }
    if (steady_now_s - startup_ready_since_s_ >= config_.startup_readiness_stable_s) {
      ever_ready_ = true;
      transition(SafetyState::kHolding, kReasonHolding);
    }
    return output();
  }
  if (state_ == SafetyState::kShuttingDown) {
    return output();
  }

  evaluate_faults(input);
  if (state_ == SafetyState::kFaultHold) {
    if (input.condition_fault_mask == 0 &&
      !(input.selected_stream_armed && !input.selected_stream_fresh))
    {
      reason_ = kReasonResetRequired;
    }
    return output();
  }

  if (state_ == SafetyState::kHolding) {
    const bool automatic_enable_ready =
      config_.auto_enable_once_from_neutral_standing_source &&
      !automatic_enable_consumed_ && input.ready && fault_mask_ == 0 &&
      input.selected_stream_fresh &&
      input.arbitration_status_fresh && input.selected_has_selection &&
      input.selected_valid && input.selected_neutral_standing;
    if (automatic_enable_ready) {
      automatic_enable_consumed_ = true;
      transition(SafetyState::kMotionEnabled, kReasonNone);
    }
    return output();
  }

  if (state_ == SafetyState::kEnabling) {
    const bool executable = input.selected_stream_fresh && input.arbitration_status_fresh &&
      input.selected_has_selection && input.selected_valid;
    if (automatic_handover_enable_) {
      if (pending_handover_valid(input)) {
        automatic_handover_enable_ = false;
        transition(SafetyState::kMotionEnabled, kReasonNone);
      } else {
        automatic_handover_enable_ = false;
        transition(SafetyState::kHolding, kReasonNoSource);
      }
    } else if (executable && input.selection_epoch > enable_selection_epoch_floor_) {
      transition(SafetyState::kMotionEnabled, kReasonNone);
    } else if (steady_now_s >= enable_deadline_s_) {
      transition(SafetyState::kHolding, kReasonNoSource);
    }
    return output();
  }

  if (state_ == SafetyState::kMotionEnabled) {
    if (input.deliberate_higher_priority_preemption &&
      input.arbitration_status_fresh &&
      input.arbitration_reason == kReasonSourceHandover &&
      input.selected_has_selection && input.selected_valid)
    {
      pending_source_id_ = input.selected_source_id;
      pending_selection_epoch_ = input.selection_epoch;
      begin_stop(StopTarget::kHandover, kReasonSourceHandover);
      return output();
    }
    if (!input.selected_stream_fresh) {
      // evaluate_faults handles an armed trusted-stream loss above.
      return output();
    }
    if ((!input.selected_has_selection || !input.selected_valid) &&
      input.arbitration_status_fresh)
    {
      const auto reason = input.arbitration_reason == kReasonNone ?
        kReasonSourceReleased : input.arbitration_reason;
      begin_stop(StopTarget::kHolding, reason);
    }
    return output();
  }

  if (state_ == SafetyState::kStopping) {
    if (stop_target_ == StopTarget::kHandover && !pending_handover_valid(input)) {
      stop_target_ = StopTarget::kHolding;
    }
    if (!input.locomotion_stably_holding) {
      stable_hold_since_s_ = -1.0;
      return output();
    }
    if (stable_hold_since_s_ < 0.0) {
      stable_hold_since_s_ = steady_now_s;
      return output();
    }
    if (steady_now_s - stable_hold_since_s_ < config_.stable_hold_dwell_s) {
      return output();
    }
    stable_hold_since_s_ = -1.0;
    switch (stop_target_) {
      case StopTarget::kHolding:
        transition(SafetyState::kHolding, reason_);
        break;
      case StopTarget::kFaultHold:
        transition(SafetyState::kFaultHold, reason_);
        break;
      case StopTarget::kShutdown:
        transition(SafetyState::kShuttingDown, kReasonShutdownRequested);
        break;
      case StopTarget::kHandover:
        automatic_handover_enable_ = true;
        transition(SafetyState::kEnabling, kReasonWaitingForSource);
        break;
    }
  }
  return output();
}

SafetyMachineOutput SafetyMachine::output() const
{
  return {state_, reason_, safety_epoch_, fault_mask_, fault_mask_ != 0};
}

}  // namespace araco_supervision
