// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include "araco_locomotion/safe_command_guard.hpp"

#include <cmath>
#include <stdexcept>

namespace araco_locomotion
{
namespace
{

constexpr std::uint8_t kHold = 0;
constexpr std::uint8_t kExecute = 1;
constexpr std::uint8_t kLimited = 2;
constexpr std::uint8_t kControlledStop = 3;
constexpr std::uint16_t kReasonNone = 0;
constexpr std::uint16_t kReasonSafeCommandStale = 13;
constexpr std::uint16_t kReasonInternalError = 27;

}  // namespace

SafeCommandGuard::SafeCommandGuard(double timeout_s)
: timeout_s_(timeout_s)
{
  if (!std::isfinite(timeout_s) || timeout_s <= 0.0) {
    throw std::invalid_argument("safe-command timeout must be finite and positive");
  }
}

void SafeCommandGuard::reset()
{
  receipt_s_ = 0.0;
  release_epoch_floor_ = 0;
  active_epoch_ = 0;
  armed_ = false;
  observed_release_ = false;
  active_ = false;
  quarantined_ = false;
  reason_ = kReasonNone;
}

bool SafeCommandGuard::accept(
  std::uint64_t safety_epoch, std::uint8_t disposition,
  bool structurally_valid, double steady_now_s)
{
  if (!std::isfinite(steady_now_s)) {
    throw std::invalid_argument("safe-command receipt time must be finite");
  }
  armed_ = true;
  receipt_s_ = steady_now_s;
  if (!structurally_valid || disposition > kControlledStop) {
    active_ = false;
    observed_release_ = false;
    quarantined_ = true;
    reason_ = kReasonInternalError;
    return false;
  }
  if (disposition == kHold || disposition == kControlledStop) {
    active_ = false;
    observed_release_ = true;
    quarantined_ = false;
    release_epoch_floor_ = safety_epoch;
    reason_ = kReasonNone;
    return true;
  }

  const bool activation_edge = !active_;
  if (!observed_release_ || quarantined_ ||
    (activation_edge && safety_epoch <= release_epoch_floor_) ||
    (!activation_edge && safety_epoch < active_epoch_))
  {
    active_ = false;
    observed_release_ = false;
    quarantined_ = true;
    reason_ = kReasonInternalError;
    return false;
  }
  if (disposition != kExecute && disposition != kLimited) {
    active_ = false;
    observed_release_ = false;
    quarantined_ = true;
    reason_ = kReasonInternalError;
    return false;
  }
  active_ = true;
  active_epoch_ = safety_epoch;
  reason_ = kReasonNone;
  return true;
}

SafeGuardResult SafeCommandGuard::evaluate(double steady_now_s)
{
  if (!std::isfinite(steady_now_s)) {
    throw std::invalid_argument("safe-command evaluation time must be finite");
  }
  bool fresh = armed_ && steady_now_s - receipt_s_ <= timeout_s_;
  if (armed_ && !fresh) {
    active_ = false;
    observed_release_ = false;
    quarantined_ = true;
    reason_ = kReasonSafeCommandStale;
  }
  return {armed_, fresh, fresh && active_ && !quarantined_, quarantined_, reason_};
}

}  // namespace araco_locomotion
