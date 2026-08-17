// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#ifndef ARACO_LOCOMOTION__SAFE_COMMAND_GUARD_HPP_
#define ARACO_LOCOMOTION__SAFE_COMMAND_GUARD_HPP_

#include <cstdint>

namespace araco_locomotion
{

struct SafeGuardResult
{
  bool armed{false};
  bool fresh{false};
  bool executable{false};
  bool quarantined{false};
  std::uint16_t reason{0};
};

class SafeCommandGuard
{
public:
  explicit SafeCommandGuard(double timeout_s);

  void reset();
  bool accept(
    std::uint64_t safety_epoch, std::uint8_t disposition,
    bool structurally_valid, double steady_now_s);
  SafeGuardResult evaluate(double steady_now_s);

private:
  double timeout_s_;
  double receipt_s_{0.0};
  std::uint64_t release_epoch_floor_{0};
  std::uint64_t active_epoch_{0};
  bool armed_{false};
  bool observed_release_{false};
  bool active_{false};
  bool quarantined_{false};
  std::uint16_t reason_{0};
};

}  // namespace araco_locomotion

#endif  // ARACO_LOCOMOTION__SAFE_COMMAND_GUARD_HPP_
