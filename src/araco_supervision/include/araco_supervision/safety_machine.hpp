// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#ifndef ARACO_SUPERVISION__SAFETY_MACHINE_HPP_
#define ARACO_SUPERVISION__SAFETY_MACHINE_HPP_

#include <cstdint>

namespace araco_supervision
{

enum class SafetyState : std::uint8_t
{
  kInitializing = 0,
  kInactive = 1,
  kHolding = 2,
  kEnabling = 3,
  kMotionEnabled = 4,
  kStopping = 5,
  kFaultHold = 6,
  kShuttingDown = 7,
};

enum class SafetyRequest : std::uint8_t
{
  kHold = 1,
  kEnableMotion = 2,
  kResetFault = 3,
  kShutdown = 4,
  kLatchedHold = 5,
};

struct SafetyMachineConfig
{
  double enable_wait_timeout_s{5.0};
  double stable_hold_dwell_s{0.25};
  bool auto_enable_once_from_neutral_standing_source{false};
};

struct SafetyMachineInput
{
  bool ready{false};
  std::uint64_t condition_fault_mask{0};
  std::uint16_t condition_reason{0};
  bool selected_stream_armed{false};
  bool selected_stream_fresh{false};
  bool selected_has_selection{false};
  bool selected_valid{false};
  bool selected_neutral_standing{false};
  std::uint32_t selected_source_id{0};
  std::uint64_t selection_epoch{0};
  bool arbitration_status_fresh{false};
  std::uint16_t arbitration_reason{0};
  bool deliberate_higher_priority_preemption{false};
  bool all_sources_released{false};
  bool locomotion_stably_holding{false};
  bool controlled_stop_available{false};
};

struct SafetyMachineOutput
{
  SafetyState state{SafetyState::kInitializing};
  std::uint16_t reason{1};
  std::uint64_t safety_epoch{0};
  std::uint64_t fault_mask{0};
  bool reset_required{false};
};

struct SafetyRequestResult
{
  bool accepted{false};
  bool completed{false};
};

class SafetyMachine
{
public:
  explicit SafetyMachine(SafetyMachineConfig config);

  void reset(double steady_now_s);
  SafetyRequestResult request(
    SafetyRequest request, const SafetyMachineInput & input, double steady_now_s);
  SafetyMachineOutput update(const SafetyMachineInput & input, double steady_now_s);
  [[nodiscard]] SafetyMachineOutput output() const;

private:
  enum class StopTarget
  {
    kHolding,
    kFaultHold,
    kHandover,
    kShutdown,
  };

  void transition(SafetyState next, std::uint16_t reason);
  void begin_stop(StopTarget target, std::uint16_t reason);
  void evaluate_faults(const SafetyMachineInput & input);
  bool pending_handover_valid(const SafetyMachineInput & input) const;

  SafetyMachineConfig config_;
  SafetyState state_{SafetyState::kInitializing};
  StopTarget stop_target_{StopTarget::kHolding};
  std::uint16_t reason_{1};
  std::uint64_t safety_epoch_{0};
  std::uint64_t fault_mask_{0};
  std::uint64_t enable_selection_epoch_floor_{0};
  std::uint64_t pending_selection_epoch_{0};
  std::uint32_t pending_source_id_{0};
  double enable_deadline_s_{0.0};
  double stable_hold_since_s_{-1.0};
  bool ever_ready_{false};
  bool automatic_handover_enable_{false};
  bool automatic_enable_consumed_{false};
};

}  // namespace araco_supervision

#endif  // ARACO_SUPERVISION__SAFETY_MACHINE_HPP_
