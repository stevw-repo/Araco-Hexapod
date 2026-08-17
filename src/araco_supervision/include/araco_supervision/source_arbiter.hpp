// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#ifndef ARACO_SUPERVISION__SOURCE_ARBITER_HPP_
#define ARACO_SUPERVISION__SOURCE_ARBITER_HPP_

#include <cstdint>
#include <vector>

namespace araco_supervision
{

struct SourceConfig
{
  std::uint32_t id{0};
  std::uint32_t priority{0};
  double timeout_s{0.0};
  bool enabled{false};
};

struct SelectionDecision
{
  std::uint64_t selection_epoch{0};
  bool has_selection{false};
  std::uint32_t source_id{0};
  std::uint32_t previous_source_id{0};
  std::uint64_t source_sequence{0};
  std::uint64_t activation_epoch{0};
  std::uint16_t reason_code{0};
  bool deliberate_higher_priority_preemption{false};
  std::vector<std::uint32_t> quarantined_source_ids;
};

class SourceArbiter
{
public:
  explicit SourceArbiter(std::vector<SourceConfig> configuration);

  void reset();

  // A structurally invalid sample is supplied with valid=false. A valid
  // inactive sample is the only event that clears quarantine and starts a new
  // publisher session; its sequence becomes that session's floor.
  bool accept(
    std::uint32_t source_id, std::uint64_t sequence, bool active,
    bool valid, double steady_now_s);

  [[nodiscard]] SelectionDecision evaluate(double steady_now_s);
  [[nodiscard]] bool all_sources_released() const;

private:
  struct SourceState
  {
    SourceConfig config;
    bool observed_release{false};
    bool active{false};
    bool quarantined{false};
    bool have_sequence{false};
    std::uint64_t sequence{0};
    std::uint64_t activation_epoch{0};
    double receipt_s{0.0};
    std::uint16_t last_loss_reason{0};
  };

  SourceState * find(std::uint32_t source_id);
  const SourceState * find(std::uint32_t source_id) const;
  bool eligible(SourceState & source, double steady_now_s);
  const SourceState * best_eligible(double steady_now_s);

  std::vector<SourceState> sources_;
  std::uint32_t selected_source_id_{0};
  std::uint64_t selection_epoch_{0};
  std::uint64_t activation_epoch_{0};
  std::uint64_t selected_at_activation_epoch_{0};
  std::uint16_t pending_loss_reason_{0};
  std::uint32_t last_previous_source_id_{0};
  std::uint16_t last_transition_reason_{0};
  bool last_deliberate_preemption_{false};
};

}  // namespace araco_supervision

#endif  // ARACO_SUPERVISION__SOURCE_ARBITER_HPP_
