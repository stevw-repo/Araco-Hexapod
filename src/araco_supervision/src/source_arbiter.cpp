// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include "araco_supervision/source_arbiter.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

namespace araco_supervision
{
namespace
{

constexpr std::uint16_t kReasonNone = 0;
constexpr std::uint16_t kReasonSourceReleased = 7;
constexpr std::uint16_t kReasonSourceStale = 8;
constexpr std::uint16_t kReasonSourceHandover = 9;
constexpr std::uint16_t kReasonSourceInvalid = 10;

}  // namespace

SourceArbiter::SourceArbiter(std::vector<SourceConfig> configuration)
{
  std::set<std::uint32_t> ids;
  std::set<std::uint32_t> priorities;
  for (const auto & item : configuration) {
    if (item.id == 0 || !std::isfinite(item.timeout_s) || item.timeout_s <= 0.0 ||
      !ids.insert(item.id).second || !priorities.insert(item.priority).second)
    {
      throw std::invalid_argument("source IDs/priorities must be unique and timeouts positive");
    }
    sources_.push_back(SourceState{item});
  }
  if (sources_.empty()) {
    throw std::invalid_argument("at least one command source is required");
  }
}

void SourceArbiter::reset()
{
  for (auto & source : sources_) {
    const auto config = source.config;
    source = SourceState{config};
  }
  selected_source_id_ = 0;
  selection_epoch_ = 0;
  activation_epoch_ = 0;
  selected_at_activation_epoch_ = 0;
  pending_loss_reason_ = kReasonNone;
  last_previous_source_id_ = 0;
  last_transition_reason_ = kReasonNone;
  last_deliberate_preemption_ = false;
}

SourceArbiter::SourceState * SourceArbiter::find(std::uint32_t source_id)
{
  const auto match = std::find_if(
    sources_.begin(), sources_.end(), [source_id](const auto & source) {
      return source.config.id == source_id;
    });
  return match == sources_.end() ? nullptr : &*match;
}

const SourceArbiter::SourceState * SourceArbiter::find(std::uint32_t source_id) const
{
  const auto match = std::find_if(
    sources_.cbegin(), sources_.cend(), [source_id](const auto & source) {
      return source.config.id == source_id;
    });
  return match == sources_.cend() ? nullptr : &*match;
}

bool SourceArbiter::accept(
  std::uint32_t source_id, std::uint64_t sequence, bool active,
  bool valid, double steady_now_s)
{
  auto * source = find(source_id);
  if (source == nullptr || !source->config.enabled || !std::isfinite(steady_now_s)) {
    return false;
  }

  source->receipt_s = steady_now_s;
  if (!valid) {
    source->active = false;
    source->quarantined = true;
    source->observed_release = false;
    source->last_loss_reason = kReasonSourceInvalid;
    if (selected_source_id_ == source_id) {
      pending_loss_reason_ = kReasonSourceInvalid;
    }
    return false;
  }

  // A valid release defines a new session even when a restarted publisher's
  // counter is lower than the previous session. No active sample can perform
  // that reset.
  if (!active) {
    source->active = false;
    source->quarantined = false;
    source->observed_release = true;
    source->have_sequence = true;
    source->sequence = sequence;
    source->last_loss_reason = kReasonSourceReleased;
    if (selected_source_id_ == source_id) {
      pending_loss_reason_ = kReasonSourceReleased;
    }
    return true;
  }

  const bool sequence_advanced = !source->have_sequence || sequence > source->sequence;
  if (!source->observed_release || source->quarantined || !sequence_advanced) {
    source->active = false;
    source->quarantined = true;
    source->observed_release = false;
    source->last_loss_reason = kReasonSourceInvalid;
    if (selected_source_id_ == source_id) {
      pending_loss_reason_ = kReasonSourceInvalid;
    }
    return false;
  }

  const bool activation_edge = !source->active;
  source->active = true;
  source->have_sequence = true;
  source->sequence = sequence;
  source->last_loss_reason = kReasonNone;
  if (activation_edge) {
    source->activation_epoch = ++activation_epoch_;
  }
  return true;
}

bool SourceArbiter::eligible(SourceState & source, double steady_now_s)
{
  if (!source.config.enabled || !source.active || source.quarantined ||
    !source.observed_release)
  {
    return false;
  }
  if (steady_now_s - source.receipt_s <= source.config.timeout_s) {
    return true;
  }
  source.active = false;
  source.quarantined = true;
  source.observed_release = false;
  source.last_loss_reason = kReasonSourceStale;
  if (selected_source_id_ == source.config.id) {
    pending_loss_reason_ = kReasonSourceStale;
  }
  return false;
}

const SourceArbiter::SourceState * SourceArbiter::best_eligible(double steady_now_s)
{
  SourceState * best = nullptr;
  for (auto & source : sources_) {
    if (eligible(source, steady_now_s) &&
      (best == nullptr || source.config.priority > best->config.priority))
    {
      best = &source;
    }
  }
  return best;
}

SelectionDecision SourceArbiter::evaluate(double steady_now_s)
{
  if (!std::isfinite(steady_now_s)) {
    throw std::invalid_argument("arbiter steady time must be finite");
  }
  const SourceState * selected = best_eligible(steady_now_s);
  const std::uint32_t next_id = selected == nullptr ? 0 : selected->config.id;
  const std::uint32_t previous_id = selected_source_id_;
  bool deliberate_preemption = false;
  std::uint16_t reason = kReasonNone;
  if (next_id != previous_id) {
    const auto * previous = find(previous_id);
    deliberate_preemption = previous != nullptr && selected != nullptr &&
      selected->config.priority > previous->config.priority &&
      selected->activation_epoch > selected_at_activation_epoch_;
    if (deliberate_preemption) {
      reason = kReasonSourceHandover;
    } else if (pending_loss_reason_ != kReasonNone) {
      reason = pending_loss_reason_;
    } else if (previous != nullptr && previous->last_loss_reason != kReasonNone) {
      reason = previous->last_loss_reason;
    }
    ++selection_epoch_;
    selected_source_id_ = next_id;
    selected_at_activation_epoch_ = activation_epoch_;
    pending_loss_reason_ = kReasonNone;
    last_previous_source_id_ = previous_id;
    last_transition_reason_ = reason;
    last_deliberate_preemption_ = deliberate_preemption;
  }

  SelectionDecision decision;
  decision.selection_epoch = selection_epoch_;
  decision.has_selection = selected != nullptr;
  decision.source_id = next_id;
  decision.previous_source_id = last_previous_source_id_;
  decision.reason_code = last_transition_reason_;
  decision.deliberate_higher_priority_preemption = last_deliberate_preemption_;
  if (selected != nullptr) {
    decision.source_sequence = selected->sequence;
    decision.activation_epoch = selected->activation_epoch;
  }
  for (const auto & source : sources_) {
    if (source.quarantined) {
      decision.quarantined_source_ids.push_back(source.config.id);
    }
  }
  std::sort(
    decision.quarantined_source_ids.begin(), decision.quarantined_source_ids.end());
  return decision;
}

bool SourceArbiter::all_sources_released() const
{
  return std::all_of(sources_.cbegin(), sources_.cend(), [](const auto & source) {
             return !source.config.enabled ||
                    (source.observed_release && !source.active && !source.quarantined);
  });
}

}  // namespace araco_supervision
