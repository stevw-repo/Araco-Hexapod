// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#ifndef ARACO_LOCOMOTION__STEADY_HEARTBEAT_HPP_
#define ARACO_LOCOMOTION__STEADY_HEARTBEAT_HPP_

#include <cmath>
#include <stdexcept>

namespace araco_locomotion
{

struct HeartbeatDecision
{
  bool publish{false};
  bool callback_delayed{false};
  double callback_gap_s{0.0};
};

class SteadyHeartbeat
{
public:
  SteadyHeartbeat(double publication_period_s, double delayed_callback_s)
  : publication_period_s_(publication_period_s),
    delayed_callback_s_(delayed_callback_s)
  {
    if (!std::isfinite(publication_period_s) || publication_period_s <= 0.0 ||
      !std::isfinite(delayed_callback_s) || delayed_callback_s <= publication_period_s)
    {
      throw std::invalid_argument(
              "heartbeat periods must be finite, positive, and leave delay margin");
    }
  }

  void reset()
  {
    initialized_ = false;
    last_callback_s_ = 0.0;
    last_publication_s_ = 0.0;
  }

  HeartbeatDecision update(double steady_now_s)
  {
    if (!std::isfinite(steady_now_s)) {
      throw std::invalid_argument("heartbeat time must be finite");
    }
    if (!initialized_) {
      initialized_ = true;
      last_callback_s_ = steady_now_s;
      last_publication_s_ = steady_now_s;
      return {true, false, 0.0};
    }
    if (steady_now_s < last_callback_s_) {
      throw std::invalid_argument("steady heartbeat time moved backwards");
    }

    const double callback_gap_s = steady_now_s - last_callback_s_;
    const bool publish = steady_now_s - last_publication_s_ + 1.0e-12 >=
      publication_period_s_;
    last_callback_s_ = steady_now_s;
    if (publish) {
      // Do not replay a burst after a delayed callback. One current health
      // sample is more useful than queued historical heartbeats.
      last_publication_s_ = steady_now_s;
    }
    return {publish, callback_gap_s > delayed_callback_s_, callback_gap_s};
  }

private:
  double publication_period_s_;
  double delayed_callback_s_;
  double last_callback_s_{0.0};
  double last_publication_s_{0.0};
  bool initialized_{false};
};

}  // namespace araco_locomotion

#endif  // ARACO_LOCOMOTION__STEADY_HEARTBEAT_HPP_
