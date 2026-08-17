// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

#include "araco_interfaces/msg/arbitration_status.hpp"
#include "araco_interfaces/msg/command_candidate.hpp"
#include "araco_interfaces/msg/selected_command.hpp"
#include "araco_supervision/command_arbiter_parameters.hpp"
#include "araco_supervision/command_policy.hpp"
#include "araco_supervision/source_arbiter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace araco_supervision
{

class CommandArbiterNode final : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit CommandArbiterNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("command_arbiter", "araco", options)
  {
  }

private:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Candidate = araco_interfaces::msg::CommandCandidate;
  using SteadyClock = std::chrono::steady_clock;

  static StaticIntent policy_intent(const Candidate & message)
  {
    const auto & twist = message.intent.planar_velocity;
    const auto & pose = message.intent.body_pose_offset;
    return {
      message.intent.gait,
      {twist.linear.x, twist.linear.y, twist.linear.z,
        twist.angular.x, twist.angular.y, twist.angular.z},
      {pose.position.x, pose.position.y, pose.position.z},
      {pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w},
    };
  }

  static double steady_now_s()
  {
    return std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count();
  }

  BodyEnvelope envelope() const
  {
    return {
      params_.body_envelope.planar_speed_hard_m_s,
      params_.body_envelope.planar_speed_hard_m_s,
      params_.body_envelope.yaw_rate_hard_rad_s,
      params_.body_envelope.yaw_rate_hard_rad_s,
      params_.body_envelope.xy_hard_m,
      params_.body_envelope.z_hard_lower_m,
      params_.body_envelope.z_hard_upper_m,
      params_.body_envelope.roll_pitch_hard_rad,
      params_.body_envelope.yaw_hard_rad,
      params_.body_envelope.xy_hard_m,
      params_.body_envelope.z_hard_lower_m,
      params_.body_envelope.z_hard_upper_m,
      params_.body_envelope.roll_pitch_hard_rad,
      params_.body_envelope.yaw_hard_rad,
      params_.body_envelope.quaternion_norm_tolerance,
      params_.body_envelope.reserved_twist_tolerance,
      params_.body_envelope.stand_velocity_tolerance,
    };
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    try {
      param_listener_ = std::make_shared<command_arbiter::ParamListener>(shared_from_this());
      params_ = param_listener_->get_params();
      policy_ = std::make_unique<SourceArbiter>(std::vector<SourceConfig>{
          SourceConfig{
            static_cast<std::uint32_t>(params_.teleop_source_id),
            static_cast<std::uint32_t>(params_.teleop_priority),
            params_.teleop_timeout_s, params_.teleop_enabled},
          SourceConfig{
            static_cast<std::uint32_t>(params_.system_test_source_id),
            static_cast<std::uint32_t>(params_.system_test_priority),
            params_.system_test_timeout_s, params_.system_test_enabled},
        });
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "parameter/source validation failed: %s", error.what());
      return CallbackReturn::FAILURE;
    }

    selected_publisher_ = create_publisher<araco_interfaces::msg::SelectedCommand>(
      "/araco/command/selected", rclcpp::QoS(1).reliable());
    status_publisher_ = create_publisher<araco_interfaces::msg::ArbitrationStatus>(
      "/araco/command/arbitration_status", rclcpp::QoS(1).reliable());
    teleop_subscription_ = create_subscription<Candidate>(
      "/araco/command/candidates/teleop", rclcpp::QoS(1).best_effort(),
      [this](Candidate::ConstSharedPtr message) {
        accept_candidate(
          static_cast<std::uint32_t>(params_.teleop_source_id), *message,
          teleop_candidate_);
      });
    system_test_subscription_ = create_subscription<Candidate>(
      "/araco/command/candidates/system_test", rclcpp::QoS(1).best_effort(),
      [this](Candidate::ConstSharedPtr message) {
        accept_candidate(
          static_cast<std::uint32_t>(params_.system_test_source_id), *message,
          system_test_candidate_);
      });
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    selected_publisher_->on_activate();
    status_publisher_->on_activate();
    policy_->reset();
    teleop_candidate_ = Candidate{};
    system_test_candidate_ = Candidate{};
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / params_.loop_rate_hz),
      [this]() {publish_selection();});
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    selected_publisher_->on_deactivate();
    status_publisher_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  void accept_candidate(
    std::uint32_t source_id, const Candidate & message, Candidate & storage)
  {
    const auto validation = validate_and_limit_static_intent(
      policy_intent(message), message.header.frame_id, envelope(), false);
    const bool valid = validation.validity == CommandValidity::kValid;
    const bool accepted = policy_->accept(
      source_id, message.sequence, message.active, valid, steady_now_s());
    if (accepted) {
      storage = message;
    }
  }

  const Candidate * candidate(std::uint32_t source_id) const
  {
    if (source_id == static_cast<std::uint32_t>(params_.teleop_source_id)) {
      return &teleop_candidate_;
    }
    if (source_id == static_cast<std::uint32_t>(params_.system_test_source_id)) {
      return &system_test_candidate_;
    }
    return nullptr;
  }

  void publish_selection()
  {
    const auto decision = policy_->evaluate(steady_now_s());
    araco_interfaces::msg::SelectedCommand selected;
    selected.header.stamp = now();
    selected.header.frame_id = "base_link";
    selected.selection_epoch = decision.selection_epoch;
    selected.has_selection = decision.has_selection;
    if (decision.has_selection) {
      const auto * source = candidate(decision.source_id);
      if (source == nullptr || source->sequence != decision.source_sequence) {
        RCLCPP_ERROR(get_logger(), "arbiter policy/candidate storage invariant failed");
        return;
      }
      selected.source_id = decision.source_id;
      selected.source_stamp = source->header.stamp;
      selected.source_sequence = source->sequence;
      selected.intent = source->intent;
    } else {
      selected.intent.body_pose_offset.orientation.w = 1.0;
    }
    selected_publisher_->publish(selected);

    araco_interfaces::msg::ArbitrationStatus status;
    status.header = selected.header;
    status.selection_epoch = decision.selection_epoch;
    status.previous_source_id = decision.previous_source_id;
    status.selected_source_id = decision.source_id;
    status.selected_activation_epoch = decision.activation_epoch;
    status.reason_code = decision.reason_code;
    status.deliberate_higher_priority_preemption =
      decision.deliberate_higher_priority_preemption;
    status.all_sources_released = policy_->all_sources_released();
    status.quarantined_source_ids = decision.quarantined_source_ids;
    status_publisher_->publish(status);
  }

  std::shared_ptr<command_arbiter::ParamListener> param_listener_;
  command_arbiter::Params params_;
  std::unique_ptr<SourceArbiter> policy_;
  rclcpp_lifecycle::LifecyclePublisher<araco_interfaces::msg::SelectedCommand>::SharedPtr
    selected_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<araco_interfaces::msg::ArbitrationStatus>::SharedPtr
    status_publisher_;
  rclcpp::Subscription<Candidate>::SharedPtr teleop_subscription_;
  rclcpp::Subscription<Candidate>::SharedPtr system_test_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  Candidate teleop_candidate_;
  Candidate system_test_candidate_;
};

}  // namespace araco_supervision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<araco_supervision::CommandArbiterNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
