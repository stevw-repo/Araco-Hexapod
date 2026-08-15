// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include <chrono>
#include <memory>

#include "araco_interfaces/msg/command_candidate.hpp"
#include "araco_interfaces/msg/selected_command.hpp"
#include "araco_supervision/command_arbiter_parameters.hpp"
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
  using SteadyClock = std::chrono::steady_clock;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    try {
      param_listener_ = std::make_shared<command_arbiter::ParamListener>(shared_from_this());
      params_ = param_listener_->get_params();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "parameter validation failed: %s", error.what());
      return CallbackReturn::FAILURE;
    }
    publisher_ = create_publisher<araco_interfaces::msg::SelectedCommand>(
      "/araco/command/selected", rclcpp::QoS(1).reliable());
    subscription_ = create_subscription<araco_interfaces::msg::CommandCandidate>(
      "/araco/command/candidates/teleop", rclcpp::QoS(1).best_effort(),
      [this](araco_interfaces::msg::CommandCandidate::ConstSharedPtr message) {
        candidate_ = *message;
        candidate_receipt_ = SteadyClock::now();
        have_candidate_ = true;
        if (!message->active) {
          observed_release_ = true;
        }
      });
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    publisher_->on_activate();
    observed_release_ = false;
    have_candidate_ = false;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / params_.loop_rate_hz),
      [this]() {publish_selection();});
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    publisher_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  void publish_selection()
  {
    const bool fresh = have_candidate_ &&
      SteadyClock::now() - candidate_receipt_ <= std::chrono::milliseconds(150);
    const bool selected = fresh && observed_release_ && candidate_.active;
    if (selected != was_selected_) {
      ++selection_epoch_;
      was_selected_ = selected;
    }
    araco_interfaces::msg::SelectedCommand output;
    output.header.stamp = now();
    output.header.frame_id = "base_link";
    output.selection_epoch = selection_epoch_;
    output.has_selection = selected;
    if (selected) {
      output.source_id = 10;
      output.source_stamp = candidate_.header.stamp;
      output.source_sequence = candidate_.sequence;
      output.intent = candidate_.intent;
    }
    publisher_->publish(output);
  }

  std::shared_ptr<command_arbiter::ParamListener> param_listener_;
  command_arbiter::Params params_;
  rclcpp_lifecycle::LifecyclePublisher<araco_interfaces::msg::SelectedCommand>::SharedPtr
    publisher_;
  rclcpp::Subscription<araco_interfaces::msg::CommandCandidate>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  araco_interfaces::msg::CommandCandidate candidate_;
  SteadyClock::time_point candidate_receipt_{};
  bool have_candidate_{false};
  bool observed_release_{false};
  bool was_selected_{false};
  uint64_t selection_epoch_{0};
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
