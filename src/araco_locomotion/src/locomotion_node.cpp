// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "araco_interfaces/msg/locomotion_status.hpp"
#include "araco_interfaces/msg/safe_command.hpp"
#include "araco_locomotion/locomotion_parameters.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace araco_locomotion
{

class LocomotionNode final : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LocomotionNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("locomotion", "araco", options)
  {
  }

private:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    try {
      param_listener_ = std::make_shared<locomotion::ParamListener>(shared_from_this());
      params_ = param_listener_->get_params();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "parameter validation failed: %s", error.what());
      return CallbackReturn::FAILURE;
    }

    const auto & names = params_.leg_joint_names;
    const auto & targets = params_.nominal_positions_rad;
    const std::set<std::string> unique_names(names.begin(), names.end());
    if (names.size() != 24 || targets.size() != 24 || unique_names.size() != 24 ||
      !std::all_of(targets.begin(), targets.end(), [](double value) {return std::isfinite(value);}))
    {
      RCLCPP_ERROR(get_logger(), "Gate 1 hold requires 24 unique finite joint targets");
      return CallbackReturn::FAILURE;
    }

    trajectory_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/leg_trajectory_controller/joint_trajectory", rclcpp::QoS(1).reliable());
    status_publisher_ = create_publisher<araco_interfaces::msg::LocomotionStatus>(
      "/araco/locomotion/status", rclcpp::QoS(1).reliable());
    safe_subscription_ = create_subscription<araco_interfaces::msg::SafeCommand>(
      "/araco/command/safe", rclcpp::QoS(1).reliable(),
      [this](araco_interfaces::msg::SafeCommand::ConstSharedPtr message) {
        processed_safety_epoch_ = message->safety_epoch;
        processed_selection_epoch_ = message->selection_epoch;
      });

    trajectory_.joint_names = names;
    trajectory_.header.frame_id = "";
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = targets;
    point.time_from_start = rclcpp::Duration::from_seconds(params_.trajectory_horizon_s);
    trajectory_.points = {point};
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    trajectory_publisher_->on_activate();
    status_publisher_->on_activate();
    sequence_ = 0;
    tick_count_ = 0;
    const auto period = std::chrono::duration<double>(1.0 / params_.loop_rate_hz);
    timer_ = rclcpp::create_timer(
      get_node_base_interface(), get_node_timers_interface(), get_clock(), period,
      std::bind(&LocomotionNode::tick, this));
    RCLCPP_INFO(get_logger(), "active in Gate 1 hold-only mode; motion generation is disabled");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    trajectory_publisher_->on_deactivate();
    status_publisher_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    safe_subscription_.reset();
    trajectory_publisher_.reset();
    status_publisher_.reset();
    param_listener_.reset();
    return CallbackReturn::SUCCESS;
  }

  void tick()
  {
    trajectory_.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    trajectory_publisher_->publish(trajectory_);
    ++tick_count_;
    if ((tick_count_ % 2U) != 0U) {
      return;
    }
    araco_interfaces::msg::LocomotionStatus status;
    status.header.stamp = now();
    status.header.frame_id = "base_link";
    status.status_sequence = ++sequence_;
    status.processed_safety_epoch = processed_safety_epoch_;
    status.processed_selection_epoch = processed_selection_epoch_;
    status.mode = araco_interfaces::msg::LocomotionStatus::MODE_HOLDING;
    status.gait = 0;
    status.gait_phase = 0.0;
    status.gait_cycle = 0;
    status.leg_kinematic_status.fill(araco_interfaces::msg::LocomotionStatus::LEG_VALID);
    status.trajectory_valid = true;
    status.reason_code = 3;
    status_publisher_->publish(status);
  }

  std::shared_ptr<locomotion::ParamListener> param_listener_;
  locomotion::Params params_;
  rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    trajectory_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<araco_interfaces::msg::LocomotionStatus>::SharedPtr
    status_publisher_;
  rclcpp::Subscription<araco_interfaces::msg::SafeCommand>::SharedPtr safe_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  trajectory_msgs::msg::JointTrajectory trajectory_;
  uint64_t sequence_{0};
  uint64_t processed_safety_epoch_{0};
  uint64_t processed_selection_epoch_{0};
  uint64_t tick_count_{0};
};

}  // namespace araco_locomotion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<araco_locomotion::LocomotionNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
