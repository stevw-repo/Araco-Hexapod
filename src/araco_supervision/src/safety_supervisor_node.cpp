// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "araco_interfaces/msg/joint_state_provenance.hpp"
#include "araco_interfaces/msg/locomotion_status.hpp"
#include "araco_interfaces/msg/safe_command.hpp"
#include "araco_interfaces/msg/safety_status.hpp"
#include "araco_interfaces/msg/selected_command.hpp"
#include "araco_interfaces/action/safety_transition.hpp"
#include "araco_supervision/safety_supervisor_parameters.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/list_hardware_components.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace araco_supervision
{

class SafetySupervisorNode final : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SafetySupervisorNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("safety_supervisor", "araco", options)
  {
  }

private:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using SafetyTransition = araco_interfaces::action::SafetyTransition;
  using SafetyGoalHandle = rclcpp_action::ServerGoalHandle<SafetyTransition>;
  using SteadyClock = std::chrono::steady_clock;
  using TimePoint = SteadyClock::time_point;
  static constexpr uint64_t kRequiredReadiness = 127;

  static bool fresh(const TimePoint & value, std::chrono::milliseconds limit)
  {
    return value != TimePoint{} && SteadyClock::now() - value <= limit;
  }

  template<typename ValuesT>
  static bool finite_values(const ValuesT & values)
  {
    return std::all_of(values.begin(), values.end(), [](double value) {
               return std::isfinite(value);
        });
  }

  static bool exact_names(
    const std::vector<std::string> & actual,
    const std::vector<std::string> & expected)
  {
    return actual.size() == expected.size() &&
           std::set<std::string>(actual.begin(), actual.end()) ==
           std::set<std::string>(expected.begin(), expected.end());
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    try {
      param_listener_ = std::make_shared<safety_supervisor::ParamListener>(shared_from_this());
      params_ = param_listener_->get_params();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "parameter validation failed: %s", error.what());
      return CallbackReturn::FAILURE;
    }
    if (params_.state_joint_names.size() != 25 || params_.leg_joint_names.size() != 24 ||
      params_.gimbal_joint_names.size() != 1 ||
      params_.gimbal_joint_names.front() != "gimbal_yaw_joint")
    {
      RCLCPP_ERROR(get_logger(), "invalid canonical 25-state / 24+1 controller partition");
      return CallbackReturn::FAILURE;
    }

    status_publisher_ = create_publisher<araco_interfaces::msg::SafetyStatus>(
      "/araco/safety/status", rclcpp::QoS(1).reliable());
    safe_publisher_ = create_publisher<araco_interfaces::msg::SafeCommand>(
      "/araco/command/safe", rclcpp::QoS(1).reliable());
    provenance_publisher_ = create_publisher<araco_interfaces::msg::JointStateProvenance>(
      "/araco/state/joint_state_provenance", rclcpp::QoS(1).reliable().transient_local());

    joint_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        joint_valid_ = exact_names(message->name, params_.state_joint_names) &&
        message->position.size() == message->name.size() && finite_values(message->position);
        joint_receipt_ = SteadyClock::now();
      });
    locomotion_subscription_ = create_subscription<araco_interfaces::msg::LocomotionStatus>(
      "/araco/locomotion/status", rclcpp::QoS(1).reliable(),
      [this](araco_interfaces::msg::LocomotionStatus::ConstSharedPtr message) {
        locomotion_valid_ = message->trajectory_valid &&
        message->mode == araco_interfaces::msg::LocomotionStatus::MODE_HOLDING;
        locomotion_receipt_ = SteadyClock::now();
      });
    clock_subscription_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS(), [this](rosgraph_msgs::msg::Clock::ConstSharedPtr message) {
        const rclcpp::Time sample(message->clock);
        clock_progressed_ = !have_clock_ || sample > last_clock_;
        if (clock_progressed_) {
          clock_receipt_ = SteadyClock::now();
          last_clock_ = sample;
          have_clock_ = true;
        }
      });
    leg_state_subscription_ =
      create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "/leg_trajectory_controller/controller_state", rclcpp::SensorDataQoS(),
      [this](control_msgs::msg::JointTrajectoryControllerState::ConstSharedPtr message) {
        leg_state_valid_ = exact_names(message->joint_names, params_.leg_joint_names) &&
        message->feedback.positions.size() == 24 && finite_values(message->feedback.positions);
        leg_state_receipt_ = SteadyClock::now();
      });
    gimbal_state_subscription_ =
      create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "/gimbal_trajectory_controller/controller_state", rclcpp::SensorDataQoS(),
      [this](control_msgs::msg::JointTrajectoryControllerState::ConstSharedPtr message) {
        gimbal_state_valid_ = exact_names(message->joint_names, params_.gimbal_joint_names) &&
        message->feedback.positions.size() == 1 && finite_values(message->feedback.positions);
        gimbal_state_receipt_ = SteadyClock::now();
      });
    selected_subscription_ = create_subscription<araco_interfaces::msg::SelectedCommand>(
      "/araco/command/selected", rclcpp::QoS(1).reliable(),
      [this](araco_interfaces::msg::SelectedCommand::ConstSharedPtr message) {
        selected_ = *message;
      });

    controller_client_ = create_client<controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers");
    hardware_client_ = create_client<controller_manager_msgs::srv::ListHardwareComponents>(
      "/controller_manager/list_hardware_components");
    transition_server_ = rclcpp_action::create_server<SafetyTransition>(
      this, "/araco/safety/transition",
      [this](const rclcpp_action::GoalUUID &,
      std::shared_ptr<const SafetyTransition::Goal> goal) {
        if (goal->request < SafetyTransition::Goal::REQUEST_HOLD ||
        goal->request > SafetyTransition::Goal::REQUEST_LATCHED_HOLD)
        {
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<SafetyGoalHandle>) {
        return rclcpp_action::CancelResponse::REJECT;
      },
      [this](const std::shared_ptr<SafetyGoalHandle> goal_handle) {
        execute_transition(goal_handle);
      });
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    status_publisher_->on_activate();
    safe_publisher_->on_activate();
    provenance_publisher_->on_activate();
    state_ = araco_interfaces::msg::SafetyStatus::STATE_INITIALIZING;
    reason_ = araco_interfaces::msg::SafetyStatus::REASON_STARTUP;
    safety_epoch_ = 0;
    tick_count_ = 0;
    ever_holding_ = false;
    fault_mask_ = 0;
    evaluation_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / params_.loop_rate_hz), [this]() {evaluate();});
    service_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {poll_services();});
    publish_status();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    evaluation_timer_.reset();
    service_timer_.reset();
    status_publisher_->on_deactivate();
    safe_publisher_->on_deactivate();
    provenance_publisher_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  void poll_services()
  {
    if (!controller_request_pending_ && controller_client_->service_is_ready()) {
      controller_request_pending_ = true;
      controller_client_->async_send_request(
        std::make_shared<controller_manager_msgs::srv::ListControllers::Request>(),
        [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future) {
          controller_request_pending_ = false;
          controller_service_valid_ = validate_controllers(*future.get());
          controller_service_receipt_ = SteadyClock::now();
        });
    }
    if (!hardware_request_pending_ && hardware_client_->service_is_ready()) {
      hardware_request_pending_ = true;
      hardware_client_->async_send_request(
        std::make_shared<controller_manager_msgs::srv::ListHardwareComponents::Request>(),
        [this](rclcpp::Client<controller_manager_msgs::srv::ListHardwareComponents>::SharedFuture
        future) {
          hardware_request_pending_ = false;
          backend_service_valid_ = validate_hardware(*future.get());
          backend_service_receipt_ = SteadyClock::now();
        });
    }
  }

  void execute_transition(const std::shared_ptr<SafetyGoalHandle> goal_handle)
  {
    const auto request = goal_handle->get_goal()->request;
    auto result = std::make_shared<SafetyTransition::Result>();
    bool accepted = false;
    if (request == SafetyTransition::Goal::REQUEST_HOLD) {
      accepted = state_ != araco_interfaces::msg::SafetyStatus::STATE_SHUTTING_DOWN;
      if (accepted) {
        if (state_ != araco_interfaces::msg::SafetyStatus::STATE_HOLDING &&
          readiness_mask_ == kRequiredReadiness)
        {
          transition_to(
            araco_interfaces::msg::SafetyStatus::STATE_HOLDING,
            araco_interfaces::msg::SafetyStatus::REASON_MANUAL_HOLD);
        } else {
          reason_ = araco_interfaces::msg::SafetyStatus::REASON_MANUAL_HOLD;
          publish_status();
        }
      }
    } else if (request == SafetyTransition::Goal::REQUEST_LATCHED_HOLD) {
      accepted = state_ != araco_interfaces::msg::SafetyStatus::STATE_SHUTTING_DOWN;
      if (accepted) {
        fault_mask_ |= araco_interfaces::msg::SafetyStatus::FAULT_SOFTWARE_LATCH;
        transition_to(
          araco_interfaces::msg::SafetyStatus::STATE_FAULT_HOLD,
          araco_interfaces::msg::SafetyStatus::REASON_SOFTWARE_LATCHED_HOLD);
      }
    } else if (request == SafetyTransition::Goal::REQUEST_RESET_FAULT) {
      const auto software_latch =
        araco_interfaces::msg::SafetyStatus::FAULT_SOFTWARE_LATCH;
      accepted = fault_mask_ == software_latch && readiness_mask_ == kRequiredReadiness;
      if (accepted) {
        fault_mask_ = 0;
        transition_to(
          araco_interfaces::msg::SafetyStatus::STATE_HOLDING,
          araco_interfaces::msg::SafetyStatus::REASON_HOLDING);
      }
    } else if (request == SafetyTransition::Goal::REQUEST_SHUTDOWN) {
      accepted = true;
      transition_to(
        araco_interfaces::msg::SafetyStatus::STATE_SHUTTING_DOWN,
        araco_interfaces::msg::SafetyStatus::REASON_SHUTDOWN_REQUESTED);
    }
    // REQUEST_ENABLE_MOTION is deliberately rejected throughout Gate 1.
    auto feedback = std::make_shared<SafetyTransition::Feedback>();
    feedback->state = state_;
    feedback->reason_code = reason_;
    goal_handle->publish_feedback(feedback);
    result->accepted = accepted;
    result->final_state = state_;
    result->reason_code = reason_;
    goal_handle->succeed(result);
  }

  bool validate_controllers(
    const controller_manager_msgs::srv::ListControllers::Response & response) const
  {
    const std::set<std::string> expected_leg = interface_names(params_.leg_joint_names);
    const std::set<std::string> expected_gimbal = interface_names(params_.gimbal_joint_names);
    bool state_broadcaster = false;
    bool leg = false;
    bool gimbal = false;
    for (const auto & controller : response.controller) {
      const std::set<std::string> claims(
        controller.claimed_interfaces.begin(), controller.claimed_interfaces.end());
      if (controller.name == "joint_state_broadcaster") {
        state_broadcaster = controller.state == "active" && claims.empty();
      } else if (controller.name == "leg_trajectory_controller") {
        leg = controller.state == "active" && claims == expected_leg;
      } else if (controller.name == "gimbal_trajectory_controller") {
        gimbal = controller.state == "active" && claims == expected_gimbal;
      }
    }
    return state_broadcaster && leg && gimbal;
  }

  static std::set<std::string> interface_names(const std::vector<std::string> & joints)
  {
    std::set<std::string> result;
    for (const auto & joint : joints) {
      result.insert(joint + "/position");
    }
    return result;
  }

  bool validate_hardware(
    const controller_manager_msgs::srv::ListHardwareComponents::Response & response) const
  {
    if (response.component.size() != 1) {
      return false;
    }
    const auto & component = response.component.front();
    if (component.name != "GazeboSimSystem" || component.state.id != 3 || component.is_async) {
      return false;
    }
    const auto expected_commands = interface_names(params_.state_joint_names);
    std::set<std::string> actual_commands;
    for (const auto & interface : component.command_interfaces) {
      if (!interface.is_available || !interface.is_claimed || interface.data_type != "double") {
        return false;
      }
      actual_commands.insert(interface.name);
    }
    std::set<std::string> expected_states;
    for (const auto & joint : params_.state_joint_names) {
      expected_states.insert(joint + "/position");
      expected_states.insert(joint + "/velocity");
      expected_states.insert(joint + "/effort");
    }
    std::set<std::string> actual_states;
    for (const auto & interface : component.state_interfaces) {
      if (!interface.is_available || interface.data_type != "double") {
        return false;
      }
      actual_states.insert(interface.name);
    }
    return actual_commands == expected_commands && actual_states == expected_states;
  }

  void evaluate()
  {
    ++tick_count_;
    uint64_t readiness = araco_interfaces::msg::SafetyStatus::READY_MODEL;
    const bool time_ready = have_clock_ && clock_progressed_ &&
      fresh(clock_receipt_, std::chrono::milliseconds(250));
    const bool joint_ready = joint_valid_ && fresh(joint_receipt_, std::chrono::milliseconds(100));
    const bool controller_ready = controller_service_valid_ && leg_state_valid_ &&
      gimbal_state_valid_ && fresh(controller_service_receipt_, std::chrono::milliseconds(110)) &&
      fresh(leg_state_receipt_, std::chrono::milliseconds(100)) &&
      fresh(gimbal_state_receipt_, std::chrono::milliseconds(100));
    const bool backend_ready = backend_service_valid_ &&
      fresh(backend_service_receipt_, std::chrono::milliseconds(110));
    const bool locomotion_ready = locomotion_valid_ &&
      fresh(locomotion_receipt_, std::chrono::milliseconds(100));
    if (backend_ready) {readiness |= araco_interfaces::msg::SafetyStatus::READY_BACKEND;}
    if (joint_ready) {readiness |= araco_interfaces::msg::SafetyStatus::READY_JOINT_STATE;}
    if (controller_ready) {readiness |= araco_interfaces::msg::SafetyStatus::READY_CONTROLLERS;}
    if (locomotion_ready) {readiness |= araco_interfaces::msg::SafetyStatus::READY_LOCOMOTION;}
    if (backend_ready && joint_ready) {
      readiness |= araco_interfaces::msg::SafetyStatus::READY_PROVENANCE;
    }
    if (time_ready) {readiness |= araco_interfaces::msg::SafetyStatus::READY_TIME;}
    readiness_mask_ = readiness;
    if (readiness_mask_ != last_logged_readiness_mask_) {
      RCLCPP_INFO(
        get_logger(),
        "readiness=%lu/%lu backend=%d joints=%d controllers=%d locomotion=%d time=%d",
        readiness_mask_, kRequiredReadiness, backend_ready, joint_ready, controller_ready,
        locomotion_ready, time_ready);
      last_logged_readiness_mask_ = readiness_mask_;
    }

    const bool initializing =
      !ever_holding_ && state_ == araco_interfaces::msg::SafetyStatus::STATE_INITIALIZING;
    const bool lost_readiness =
      ever_holding_ && readiness_mask_ != kRequiredReadiness &&
      state_ == araco_interfaces::msg::SafetyStatus::STATE_HOLDING;
    if (!ever_holding_ && readiness_mask_ == kRequiredReadiness) {
      transition_to(araco_interfaces::msg::SafetyStatus::STATE_HOLDING,
        araco_interfaces::msg::SafetyStatus::REASON_HOLDING);
      ever_holding_ = true;
    } else if (initializing) {
      transition_to(araco_interfaces::msg::SafetyStatus::STATE_INACTIVE,
        araco_interfaces::msg::SafetyStatus::REASON_INACTIVE);
    } else if (lost_readiness) {
      classify_fault(joint_ready, controller_ready, backend_ready, locomotion_ready, time_ready);
      transition_to(araco_interfaces::msg::SafetyStatus::STATE_FAULT_HOLD, reason_);
    }

    publish_safe_command();
    if ((tick_count_ % 10U) == 0U) {publish_status();}
    if ((tick_count_ % 100U) == 0U || provenance_epoch_ == 0) {publish_provenance();}
  }

  void classify_fault(bool joint, bool controller, bool backend, bool locomotion, bool time)
  {
    if (!joint) {
      fault_mask_ |= araco_interfaces::msg::SafetyStatus::FAULT_JOINT_STATE;
      reason_ = araco_interfaces::msg::SafetyStatus::REASON_JOINT_STATE_STALE;
    }
    if (!controller) {
      fault_mask_ |= araco_interfaces::msg::SafetyStatus::FAULT_CONTROLLER;
      reason_ = araco_interfaces::msg::SafetyStatus::REASON_CONTROLLER_NOT_READY;
    }
    if (!backend) {
      fault_mask_ |= araco_interfaces::msg::SafetyStatus::FAULT_BACKEND;
      reason_ = araco_interfaces::msg::SafetyStatus::REASON_BACKEND_FAULT;
    }
    if (!locomotion) {
      fault_mask_ |= araco_interfaces::msg::SafetyStatus::FAULT_LOCOMOTION;
      reason_ = araco_interfaces::msg::SafetyStatus::REASON_LOCOMOTION_STALE;
    }
    if (!time) {
      fault_mask_ |= araco_interfaces::msg::SafetyStatus::FAULT_TIME;
      reason_ = araco_interfaces::msg::SafetyStatus::REASON_TIME_DISCONTINUITY;
    }
  }

  void transition_to(uint8_t state, uint16_t reason)
  {
    if (state_ == state && reason_ == reason) {return;}
    state_ = state;
    reason_ = reason;
    ++safety_epoch_;
    RCLCPP_INFO(
      get_logger(), "safety transition state=%u reason=%u epoch=%lu",
      state_, reason_, safety_epoch_);
    publish_status();
  }

  void publish_safe_command()
  {
    araco_interfaces::msg::SafeCommand output;
    output.header.stamp = now();
    output.header.frame_id = "base_link";
    output.safety_epoch = safety_epoch_;
    output.selection_epoch = selected_.selection_epoch;
    output.disposition = araco_interfaces::msg::SafeCommand::DISPOSITION_HOLD;
    output.reason_code = reason_;
    output.source_id = selected_.has_selection ? selected_.source_id : 0;
    output.intent.gait = 0;
    output.intent.body_pose_offset.orientation.w = 1.0;
    safe_publisher_->publish(output);
  }

  void publish_status()
  {
    araco_interfaces::msg::SafetyStatus output;
    output.header.stamp = now();
    output.header.frame_id = "base_link";
    output.safety_epoch = safety_epoch_;
    output.state = state_;
    output.disposition = araco_interfaces::msg::SafeCommand::DISPOSITION_HOLD;
    output.reason_code = reason_;
    output.selected_source_id = selected_.has_selection ? selected_.source_id : 0;
    output.readiness_mask = readiness_mask_;
    output.required_readiness_mask = kRequiredReadiness;
    output.fault_mask = fault_mask_;
    output.reset_required = fault_mask_ != 0;
    status_publisher_->publish(output);
  }

  void publish_provenance()
  {
    araco_interfaces::msg::JointStateProvenance output;
    output.header.stamp = now();
    output.header.frame_id = "base_link";
    output.provenance_epoch = ++provenance_epoch_;
    output.joint_names = params_.state_joint_names;
    const auto source = araco_interfaces::msg::JointStateProvenance::SOURCE_SIMULATED_PHYSICS;
    output.position_source.assign(output.joint_names.size(), source);
    output.velocity_source.assign(output.joint_names.size(), source);
    output.effort_source.assign(output.joint_names.size(), source);
    provenance_publisher_->publish(output);
  }

  std::shared_ptr<safety_supervisor::ParamListener> param_listener_;
  safety_supervisor::Params params_;
  rclcpp_lifecycle::LifecyclePublisher<araco_interfaces::msg::SafetyStatus>::SharedPtr
    status_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<araco_interfaces::msg::SafeCommand>::SharedPtr
    safe_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<araco_interfaces::msg::JointStateProvenance>::SharedPtr
    provenance_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::Subscription<araco_interfaces::msg::LocomotionStatus>::SharedPtr locomotion_subscription_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    leg_state_subscription_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    gimbal_state_subscription_;
  rclcpp::Subscription<araco_interfaces::msg::SelectedCommand>::SharedPtr selected_subscription_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListHardwareComponents>::SharedPtr hardware_client_;
  rclcpp_action::Server<SafetyTransition>::SharedPtr transition_server_;
  rclcpp::TimerBase::SharedPtr evaluation_timer_;
  rclcpp::TimerBase::SharedPtr service_timer_;
  araco_interfaces::msg::SelectedCommand selected_;
  rclcpp::Time last_clock_{0, 0, RCL_ROS_TIME};
  TimePoint joint_receipt_{};
  TimePoint locomotion_receipt_{};
  TimePoint clock_receipt_{};
  TimePoint leg_state_receipt_{};
  TimePoint gimbal_state_receipt_{};
  TimePoint controller_service_receipt_{};
  TimePoint backend_service_receipt_{};
  bool joint_valid_{false};
  bool locomotion_valid_{false};
  bool clock_progressed_{false};
  bool have_clock_{false};
  bool leg_state_valid_{false};
  bool gimbal_state_valid_{false};
  bool controller_service_valid_{false};
  bool backend_service_valid_{false};
  bool controller_request_pending_{false};
  bool hardware_request_pending_{false};
  bool ever_holding_{false};
  uint8_t state_{araco_interfaces::msg::SafetyStatus::STATE_INITIALIZING};
  uint16_t reason_{araco_interfaces::msg::SafetyStatus::REASON_STARTUP};
  uint64_t readiness_mask_{0};
  uint64_t last_logged_readiness_mask_{0};
  uint64_t fault_mask_{0};
  uint64_t safety_epoch_{0};
  uint64_t provenance_epoch_{0};
  uint64_t tick_count_{0};
};

}  // namespace araco_supervision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<araco_supervision::SafetySupervisorNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
