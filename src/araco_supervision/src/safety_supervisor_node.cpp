// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "araco_interfaces/msg/arbitration_status.hpp"
#include "araco_interfaces/msg/joint_state_provenance.hpp"
#include "araco_interfaces/msg/locomotion_status.hpp"
#include "araco_interfaces/msg/safe_command.hpp"
#include "araco_interfaces/msg/safety_status.hpp"
#include "araco_interfaces/msg/selected_command.hpp"
#include "araco_interfaces/action/safety_transition.hpp"
#include "araco_supervision/command_policy.hpp"
#include "araco_supervision/safety_machine.hpp"
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

  static std::chrono::milliseconds timeout(double seconds)
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(seconds));
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

  static StaticIntent policy_intent(
    const araco_interfaces::msg::SelectedCommand & message)
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
      message.intent.gimbal_yaw_rad,
    };
  }

  BodyEnvelope body_envelope() const
  {
    return {
      params_.body_envelope.planar_speed_normal_m_s,
      params_.body_envelope.planar_speed_hard_m_s,
      params_.body_envelope.yaw_rate_normal_rad_s,
      params_.body_envelope.yaw_rate_hard_rad_s,
      params_.body_envelope.gimbal_yaw_normal_rad,
      params_.body_envelope.gimbal_yaw_hard_rad,
      params_.body_envelope.xy_normal_m,
      params_.body_envelope.z_normal_lower_m,
      params_.body_envelope.z_normal_upper_m,
      params_.body_envelope.roll_pitch_normal_rad,
      params_.body_envelope.yaw_normal_rad,
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

  static void assign_limited_intent(
    const PolicyResult & policy,
    araco_interfaces::msg::MotionIntent & output)
  {
    output.gait = policy.intent.gait;
    output.planar_velocity.linear.x = policy.intent.twist[0];
    output.planar_velocity.linear.y = policy.intent.twist[1];
    output.planar_velocity.angular.z = policy.intent.twist[5];
    output.body_pose_offset.position.x = policy.intent.position_m[0];
    output.body_pose_offset.position.y = policy.intent.position_m[1];
    output.body_pose_offset.position.z = policy.intent.position_m[2];
    output.body_pose_offset.orientation.x = policy.intent.orientation.x;
    output.body_pose_offset.orientation.y = policy.intent.orientation.y;
    output.body_pose_offset.orientation.z = policy.intent.orientation.z;
    output.body_pose_offset.orientation.w = policy.intent.orientation.w;
    output.gimbal_yaw_rad = policy.intent.gimbal_yaw_rad;
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
      params_.joint_state_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        joint_valid_ = exact_names(message->name, params_.state_joint_names) &&
        message->position.size() == message->name.size() && finite_values(message->position);
        joint_receipt_ = SteadyClock::now();
      });
    locomotion_subscription_ = create_subscription<araco_interfaces::msg::LocomotionStatus>(
      "/araco/locomotion/status", rclcpp::QoS(1).reliable(),
      [this](araco_interfaces::msg::LocomotionStatus::ConstSharedPtr message) {
        locomotion_mode_ = message->mode;
        locomotion_reason_ = message->reason_code;
        locomotion_valid_ = message->trajectory_valid &&
        message->mode >= araco_interfaces::msg::LocomotionStatus::MODE_HOLDING &&
        message->mode <= araco_interfaces::msg::LocomotionStatus::MODE_STOPPING;
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
        selected_receipt_ = SteadyClock::now();
        selected_policy_ = message->has_selection ?
        validate_and_limit_static_intent(
          policy_intent(*message), message->header.frame_id, body_envelope(), true) :
        PolicyResult{};
        selected_valid_ = !message->has_selection ||
        selected_policy_.validity != CommandValidity::kInvalid;
      });
    arbitration_subscription_ =
      create_subscription<araco_interfaces::msg::ArbitrationStatus>(
      "/araco/command/arbitration_status", rclcpp::QoS(1).reliable(),
      [this](araco_interfaces::msg::ArbitrationStatus::ConstSharedPtr message) {
        arbitration_ = *message;
        arbitration_receipt_ = SteadyClock::now();
      });

    controller_client_ = create_client<controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers");
    hardware_client_ = create_client<controller_manager_msgs::srv::ListHardwareComponents>(
      "/controller_manager/list_hardware_components");
    transition_server_ = rclcpp_action::create_server<SafetyTransition>(
      this, "/araco/safety/transition",
      [this](const rclcpp_action::GoalUUID &,
      std::shared_ptr<const SafetyTransition::Goal> goal) {
        if (pending_goal_) {
          return rclcpp_action::GoalResponse::REJECT;
        }
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
    machine_ = std::make_unique<SafetyMachine>(SafetyMachineConfig{
        5.0, params_.stable_hold_dwell_s,
        params_.auto_enable_once_from_neutral_standing_source,
        params_.startup_readiness_stable_s});
    machine_->reset(steady_now_s());
    sync_machine_output();
    tick_count_ = 0;
    fault_mask_ = 0;
    selected_ = araco_interfaces::msg::SelectedCommand{};
    selected_policy_ = PolicyResult{};
    selected_valid_ = false;
    selected_receipt_ = TimePoint{};
    arbitration_ = araco_interfaces::msg::ArbitrationStatus{};
    arbitration_receipt_ = TimePoint{};
    joint_valid_ = false;
    joint_receipt_ = TimePoint{};
    locomotion_valid_ = false;
    locomotion_receipt_ = TimePoint{};
    have_clock_ = false;
    clock_progressed_ = false;
    last_clock_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    clock_receipt_ = TimePoint{};
    leg_state_valid_ = false;
    leg_state_receipt_ = TimePoint{};
    gimbal_state_valid_ = false;
    gimbal_state_receipt_ = TimePoint{};
    controller_service_valid_ = false;
    controller_service_receipt_ = TimePoint{};
    backend_service_valid_ = false;
    backend_service_receipt_ = TimePoint{};
    controller_request_pending_ = false;
    hardware_request_pending_ = false;
    poll_controller_next_ = true;
    readiness_mask_ = 0;
    last_logged_readiness_mask_ = 0;
    evaluation_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / params_.loop_rate_hz), [this]() {evaluate();});
    service_timer_ = create_wall_timer(
      std::chrono::duration<double>(params_.controller_manager_validation_period_s / 2.0),
      [this]() {poll_services();});
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
    if (pending_goal_) {
      finish_pending_transition(false);
    }
    return CallbackReturn::SUCCESS;
  }

  void poll_services()
  {
    if (poll_controller_next_) {
      const bool service_ready = controller_client_->service_is_ready();
      if (!service_ready) {
        controller_service_valid_ = false;
      } else if (!controller_request_pending_) {
        controller_request_pending_ = true;
        controller_client_->async_send_request(
          std::make_shared<controller_manager_msgs::srv::ListControllers::Request>(),
          [this](
            rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future) {
            controller_request_pending_ = false;
            controller_service_valid_ = validate_controllers(*future.get());
            controller_service_receipt_ = SteadyClock::now();
          });
      }
    } else {
      const bool service_ready = hardware_client_->service_is_ready();
      if (!service_ready) {
        backend_service_valid_ = false;
      } else if (!hardware_request_pending_) {
        hardware_request_pending_ = true;
        hardware_client_->async_send_request(
          std::make_shared<controller_manager_msgs::srv::ListHardwareComponents::Request>(),
          [this](
            rclcpp::Client<controller_manager_msgs::srv::ListHardwareComponents>::SharedFuture
            future) {
            hardware_request_pending_ = false;
            backend_service_valid_ = validate_hardware(*future.get());
            backend_service_receipt_ = SteadyClock::now();
          });
      }
    }
    poll_controller_next_ = !poll_controller_next_;
  }

  void execute_transition(const std::shared_ptr<SafetyGoalHandle> goal_handle)
  {
    const auto request = goal_handle->get_goal()->request;
    const auto input = machine_input();
    const auto transition = machine_->request(
      static_cast<SafetyRequest>(request), input, steady_now_s());
    sync_machine_output();
    auto feedback = std::make_shared<SafetyTransition::Feedback>();
    feedback->state = state_;
    feedback->reason_code = reason_;
    goal_handle->publish_feedback(feedback);
    if (!transition.accepted) {
      auto result = std::make_shared<SafetyTransition::Result>();
      result->accepted = false;
      result->final_state = state_;
      result->reason_code = reason_;
      goal_handle->succeed(result);
      return;
    }
    pending_goal_ = goal_handle;
    pending_request_ = request;
    pending_feedback_state_ = state_;
    pending_feedback_reason_ = reason_;
    complete_pending_transition_if_terminal();
  }

  bool pending_target_reached() const
  {
    switch (pending_request_) {
      case SafetyTransition::Goal::REQUEST_HOLD:
      case SafetyTransition::Goal::REQUEST_RESET_FAULT:
        return state_ == araco_interfaces::msg::SafetyStatus::STATE_HOLDING;
      case SafetyTransition::Goal::REQUEST_ENABLE_MOTION:
        return state_ == araco_interfaces::msg::SafetyStatus::STATE_MOTION_ENABLED;
      case SafetyTransition::Goal::REQUEST_SHUTDOWN:
        return state_ == araco_interfaces::msg::SafetyStatus::STATE_SHUTTING_DOWN;
      case SafetyTransition::Goal::REQUEST_LATCHED_HOLD:
        return state_ == araco_interfaces::msg::SafetyStatus::STATE_FAULT_HOLD;
      default:
        return false;
    }
  }

  bool pending_transition_failed() const
  {
    switch (pending_request_) {
      case SafetyTransition::Goal::REQUEST_ENABLE_MOTION:
        return state_ != araco_interfaces::msg::SafetyStatus::STATE_ENABLING &&
               state_ != araco_interfaces::msg::SafetyStatus::STATE_MOTION_ENABLED;
      case SafetyTransition::Goal::REQUEST_HOLD:
        return state_ != araco_interfaces::msg::SafetyStatus::STATE_STOPPING &&
               state_ != araco_interfaces::msg::SafetyStatus::STATE_HOLDING;
      case SafetyTransition::Goal::REQUEST_LATCHED_HOLD:
        return state_ != araco_interfaces::msg::SafetyStatus::STATE_STOPPING &&
               state_ != araco_interfaces::msg::SafetyStatus::STATE_FAULT_HOLD;
      case SafetyTransition::Goal::REQUEST_SHUTDOWN:
        return state_ != araco_interfaces::msg::SafetyStatus::STATE_STOPPING &&
               state_ != araco_interfaces::msg::SafetyStatus::STATE_SHUTTING_DOWN;
      default:
        return false;
    }
  }

  void finish_pending_transition(bool reached)
  {
    if (!pending_goal_) {
      return;
    }
    auto result = std::make_shared<SafetyTransition::Result>();
    result->accepted = reached;
    result->final_state = state_;
    result->reason_code = reason_;
    if (reached) {
      pending_goal_->succeed(result);
    } else {
      pending_goal_->abort(result);
    }
    pending_goal_.reset();
    pending_request_ = 0;
  }

  void complete_pending_transition_if_terminal()
  {
    if (!pending_goal_) {
      return;
    }
    if (state_ != pending_feedback_state_ || reason_ != pending_feedback_reason_) {
      auto feedback = std::make_shared<SafetyTransition::Feedback>();
      feedback->state = state_;
      feedback->reason_code = reason_;
      pending_goal_->publish_feedback(feedback);
      pending_feedback_state_ = state_;
      pending_feedback_reason_ = reason_;
    }
    if (pending_target_reached()) {
      finish_pending_transition(true);
    } else if (pending_transition_failed()) {
      finish_pending_transition(false);
    }
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
      if (!interface.is_available || interface.data_type != "double") {
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
    const auto clock_timeout = timeout(params_.clock_progress_timeout_s);
    const auto joint_timeout = timeout(params_.joint_state_timeout_s);
    const auto controller_timeout = timeout(params_.controller_state_timeout_s);
    const auto locomotion_timeout = timeout(params_.locomotion_status_timeout_s);
    uint64_t readiness = araco_interfaces::msg::SafetyStatus::READY_MODEL;
    const bool time_ready = have_clock_ && clock_progressed_ &&
      fresh(clock_receipt_, clock_timeout);
    const bool joint_ready = joint_valid_ && fresh(joint_receipt_, joint_timeout);
    const bool controller_ready = controller_service_valid_ &&
      controller_client_->service_is_ready() && leg_state_valid_ && gimbal_state_valid_ &&
      fresh(leg_state_receipt_, controller_timeout) &&
      fresh(gimbal_state_receipt_, controller_timeout);
    const bool backend_ready = backend_service_valid_ && hardware_client_->service_is_ready();
    const bool locomotion_ready = locomotion_valid_ &&
      fresh(locomotion_receipt_, locomotion_timeout);
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
      const double locomotion_age_ms = locomotion_receipt_ == TimePoint{} ? -1.0 :
      std::chrono::duration<double, std::milli>(
        SteadyClock::now() - locomotion_receipt_).count();
      RCLCPP_INFO(
        get_logger(),
        "readiness=%lu/%lu backend=%d joints=%d controllers=%d locomotion=%d "
        "locomotion_age_ms=%.3f time=%d",
        readiness_mask_, kRequiredReadiness, backend_ready, joint_ready, controller_ready,
        locomotion_ready, locomotion_age_ms, time_ready);
      last_logged_readiness_mask_ = readiness_mask_;
    }

    machine_->update(machine_input(), steady_now_s());
    sync_machine_output();
    complete_pending_transition_if_terminal();

    publish_safe_command();
    if ((tick_count_ % 10U) == 0U) {publish_status();}
    if ((tick_count_ % 100U) == 0U || provenance_epoch_ == 0) {publish_provenance();}
  }

  SafetyMachineInput machine_input() const
  {
    const auto selected_timeout = timeout(params_.selected_command_timeout_s);
    const auto clock_timeout = timeout(params_.clock_progress_timeout_s);
    const auto joint_timeout = timeout(params_.joint_state_timeout_s);
    const auto controller_timeout = timeout(params_.controller_state_timeout_s);
    const auto locomotion_timeout = timeout(params_.locomotion_status_timeout_s);
    SafetyMachineInput input;
    input.ready = readiness_mask_ == kRequiredReadiness;
    input.selected_stream_armed = selected_receipt_ != TimePoint{};
    input.selected_stream_fresh = fresh(selected_receipt_, selected_timeout);
    input.selected_has_selection = selected_.has_selection;
    input.selected_valid = selected_valid_ &&
      selected_policy_.validity != CommandValidity::kInvalid;
    input.selected_neutral_standing = input.selected_valid &&
      selected_policy_.intent.gait == araco_interfaces::msg::MotionIntent::GAIT_STAND &&
      std::abs(selected_policy_.intent.twist[0]) <=
      params_.body_envelope.stand_velocity_tolerance &&
      std::abs(selected_policy_.intent.twist[1]) <=
      params_.body_envelope.stand_velocity_tolerance &&
      std::abs(selected_policy_.intent.twist[5]) <=
      params_.body_envelope.stand_velocity_tolerance &&
      std::abs(selected_policy_.intent.position_m[0]) <=
      params_.body_envelope.stand_velocity_tolerance &&
      std::abs(selected_policy_.intent.position_m[1]) <=
      params_.body_envelope.stand_velocity_tolerance &&
      std::abs(selected_policy_.rpy_rad[0]) <=
      params_.body_envelope.stand_velocity_tolerance &&
      std::abs(selected_policy_.rpy_rad[1]) <=
      params_.body_envelope.stand_velocity_tolerance &&
      std::abs(selected_policy_.rpy_rad[2]) <=
      params_.body_envelope.stand_velocity_tolerance &&
      std::abs(selected_policy_.intent.gimbal_yaw_rad) <=
      params_.body_envelope.stand_velocity_tolerance;
    input.selected_source_id = selected_.has_selection ? selected_.source_id : 0;
    input.selection_epoch = selected_.selection_epoch;
    input.arbitration_status_fresh = fresh(arbitration_receipt_, selected_timeout) &&
      arbitration_.selection_epoch == selected_.selection_epoch;
    input.arbitration_reason = arbitration_.reason_code;
    input.deliberate_higher_priority_preemption =
      arbitration_.deliberate_higher_priority_preemption;
    input.all_sources_released = input.arbitration_status_fresh &&
      arbitration_.all_sources_released;
    input.locomotion_stably_holding = locomotion_valid_ &&
      (locomotion_mode_ == araco_interfaces::msg::LocomotionStatus::MODE_HOLDING ||
      locomotion_mode_ == araco_interfaces::msg::LocomotionStatus::MODE_STANDING);
    const bool time_ready = have_clock_ && clock_progressed_ &&
      fresh(clock_receipt_, clock_timeout);
    const bool controller_ready = controller_service_valid_ &&
      controller_client_->service_is_ready() && leg_state_valid_ && gimbal_state_valid_ &&
      fresh(leg_state_receipt_, controller_timeout) &&
      fresh(gimbal_state_receipt_, controller_timeout);
    const bool backend_ready = backend_service_valid_ && hardware_client_->service_is_ready();
    const bool locomotion_ready = locomotion_valid_ &&
      fresh(locomotion_receipt_, locomotion_timeout);
    input.controlled_stop_available =
      time_ready && controller_ready && backend_ready && locomotion_ready;

    if (state_ != araco_interfaces::msg::SafetyStatus::STATE_INITIALIZING &&
      state_ != araco_interfaces::msg::SafetyStatus::STATE_INACTIVE &&
      state_ != araco_interfaces::msg::SafetyStatus::STATE_SHUTTING_DOWN)
    {
      if (!(joint_valid_ && fresh(joint_receipt_, joint_timeout))) {
        input.condition_fault_mask |= araco_interfaces::msg::SafetyStatus::FAULT_JOINT_STATE;
        input.condition_reason = joint_valid_ ?
          araco_interfaces::msg::SafetyStatus::REASON_JOINT_STATE_STALE :
          araco_interfaces::msg::SafetyStatus::REASON_JOINT_STATE_INVALID;
      }
      if (!(controller_service_valid_ && controller_client_->service_is_ready() &&
        leg_state_valid_ && gimbal_state_valid_ &&
        fresh(leg_state_receipt_, controller_timeout) &&
        fresh(gimbal_state_receipt_, controller_timeout)))
      {
        input.condition_fault_mask |= araco_interfaces::msg::SafetyStatus::FAULT_CONTROLLER;
        input.condition_reason = araco_interfaces::msg::SafetyStatus::REASON_CONTROLLER_NOT_READY;
      }
      if (!(backend_service_valid_ && hardware_client_->service_is_ready())) {
        input.condition_fault_mask |= araco_interfaces::msg::SafetyStatus::FAULT_BACKEND;
        input.condition_reason = araco_interfaces::msg::SafetyStatus::REASON_BACKEND_FAULT;
      }
      if (fresh(locomotion_receipt_, locomotion_timeout) &&
        (locomotion_reason_ ==
        araco_interfaces::msg::SafetyStatus::REASON_KINEMATICS_INVALID ||
        locomotion_reason_ == araco_interfaces::msg::SafetyStatus::REASON_JOINT_LIMIT))
      {
        input.condition_fault_mask |= araco_interfaces::msg::SafetyStatus::FAULT_KINEMATICS;
        input.condition_reason = locomotion_reason_;
      } else {
        if (!(locomotion_valid_ &&
          fresh(locomotion_receipt_, locomotion_timeout)))
        {
          input.condition_fault_mask |= araco_interfaces::msg::SafetyStatus::FAULT_LOCOMOTION;
          input.condition_reason = araco_interfaces::msg::SafetyStatus::REASON_LOCOMOTION_STALE;
        }
      }
      if (!(have_clock_ && clock_progressed_ &&
        fresh(clock_receipt_, clock_timeout)))
      {
        input.condition_fault_mask |= araco_interfaces::msg::SafetyStatus::FAULT_TIME;
        input.condition_reason = araco_interfaces::msg::SafetyStatus::REASON_TIME_DISCONTINUITY;
      }
      if (input.selected_stream_fresh && selected_.has_selection && !selected_valid_) {
        input.condition_fault_mask |=
          araco_interfaces::msg::SafetyStatus::FAULT_SELECTED_COMMAND_STREAM;
        input.condition_reason = araco_interfaces::msg::SafetyStatus::REASON_SOURCE_INVALID;
      }
    }
    return input;
  }

  bool arbitration_synchronized(std::chrono::milliseconds timeout) const
  {
    return fresh(arbitration_receipt_, timeout) &&
           arbitration_.selection_epoch == selected_.selection_epoch;
  }

  static double steady_now_s()
  {
    return std::chrono::duration<double>(
      SteadyClock::now().time_since_epoch()).count();
  }

  void sync_machine_output()
  {
    const auto current = machine_->output();
    const auto previous_state = state_;
    state_ = static_cast<std::uint8_t>(current.state);
    reason_ = current.reason;
    safety_epoch_ = current.safety_epoch;
    fault_mask_ = current.fault_mask;
    if (state_ != previous_state) {
      RCLCPP_INFO(
        get_logger(), "safety transition state=%u reason=%u epoch=%lu",
        state_, reason_, safety_epoch_);
      publish_status();
    }
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
    if (state_ == araco_interfaces::msg::SafetyStatus::STATE_STOPPING) {
      output.disposition = araco_interfaces::msg::SafeCommand::DISPOSITION_CONTROLLED_STOP;
    }
    const auto selected_timeout = timeout(params_.selected_command_timeout_s);
    const bool synchronized = arbitration_synchronized(selected_timeout);
    const bool executable =
      state_ == araco_interfaces::msg::SafetyStatus::STATE_MOTION_ENABLED &&
      fresh(selected_receipt_, selected_timeout) && selected_.has_selection && selected_valid_ &&
      selected_policy_.validity != CommandValidity::kInvalid && synchronized;
    if (state_ == araco_interfaces::msg::SafetyStatus::STATE_MOTION_ENABLED &&
      !synchronized)
    {
      output.reason_code =
        araco_interfaces::msg::SafetyStatus::REASON_SELECTED_COMMAND_STALE;
    }
    if (executable) {
      output.disposition = selected_policy_.validity == CommandValidity::kLimited ?
        araco_interfaces::msg::SafeCommand::DISPOSITION_LIMITED :
        araco_interfaces::msg::SafeCommand::DISPOSITION_EXECUTE;
      output.reason_code = selected_policy_.validity == CommandValidity::kLimited ?
        araco_interfaces::msg::SafetyStatus::REASON_COMMAND_LIMITED :
        araco_interfaces::msg::SafetyStatus::REASON_NONE;
      output.source_stamp = selected_.source_stamp;
      output.source_sequence = selected_.source_sequence;
      assign_limited_intent(selected_policy_, output.intent);
    }
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
    if (state_ == araco_interfaces::msg::SafetyStatus::STATE_STOPPING) {
      output.disposition = araco_interfaces::msg::SafeCommand::DISPOSITION_CONTROLLED_STOP;
    }
    const auto selected_timeout = timeout(params_.selected_command_timeout_s);
    const bool synchronized = arbitration_synchronized(selected_timeout);
    if (state_ == araco_interfaces::msg::SafetyStatus::STATE_MOTION_ENABLED &&
      selected_valid_ && selected_.has_selection && synchronized)
    {
      output.disposition = selected_policy_.validity == CommandValidity::kLimited ?
        araco_interfaces::msg::SafeCommand::DISPOSITION_LIMITED :
        araco_interfaces::msg::SafeCommand::DISPOSITION_EXECUTE;
      output.reason_code = selected_policy_.validity == CommandValidity::kLimited ?
        araco_interfaces::msg::SafetyStatus::REASON_COMMAND_LIMITED :
        araco_interfaces::msg::SafetyStatus::REASON_NONE;
    }
    if (state_ == araco_interfaces::msg::SafetyStatus::STATE_MOTION_ENABLED &&
      !synchronized)
    {
      output.reason_code =
        araco_interfaces::msg::SafetyStatus::REASON_SELECTED_COMMAND_STALE;
    }
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
  rclcpp::Subscription<araco_interfaces::msg::ArbitrationStatus>::SharedPtr
    arbitration_subscription_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListHardwareComponents>::SharedPtr hardware_client_;
  rclcpp_action::Server<SafetyTransition>::SharedPtr transition_server_;
  rclcpp::TimerBase::SharedPtr evaluation_timer_;
  rclcpp::TimerBase::SharedPtr service_timer_;
  araco_interfaces::msg::SelectedCommand selected_;
  araco_interfaces::msg::ArbitrationStatus arbitration_;
  PolicyResult selected_policy_;
  std::unique_ptr<SafetyMachine> machine_;
  std::shared_ptr<SafetyGoalHandle> pending_goal_;
  rclcpp::Time last_clock_{0, 0, RCL_ROS_TIME};
  TimePoint joint_receipt_{};
  TimePoint locomotion_receipt_{};
  TimePoint clock_receipt_{};
  TimePoint leg_state_receipt_{};
  TimePoint gimbal_state_receipt_{};
  TimePoint controller_service_receipt_{};
  TimePoint backend_service_receipt_{};
  TimePoint selected_receipt_{};
  TimePoint arbitration_receipt_{};
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
  bool poll_controller_next_{true};
  bool selected_valid_{false};
  uint8_t locomotion_mode_{araco_interfaces::msg::LocomotionStatus::MODE_INACTIVE};
  uint16_t locomotion_reason_{araco_interfaces::msg::SafetyStatus::REASON_STARTUP};
  uint8_t pending_request_{0};
  uint8_t pending_feedback_state_{0};
  uint16_t pending_feedback_reason_{0};
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
