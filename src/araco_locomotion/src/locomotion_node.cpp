// Copyright 2026 Araco Hexapod contributors
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <array>
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
#include "araco_locomotion/safe_command_guard.hpp"
#include "araco_locomotion/steady_heartbeat.hpp"
#include "araco_locomotion/standing_target.hpp"
#include "araco_locomotion/tripod_gait.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
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
  using SteadyClock = std::chrono::steady_clock;
  using TimePoint = SteadyClock::time_point;

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
    const auto & oracle = params_.nominal_positions_rad;
    const std::set<std::string> unique_names(names.begin(), names.end());
    if (names.size() != kLegJointCount || oracle.size() != kLegJointCount ||
      unique_names.size() != kLegJointCount ||
      params_.leg_geometry_m.size() != kJointsPerLeg ||
      params_.leg_mount_positions_base_m.size() != kLegCount * 3 ||
      params_.leg_mount_yaw_rad.size() != kLegCount ||
      params_.standing_foot_targets_base_m.size() != kLegCount * 3 ||
      params_.standing_foot_pitch_rad.size() != kLegCount ||
      params_.joint_lower_rad.size() != kLegJointCount ||
      params_.joint_upper_rad.size() != kLegJointCount ||
      params_.joint_command_rate_cap_rad_s.size() != kLegJointCount ||
      params_.gimbal_joint_name != "gimbal_yaw_joint" ||
      !std::isfinite(params_.gimbal_lower_rad) ||
      !std::isfinite(params_.gimbal_upper_rad) ||
      params_.gimbal_lower_rad >= params_.gimbal_upper_rad ||
      !std::all_of(oracle.begin(), oracle.end(), [](double value) {return std::isfinite(value);}))
    {
      RCLCPP_ERROR(get_logger(), "standing/body-pose inputs have invalid dimensions or values");
      return CallbackReturn::FAILURE;
    }

    request_.geometry = {
      params_.leg_geometry_m[0], params_.leg_geometry_m[1],
      params_.leg_geometry_m[2], params_.leg_geometry_m[3]};
    request_.solver_options = {
      params_.ik_position_tolerance_m,
      params_.ik_singularity_threshold,
      params_.ik_near_limit_margin_rad};
    if (params_.standing_branch == "knee_down") {
      request_.branch = araco_kinematics::Branch::kKneeDown;
    } else if (params_.standing_branch == "knee_up") {
      request_.branch = araco_kinematics::Branch::kKneeUp;
    } else {
      RCLCPP_ERROR(get_logger(), "unknown standing IK branch: %s", params_.standing_branch.c_str());
      return CallbackReturn::FAILURE;
    }
    request_.oracle_tolerance_rad = params_.standing_oracle_tolerance_rad;
    for (std::size_t leg = 0; leg < kLegCount; ++leg) {
      request_.mounts[leg] = {
        {
          params_.leg_mount_positions_base_m[leg * 3],
          params_.leg_mount_positions_base_m[leg * 3 + 1],
          params_.leg_mount_positions_base_m[leg * 3 + 2],
        },
        params_.leg_mount_yaw_rad[leg],
      };
      request_.foot_targets_base_m[leg] = {
        params_.standing_foot_targets_base_m[leg * 3],
        params_.standing_foot_targets_base_m[leg * 3 + 1],
        params_.standing_foot_targets_base_m[leg * 3 + 2],
      };
      request_.foot_pitch_rad[leg] = params_.standing_foot_pitch_rad[leg];
      for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
        const std::size_t index = leg * kJointsPerLeg + joint;
        request_.limits[leg][joint] = {
          params_.joint_lower_rad[index], params_.joint_upper_rad[index]};
        request_.oracle_joints_rad[index] = oracle[index];
      }
    }
    const std::array<double, kLegJointCount> empty_previous{};
    standing_result_ = compute_standing_target(request_, empty_previous);
    if (!standing_result_.committed) {
      RCLCPP_ERROR(get_logger(), "computed standing target is invalid");
      return CallbackReturn::FAILURE;
    }
    gait_config_.base_cadence_hz = params_.gait_base_cadence_hz;
    gait_config_.maximum_cadence_hz = params_.gait_maximum_cadence_hz;
    gait_config_.cadence_rate_hz_s = params_.gait_cadence_rate_hz_s;
    gait_config_.preferred_maximum_stride_scale =
      params_.gait_preferred_maximum_stride_scale;
    gait_config_.motion_deadband_m_s = params_.gait_motion_deadband_m_s;
    gait_config_.duty_factor = params_.gait_duty_factor;
    gait_config_.maximum_stride_m = params_.gait_maximum_stride_m;
    gait_config_.swing_clearance_m = params_.gait_swing_clearance_m;
    gait_config_.planar_command_scale_m_s = params_.gait_planar_command_scale_m_s;
    gait_config_.yaw_command_scale_rad_s = params_.gait_yaw_command_scale_rad_s;
    gait_config_.translation_acceleration_m_s2 = params_.translation_acceleration_m_s2;
    gait_config_.translation_stop_deceleration_m_s2 =
      params_.translation_stop_deceleration_m_s2;
    gait_config_.yaw_acceleration_rad_s2 = params_.yaw_acceleration_rad_s2;
    gait_config_.yaw_stop_deceleration_rad_s2 = params_.yaw_stop_deceleration_rad_s2;
    gait_config_.operator_input_pre_filtered = params_.operator_input_pre_filtered;
    gait_config_.stable_hold_dwell_s = params_.stable_hold_dwell_s;

    trajectory_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/leg_trajectory_controller/joint_trajectory", rclcpp::QoS(1).reliable());
    gimbal_trajectory_publisher_ =
      create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/gimbal_trajectory_controller/joint_trajectory", rclcpp::QoS(1).reliable());
    status_publisher_ = create_publisher<araco_interfaces::msg::LocomotionStatus>(
      "/araco/locomotion/status", rclcpp::QoS(1).reliable());
    foot_targets_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/araco/debug/foot_targets_body", rclcpp::QoS(1).best_effort());
    safe_subscription_ = create_subscription<araco_interfaces::msg::SafeCommand>(
      "/araco/command/safe", rclcpp::QoS(1).reliable(),
      [this](araco_interfaces::msg::SafeCommand::ConstSharedPtr message) {
        const bool valid = valid_safe_command(*message);
        const bool accepted = safe_guard_->accept(
          message->safety_epoch, message->disposition, valid, steady_now_s());
        if (accepted) {
          safe_command_ = *message;
          processed_safety_epoch_ = message->safety_epoch;
          processed_selection_epoch_ = message->selection_epoch;
        }
        guard_result_ = safe_guard_->evaluate(steady_now_s());
      });

    safe_guard_ = std::make_unique<SafeCommandGuard>(params_.safe_command_timeout_s);

    trajectory_.joint_names = names;
    trajectory_.header.frame_id = "";
    update_trajectory(standing_result_);
    gimbal_trajectory_.joint_names = {params_.gimbal_joint_name};
    gimbal_trajectory_.header.frame_id = "";
    update_gimbal_trajectory(false);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    trajectory_publisher_->on_activate();
    gimbal_trajectory_publisher_->on_activate();
    status_publisher_->on_activate();
    foot_targets_publisher_->on_activate();
    sequence_ = 0;
    tick_count_ = 0;
    kinematics_fault_ = false;
    workspace_limited_ = false;
    workspace_recovery_latched_ = false;
    walking_posture_suppressed_ = false;
    gait_state_ = TripodState{};
    gait_mode_ = GaitMode::kHolding;
    applied_gimbal_yaw_rad_ = 0.0;
    safe_guard_->reset();
    guard_result_ = SafeGuardResult{};
    status_heartbeat_.reset();
    last_motion_tick_s_ = -1.0;
    last_timing_warning_s_ = -1.0;
    maximum_motion_gap_s_ = 0.0;
    maximum_motion_execution_s_ = 0.0;
    maximum_heartbeat_gap_s_ = 0.0;
    const auto period = std::chrono::duration<double>(1.0 / params_.loop_rate_hz);
    timer_ = rclcpp::create_timer(
      get_node_base_interface(), get_node_timers_interface(), get_clock(), period,
      std::bind(&LocomotionNode::tick, this));
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(10), [this]() {check_safe_watchdog();});
    RCLCPP_INFO(
      get_logger(),
      "active in Gate 2 computed-standing mode; "
      "Phase 4 planted-foot body-pose ready; Phase 5 tripod gait ready; "
      "responsive cadence/stride scheduler ready; "
      "max standing oracle error %.3g rad",
      standing_result_.maximum_oracle_error_rad);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    watchdog_timer_.reset();
    RCLCPP_INFO(
      get_logger(),
      "locomotion timing summary: maximum motion gap=%.3f ms, "
      "maximum motion execution=%.3f ms, maximum heartbeat gap=%.3f ms",
      maximum_motion_gap_s_ * 1000.0, maximum_motion_execution_s_ * 1000.0,
      maximum_heartbeat_gap_s_ * 1000.0);
    trajectory_publisher_->on_deactivate();
    gimbal_trajectory_publisher_->on_deactivate();
    status_publisher_->on_deactivate();
    foot_targets_publisher_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    watchdog_timer_.reset();
    safe_subscription_.reset();
    trajectory_publisher_.reset();
    gimbal_trajectory_publisher_.reset();
    status_publisher_.reset();
    foot_targets_publisher_.reset();
    param_listener_.reset();
    return CallbackReturn::SUCCESS;
  }

  static std::array<double, 3> rpy_from_quaternion(
    const geometry_msgs::msg::Quaternion & quaternion)
  {
    const double sinr_cosp = 2.0 *
      (quaternion.w * quaternion.x + quaternion.y * quaternion.z);
    const double cosr_cosp = 1.0 - 2.0 *
      (quaternion.x * quaternion.x + quaternion.y * quaternion.y);
    const double sinp = 2.0 *
      (quaternion.w * quaternion.y - quaternion.z * quaternion.x);
    return {
      std::atan2(sinr_cosp, cosr_cosp),
      std::abs(sinp) >= 1.0 ? std::copysign(1.5707963267948966, sinp) : std::asin(sinp),
      std::atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)),
    };
  }

  BodyPoseOffset requested_pose() const
  {
    const auto & pose = safe_command_.intent.body_pose_offset;
    const auto rpy = rpy_from_quaternion(pose.orientation);
    return {{pose.position.x, pose.position.y, pose.position.z}, rpy[0], rpy[1], rpy[2]};
  }

  bool requested_pose_is_neutral() const
  {
    const auto pose = requested_pose();
    constexpr double kPositionToleranceM = 1.0e-4;
    constexpr double kAngleToleranceRad = 1.0e-3;
    return std::abs(pose.translation_m.x) <= kPositionToleranceM &&
           std::abs(pose.translation_m.y) <= kPositionToleranceM &&
           std::abs(pose.translation_m.z) <= kPositionToleranceM &&
           std::abs(pose.roll_rad) <= kAngleToleranceRad &&
           std::abs(pose.pitch_rad) <= kAngleToleranceRad &&
           std::abs(pose.yaw_rad) <= kAngleToleranceRad;
  }

  static BodyPoseOffset interpolate(
    const BodyPoseOffset & from, const BodyPoseOffset & to, double scale)
  {
    return {
      {
        from.translation_m.x + scale * (to.translation_m.x - from.translation_m.x),
        from.translation_m.y + scale * (to.translation_m.y - from.translation_m.y),
        from.translation_m.z + scale * (to.translation_m.z - from.translation_m.z),
      },
      from.roll_rad + scale * (to.roll_rad - from.roll_rad),
      from.pitch_rad + scale * (to.pitch_rad - from.pitch_rad),
      from.yaw_rad + scale * (to.yaw_rad - from.yaw_rad),
    };
  }

  BodyPoseOffset shaped_pose(
    const BodyPoseOffset & target, bool allow_pre_filtered = true) const
  {
    if (params_.operator_input_pre_filtered && allow_pre_filtered) {
      return target;
    }
    const double linear_step = params_.body_translation_rate_m_s / params_.loop_rate_hz;
    const double angular_step = params_.body_angular_rate_rad_s / params_.loop_rate_hz;
    const double dx = target.translation_m.x - applied_pose_.translation_m.x;
    const double dy = target.translation_m.y - applied_pose_.translation_m.y;
    const double dz = target.translation_m.z - applied_pose_.translation_m.z;
    const double linear_distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double dr = target.roll_rad - applied_pose_.roll_rad;
    const double dp = target.pitch_rad - applied_pose_.pitch_rad;
    const double dyaw = target.yaw_rad - applied_pose_.yaw_rad;
    const double angular_distance = std::sqrt(dr * dr + dp * dp + dyaw * dyaw);
    const double linear_scale = linear_distance > linear_step ? linear_step / linear_distance : 1.0;
    const double angular_scale = angular_distance > angular_step ?
      angular_step / angular_distance : 1.0;
    BodyPoseOffset result = applied_pose_;
    result.translation_m.x += linear_scale * dx;
    result.translation_m.y += linear_scale * dy;
    result.translation_m.z += linear_scale * dz;
    result.roll_rad += angular_scale * dr;
    result.pitch_rad += angular_scale * dp;
    result.yaw_rad += angular_scale * dyaw;
    return result;
  }

  bool within_joint_rate(const StandingResult & candidate) const
  {
    for (std::size_t index = 0; index < kLegJointCount; ++index) {
      const double allowed = params_.joint_command_rate_cap_rad_s[index] /
        params_.loop_rate_hz + 1.0e-12;
      if (std::abs(candidate.joints_rad[index] - standing_result_.joints_rad[index]) > allowed) {
        return false;
      }
    }
    return true;
  }

  bool advance_body_pose(const BodyPoseOffset & target)
  {
    const BodyPoseOffset proposed = shaped_pose(target);
    StandingResult candidate = compute_body_pose_target(
      request_, proposed, standing_result_.joints_rad);
    if (candidate.committed && within_joint_rate(candidate)) {
      applied_pose_ = proposed;
      standing_result_ = candidate;
      update_trajectory(candidate);
      return true;
    }

    double lower = 0.0;
    double upper = 1.0;
    StandingResult best = standing_result_;
    BodyPoseOffset best_pose = applied_pose_;
    bool found = false;
    for (std::size_t iteration = 0; iteration < 16; ++iteration) {
      const double scale = 0.5 * (lower + upper);
      const BodyPoseOffset trial_pose = interpolate(applied_pose_, proposed, scale);
      StandingResult trial = compute_body_pose_target(
        request_, trial_pose, standing_result_.joints_rad);
      if (trial.committed && within_joint_rate(trial)) {
        found = true;
        lower = scale;
        best = trial;
        best_pose = trial_pose;
      } else {
        upper = scale;
      }
    }
    if (found) {
      applied_pose_ = best_pose;
      standing_result_ = best;
      update_trajectory(best);
    }
    return found;
  }

  bool advance_gait(bool request_walk, bool request_stop)
  {
    const PlanarVelocity requested{
      safe_command_.intent.planar_velocity.linear.x,
      safe_command_.intent.planar_velocity.linear.y,
      safe_command_.intent.planar_velocity.angular.z,
    };
    const BodyPoseOffset target_pose = request_walk ?
      (walking_posture_suppressed_ ? BodyPoseOffset{} : requested_pose()) : applied_pose_;
    const BodyPoseOffset proposed_pose = shaped_pose(target_pose);
    TripodStep best_gait;
    StandingResult best_candidate = standing_result_;
    BodyPoseOffset best_pose = applied_pose_;
    bool found = false;
    double lower = 0.0;
    double upper = 1.0;
    for (std::size_t iteration = 0; iteration < 17; ++iteration) {
      const double scale = iteration == 0 ? 1.0 : 0.5 * (lower + upper);
      const auto trial_gait = advance_tripod(
        gait_config_, gait_state_, request_.foot_targets_base_m, requested,
        request_walk, request_stop, scale / params_.loop_rate_hz);
      const BodyPoseOffset trial_pose = interpolate(applied_pose_, proposed_pose, scale);
      const StandingResult trial_candidate = trial_gait.valid ? compute_foot_pose_target(
        request_, trial_gait.foot_targets_base_m, trial_pose,
        standing_result_.joints_rad) : StandingResult{};
      if (trial_gait.valid && trial_candidate.committed &&
        within_joint_rate(trial_candidate))
      {
        found = true;
        lower = scale;
        best_gait = trial_gait;
        best_candidate = trial_candidate;
        best_pose = trial_pose;
        if (iteration == 0) {
          break;
        }
      } else {
        upper = scale;
      }
    }
    if (!found) {
      return false;
    }
    gait_state_ = best_gait.state;
    gait_mode_ = best_gait.mode;
    applied_pose_ = best_pose;
    standing_result_ = best_candidate;
    update_trajectory(best_candidate);
    return true;
  }

  bool advance_workspace_recovery()
  {
    const BodyPoseOffset neutral_pose{};
    const BodyPoseOffset proposed_pose = shaped_pose(neutral_pose, false);
    TripodStep best_gait;
    StandingResult best_candidate = standing_result_;
    BodyPoseOffset best_pose = applied_pose_;
    bool found = false;
    double lower = 0.0;
    double upper = 1.0;
    for (std::size_t iteration = 0; iteration < 17; ++iteration) {
      const double scale = iteration == 0 ? 1.0 : 0.5 * (lower + upper);
      const auto trial_gait = retreat_tripod_to_nominal(
        gait_config_, gait_state_, request_.foot_targets_base_m,
        scale / params_.loop_rate_hz);
      const BodyPoseOffset trial_pose = interpolate(applied_pose_, proposed_pose, scale);
      const StandingResult trial_candidate = trial_gait.valid ? compute_foot_pose_target(
        request_, trial_gait.foot_targets_base_m, trial_pose,
        standing_result_.joints_rad) : StandingResult{};
      if (trial_gait.valid && trial_candidate.committed &&
        within_joint_rate(trial_candidate))
      {
        found = true;
        lower = scale;
        best_gait = trial_gait;
        best_candidate = trial_candidate;
        best_pose = trial_pose;
        if (iteration == 0) {
          break;
        }
      } else {
        upper = scale;
      }
    }
    if (!found) {
      return false;
    }
    gait_state_ = best_gait.state;
    gait_mode_ = best_gait.mode;
    applied_pose_ = best_pose;
    standing_result_ = best_candidate;
    update_trajectory(best_candidate);
    return true;
  }

  void update_trajectory(const StandingResult & result)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(result.joints_rad.begin(), result.joints_rad.end());
    point.time_from_start = rclcpp::Duration::from_seconds(params_.trajectory_horizon_s);
    trajectory_.points = {point};
  }

  void update_gimbal_trajectory(bool executing)
  {
    const double requested = executing ? safe_command_.intent.gimbal_yaw_rad : 0.0;
    const double target = std::clamp(
      requested, params_.gimbal_lower_rad, params_.gimbal_upper_rad);
    if (executing && params_.operator_input_pre_filtered) {
      // Body posture and gimbal are derived from the same filtered axis-4
      // state. Preserve that synchronization instead of independently slewing
      // the gimbal a second time.
      applied_gimbal_yaw_rad_ = target;
    } else {
      const double maximum_step =
        params_.gimbal_command_rate_cap_rad_s / params_.loop_rate_hz;
      applied_gimbal_yaw_rad_ += std::clamp(
        target - applied_gimbal_yaw_rad_, -maximum_step, maximum_step);
    }
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {applied_gimbal_yaw_rad_};
    point.time_from_start = rclcpp::Duration::from_seconds(params_.trajectory_horizon_s);
    gimbal_trajectory_.points = {point};
  }

  bool safe_command_executable() const
  {
    return guard_result_.executable &&
           (safe_command_.disposition == araco_interfaces::msg::SafeCommand::DISPOSITION_EXECUTE ||
           safe_command_.disposition == araco_interfaces::msg::SafeCommand::DISPOSITION_LIMITED) &&
           safe_command_.intent.gait <= 1;
  }

  bool tripod_requested(bool executing) const
  {
    return executing && safe_command_.intent.gait == 1 &&
           (std::hypot(
      safe_command_.intent.planar_velocity.linear.x,
      safe_command_.intent.planar_velocity.linear.y) > 1.0e-9 ||
           std::abs(safe_command_.intent.planar_velocity.angular.z) > 1.0e-9);
  }

  void warn_timing_delay(
    const char * source, double gap_s, double execution_s, double steady_now_s)
  {
    if (last_timing_warning_s_ >= 0.0 && steady_now_s - last_timing_warning_s_ < 1.0) {
      return;
    }
    last_timing_warning_s_ = steady_now_s;
    RCLCPP_WARN(
      get_logger(),
      "locomotion timing delay source=%s gap=%.3f ms execution=%.3f ms; "
      "motion uses ROS time and health heartbeat uses steady time",
      source, gap_s * 1000.0, execution_s * 1000.0);
  }

  void tick()
  {
    const double tick_started_s = steady_now_s();
    if (last_motion_tick_s_ >= 0.0) {
      const double motion_gap_s = tick_started_s - last_motion_tick_s_;
      maximum_motion_gap_s_ = std::max(maximum_motion_gap_s_, motion_gap_s);
      if (motion_gap_s > 0.05) {
        warn_timing_delay("motion_timer", motion_gap_s, 0.0, tick_started_s);
      }
    }
    last_motion_tick_s_ = tick_started_s;

    guard_result_ = safe_guard_->evaluate(tick_started_s);
    const bool executing = safe_command_executable();
    const bool request_tripod = tripod_requested(executing);
    const bool gait_active = gait_state_.walking || gait_state_.stopping ||
      (gait_state_.hold_dwell_s > 0.0 &&
      gait_state_.hold_dwell_s < gait_config_.stable_hold_dwell_s);
    if (walking_posture_suppressed_ &&
      (!request_tripod || requested_pose_is_neutral()))
    {
      walking_posture_suppressed_ = false;
    }
    if (workspace_recovery_latched_ && !gait_active) {
      workspace_recovery_latched_ = false;
      walking_posture_suppressed_ = request_tripod && !requested_pose_is_neutral();
      if (walking_posture_suppressed_) {
        RCLCPP_WARN(
          get_logger(),
          "workspace retreat completed; resuming planar gait at neutral body posture; "
          "center posture controls to re-enable walking posture offsets");
      }
    }
    if (!standing_result_.committed) {
      // Losing the last complete 24-joint commit is an internal invariant
      // failure. This remains a latched safety fault.
      kinematics_fault_ = true;
      workspace_limited_ = false;
    } else if (workspace_recovery_latched_) {
      kinematics_fault_ = false;
      workspace_limited_ = true;
      if (!advance_workspace_recovery()) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "workspace retreat could not make an inward rate-limited step; "
          "holding the last complete valid trajectory");
      }
    } else if (request_tripod || gait_active) {
      kinematics_fault_ = false;
      if (!advance_gait(request_tripod, !request_tripod)) {
        workspace_recovery_latched_ = true;
        workspace_limited_ = true;
        static_cast<void>(advance_workspace_recovery());
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "gait/posture request reached the IK workspace boundary; "
          "retreating to nominal stance before automatic planar-gait retry");
      } else {
        workspace_limited_ = walking_posture_suppressed_;
      }
    } else if (executing) {
      kinematics_fault_ = false;
      workspace_limited_ = !advance_body_pose(requested_pose());
      if (workspace_limited_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "body-pose request reached the IK workspace boundary; "
          "holding the last complete valid trajectory and retrying");
      }
      gait_mode_ = GaitMode::kHolding;
    } else {
      kinematics_fault_ = false;
      workspace_limited_ = false;
      gait_mode_ = GaitMode::kHolding;
    }

    trajectory_.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    trajectory_publisher_->publish(trajectory_);
    update_gimbal_trajectory(executing);
    gimbal_trajectory_.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    gimbal_trajectory_publisher_->publish(gimbal_trajectory_);
    ++tick_count_;
    if ((tick_count_ % 2U) == 0U) {
      publish_foot_targets();
    }
    const double execution_s = steady_now_s() - tick_started_s;
    maximum_motion_execution_s_ = std::max(maximum_motion_execution_s_, execution_s);
    if (execution_s > 0.01) {
      warn_timing_delay("motion_execution", 0.0, execution_s, steady_now_s());
    }
  }

  void publish_status(bool executing, bool tripod_requested)
  {
    araco_interfaces::msg::LocomotionStatus status;
    status.header.stamp = now();
    status.header.frame_id = "base_link";
    status.status_sequence = ++sequence_;
    status.processed_safety_epoch = processed_safety_epoch_;
    status.processed_selection_epoch = processed_selection_epoch_;
    if (guard_result_.reason == 27) {
      status.mode = araco_interfaces::msg::LocomotionStatus::MODE_FAULT;
    } else if (kinematics_fault_) {
      status.mode = araco_interfaces::msg::LocomotionStatus::MODE_FAULT;
    } else if (gait_mode_ == GaitMode::kStarting) {
      status.mode = araco_interfaces::msg::LocomotionStatus::MODE_STARTING;
    } else if (gait_mode_ == GaitMode::kWalking) {
      status.mode = araco_interfaces::msg::LocomotionStatus::MODE_WALKING;
    } else if (gait_mode_ == GaitMode::kStopping) {
      status.mode = araco_interfaces::msg::LocomotionStatus::MODE_STOPPING;
    } else {
      status.mode = executing ? araco_interfaces::msg::LocomotionStatus::MODE_STANDING :
        araco_interfaces::msg::LocomotionStatus::MODE_HOLDING;
    }
    status.gait = tripod_requested || gait_state_.walking ? 1 : 0;
    status.gait_phase = gait_state_.phase;
    status.gait_cycle = gait_state_.cycle;
    status.gait_cadence_hz = gait_state_.cadence_hz;
    status.gait_maximum_stride_scale = gait_state_.maximum_stride_scale;
    status.gait_maximum_clearance_m = gait_state_.maximum_clearance_m;
    status.gait_applied_velocity_scale = gait_state_.applied_velocity_scale;
    for (std::size_t leg = 0; leg < kLegCount; ++leg) {
      status.leg_kinematic_status[leg] =
        static_cast<std::uint8_t>(standing_result_.leg_status[leg]);
    }
    status.trajectory_valid = !kinematics_fault_ && standing_result_.committed;
    status.reason_code = kinematics_fault_ ? 16 :
      (workspace_limited_ ? 11 : guard_result_.reason);
    if (status.reason_code == 0) {
      status.reason_code = executing ? 0 :
        (guard_result_.fresh ? safe_command_.reason_code : 3);
    }
    status_publisher_->publish(status);
  }

  static double steady_now_s()
  {
    return std::chrono::duration<double>(
      SteadyClock::now().time_since_epoch()).count();
  }

  bool valid_safe_command(const araco_interfaces::msg::SafeCommand & message) const
  {
    const auto & twist = message.intent.planar_velocity;
    const auto & pose = message.intent.body_pose_offset;
    const std::array<double, 14> values{
      twist.linear.x, twist.linear.y, twist.linear.z,
      twist.angular.x, twist.angular.y, twist.angular.z,
      pose.position.x, pose.position.y, pose.position.z,
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
      message.intent.gimbal_yaw_rad};
    const double quaternion_norm = std::sqrt(
      pose.orientation.x * pose.orientation.x +
      pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z +
      pose.orientation.w * pose.orientation.w);
    const bool execute =
      message.disposition == araco_interfaces::msg::SafeCommand::DISPOSITION_EXECUTE ||
      message.disposition == araco_interfaces::msg::SafeCommand::DISPOSITION_LIMITED;
    return message.header.frame_id == "base_link" && message.disposition <= 3 &&
           message.reason_code <= 30 && message.intent.gait <= 1 &&
           (!execute || message.source_id != 0) &&
           message.intent.gimbal_yaw_rad >= params_.gimbal_lower_rad &&
           message.intent.gimbal_yaw_rad <= params_.gimbal_upper_rad &&
           std::all_of(values.begin(), values.end(), [](double value) {
               return std::isfinite(value);
           }) && std::abs(quaternion_norm - 1.0) <= 1.0e-6;
  }

  void check_safe_watchdog()
  {
    const double steady_s = steady_now_s();
    const auto heartbeat = status_heartbeat_.update(steady_s);
    maximum_heartbeat_gap_s_ = std::max(
      maximum_heartbeat_gap_s_, heartbeat.callback_gap_s);
    if (heartbeat.callback_delayed) {
      warn_timing_delay("health_heartbeat", heartbeat.callback_gap_s, 0.0, steady_s);
    }
    const auto previous_reason = guard_result_.reason;
    guard_result_ = safe_guard_->evaluate(steady_s);
    if (guard_result_.reason != 0 && guard_result_.reason != previous_reason) {
      RCLCPP_WARN(
        get_logger(), "safe-command authority revoked locally; reason=%u",
        guard_result_.reason);
      publish_status(false, false);
    } else if (heartbeat.publish) {
      const bool executing = safe_command_executable();
      publish_status(executing, tripod_requested(executing));
    }
  }

  void publish_foot_targets()
  {
    geometry_msgs::msg::PoseArray output;
    output.header.stamp = now();
    output.header.frame_id = "base_link";
    output.poses.resize(kLegCount);
    for (std::size_t leg = 0; leg < kLegCount; ++leg) {
      output.poses[leg].position.x = standing_result_.foot_targets_body_m[leg].x;
      output.poses[leg].position.y = standing_result_.foot_targets_body_m[leg].y;
      output.poses[leg].position.z = standing_result_.foot_targets_body_m[leg].z;
      output.poses[leg].orientation.w = 1.0;
    }
    foot_targets_publisher_->publish(output);
  }

  std::shared_ptr<locomotion::ParamListener> param_listener_;
  locomotion::Params params_;
  rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    trajectory_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    gimbal_trajectory_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<araco_interfaces::msg::LocomotionStatus>::SharedPtr
    status_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr
    foot_targets_publisher_;
  rclcpp::Subscription<araco_interfaces::msg::SafeCommand>::SharedPtr safe_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  SteadyHeartbeat status_heartbeat_{0.02, 0.05};
  trajectory_msgs::msg::JointTrajectory trajectory_;
  trajectory_msgs::msg::JointTrajectory gimbal_trajectory_;
  araco_interfaces::msg::SafeCommand safe_command_;
  StandingRequest request_;
  StandingResult standing_result_;
  BodyPoseOffset applied_pose_{};
  TripodConfig gait_config_{};
  TripodState gait_state_{};
  GaitMode gait_mode_{GaitMode::kHolding};
  std::unique_ptr<SafeCommandGuard> safe_guard_;
  SafeGuardResult guard_result_;
  bool kinematics_fault_{false};
  bool workspace_limited_{false};
  bool workspace_recovery_latched_{false};
  bool walking_posture_suppressed_{false};
  double applied_gimbal_yaw_rad_{0.0};
  double last_motion_tick_s_{-1.0};
  double last_timing_warning_s_{-1.0};
  double maximum_motion_gap_s_{0.0};
  double maximum_motion_execution_s_{0.0};
  double maximum_heartbeat_gap_s_{0.0};
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
