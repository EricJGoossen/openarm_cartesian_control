#pragma once

#include <mutex>
#include <optional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "cartesian_control_msgs/action/follow_cartesian_trajectory.hpp"
#include "cartesian_control_msgs/msg/cartesian_trajectory.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "openarm_cartesian_control/cartesian_impedance.hpp"

namespace openarm_cartesian_control {

class OpenArmCartesianController : public controller_interface::ControllerInterface {
 public:
  OpenArmCartesianController() = default;
  ~OpenArmCartesianController() = default;

  // Lifecycle
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;

  // RT control loop
  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  // Hardware interface claims
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

 private:
  using FollowCartesianTrajectoryAction =
      cartesian_control_msgs::action::FollowCartesianTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowCartesianTrajectoryAction>;

  // Configuration 
  std::vector<std::string> joint_names_;
  std::optional<CartesianImpedance> impedance_controller_;

  // Trajectory tracking 
  using CartesianTrajectory = cartesian_control_msgs::msg::CartesianTrajectory;
  realtime_tools::RealtimeBuffer<std::shared_ptr<CartesianTrajectory>> trajectory_buffer_;
  rclcpp::Time trajectory_start_time_;

  // Current Cartesian reference 
  Eigen::Vector3d    x_ref_pos_;
  Eigen::Quaterniond x_ref_quat_;
  Eigen::Vector3d    x_ref_linvel_;
  Eigen::Vector3d    x_ref_angvel_;

  // Start Cartesian pose
  Eigen::Vector3d    x_start_pos_;
  Eigen::Quaterniond x_start_quat_;

  // Action server 
  rclcpp_action::Server<FollowCartesianTrajectoryAction>::SharedPtr action_server_;

  mutable std::mutex goal_mutex_;
  std::shared_ptr<GoalHandle> active_goal_;  // guarded by goal_mutex_

  // Feedback 
  std::shared_ptr<FollowCartesianTrajectoryAction::Feedback> feedback_msg_;
  size_t feedback_count_{0};
  static constexpr size_t kFeedbackStride = 6;  

  // Parameter callback 
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Helpers 
  void interpolateTrajectory(
      const CartesianTrajectory& traj, const rclcpp::Time& time);
  void updateFeedback();

  rcl_interfaces::msg::SetParametersResult onParameterChange(
      const std::vector<rclcpp::Parameter>& params);

  rclcpp_action::GoalResponse onGoalRequest(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const FollowCartesianTrajectoryAction::Goal> goal);

  rclcpp_action::CancelResponse onCancelRequest(
      std::shared_ptr<GoalHandle> goal_handle);

  void onGoalAccepted(std::shared_ptr<GoalHandle> goal_handle);
};

}  // namespace openarm_cartesian_control