#include "openarm_cartesian_control/openarm_cartesian_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <string>
#include <algorithm>

namespace openarm_cartesian_control {

namespace {

ImpedanceGains makeImpedanceGains(
    const std::vector<double>& k_gains, const std::vector<double>& d_gains) {
  ImpedanceGains gains;
  std::copy(k_gains.begin(), k_gains.end(), gains.k.begin());
  std::copy(d_gains.begin(), d_gains.end(), gains.d.begin());
  return gains;
}

}  // namespace

// Lifecycle

controller_interface::CallbackReturn OpenArmCartesianController::on_init() {
  // Declare params
  auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
  auto_declare<std::string>("robot_description", "");
  auto_declare<std::vector<double>>(
      "cartesian_position_lower_limits", std::vector<double>(3, -INFINITY));
  auto_declare<std::vector<double>>(
      "cartesian_position_upper_limits", std::vector<double>(3, INFINITY));
  auto_declare<std::vector<double>>(
      "cartesian_torque_limits", std::vector<double>(6, INFINITY));
  auto_declare<std::vector<double>>(
      "joint_torque_limits", std::vector<double>(7, INFINITY));
  auto_declare<std::vector<double>>("k_gains", std::vector<double>(6, 0.0));
  auto_declare<std::vector<double>>("d_gains", std::vector<double>(6, 0.0));
  auto_declare<std::string>("ee_frame_name", "tool0");
  auto_declare<std::string>("imu_topic", "imu");
  auto_declare<std::string>("odom_topic", "odom");
  auto_declare<std::vector<double>>("imu_to_mount_rpy", std::vector<double>{0.0, 0.0, 0.0});
  auto_declare<std::string>("command_source", "action");
  auto_declare<std::string>("command_pose_topic", "command_pose");

  // Read params
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  const auto cartesian_position_lower_limits =
      get_node()->get_parameter("cartesian_position_lower_limits").as_double_array();
  const auto cartesian_position_upper_limits =
      get_node()->get_parameter("cartesian_position_upper_limits").as_double_array();
  const auto cartesian_torque_limits =
      get_node()->get_parameter("cartesian_torque_limits").as_double_array();
  const auto joint_torque_limits =
      get_node()->get_parameter("joint_torque_limits").as_double_array();

  // Load and parse URDF
  const std::string urdf_string =
      get_node()->get_parameter("robot_description").as_string();
  if (urdf_string.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_description parameter is empty");
    return CallbackReturn::ERROR;
  }

  // Construct impedance controller
  try {
    impedance_controller_.emplace(
      urdf_string,
      joint_names_,
      get_node()->get_parameter("ee_frame_name").as_string(),
      cartesian_position_lower_limits,
      cartesian_position_upper_limits,
      cartesian_torque_limits,
      joint_torque_limits);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Failed to construct CartesianImpedance: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // Seed RT gain buffer from config; update() applies them on the RT thread.
  const auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  const auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  std::string gain_error;
  if (!validateGainVector(k_gains, "k_gains", gain_error) ||
      !validateGainVector(d_gains, "d_gains", gain_error)) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", gain_error.c_str());
    return CallbackReturn::ERROR;
  }
  impedance_gains_buffer_.writeFromNonRT(makeImpedanceGains(k_gains, d_gains));

  const auto imu_to_mount_rpy =
      get_node()->get_parameter("imu_to_mount_rpy").as_double_array();
  if (imu_to_mount_rpy.size() == 3) {
    imu_to_mount_rotation_ =
        Eigen::AngleAxisd(imu_to_mount_rpy[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(imu_to_mount_rpy[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(imu_to_mount_rpy[0], Eigen::Vector3d::UnitX());
  }

  // Register param callback so gains can be updated at runtime via:
  //   ros2 param set /<controller_name> k_gains "[100,100,100,10,10,10]"
  param_callback_handle_ = get_node()->add_on_set_parameters_callback(
      std::bind(&OpenArmCartesianController::onParameterChange,
                this, std::placeholders::_1));

  // Register action server for trajectory goals
  action_server_ = rclcpp_action::create_server<FollowCartesianTrajectoryAction>(
      get_node(),
      "follow_cartesian_trajectory",
      std::bind(&OpenArmCartesianController::onGoalRequest,
                this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&OpenArmCartesianController::onCancelRequest,
                this, std::placeholders::_1),
      std::bind(&OpenArmCartesianController::onGoalAccepted,
                this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OpenArmCartesianController::on_activate(
    const rclcpp_lifecycle::State& /* state */) {
  // Zero effort outputs
  for (auto& cmd : command_interfaces_) {
    (void)cmd.set_value(0.0);
  }

  // Clear stale state from a previous activation
  trajectory_buffer_.writeFromNonRT(nullptr);
  pose_command_buffer_.writeFromNonRT(PoseCommandRef{});
  feedback_msg_ = std::make_shared<FollowCartesianTrajectoryAction::Feedback>();
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_.reset();
  }
  feedback_count_ = 0;
  mount_motion_buffer_.writeFromNonRT(MountMotion{});
  impedance_gains_buffer_.writeFromNonRT(
      makeImpedanceGains(
          get_node()->get_parameter("k_gains").as_double_array(),
          get_node()->get_parameter("d_gains").as_double_array()));

  // Seed FK from current joint positions so the arm holds in place
  // until the first trajectory arrives
  Eigen::VectorXd q(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    q[i] = state_interfaces_[i * 2].get_value();
  }
  impedance_controller_->initFK(q);
  x_ref_pos_  = impedance_controller_->getPose();
  x_ref_quat_ = impedance_controller_->getOrientation();
  x_ref_linvel_.setZero();
  x_ref_angvel_.setZero();

  trajectory_start_time_ = get_node()->get_clock()->now();

  RCLCPP_INFO(get_node()->get_logger(),
      "OpenArm Cartesian Controller activated (command_source=%s)",
      command_source_ == CommandSource::ACTION ? "action" : "topic");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OpenArmCartesianController::on_deactivate(
    const rclcpp_lifecycle::State& /* state */) {
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (active_goal_) {
      auto result = std::make_shared<FollowCartesianTrajectoryAction::Result>();
      result->error_code = -1;
      result->error_string = "Controller deactivated";
      active_goal_->abort(result);
      active_goal_.reset();
    }
  }
  for (auto& cmd : command_interfaces_) {
    (void)cmd.set_value(0.0);
  }
  RCLCPP_INFO(get_node()->get_logger(), "OpenArm Cartesian Controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OpenArmCartesianController::on_configure(
    const rclcpp_lifecycle::State& /* state */) {
  const std::string action_name =
    std::string("/") + get_node()->get_name() + "/follow_cartesian_trajectory";

  action_server_ = rclcpp_action::create_server<FollowCartesianTrajectoryAction>(
      get_node(),
      action_name,
      std::bind(&OpenArmCartesianController::onGoalRequest,
                this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&OpenArmCartesianController::onCancelRequest,
                this, std::placeholders::_1),
      std::bind(&OpenArmCartesianController::onGoalAccepted,
                this, std::placeholders::_1));

  const std::string imu_topic = get_node()->get_parameter("imu_topic").as_string();
  const std::string odom_topic = get_node()->get_parameter("odom_topic").as_string();

  imu_sub_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&OpenArmCartesianController::imuCallback, this, std::placeholders::_1));

  odom_sub_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&OpenArmCartesianController::odomCallback, this, std::placeholders::_1));

  const std::string command_source_param =
      get_node()->get_parameter("command_source").as_string();
  command_source_ = parseCommandSource(command_source_param);
  if (command_source_ == CommandSource::ACTION &&
      command_source_param != "action") {
    RCLCPP_WARN(get_node()->get_logger(),
        "Unknown command_source '%s'; defaulting to 'action'",
        command_source_param.c_str());
  }

  if (command_source_ == CommandSource::TOPIC) {
    const std::string pose_topic =
        get_node()->get_parameter("command_pose_topic").as_string();
    pose_command_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&OpenArmCartesianController::poseCommandCallback,
                  this, std::placeholders::_1));
    RCLCPP_INFO(get_node()->get_logger(),
        "Pose commands from topic '%s'", pose_topic.c_str());
  } else {
    RCLCPP_INFO(get_node()->get_logger(),
        "Pose commands from follow_cartesian_trajectory action");
  }

  RCLCPP_INFO(get_node()->get_logger(),
      "Subscribed to mount IMU '%s' and odom '%s'", imu_topic.c_str(), odom_topic.c_str());

  return CallbackReturn::SUCCESS;
}

// Hardware Interface Configuration

controller_interface::InterfaceConfiguration
OpenArmCartesianController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& name : joint_names_) {
    config.names.push_back(name + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
OpenArmCartesianController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& name : joint_names_) {
    config.names.push_back(name + "/position");
    config.names.push_back(name + "/velocity");
  }
  return config;
}

// RT Control Loop 

controller_interface::return_type OpenArmCartesianController::update(
    const rclcpp::Time& time, const rclcpp::Duration& /* period */) {
  // Read joint state from hardware
  Eigen::VectorXd q(joint_names_.size());
  Eigen::VectorXd dq(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    q[i]  = state_interfaces_[i * 2].get_value();
    dq[i] = state_interfaces_[i * 2 + 1].get_value();
  }

  // Advance reference pose from action trajectory or topic command
  auto trajectory_ptr = trajectory_buffer_.readFromRT();
  const bool trajectory_active =
      command_source_ == CommandSource::ACTION && trajectory_ptr && *trajectory_ptr;
  if (trajectory_active) {
    interpolateTrajectory(**trajectory_ptr, time);
  } else if (command_source_ == CommandSource::TOPIC) {
    if (const PoseCommandRef* ptr = pose_command_buffer_.readFromRT()) {
      if (ptr->valid) {
        x_ref_pos_ = ptr->position;
        x_ref_quat_ = ptr->orientation;
        x_ref_linvel_.setZero();
        x_ref_angvel_.setZero();
      }
    }
  }

  // Apply latest gains on the RT thread, then compute torques.
  applyImpedanceGains(readImpedanceGainsFromRT());
  const MountMotion mount = readMountMotionFromRT();
  const Eigen::VectorXd torques = impedance_controller_->computeControl(
      q, dq, mount, x_ref_pos_, x_ref_quat_, x_ref_linvel_, x_ref_angvel_);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    (void)command_interfaces_[i].set_value(torques[i]);
  }

  std::shared_ptr<GoalHandle> goal;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    goal = active_goal_;
  }

  if (command_source_ != CommandSource::ACTION || !trajectory_active || !goal) {
    return controller_interface::return_type::OK;
  }

  // Handle client-requested cancellation
  if (goal->is_canceling()) {
    auto result = std::make_shared<FollowCartesianTrajectoryAction::Result>();
    result->error_code = FollowCartesianTrajectoryAction::Result::SUCCESSFUL;
    goal->canceled(result);

    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      active_goal_ = nullptr;
    }

    trajectory_buffer_.writeFromNonRT(nullptr);
    return controller_interface::return_type::OK;
  }

  // Check for trajectory completion
  const auto& points = (*trajectory_ptr)->points;
  const rclcpp::Duration elapsed = time - trajectory_start_time_;
  if (rclcpp::Duration(points.back().time_from_start) <= elapsed) {
    auto result = std::make_shared<FollowCartesianTrajectoryAction::Result>();
    result->error_code = FollowCartesianTrajectoryAction::Result::SUCCESSFUL;
    goal->succeed(result);

    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      active_goal_ = nullptr;
    }

    trajectory_buffer_.writeFromNonRT(nullptr);
    return controller_interface::return_type::OK;
  }

  // Publish feedback at reduced rate (~43 Hz at 300 Hz control rate)
  if (++feedback_count_ % kFeedbackStride != 0) {
    return controller_interface::return_type::OK;
  }

  updateFeedback();
  goal->publish_feedback(feedback_msg_);

  return controller_interface::return_type::OK;
}

// Trajectory Helpers

void OpenArmCartesianController::interpolateTrajectory(
    const CartesianTrajectory& traj, const rclcpp::Time& time) {
  const auto& points = traj.points;
  const rclcpp::Duration elapsed = time - trajectory_start_time_;

  // Find the last waypoint whose time_from_start <= elapsed
  size_t seg = 0;
  for (size_t i = 0; i < points.size(); ++i) {
    if (rclcpp::Duration(points[i].time_from_start) <= elapsed) {
      seg = i;
    } else {
      break;
    }
  }

  auto set_from_point = [&](const auto& p) {
    x_ref_pos_    = Eigen::Vector3d(p.pose.position.x, p.pose.position.y, p.pose.position.z);
    x_ref_quat_   = Eigen::Quaterniond(p.pose.orientation.w, p.pose.orientation.x,
                                        p.pose.orientation.y, p.pose.orientation.z);
    x_ref_linvel_ = Eigen::Vector3d(p.twist.linear.x,  p.twist.linear.y,  p.twist.linear.z);
    x_ref_angvel_ = Eigen::Vector3d(p.twist.angular.x, p.twist.angular.y, p.twist.angular.z);
  };

  if (rclcpp::Duration(points.back().time_from_start) <= elapsed) {
    // Past end of trajectory: clamp to last point with zero velocity
    set_from_point(points.back());
    x_ref_linvel_.setZero();
    x_ref_angvel_.setZero();
  } else if (seg + 1 < points.size()) {
    // Interpolate between seg and seg+1
    const auto& p0 = points[seg];
    const auto& p1 = points[seg + 1];

    const double t0    = rclcpp::Duration(p0.time_from_start).seconds();
    const double t1    = rclcpp::Duration(p1.time_from_start).seconds();
    const double alpha = (elapsed.seconds() - t0) / (t1 - t0);

    x_ref_pos_ =
        Eigen::Vector3d(p0.pose.position.x, p0.pose.position.y, p0.pose.position.z) * (1 - alpha) +
        Eigen::Vector3d(p1.pose.position.x, p1.pose.position.y, p1.pose.position.z) * alpha;

    x_ref_quat_ =
        Eigen::Quaterniond(p0.pose.orientation.w, p0.pose.orientation.x,
                           p0.pose.orientation.y, p0.pose.orientation.z)
        .slerp(alpha,
               Eigen::Quaterniond(p1.pose.orientation.w, p1.pose.orientation.x,
                                  p1.pose.orientation.y, p1.pose.orientation.z));

    x_ref_linvel_ =
        Eigen::Vector3d(p0.twist.linear.x, p0.twist.linear.y, p0.twist.linear.z) * (1 - alpha) +
        Eigen::Vector3d(p1.twist.linear.x, p1.twist.linear.y, p1.twist.linear.z) * alpha;

    x_ref_angvel_ =
        Eigen::Vector3d(p0.twist.angular.x, p0.twist.angular.y, p0.twist.angular.z) * (1 - alpha) +
        Eigen::Vector3d(p1.twist.angular.x, p1.twist.angular.y, p1.twist.angular.z) * alpha;
  }
}

void OpenArmCartesianController::updateFeedback() {
  // Desired reference
  feedback_msg_->desired.pose.position.x    = x_ref_pos_.x();
  feedback_msg_->desired.pose.position.y    = x_ref_pos_.y();
  feedback_msg_->desired.pose.position.z    = x_ref_pos_.z();
  feedback_msg_->desired.pose.orientation.w = x_ref_quat_.w();
  feedback_msg_->desired.pose.orientation.x = x_ref_quat_.x();
  feedback_msg_->desired.pose.orientation.y = x_ref_quat_.y();
  feedback_msg_->desired.pose.orientation.z = x_ref_quat_.z();
  feedback_msg_->desired.twist.linear.x     = x_ref_linvel_.x();
  feedback_msg_->desired.twist.linear.y     = x_ref_linvel_.y();
  feedback_msg_->desired.twist.linear.z     = x_ref_linvel_.z();
  feedback_msg_->desired.twist.angular.x    = x_ref_angvel_.x();
  feedback_msg_->desired.twist.angular.y    = x_ref_angvel_.y();
  feedback_msg_->desired.twist.angular.z    = x_ref_angvel_.z();

  // Actual pose (cached from last computeControl call)
  const Eigen::Vector3d    actual_pos  = impedance_controller_->getPose();
  const Eigen::Quaterniond actual_quat = impedance_controller_->getOrientation();
  feedback_msg_->actual.pose.position.x    = actual_pos.x();
  feedback_msg_->actual.pose.position.y    = actual_pos.y();
  feedback_msg_->actual.pose.position.z    = actual_pos.z();
  feedback_msg_->actual.pose.orientation.w = actual_quat.w();
  feedback_msg_->actual.pose.orientation.x = actual_quat.x();
  feedback_msg_->actual.pose.orientation.y = actual_quat.y();
  feedback_msg_->actual.pose.orientation.z = actual_quat.z();

  // Position error
  const Eigen::Vector3d pos_err = x_ref_pos_ - actual_pos;
  feedback_msg_->error.pose.position.x = pos_err.x();
  feedback_msg_->error.pose.position.y = pos_err.y();
  feedback_msg_->error.pose.position.z = pos_err.z();

  // Orientation error as axis-angle (log map of q_err = q_ref * q_actual^{-1})
  const Eigen::Quaterniond q_err   = x_ref_quat_ * actual_quat.inverse();
  const Eigen::AngleAxisd  aa_err(q_err);
  const Eigen::Vector3d    rot_err = aa_err.axis() * aa_err.angle();
  feedback_msg_->error.pose.orientation.x = rot_err.x();
  feedback_msg_->error.pose.orientation.y = rot_err.y();
  feedback_msg_->error.pose.orientation.z = rot_err.z();
  feedback_msg_->error.pose.orientation.w = 0.0;
}

// Action Server Callbacks

rclcpp_action::GoalResponse OpenArmCartesianController::onGoalRequest(
    const rclcpp_action::GoalUUID& /* uuid */,
    std::shared_ptr<const FollowCartesianTrajectoryAction::Goal> goal) {
  if (command_source_ == CommandSource::TOPIC) {
    RCLCPP_WARN(get_node()->get_logger(),
        "Rejected trajectory goal: command_source is 'topic'");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->trajectory.points.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "Rejected empty trajectory");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OpenArmCartesianController::onCancelRequest(
    std::shared_ptr<GoalHandle> /* goal_handle */) {
  // Cancellation is handled in update() via is_canceling()
  return rclcpp_action::CancelResponse::ACCEPT;
}

void OpenArmCartesianController::onGoalAccepted(std::shared_ptr<GoalHandle> goal_handle) {
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (active_goal_) {
    auto result = std::make_shared<FollowCartesianTrajectoryAction::Result>();
    result->error_code = FollowCartesianTrajectoryAction::Result::INVALID_GOAL;
    result->error_string = "Preempted by new goal";
    active_goal_->abort(result);
  }
  trajectory_start_time_ = get_node()->get_clock()->now();
  active_goal_ = goal_handle;

  auto traj = std::make_shared<CartesianTrajectory>(goal_handle->get_goal()->trajectory);
  cartesian_control_msgs::msg::CartesianTrajectoryPoint start_pt;
  
  start_pt.pose.position.x    = x_ref_pos_.x();
  start_pt.pose.position.y    = x_ref_pos_.y();
  start_pt.pose.position.z    = x_ref_pos_.z();
  start_pt.pose.orientation.w = x_ref_quat_.w();
  start_pt.pose.orientation.x = x_ref_quat_.x();
  start_pt.pose.orientation.y = x_ref_quat_.y();
  start_pt.pose.orientation.z = x_ref_quat_.z();

  traj->points.insert(traj->points.begin(), start_pt);
  trajectory_buffer_.writeFromNonRT(traj);
}

// Parameter Callbacks

rcl_interfaces::msg::SetParametersResult
OpenArmCartesianController::onParameterChange(const std::vector<rclcpp::Parameter>& params) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  ImpedanceGains gains = readImpedanceGainsFromRT();
  if (const ImpedanceGains* ptr = impedance_gains_buffer_.readFromNonRT()) {
    gains = *ptr;
  }

  bool gains_updated = false;
  for (const auto& p : params) {
    if (p.get_name() == "k_gains") {
      std::string reason;
      const auto values = p.as_double_array();
      if (!validateGainVector(values, "k_gains", reason)) {
        result.successful = false;
        result.reason = reason;
        return result;
      }
      std::copy(values.begin(), values.end(), gains.k.begin());
      gains_updated = true;
    } else if (p.get_name() == "d_gains") {
      std::string reason;
      const auto values = p.as_double_array();
      if (!validateGainVector(values, "d_gains", reason)) {
        result.successful = false;
        result.reason = reason;
        return result;
      }
      std::copy(values.begin(), values.end(), gains.d.begin());
      gains_updated = true;
    }
  }

  if (gains_updated) {
    impedance_gains_buffer_.writeFromNonRT(gains);
    RCLCPP_INFO(get_node()->get_logger(), "Updated impedance gains via rosparam");
  }
  return result;
}

bool OpenArmCartesianController::validateGainVector(
    const std::vector<double>& gains,
    const char* param_name,
    std::string& reason) {
  if (gains.size() != kCartesianImpedanceDoF) {
    reason = std::string(param_name) + " must have exactly " +
             std::to_string(kCartesianImpedanceDoF) + " elements [x, y, z, rx, ry, rz]; got " +
             std::to_string(gains.size());
    return false;
  }
  return true;
}

ImpedanceGains OpenArmCartesianController::readImpedanceGainsFromRT() const {
  if (const ImpedanceGains* ptr = impedance_gains_buffer_.readFromRT()) {
    return *ptr;
  }
  return ImpedanceGains{};
}

void OpenArmCartesianController::applyImpedanceGains(const ImpedanceGains& gains) {
  impedance_controller_->setStiffness(gains.k);
  impedance_controller_->setDamping(gains.d);
}

MountMotion OpenArmCartesianController::readMountMotionFromRT() const {
  MountMotion mount;
  if (const MountMotion* ptr = mount_motion_buffer_.readFromRT()) {
    mount = *ptr;
  }

  if (!mount.has_imu) {
    mount.orientation = Eigen::Quaterniond::Identity();
    mount.angular_velocity.setZero();
    mount.linear_acceleration.setZero();
  }
  if (!mount.has_odom) {
    mount.linear_velocity.setZero();
  }

  return mount;
}

void OpenArmCartesianController::imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg) {
  MountMotion mount;
  if (const MountMotion* ptr = mount_motion_buffer_.readFromNonRT()) {
    mount = *ptr;
  }

  const Eigen::Quaterniond q_imu(
      msg->orientation.w, msg->orientation.x,
      msg->orientation.y, msg->orientation.z);
  if (q_imu.norm() > 1e-6) {
    mount.orientation = imu_to_mount_rotation_ * q_imu.normalized();
  }

  mount.angular_velocity =
      imu_to_mount_rotation_ * Eigen::Vector3d(
          msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  mount.linear_acceleration =
      imu_to_mount_rotation_ * Eigen::Vector3d(
          msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  mount.has_imu = true;

  mount_motion_buffer_.writeFromNonRT(mount);
}

void OpenArmCartesianController::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  MountMotion mount;
  if (const MountMotion* ptr = mount_motion_buffer_.readFromNonRT()) {
    mount = *ptr;
  }

  mount.linear_velocity =
      imu_to_mount_rotation_ * Eigen::Vector3d(
          msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);

  if (!mount.has_imu) {
    const Eigen::Quaterniond q_odom(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    if (q_odom.norm() > 1e-6) {
      mount.orientation = imu_to_mount_rotation_ * q_odom.normalized();
    }
    mount.angular_velocity =
        imu_to_mount_rotation_ * Eigen::Vector3d(
            msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
  }

  mount.has_odom = true;
  mount_motion_buffer_.writeFromNonRT(mount);
}

OpenArmCartesianController::CommandSource
OpenArmCartesianController::parseCommandSource(const std::string& value) {
  if (value == "topic") {
    return CommandSource::TOPIC;
  }
  return CommandSource::ACTION;
}

void OpenArmCartesianController::poseCommandCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  PoseCommandRef ref;
  if (const PoseCommandRef* ptr = pose_command_buffer_.readFromNonRT()) {
    ref = *ptr;
  }

  ref.position = Eigen::Vector3d(
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  ref.orientation = Eigen::Quaterniond(
      msg->pose.orientation.w, msg->pose.orientation.x,
      msg->pose.orientation.y, msg->pose.orientation.z);
  if (ref.orientation.norm() > 1e-6) {
    ref.orientation.normalize();
  } else {
    ref.orientation = Eigen::Quaterniond::Identity();
  }
  ref.valid = true;

  pose_command_buffer_.writeFromNonRT(ref);
}

}  // namespace openarm_cartesian_control

// Registers this class with pluginlib so the controller manager can
// load it by name at runtime from the plugins XML.
PLUGINLIB_EXPORT_CLASS(
  openarm_cartesian_control::OpenArmCartesianController,
  controller_interface::ControllerInterface)