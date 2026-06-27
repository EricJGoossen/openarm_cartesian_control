#pragma once

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

namespace openarm_cartesian_control {

/// Measured motion of the arm mount in world frame (assembled from external sensors).
struct MountMotion {
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_acceleration{Eigen::Vector3d::Zero()};

  bool has_imu{false};
  bool has_odom{false};
};

}  // namespace openarm_cartesian_control
