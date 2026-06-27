#pragma once

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

namespace openarm_cartesian_control {

/// Latest end-effector pose command from a topic (position + orientation only).
struct PoseCommandRef {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  bool valid{false};
};

}  // namespace openarm_cartesian_control
