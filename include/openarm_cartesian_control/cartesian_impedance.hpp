#pragma once

#include <string>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace openarm_cartesian_control {

class CartesianImpedance {
 public:
  CartesianImpedance(
      const std::string& urdf_string,
      const std::vector<std::string>& joint_names,
      const std::string& ee_frame_name,
      const std::vector<double>& cartesian_position_lower_limits,
      const std::vector<double>& cartesian_position_upper_limits,
      const std::vector<double>& cartesian_torque_limits,
      const std::vector<double>& joint_torque_limits,
      bool do_gravity_compensation = true,
      bool do_damping = true,
      bool do_stiffness = true);

  const Eigen::Vector3d&    getPose()        const { return last_pos_; }
  const Eigen::Quaterniond& getOrientation() const { return last_quat_; }

  void initFK(const Eigen::VectorXd& q);
  void setStiffness(const std::vector<double>& k_gains);
  void setDamping(const std::vector<double>& d_gains);

  Eigen::VectorXd computeControl(
      const Eigen::VectorXd& q,
      const Eigen::VectorXd& dq,
      const Eigen::Vector3d& x_ref_pos,
      const Eigen::Quaterniond& x_ref_quat,
      const Eigen::Vector3d& x_ref_linvel,
      const Eigen::Vector3d& x_ref_angvel);
    
 private:
  // Limits
  const std::vector<double> cartesian_position_lower_limits_;
  const std::vector<double> cartesian_position_upper_limits_;
  const std::vector<double> cartesian_torque_limits_;
  const std::vector<double> joint_torque_limits_;

  // Control flags
  bool do_gravity_compensation_;
  bool do_damping_;
  bool do_stiffness_;

  // Pinocchio
  pinocchio::Model      pinocchio_model_;
  pinocchio::Data       pinocchio_data_;
  pinocchio::FrameIndex ee_frame_id_;

  // Impedance gains
  Eigen::Matrix<double, 6, 6> K_;
  Eigen::Matrix<double, 6, 6> D_;

  // Cached EE pose 
  Eigen::Vector3d    last_pos_;
  Eigen::Quaterniond last_quat_;

  // Helper functions
  void clamp(Eigen::Ref<Eigen::VectorXd> v, const std::vector<double>& limits);
  void clamp(Eigen::Ref<Eigen::VectorXd> v,
          const std::vector<double>& lower,
          const std::vector<double>& upper);

};

} // namespace openarm_cartesian_control