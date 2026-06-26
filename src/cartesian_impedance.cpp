#include "openarm_cartesian_control/cartesian_impedance.hpp"

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include <stdexcept>
#include <iostream>
#include <cmath>

namespace openarm_cartesian_control {

CartesianImpedance::CartesianImpedance(
    const std::string& urdf_string,
    const std::string& ee_frame_name,
    const std::vector<double>& cartesian_position_lower_limits,
    const std::vector<double>& cartesian_position_upper_limits,
    const std::vector<double>& cartesian_torque_limits,
    const std::vector<double>& joint_torque_limits,
    bool do_gravity_compensation,
    bool do_damping,
    bool do_stiffness)
  : cartesian_position_lower_limits_(cartesian_position_lower_limits),
    cartesian_position_upper_limits_(cartesian_position_upper_limits),
    cartesian_torque_limits_(cartesian_torque_limits),
    joint_torque_limits_(joint_torque_limits),
    do_gravity_compensation_(do_gravity_compensation),
    do_damping_(do_damping),
    do_stiffness_(do_stiffness),
    K_(Eigen::Matrix<double, 6, 6>::Zero()),
    D_(Eigen::Matrix<double, 6, 6>::Zero())
{
  try {
    pinocchio::urdf::buildModelFromXML(urdf_string, pinocchio_model_);
    pinocchio_data_ = pinocchio::Data(pinocchio_model_);
    ee_frame_id_ = pinocchio_model_.getFrameId(ee_frame_name);
    if (ee_frame_id_ == pinocchio_model_.nframes) {
      throw std::runtime_error("EE frame '" + ee_frame_name + "' not found in URDF");
    }
  } catch (const std::exception& e) {
    throw std::runtime_error(std::string("CartesianImpedance init failed: ") + e.what());
  }
}

void CartesianImpedance::initFK(const Eigen::VectorXd& q) {
  pinocchio::forwardKinematics(pinocchio_model_, pinocchio_data_, q);
  pinocchio::updateFramePlacements(pinocchio_model_, pinocchio_data_);
  const auto& oMf = pinocchio_data_.oMf[ee_frame_id_];
  last_pos_  = oMf.translation();
  last_quat_ = Eigen::Quaterniond(oMf.rotation());
}

void CartesianImpedance::setStiffness(const std::vector<double>& k_gains) {
  K_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(k_gains.data()).asDiagonal();
}

void CartesianImpedance::setDamping(const std::vector<double>& d_gains) {
  D_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(d_gains.data()).asDiagonal();
}

Eigen::VectorXd CartesianImpedance::computeControl(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& dq,
    const Eigen::Vector3d& x_ref_pos,
    const Eigen::Quaterniond& x_ref_quat,
    const Eigen::Vector3d& x_ref_linvel,
    const Eigen::Vector3d& x_ref_angvel)
{
  // Clamp reference position to workspace limits 
  Eigen::Vector3d ref_pos = x_ref_pos;
  clamp(ref_pos, cartesian_position_lower_limits_, cartesian_position_upper_limits_);

  // computeJointJacobians runs FK internally; updateFramePlacements gives oMf
  pinocchio::computeJointJacobians(pinocchio_model_, pinocchio_data_, q);
  pinocchio::updateFramePlacements(pinocchio_model_, pinocchio_data_);
  const pinocchio::SE3& oMf = pinocchio_data_.oMf[ee_frame_id_];

  // Cache for getPose() / getOrientation()
  last_pos_  = oMf.translation();
  last_quat_ = Eigen::Quaterniond(oMf.rotation());

  // 6 x nv Jacobian in world-aligned frame (linear rows first, then angular)
  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, pinocchio_model_.nv);
  J.setZero();
  pinocchio::getFrameJacobian(
      pinocchio_model_, pinocchio_data_, ee_frame_id_,
      pinocchio::LOCAL_WORLD_ALIGNED, J);

  const Eigen::Vector3d pos_err = ref_pos - oMf.translation();

  // Orientation error via quaternion log map; ensure shortest-path rotation
  Eigen::Quaterniond q_err = x_ref_quat * Eigen::Quaterniond(oMf.rotation()).inverse();
  if (q_err.w() < 0.0) q_err.coeffs() = -q_err.coeffs();
  const Eigen::AngleAxisd aa_err(q_err);
  const Eigen::Vector3d rot_err = aa_err.axis() * aa_err.angle();

  const Eigen::Matrix<double, 6, 1> cart_vel = J * dq;
  const Eigen::Vector3d linvel_err = x_ref_linvel - cart_vel.head<3>();
  const Eigen::Vector3d angvel_err = x_ref_angvel - cart_vel.tail<3>();

  Eigen::Matrix<double, 6, 1> dx;
  dx << pos_err, rot_err;

  Eigen::Matrix<double, 6, 1> dxdot;
  dxdot << linvel_err, angvel_err;

  Eigen::Matrix<double, 6, 1> F = Eigen::Matrix<double, 6, 1>::Zero();
  if (do_stiffness_) F += K_ * dx;
  if (do_damping_)   F += D_ * dxdot;

  // Clamp wrench before mapping to joint space
  clamp(F, cartesian_torque_limits_);

  Eigen::VectorXd tau = J.transpose() * F;

  if (do_gravity_compensation_) {
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(pinocchio_model_.nv);
    tau += pinocchio::rnea(pinocchio_model_, pinocchio_data_, q, zero, zero);
  }

  clamp(tau, joint_torque_limits_);

  return tau;
}

void CartesianImpedance::clamp(Eigen::Ref<Eigen::VectorXd> v, const std::vector<double>& limits) {
  std::vector<double> lower(limits.size());
  std::transform(limits.begin(), limits.end(), lower.begin(), std::negate<double>());
  clamp(v, lower, limits);
}

void CartesianImpedance::clamp(Eigen::Ref<Eigen::VectorXd> v,
    const std::vector<double>& lower_limits,
    const std::vector<double>& upper_limits) {
  for (size_t i = 0; i < lower_limits.size(); ++i) {
    if (v(i) < lower_limits[i]) {
      std::cerr << "[CartesianImpedance] Clamping [" << i << "] to lower limit " << lower_limits[i] << "\n";
      v(i) = lower_limits[i];
    } else if (v(i) > upper_limits[i]) {
      std::cerr << "[CartesianImpedance] Clamping [" << i << "] to upper limit " << upper_limits[i] << "\n";
      v(i) = upper_limits[i];
    }
  }
}

}  // namespace openarm_cartesian_control