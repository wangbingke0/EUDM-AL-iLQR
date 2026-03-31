/**
 * @file cartesian_dynamics.cpp
 * @brief Implementation of vehicle kinematic model in Cartesian coordinates
 */

#include "ilqr_planner/cartesian_dynamics.h"

#include <cmath>
#include "altro/utils/utils.hpp"

namespace planning {
namespace ilqr {

void CartesianDynamics::Evaluate(const altro::VectorXdRef& x, const altro::VectorXdRef& u,
                                  float t, Eigen::Ref<altro::VectorXd> xdot) {
  ALTRO_UNUSED(t);
  
  // Extract states
  double yaw = x(kYaw);
  double v = x(kVel);
  double a = x(kAcc);
  double kappa = x(kKappa);
  
  // Extract controls
  double jerk = u(kJerk);
  double dkappa = u(kDKappa);
  
  // Kinematic equations
  // dx/dt = v * cos(yaw)
  xdot(kPosX) = v * std::cos(yaw);
  
  // dy/dt = v * sin(yaw)
  xdot(kPosY) = v * std::sin(yaw);
  
  // dyaw/dt = v * kappa
  xdot(kYaw) = v * kappa;
  
  // dv/dt = a
  xdot(kVel) = a;
  
  // da/dt = jerk (control input)
  xdot(kAcc) = jerk;
  
  // dkappa/dt = dkappa (control input)
  xdot(kKappa) = dkappa;
}

void CartesianDynamics::Jacobian(const altro::VectorXdRef& x, const altro::VectorXdRef& u,
                                  float t, Eigen::Ref<altro::MatrixXd> jac) {
  ALTRO_UNUSED(t);
  ALTRO_UNUSED(u);
  
  // Extract states
  double yaw = x(kYaw);
  double v = x(kVel);
  double kappa = x(kKappa);
  
  // Initialize Jacobian to zero
  jac.setZero();
  
  // Jacobian matrix: [df/dx, df/du]
  // Size: n x (n + m) = 6 x 8
  
  // Partial derivatives of xdot(0) = v * cos(yaw)
  jac(kPosX, kYaw) = -v * std::sin(yaw);  // d/dyaw
  jac(kPosX, kVel) = std::cos(yaw);       // d/dv
  
  // Partial derivatives of xdot(1) = v * sin(yaw)
  jac(kPosY, kYaw) = v * std::cos(yaw);   // d/dyaw
  jac(kPosY, kVel) = std::sin(yaw);       // d/dv
  
  // Partial derivatives of xdot(2) = v * kappa
  jac(kYaw, kVel) = kappa;                // d/dv
  jac(kYaw, kKappa) = v;                  // d/dkappa
  
  // Partial derivatives of xdot(3) = a
  jac(kVel, kAcc) = 1.0;                  // d/da
  
  // Partial derivatives of xdot(4) = jerk (control)
  jac(kAcc, NStates + kJerk) = 1.0;       // d/djerk
  
  // Partial derivatives of xdot(5) = dkappa (control)
  jac(kKappa, NStates + kDKappa) = 1.0;   // d/ddkappa
}

void CartesianDynamics::Hessian(const altro::VectorXdRef& x, const altro::VectorXdRef& u,
                                 float t, const altro::VectorXdRef& b,
                                 Eigen::Ref<altro::MatrixXd> hess) {
  ALTRO_UNUSED(t);
  ALTRO_UNUSED(u);
  
  // Extract states
  double yaw = x(kYaw);
  double v = x(kVel);
  
  // Initialize Hessian to zero
  hess.setZero();
  
  // Hessian of the Lagrangian: sum_i b_i * H_i(f_i)
  // where H_i is the Hessian of the i-th component of f
  
  // f_0 = v * cos(yaw)
  // d²f_0/dyaw² = -v * cos(yaw)
  // d²f_0/dyaw_dv = -sin(yaw)
  // d²f_0/dv_dyaw = -sin(yaw)
  hess(kYaw, kYaw) += b(kPosX) * (-v * std::cos(yaw));
  hess(kYaw, kVel) += b(kPosX) * (-std::sin(yaw));
  hess(kVel, kYaw) += b(kPosX) * (-std::sin(yaw));
  
  // f_1 = v * sin(yaw)
  // d²f_1/dyaw² = -v * sin(yaw)
  // d²f_1/dyaw_dv = cos(yaw)
  // d²f_1/dv_dyaw = cos(yaw)
  hess(kYaw, kYaw) += b(kPosY) * (-v * std::sin(yaw));
  hess(kYaw, kVel) += b(kPosY) * std::cos(yaw);
  hess(kVel, kYaw) += b(kPosY) * std::cos(yaw);
  
  // f_2 = v * kappa
  // d²f_2/dv_dkappa = 1
  // d²f_2/dkappa_dv = 1
  hess(kVel, kKappa) += b(kYaw);
  hess(kKappa, kVel) += b(kYaw);
  
  // f_3, f_4, f_5 are linear, so Hessians are zero
}

}  // namespace ilqr
}  // namespace planning

