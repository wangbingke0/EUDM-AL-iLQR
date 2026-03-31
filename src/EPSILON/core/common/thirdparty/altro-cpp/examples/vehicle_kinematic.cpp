// Vehicle Kinematic Model Implementation

#include "examples/vehicle_kinematic.hpp"

#include <cmath>

#include "altro/utils/utils.hpp"

namespace altro {
namespace examples {

void VehicleKinematic::Evaluate(const VectorXdRef& x, const VectorXdRef& u,
                                 const float t, Eigen::Ref<VectorXd> xdot) {
  ALTRO_UNUSED(t);
  
  // Extract states
  double yaw = x(kYaw);
  double v = x(kVel);
  double a = x(kAcc);
  double kappa = x(kKappa);
  
  // Extract controls
  double da = u(kJerk);
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
  
  // da/dt = da (jerk, control input)
  xdot(kAcc) = da;
  
  // dkappa/dt = dkappa (control input)
  xdot(kKappa) = dkappa;
}

void VehicleKinematic::Jacobian(const VectorXdRef& x, const VectorXdRef& u,
                                 const float t, Eigen::Ref<MatrixXd> jac) {
  ALTRO_UNUSED(t);
  ALTRO_UNUSED(u);
  
  // Extract states
  double yaw = x(kYaw);
  double v = x(kVel);
  double kappa = x(kKappa);
  
  // Jacobian matrix: [df/dx, df/du]
  // Size: n x (n + m) = 6 x 8
  
  // Partial derivatives of xdot(0) = v * cos(yaw)
  // d/dyaw: -v * sin(yaw)
  jac(kPosX, kYaw) = -v * std::sin(yaw);
  // d/dv: cos(yaw)
  jac(kPosX, kVel) = std::cos(yaw);
  
  // Partial derivatives of xdot(1) = v * sin(yaw)
  // d/dyaw: v * cos(yaw)
  jac(kPosY, kYaw) = v * std::cos(yaw);
  // d/dv: sin(yaw)
  jac(kPosY, kVel) = std::sin(yaw);
  
  // Partial derivatives of xdot(2) = v * kappa
  // d/dv: kappa
  jac(kYaw, kVel) = kappa;
  // d/dkappa: v
  jac(kYaw, kKappa) = v;
  
  // Partial derivatives of xdot(3) = a
  // d/da: 1
  jac(kVel, kAcc) = 1.0;
  
  // Partial derivatives of xdot(4) = da (control)
  // d/d(da): 1 (column index = n + kJerk = 6 + 0 = 6)
  jac(kAcc, NStates + kJerk) = 1.0;
  
  // Partial derivatives of xdot(5) = dkappa (control)
  // d/d(dkappa): 1 (column index = n + kDKappa = 6 + 1 = 7)
  jac(kKappa, NStates + kDKappa) = 1.0;
}

void VehicleKinematic::Hessian(const VectorXdRef& x, const VectorXdRef& u,
                                const float t, const VectorXdRef& b,
                                Eigen::Ref<MatrixXd> hess) {
  ALTRO_UNUSED(t);
  ALTRO_UNUSED(u);
  
  // Extract states
  double yaw = x(kYaw);
  double v = x(kVel);
  
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

}  // namespace examples
}  // namespace altro

