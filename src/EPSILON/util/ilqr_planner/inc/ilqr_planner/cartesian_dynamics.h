/**
 * @file cartesian_dynamics.h
 * @brief Vehicle kinematic model in Cartesian coordinates for trajectory optimization
 * 
 * State: [x, y, yaw, v, a, kappa] (6 states)
 * Control: [jerk, dkappa] (2 controls)
 * 
 * The model uses a kinematic bicycle model with continuous acceleration and curvature.
 */

#ifndef ILQR_PLANNER_CARTESIAN_DYNAMICS_H_
#define ILQR_PLANNER_CARTESIAN_DYNAMICS_H_

#include "altro/problem/dynamics.hpp"
#include "altro/eigentypes.hpp"

namespace planning {
namespace ilqr {

/**
 * @brief Vehicle kinematic model in Cartesian (x, y, t) coordinate system
 * 
 * State vector: x = [x, y, yaw, v, a, kappa]
 *   - x: X position (m)
 *   - y: Y position (m)  
 *   - yaw: Heading angle (rad)
 *   - v: Velocity (m/s)
 *   - a: Acceleration (m/s^2)
 *   - kappa: Curvature (1/m)
 * 
 * Control vector: u = [jerk, dkappa]
 *   - jerk: Rate of change of acceleration (m/s^3)
 *   - dkappa: Rate of change of curvature (1/m/s)
 * 
 * Dynamics:
 *   dx/dt = v * cos(yaw)
 *   dy/dt = v * sin(yaw)
 *   dyaw/dt = v * kappa
 *   dv/dt = a
 *   da/dt = jerk
 *   dkappa/dt = dkappa
 */
class CartesianDynamics : public altro::problem::ContinuousDynamics {
 public:
  // Dimensions
  static constexpr int NStates = 6;
  static constexpr int NControls = 2;
  
  // State indices
  static constexpr int kPosX = 0;
  static constexpr int kPosY = 1;
  static constexpr int kYaw = 2;
  static constexpr int kVel = 3;
  static constexpr int kAcc = 4;
  static constexpr int kKappa = 5;
  
  // Control indices
  static constexpr int kJerk = 0;
  static constexpr int kDKappa = 1;

  CartesianDynamics() = default;

  int StateDimension() const override { return NStates; }
  int ControlDimension() const override { return NControls; }

  /**
   * @brief Evaluate the continuous dynamics
   * xdot = f(x, u)
   */
  void Evaluate(const altro::VectorXdRef& x, const altro::VectorXdRef& u, float t,
                Eigen::Ref<altro::VectorXd> xdot) override;

  /**
   * @brief Compute Jacobian of dynamics
   * jac = [df/dx, df/du]
   */
  void Jacobian(const altro::VectorXdRef& x, const altro::VectorXdRef& u, float t,
                Eigen::Ref<altro::MatrixXd> jac) override;

  /**
   * @brief Compute Hessian of Lagrangian for dynamics
   * hess = sum_i b_i * H_i(f_i)
   */
  void Hessian(const altro::VectorXdRef& x, const altro::VectorXdRef& u, float t,
               const altro::VectorXdRef& b, Eigen::Ref<altro::MatrixXd> hess) override;

  bool HasHessian() const override { return true; }
};

}  // namespace ilqr
}  // namespace planning

#endif  // ILQR_PLANNER_CARTESIAN_DYNAMICS_H_

