// Vehicle Kinematic Model for Trajectory Optimization
// State: [x, y, yaw, v, a, kappa] (6 states)
// Control: [da, dkappa] (2 controls)
// da: derivative of acceleration (jerk)
// dkappa: derivative of curvature

#pragma once

#include "altro/eigentypes.hpp"
#include "altro/problem/dynamics.hpp"

namespace altro {
namespace examples {

/**
 * @brief Vehicle kinematic model with continuous acceleration and curvature
 *
 * Has 6 states and 2 controls.
 *
 * # State Vector
 * x = [x, y, yaw, v, a, kappa]
 * where:
 *   - x, y: position
 *   - yaw: heading angle
 *   - v: velocity
 *   - a: acceleration
 *   - kappa: curvature (1/turning_radius)
 *
 * # Control Vector
 * u = [da, dkappa]
 * where:
 *   - da: jerk (derivative of acceleration)
 *   - dkappa: derivative of curvature
 *
 * # Mathematical Formulation
 * \f[
 * \begin{bmatrix} 
 *   \dot{x} \\ \dot{y} \\ \dot{\psi} \\ \dot{v} \\ \dot{a} \\ \dot{\kappa}
 * \end{bmatrix} =
 * \begin{bmatrix} 
 *   v \cos(\psi) \\ 
 *   v \sin(\psi) \\ 
 *   v \kappa \\
 *   a \\
 *   da \\
 *   d\kappa
 * \end{bmatrix}
 * \f]
 */
class VehicleKinematic : public problem::ContinuousDynamics {
 public:
  VehicleKinematic() = default;
  
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
  static constexpr int kJerk = 0;      // da
  static constexpr int kDKappa = 1;    // dkappa

  int StateDimension() const override { return NStates; }
  int ControlDimension() const override { return NControls; }

  void Evaluate(const VectorXdRef& x, const VectorXdRef& u, const float t,
                Eigen::Ref<VectorXd> xdot) override;
  void Jacobian(const VectorXdRef& x, const VectorXdRef& u, const float t,
                Eigen::Ref<MatrixXd> jac) override;
  void Hessian(const VectorXdRef& x, const VectorXdRef& u, const float t,
               const VectorXdRef& b, Eigen::Ref<MatrixXd> hess) override;
  bool HasHessian() const override { return true; }
};

}  // namespace examples
}  // namespace altro

