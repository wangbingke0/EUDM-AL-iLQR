// Vehicle Trajectory Optimization Problem Definition
// State: [x, y, yaw, v, a, kappa] (6 states)
// Control: [da, dkappa] (2 controls)

#pragma once

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "altro/eigentypes.hpp"
#include "altro/problem/problem.hpp"
#include "altro/augmented_lagrangian/al_solver.hpp"
#include "altro/augmented_lagrangian/al_problem.hpp"
#include "altro/common/trajectory.hpp"
#include "altro/ilqr/ilqr.hpp"
#include "altro/problem/discretized_model.hpp"
#include "examples/basic_constraints.hpp"
#include "examples/obstacle_constraints.hpp"
#include "examples/quadratic_cost.hpp"
#include "examples/vehicle_kinematic.hpp"

namespace altro {
namespace problems {

/**
 * @brief Vehicle trajectory optimization problem
 * 
 * State: [x, y, yaw, v, a, kappa]
 * Control: [da, dkappa]
 * 
 * Scenarios:
 * - kLaneChange: Lane change maneuver
 * - kParking: Parking maneuver (90 degree turn)
 * - kSCurve: S-curve path following
 */
class VehicleTrajectoryProblem {
 public:
  static constexpr int NStates = 6;
  static constexpr int NControls = 2;

  VehicleTrajectoryProblem();

  enum Scenario { kLaneChange, kParking, kSCurve };

  using ModelType = altro::problem::DiscretizedModel<altro::examples::VehicleKinematic>;
  using CostFunType = altro::examples::QuadraticCost;

  // Problem Data
  static constexpr int HEAP = Eigen::Dynamic;
  const int n = NStates;
  const int m = NControls;

  int N = 100;  // Number of time steps
  ModelType model = ModelType(altro::examples::VehicleKinematic());

  // Cost matrices
  Eigen::Matrix<double, NStates, NStates> Q = 
      Eigen::Matrix<double, NStates, 1>::Constant(1e-2).asDiagonal();
  Eigen::Matrix<double, NControls, NControls> R = 
      Eigen::Matrix<double, NControls, 1>::Constant(1e-1).asDiagonal();
  Eigen::Matrix<double, NStates, NStates> Qf = 
      Eigen::Matrix<double, NStates, 1>::Constant(100).asDiagonal();

  // State and control limits
  double v_max = 10.0;    // max velocity [m/s]
  double v_min = -2.0;    // min velocity (reverse) [m/s]
  double a_max = 3.0;     // max acceleration [m/s²]
  double a_min = -5.0;    // max deceleration [m/s²]
  double kappa_max = 0.5; // max curvature [1/m] (corresponds to ~2m turning radius)
  double jerk_max = 5.0;  // max jerk [m/s³]
  double dkappa_max = 0.5; // max curvature rate [1/m/s]

  // Boundary states
  Eigen::Matrix<double, NStates, 1> x0;  // Initial state
  Eigen::Matrix<double, NStates, 1> xf;  // Goal state
  Eigen::Matrix<double, NControls, 1> u0;  // Initial control guess
  Eigen::Matrix<double, NControls, 1> uref;  // Reference control

  // Cost function pointers
  std::shared_ptr<examples::QuadraticCost> qcost;
  std::shared_ptr<examples::QuadraticCost> qterm;

  // Control bounds
  std::vector<double> u_lb;
  std::vector<double> u_ub;

  // Obstacle data (for Lane Change with obstacle avoidance)
  Eigen::VectorXd obs_x;  // x-coordinates of obstacles
  Eigen::VectorXd obs_y;  // y-coordinates of obstacles  
  Eigen::VectorXd obs_r;  // radii of obstacles (including safety margin)
  altro::examples::CircleConstraint obstacles;

  // Create problem
  altro::problem::Problem MakeProblem(bool add_constraints = true);

  // Create initial trajectory
  template <int n_size = NStates, int m_size = NControls>
  altro::Trajectory<n_size, m_size> InitialTrajectory();

  // Create solvers
  template <int n_size = NStates, int m_size = NControls>
  altro::ilqr::iLQR<n_size, m_size> MakeSolver(bool alcost = false);

  template <int n_size = NStates, int m_size = NControls>
  altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size> MakeALSolver();

  // Set scenario
  void SetScenario(Scenario scenario) { scenario_ = scenario; }
  Scenario GetScenario() const { return scenario_; }

  // Get time step
  float GetTimeStep() const { return tf / N; }
  float GetFinalTime() const { return tf; }

 private:
  Scenario scenario_ = kLaneChange;
  float tf = 5.0;  // Final time [s]
  bool s_curve_waypoints_ = false;  // Flag for S-curve waypoint constraints
};

// Template implementations

template <int n_size, int m_size>
altro::Trajectory<n_size, m_size> VehicleTrajectoryProblem::InitialTrajectory() {
  altro::Trajectory<n_size, m_size> Z(n, m, N);
  
  // Set initial control guess
  for (int k = 0; k < N; ++k) {
    Z.Control(k) = u0;
  }
  
  float h = GetTimeStep();
  Z.SetUniformStep(h);
  return Z;
}

template <int n_size, int m_size>
altro::ilqr::iLQR<n_size, m_size> VehicleTrajectoryProblem::MakeSolver(bool alcost) {
  altro::problem::Problem prob = MakeProblem();
  if (alcost) {
    prob = altro::augmented_lagrangian::BuildAugLagProblem<n_size, m_size>(prob);
  }
  altro::ilqr::iLQR<n_size, m_size> solver(prob);

  std::shared_ptr<altro::Trajectory<n_size, m_size>> traj_ptr =
      std::make_shared<altro::Trajectory<n_size, m_size>>(InitialTrajectory<n_size, m_size>());

  solver.SetTrajectory(traj_ptr);
  solver.Rollout();
  return solver;
}

template <int n_size, int m_size>
altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size>
VehicleTrajectoryProblem::MakeALSolver() {
  altro::problem::Problem prob = MakeProblem(true);
  altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size> solver_al(prob);
  solver_al.SetTrajectory(
      std::make_shared<altro::Trajectory<NStates, NControls>>(InitialTrajectory()));
  solver_al.GetiLQRSolver().Rollout();

  return solver_al;
}

}  // namespace problems
}  // namespace altro

