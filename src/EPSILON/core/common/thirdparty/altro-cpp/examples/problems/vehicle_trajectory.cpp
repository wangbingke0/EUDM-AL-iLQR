// Vehicle Trajectory Optimization Problem Implementation

#include "examples/problems/vehicle_trajectory.hpp"

namespace altro {
namespace problems {

VehicleTrajectoryProblem::VehicleTrajectoryProblem() {
  // Initialize default values
  x0.setZero();
  xf.setZero();
  u0.setZero();
  uref.setZero();
  
  // Default control bounds
  u_lb = {-jerk_max, -dkappa_max};
  u_ub = {+jerk_max, +dkappa_max};
}

altro::problem::Problem VehicleTrajectoryProblem::MakeProblem(bool add_constraints) {
  float h;
  s_curve_waypoints_ = false;  // Reset waypoint flag
  
  if (scenario_ == kLaneChange) {
    // Lane change scenario with obstacle avoidance
    // Start at origin, avoid a stopped vehicle ahead, change to left lane
    tf = 5.0;
    N = 100;
    h = GetTimeStep();
    
    // Initial state: starting with 5 m/s velocity
    x0 << 0, 0, 0, 5.0, 0, 0;
    
    // Goal state: 35m ahead, 3.5m to the left (lane width)
    xf << 35, 3.5, 0, 5.0, 0, 0;
    
    // Initial control guess - slight curvature to help avoid obstacle
    u0 << 0.0, 0.02;
    uref << 0.0, 0.0;
    
    // Cost weights
    Q.diagonal() << 0.1, 0.5, 0.5, 0.3, 0.1, 0.1;
    Q *= h;
    R.diagonal() << 0.1, 0.3;
    R *= h;
    Qf.diagonal() << 100, 100, 100, 50, 10, 10;
    
    // Control bounds
    u_lb = {-jerk_max, -dkappa_max};
    u_ub = {+jerk_max, +dkappa_max};
    
    // ===== Add obstacle (stopped vehicle ahead) =====
    // Vehicle dimensions: ~4.5m x 1.8m, we model as a circle with safety margin
    // Obstacle position: 15m ahead in the current lane (y=0)
    // Radius: ~2.5m (vehicle half-diagonal + safety buffer)
    constexpr int num_obstacles = 1;
    obs_x = Eigen::VectorXd(num_obstacles);
    obs_y = Eigen::VectorXd(num_obstacles);
    obs_r = Eigen::VectorXd(num_obstacles);
    
    obs_x << 15.0;   // x position of obstacle
    obs_y << 1.5;    // y position (moved up, partially blocking lane change)
    obs_r << 3.0;    // radius with safety margin
    
    // Create obstacle constraint
    obstacles = altro::examples::CircleConstraint();
    for (int i = 0; i < num_obstacles; ++i) {
      obstacles.AddObstacle(obs_x(i), obs_y(i), obs_r(i));
    }
    
  } else if (scenario_ == kParking) {
    // Parking scenario (90 degree turn)
    tf = 6.0;
    N = 120;
    h = GetTimeStep();
    
    // Initial state: moving forward slowly
    x0 << 0, 0, 0, 2.0, 0, 0;
    
    // Goal state: turned 90 degrees, moved to parking spot
    xf << 5.0, 5.0, M_PI/2, 0, 0, 0;  // End stationary
    
    // Initial control guess
    u0 << 0.0, 0.01;
    uref << 0.0, 0.0;
    
    // Cost weights
    Q.diagonal() << 0.5, 0.5, 0.5, 0.5, 0.2, 0.2;
    Q *= h;
    R.diagonal() << 0.1, 0.2;
    R *= h;
    Qf.diagonal() << 200, 200, 200, 100, 50, 50;
    
    // Tighter control bounds for parking
    u_lb = {-3.0, -0.3};
    u_ub = {+3.0, +0.3};
    
  } else if (scenario_ == kSCurve) {
    // S-curve maneuver: Double lane change (left then right)
    tf = 10.0;
    N = 200;
    h = GetTimeStep();
    
    // Initial state: moving forward at 4 m/s
    x0 << 0, 0, 0, 4.0, 0, 0;
    
    // Goal state: completed S-curve, back to original lane
    xf << 50, 0, 0, 4.0, 0, 0;
    
    // Initial control guess - small curvature to help convergence
    u0 << 0.0, 0.01;
    uref << 0.0, 0.0;
    
    // Cost weights
    Q.diagonal() << 0.1, 0.1, 0.3, 0.3, 0.1, 0.1;
    Q *= h;
    R.diagonal() << 0.05, 0.1;
    R *= h;
    Qf.diagonal() << 200, 200, 100, 50, 20, 20;
    
    // Control bounds
    u_lb = {-jerk_max, -dkappa_max};
    u_ub = {+jerk_max, +dkappa_max};
    
    // S-curve waypoints will be added as constraints below
    s_curve_waypoints_ = true;
  }

  // Create problem with correct N
  altro::problem::Problem prob(N);

  // Cost Function
  for (int k = 0; k < N; ++k) {
    qcost = std::make_shared<examples::QuadraticCost>(
        examples::QuadraticCost::LQRCost(Q, R, xf, uref));
    prob.SetCostFunction(qcost, k);
  }
  
  // Terminal cost
  qterm = std::make_shared<examples::QuadraticCost>(
      examples::QuadraticCost::LQRCost(Qf, R * 0, xf, uref, true));
  prob.SetCostFunction(qterm, N);

  // Dynamics
  for (int k = 0; k < N; ++k) {
    prob.SetDynamics(std::make_shared<ModelType>(model), k);
  }

  // Constraints
  if (add_constraints) {
    // Control bounds
    for (int k = 0; k < N; ++k) {
      prob.SetConstraint(std::make_shared<altro::examples::ControlBound>(u_lb, u_ub), k);
    }
    
    // Goal constraint
    prob.SetConstraint(std::make_shared<examples::GoalConstraint>(xf), N);
    
    // Obstacle avoidance constraints (for Lane Change scenario)
    if (scenario_ == kLaneChange && obs_x.size() > 0) {
      // Add obstacle constraint to all knot points except terminal
      for (int k = 1; k < N; ++k) {
        std::shared_ptr<altro::constraints::Constraint<altro::constraints::Inequality>> obs_con =
            std::make_shared<altro::examples::CircleConstraint>(obstacles);
        prob.SetConstraint(obs_con, k);
      }
    }
    
    // S-curve waypoint constraints (for double lane change)
    if (s_curve_waypoints_) {
      // Waypoint 1: At 1/4 of trajectory, move to left lane (y = 3.5m)
      int k1 = N / 4;
      prob.SetConstraint(
          std::make_shared<altro::examples::WaypointConstraint>(12.5, 3.5), k1);
      
      // Waypoint 2: At 3/4 of trajectory, move to right lane (y = -3.5m)
      int k2 = 3 * N / 4;
      prob.SetConstraint(
          std::make_shared<altro::examples::WaypointConstraint>(37.5, -3.5), k2);
    }
  }

  // Initial State
  prob.SetInitialState(x0);

  return prob;
}

}  // namespace problems
}  // namespace altro

