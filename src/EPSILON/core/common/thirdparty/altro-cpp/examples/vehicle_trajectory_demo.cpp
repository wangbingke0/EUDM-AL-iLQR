// Vehicle Trajectory Optimization Demo
// Demonstrates trajectory optimization with a 6-state vehicle kinematic model
// State: [x, y, yaw, v, a, kappa]
// Control: [da, dkappa] (jerk and curvature rate)

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fstream>
#include <iostream>
#include <chrono>

#include "altro/augmented_lagrangian/al_solver.hpp"
#include "altro/common/solver_options.hpp"
#include "altro/ilqr/ilqr.hpp"
#include "examples/problems/vehicle_trajectory.hpp"

// Save trajectory to CSV file for visualization
void SaveTrajectory(const std::string& filename,
                    const altro::Trajectory<6, 2>& traj,
                    const altro::problems::VehicleTrajectoryProblem& prob_def) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    fmt::print("Failed to open file: {}\n", filename);
    return;
  }

  // Header
  file << "# Vehicle Trajectory Data\n";
  file << "# N=" << prob_def.N << "\n";
  file << "# tf=" << prob_def.GetFinalTime() << "\n";
  file << "# Scenario=" << static_cast<int>(prob_def.GetScenario()) << "\n";
  file << "# Columns: x, y, yaw, v, a, kappa, da, dkappa, t\n";

  float h = prob_def.GetTimeStep();
  
  // Write trajectory data
  for (int k = 0; k <= prob_def.N; ++k) {
    const auto& state = traj.State(k);
    file << state(0) << ","   // x
         << state(1) << ","   // y
         << state(2) << ","   // yaw
         << state(3) << ","   // v
         << state(4) << ","   // a
         << state(5);         // kappa
    
    if (k < prob_def.N) {
      const auto& control = traj.Control(k);
      file << "," << control(0) << "," << control(1);  // da, dkappa
    } else {
      file << ",0.0,0.0";  // Terminal state has no control
    }
    file << "," << (k * h) << "\n";  // time
  }

  // Write goal state
  file << "# Goal: x, y, yaw, v, a, kappa\n";
  file << "GOAL," << prob_def.xf(0) << "," << prob_def.xf(1) << ","
       << prob_def.xf(2) << "," << prob_def.xf(3) << ","
       << prob_def.xf(4) << "," << prob_def.xf(5) << "\n";

  // Write obstacle info (if any)
  if (prob_def.obs_x.size() > 0) {
    file << "# Obstacles: x, y, radius\n";
    for (int i = 0; i < prob_def.obs_x.size(); ++i) {
      file << "OBS," << prob_def.obs_x(i) << "," << prob_def.obs_y(i) << "," 
           << prob_def.obs_r(i) << "\n";
    }
  }

  file.close();
  fmt::print("Trajectory saved to: {}\n", filename);
}

// Print scenario description
void PrintScenarioInfo(altro::problems::VehicleTrajectoryProblem::Scenario scenario) {
  fmt::print("\n");
  switch (scenario) {
    case altro::problems::VehicleTrajectoryProblem::kLaneChange:
      fmt::print("Scenario: Lane Change with Obstacle Avoidance\n");
      fmt::print("  - Start: Origin with 5 m/s velocity\n");
      fmt::print("  - Obstacle: Stopped vehicle at (15m, 0m)\n");
      fmt::print("  - Goal: 35m forward, 3.5m lateral (left lane)\n");
      break;
    case altro::problems::VehicleTrajectoryProblem::kParking:
      fmt::print("Scenario: Parking Maneuver\n");
      fmt::print("  - Start: Origin with 2 m/s velocity\n");
      fmt::print("  - Goal: 90-degree turn, end stationary\n");
      break;
    case altro::problems::VehicleTrajectoryProblem::kSCurve:
      fmt::print("Scenario: S-Curve\n");
      fmt::print("  - Start: Origin with 3 m/s velocity\n");
      fmt::print("  - Goal: 40m forward, return to original lane\n");
      break;
  }
  fmt::print("\n");
}

int main(int argc, char* argv[]) {
  fmt::print("================================================\n");
  fmt::print("ALTRO C++ - Vehicle Trajectory Optimization Demo\n");
  fmt::print("================================================\n\n");
  
  fmt::print("Vehicle Kinematic Model:\n");
  fmt::print("  State:   [x, y, yaw, v, a, kappa] (6 states)\n");
  fmt::print("  Control: [da, dkappa] (jerk, curvature rate)\n");
  fmt::print("================================================\n");

  // Parse command line arguments for scenario selection
  auto scenario = altro::problems::VehicleTrajectoryProblem::kLaneChange;
  if (argc > 1) {
    int scenario_id = std::atoi(argv[1]);
    switch (scenario_id) {
      case 0:
        scenario = altro::problems::VehicleTrajectoryProblem::kLaneChange;
        break;
      case 1:
        scenario = altro::problems::VehicleTrajectoryProblem::kParking;
        break;
      case 2:
        scenario = altro::problems::VehicleTrajectoryProblem::kSCurve;
        break;
      default:
        fmt::print("Unknown scenario {}. Using default (Lane Change).\n", scenario_id);
    }
  }

  // Create problem
  altro::problems::VehicleTrajectoryProblem prob_def;
  prob_def.SetScenario(scenario);
  PrintScenarioInfo(scenario);
  
  const bool add_constraints = true;
  altro::problem::Problem prob = prob_def.MakeProblem(add_constraints);

  // Create solver (Augmented Lagrangian with iLQR)
  constexpr int NStates = altro::problems::VehicleTrajectoryProblem::NStates;
  constexpr int NControls = altro::problems::VehicleTrajectoryProblem::NControls;
  altro::augmented_lagrangian::AugmentedLagrangianiLQR<NStates, NControls> solver(prob);

  // Set initial trajectory (N is now set correctly after MakeProblem)
  std::shared_ptr<altro::Trajectory<NStates, NControls>> traj_ptr =
      std::make_shared<altro::Trajectory<NStates, NControls>>(
          prob_def.n, prob_def.m, prob_def.N);
  for (int k = 0; k < prob_def.N; ++k) {
    traj_ptr->Control(k) = prob_def.u0;
  }
  traj_ptr->SetUniformStep(prob_def.GetTimeStep());
  solver.SetTrajectory(traj_ptr);

  // Rollout initial trajectory
  fmt::print("Rolling out initial trajectory...\n");
  solver.GetiLQRSolver().Rollout();

  // Save initial trajectory
  SaveTrajectory("/tmp/vehicle_initial.csv", *traj_ptr, prob_def);

  // Configure solver
  solver.SetPenalty(1.0);  // Start with lower penalty for better convergence
  solver.SetPenaltyScaling(10.0);  // Scale penalty more aggressively
  solver.GetOptions().verbose = altro::LogLevel::kInner;
  solver.GetOptions().cost_tolerance = 1e-4;
  solver.GetOptions().gradient_tolerance = 1e-2;  // Relaxed gradient tolerance
  solver.GetOptions().constraint_tolerance = 0.1;  // Relaxed constraint tolerance
  solver.GetOptions().max_iterations_inner = 100;
  solver.GetOptions().max_iterations_outer = 50;

  // Solve
  fmt::print("\nSolving trajectory optimization problem...\n\n");
  auto start = std::chrono::high_resolution_clock::now();
  solver.Solve();
  auto stop = std::chrono::high_resolution_clock::now();

  std::chrono::microseconds duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  // Print results
  fmt::print("\n================================================\n");
  fmt::print("Results:\n");
  fmt::print("================================================\n");
  fmt::print("Status: {}\n",
             solver.GetStatus() == altro::SolverStatus::kSolved ? "SOLVED" : "NOT SOLVED");
  fmt::print("Total iterations: {}\n", solver.GetStats().iterations_total);
  fmt::print("Final cost: {:.6f}\n", solver.GetiLQRSolver().Cost());
  fmt::print("Max violation: {:.6e}\n", solver.MaxViolation());
  fmt::print("Solve time: {:.3f} ms\n", duration.count() / 1000.0);

  // Print initial and final states
  fmt::print("\nState comparison:\n");
  fmt::print("  Initial state: x={:.3f}, y={:.3f}, yaw={:.3f}°, v={:.3f}, a={:.3f}, κ={:.4f}\n",
             traj_ptr->State(0)(0), traj_ptr->State(0)(1),
             traj_ptr->State(0)(2) * 180.0 / M_PI,
             traj_ptr->State(0)(3), traj_ptr->State(0)(4), traj_ptr->State(0)(5));
  fmt::print("  Final state:   x={:.3f}, y={:.3f}, yaw={:.3f}°, v={:.3f}, a={:.3f}, κ={:.4f}\n",
             traj_ptr->State(prob_def.N)(0), traj_ptr->State(prob_def.N)(1),
             traj_ptr->State(prob_def.N)(2) * 180.0 / M_PI,
             traj_ptr->State(prob_def.N)(3), traj_ptr->State(prob_def.N)(4),
             traj_ptr->State(prob_def.N)(5));
  fmt::print("  Goal state:    x={:.3f}, y={:.3f}, yaw={:.3f}°, v={:.3f}, a={:.3f}, κ={:.4f}\n",
             prob_def.xf(0), prob_def.xf(1), prob_def.xf(2) * 180.0 / M_PI,
             prob_def.xf(3), prob_def.xf(4), prob_def.xf(5));
  fmt::print("================================================\n\n");

  // Save final trajectory
  SaveTrajectory("/tmp/vehicle_final.csv", *traj_ptr, prob_def);

  // Print visualization instructions
  fmt::print("To visualize the trajectory, run:\n");
  fmt::print("  python3 /home/wbk/altro-cpp/visualize_vehicle_trajectory.py\n\n");
  fmt::print("To run different scenarios:\n");
  fmt::print("  ./vehicle_trajectory_demo 0  # Lane Change\n");
  fmt::print("  ./vehicle_trajectory_demo 1  # Parking\n");
  fmt::print("  ./vehicle_trajectory_demo 2  # S-Curve\n\n");

  return 0;
}

