// Copyright [2021] Optimus Ride Inc.
// Demo with trajectory visualization output

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fstream>
#include <iostream>

#include "altro/augmented_lagrangian/al_solver.hpp"
#include "altro/common/solver_options.hpp"
#include "altro/ilqr/ilqr.hpp"
#include "examples/problems/unicycle.hpp"

void SaveTrajectory(const std::string& filename, 
                    const altro::Trajectory<3, 2>& traj,
                    const altro::problems::UnicycleProblem& prob_def) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    fmt::print("Failed to open file: {}\n", filename);
    return;
  }

  // Header
  file << "# Unicycle Trajectory Data\n";
  file << "# N=" << prob_def.N << "\n";
  file << "# x, y, theta, v, omega\n";

  // Write trajectory
  for (int k = 0; k <= prob_def.N; ++k) {
    const auto& state = traj.State(k);
    file << state(0) << "," << state(1) << "," << state(2);
    
    if (k < prob_def.N) {
      const auto& control = traj.Control(k);
      file << "," << control(0) << "," << control(1);
    } else {
      file << ",0.0,0.0";
    }
    file << "\n";
  }

  // Write obstacle info
  file << "# Obstacles: cx, cy, cr\n";
  for (int i = 0; i < prob_def.cx.size(); ++i) {
    file << "OBS," << prob_def.cx(i) << "," << prob_def.cy(i) << "," 
         << prob_def.cr(i) << "\n";
  }

  // Write goal
  file << "# Goal: x, y, theta\n";
  file << "GOAL," << prob_def.xf(0) << "," << prob_def.xf(1) << "," 
       << prob_def.xf(2) << "\n";

  file.close();
  fmt::print("Trajectory saved to: {}\n", filename);
}

int main() {
  fmt::print("========================================\n");
  fmt::print("ALTRO C++ - Unicycle Obstacle Avoidance\n");
  fmt::print("========================================\n\n");

  // Create problem
  altro::problems::UnicycleProblem prob_def;
  prob_def.SetScenario(altro::problems::UnicycleProblem::kThreeObstacles);
  const bool add_constraints = true;
  altro::problem::Problem prob = prob_def.MakeProblem(add_constraints);

  // Create solver
  constexpr int NStates = altro::problems::UnicycleProblem::NStates;
  constexpr int NControls = altro::problems::UnicycleProblem::NControls;
  altro::augmented_lagrangian::AugmentedLagrangianiLQR<NStates, NControls> solver(prob);
  
  // Set initial trajectory
  std::shared_ptr<altro::Trajectory<NStates, NControls>> traj_ptr =
      std::make_shared<altro::Trajectory<NStates, NControls>>(prob_def.InitialTrajectory());
  solver.SetTrajectory(traj_ptr);
  
  // IMPORTANT: Rollout the initial trajectory to get the full state sequence
  fmt::print("Rolling out initial trajectory...\n");
  solver.GetiLQRSolver().Rollout();

  // Save initial trajectory (after rollout)
  SaveTrajectory("/tmp/unicycle_initial.csv", *traj_ptr, prob_def);

  // Configure solver
  solver.SetPenalty(10.0);
  solver.GetOptions().verbose = altro::LogLevel::kInner;
  solver.GetOptions().cost_tolerance = 1e-4;
  solver.GetOptions().gradient_tolerance = 1e-3;
  solver.GetOptions().constraint_tolerance = 1e-3;

  // Solve
  fmt::print("\nSolving trajectory optimization problem...\n\n");
  auto start = std::chrono::high_resolution_clock::now();
  solver.Solve();
  auto stop = std::chrono::high_resolution_clock::now();
  
  std::chrono::microseconds duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  // Print results
  fmt::print("\n========================================\n");
  fmt::print("Results:\n");
  fmt::print("========================================\n");
  fmt::print("Status: {}\n", solver.GetStatus() == altro::SolverStatus::kSolved ? "Solved" : "Not Solved");
  fmt::print("Total iterations: {}\n", solver.GetStats().iterations_total);
  fmt::print("Final cost: {:.6f}\n", solver.GetiLQRSolver().Cost());
  fmt::print("Max violation: {:.6e}\n", solver.MaxViolation());
  fmt::print("Solve time: {:.3f} ms\n", duration.count() / 1000.0);
  
  // Print initial and final states
  fmt::print("\nInitial state: ({:.3f}, {:.3f}, {:.3f})\n", 
             traj_ptr->State(0)(0), traj_ptr->State(0)(1), traj_ptr->State(0)(2));
  fmt::print("Final state: ({:.3f}, {:.3f}, {:.3f})\n", 
             traj_ptr->State(prob_def.N)(0), traj_ptr->State(prob_def.N)(1), 
             traj_ptr->State(prob_def.N)(2));
  fmt::print("Goal state: ({:.3f}, {:.3f}, {:.3f})\n", 
             prob_def.xf(0), prob_def.xf(1), prob_def.xf(2));
  fmt::print("========================================\n\n");

  // Save final trajectory
  SaveTrajectory("/tmp/unicycle_final.csv", *traj_ptr, prob_def);

  fmt::print("Run visualization:\n");
  fmt::print("  python3 /home/wbk/altro-cpp/visualize_trajectory.py\n\n");

  return 0;
}


