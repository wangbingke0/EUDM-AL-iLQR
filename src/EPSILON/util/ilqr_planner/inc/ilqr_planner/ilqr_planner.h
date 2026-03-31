/**
 * @file ilqr_planner.h
 * @brief iLQR-based trajectory planner using altro-cpp in Cartesian coordinates
 * 
 * This planner uses the spatio-temporal corridor from SSC and optimizes
 * the trajectory using Augmented Lagrangian iLQR (AL-iLQR) from altro-cpp.
 * The optimization is performed in Cartesian (x, y, t) coordinates.
 */

#ifndef ILQR_PLANNER_ILQR_PLANNER_H_
#define ILQR_PLANNER_ILQR_PLANNER_H_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

// ROS
#include <ros/ros.h>

// EPSILON common
#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"
#include "common/trajectory/frenet_traj.h"

// SSC dependencies
#include "ssc_planner/map_interface.h"
#include "ssc_planner/ssc_map.h"

// altro-cpp
#include "altro/augmented_lagrangian/al_solver.hpp"
#include "altro/common/trajectory.hpp"
#include "altro/problem/problem.hpp"
#include "altro/problem/discretized_model.hpp"

// iLQR components
#include "ilqr_planner/cartesian_dynamics.h"
#include "ilqr_planner/cartesian_corridor_constraint.h"

// altro-cpp examples (for cost functions)
#include "examples/quadratic_cost.hpp"
#include "examples/basic_constraints.hpp"

namespace planning {
namespace ilqr {

class IlqrLqrWarmStart;
class IlqrAlIlqrSolver;

/**
 * @brief Configuration for iLQR planner
 */
struct IlqrPlannerConfig {
  // Cost weights for state tracking
  double weight_x = 1.0;           // X position tracking
  double weight_y = 10.0;          // Y position tracking
  double weight_yaw = 0.1;         // Yaw tracking
  double weight_v = 1.0;           // Velocity tracking
  double weight_a = 0.1;           // Acceleration weight
  double weight_kappa = 0.1;       // Curvature weight
  
  // Control weights
  double weight_jerk = 0.01;       // Jerk (control)
  double weight_dkappa = 0.1;      // Curvature rate (control)
  
  // Terminal cost weights (usually larger)
  double weight_terminal_x = 100.0;
  double weight_terminal_y = 100.0;
  double weight_terminal_yaw = 10.0;
  double weight_terminal_v = 10.0;
  double weight_terminal_a = 1.0;
  double weight_terminal_kappa = 1.0;

  // Dynamic limits
  double max_velocity = 30.0;      // m/s
  double min_velocity = 0.1;       // m/s
  double max_acceleration = 3.0;   // m/s^2
  double min_acceleration = -5.0;  // m/s^2
  double max_curvature = 0.2;      // 1/m (corresponds to ~5m turning radius)
  double max_jerk = 5.0;           // m/s^3
  double max_dkappa = 0.5;         // 1/m/s

  // AL-iLQR solver settings
  int max_outer_iterations = 20;
  int max_inner_iterations = 100;
  double constraint_tolerance = 1e-4;
  double cost_tolerance = 1e-4;
  double initial_penalty = 1.0;
  double penalty_scaling = 10.0;

  // Trajectory settings
  double time_horizon = 5.0;       // seconds
  double dt = 0.1;                 // time step
  int num_knot_points = 50;        // number of trajectory points
};

/**
 * @brief Cartesian trajectory from iLQR optimization
 */
struct CartesianTrajectory {
  std::vector<Eigen::Matrix<double, 6, 1>> states;  // [x, y, yaw, v, a, kappa]
  std::vector<Eigen::Matrix<double, 2, 1>> controls; // [jerk, dkappa]
  std::vector<double> times;
  double dt = 0.1;
  
  int size() const { return states.size(); }
  bool empty() const { return states.empty(); }
  void clear() { states.clear(); controls.clear(); times.clear(); }
};

/**
 * @brief Frenet trajectory converted from Cartesian iLQR result
 */
class IlqrFrenetTrajectory : public common::FrenetTrajectory {
 public:
  IlqrFrenetTrajectory() = default;
  
  /**
   * @brief Set trajectory from Cartesian trajectory and convert to Frenet
   */
  void SetFromCartesianTrajectory(const CartesianTrajectory& traj,
                                   const common::StateTransformer& stf,
                                   double time_origin);

  bool IsValid() const { return is_valid_; }

  ErrorType GetState(const decimal_t& t, common::State* state) const override;
  ErrorType GetFrenetState(const decimal_t& t, common::FrenetState* fs) const override;
  
  decimal_t begin() const override { return time_stamps_.empty() ? 0.0 : time_stamps_.front(); }
  decimal_t end() const override { return time_stamps_.empty() ? 0.0 : time_stamps_.back(); }

  std::vector<decimal_t> variables() const override { return {}; }
  void set_variables(const std::vector<decimal_t>& variables) override {}
  void Jerk(decimal_t* j_lon, decimal_t* j_lat) const override {
    *j_lon = 0.0;
    *j_lat = 0.0;
  }

  // Get Cartesian states for visualization
  const std::vector<common::State>& GetCartesianStates() const { return cartesian_states_; }

 private:
  bool is_valid_ = false;
  std::vector<double> time_stamps_;
  std::vector<common::State> cartesian_states_;
  std::vector<common::FrenetState> frenet_states_;
  common::StateTransformer stf_;
  
  int FindTimeIndex(double t) const;
  common::State InterpolateCartesianState(int idx, double t) const;
  common::FrenetState InterpolateFrenetState(int idx, double t) const;
};

/**
 * @brief iLQR trajectory planner using altro-cpp in Cartesian coordinates
 */
class IlqrPlanner : public Planner {
 public:
  using FrenetState = common::FrenetState;
  using State = common::State;
  using Vehicle = common::Vehicle;
  using Lane = common::Lane;
  using LateralBehavior = common::LateralBehavior;

  // Dimensions for altro-cpp
  static constexpr int kStateDim = 6;
  static constexpr int kControlDim = 2;

  IlqrPlanner() = default;
  ~IlqrPlanner() = default;

  // Planner interface
  std::string Name() override { return "ilqr_planner"; }
  ErrorType Init(const std::string config_path) override;
  ErrorType RunOnce() override;

  // Setters
  ErrorType set_map_interface(SscPlannerMapItf* map_itf);
  ErrorType set_initial_state(const State& state);
  void set_config(const IlqrPlannerConfig& cfg) { cfg_ = cfg; }

  // Getters
  SscMap* p_ssc_map() const { return p_ssc_map_; }
  FrenetState ego_frenet_state() const { return ego_frenet_state_; }
  Vehicle ego_vehicle() const { return ego_vehicle_; }
  common::FsVehicle fs_ego_vehicle() const { return fs_ego_vehicle_; }
  common::StateTransformer state_transformer() const { return stf_; }
  decimal_t time_origin() const { return time_origin_; }
  decimal_t time_cost() const { return time_cost_; }
  FrenetState initial_frenet_state() const { return initial_frenet_state_; }
  const IlqrPlannerConfig& config() const { return cfg_; }
  Lane reference_lane() const { return nav_lane_local_; }

  // Get optimized trajectory
  std::unique_ptr<common::FrenetTrajectory> trajectory() const {
    return std::unique_ptr<IlqrFrenetTrajectory>(
        new IlqrFrenetTrajectory(optimized_trajectory_));
  }

  // Get forward trajectories (for visualization compatibility with SSC)
  vec_E<vec_E<common::FsVehicle>> forward_trajs_fs() const { 
    return forward_trajs_fs_; 
  }

  vec_E<std::unordered_map<int, vec_E<common::FsVehicle>>> 
  surround_forward_trajs_fs() const {
    return surround_forward_trajs_fs_;
  }

  // Get Cartesian trajectory for visualization
  const CartesianTrajectory& GetCartesianTrajectory() const { return cartesian_traj_; }

  // Get corridor polygons for visualization
  const CartesianCorridor& GetCartesianCorridor() const { return cartesian_corridor_; }

  // Get reference trajectory in Cartesian coordinates
  const std::vector<State>& GetReferenceStates() const { return reference_cartesian_states_; }

  // Get LQR warm start trajectory for visualization
  const CartesianTrajectory& GetLQRWarmStartTrajectory() const { return lqr_warm_start_traj_; }

  // Get corridor cubes for visualization (Frenet)
  const vec_E<common::SpatioTemporalSemanticCubeNd<2>>& GetCorridorCubes() const {
    return corridor_cubes_;
  }

 private:
  friend class IlqrLqrWarmStart;
  friend class IlqrAlIlqrSolver;

  // Run the AL-iLQR solver
  ErrorType RunALiLQROptimization();
  
  // Build the trajectory optimization problem
  ErrorType BuildProblem();
  
  // Convert reference trajectory from Frenet to Cartesian
  ErrorType ConvertReferenceToCartesian();
  
  // Sample reference trajectory by time-based interpolation
  // Samples at fixed time intervals (e.g., 50 points at 0.1s intervals for 5 seconds)
  ErrorType SampleReferenceTrajectoryByTime(int num_points, double dt, 
                                            const std::vector<common::State>& original_trajectory);
  
  // Interpolate state at a specific time from trajectory
  ErrorType InterpolateStateAtTime(const std::vector<common::State>& trajectory, 
                                   double target_time, common::State* output);
  
  // Sample reference trajectory to fixed number of points (50 points, 5 seconds, 0.1s per point)
  // [DEPRECATED] Use SampleReferenceTrajectoryByTime instead
  // Sampling rule: if v < 10 m/s, use 10 m/s; if v >= 10 m/s, use actual velocity
  // extended_trajectory: the extended trajectory to sample from (can be empty, will use reference_frenet_states_ if empty)
  ErrorType SampleReferenceTrajectoryToFixedPoints(int num_points, double dt, double min_sampling_velocity,
                                                    const std::vector<common::State>* extended_trajectory = nullptr);
  
  // Extend a trajectory to ensure minimum length (used before sampling)
  // [DEPRECATED] No longer needed with time-based sampling
  void ExtendTrajectoryToMinimumLength(std::vector<common::State>* trajectory, double min_length_meters);
  
  // Extend reference trajectory to ensure minimum length (deprecated, replaced by SampleReferenceTrajectoryByTime)
  void ExtendReferenceTrajectoryToMinimumLength(double min_length_meters);

  // State transformation
  ErrorType StateTransformForInputData();

  // Configuration
  IlqrPlannerConfig cfg_;

  // Map interface
  SscPlannerMapItf* map_itf_ = nullptr;
  bool map_valid_ = false;
  SscMap* p_ssc_map_ = nullptr;

  // Vehicle data
  Vehicle ego_vehicle_;
  LateralBehavior ego_behavior_;
  FrenetState ego_frenet_state_;
  Lane nav_lane_local_;
  common::StateTransformer stf_;

  // Initial state
  State initial_state_;
  bool has_initial_state_ = false;
  FrenetState initial_frenet_state_;

  // Frenet space data
  common::FsVehicle fs_ego_vehicle_;
  vec_E<vec_E<common::FsVehicle>> forward_trajs_fs_;
  vec_E<std::unordered_map<int, vec_E<common::FsVehicle>>> surround_forward_trajs_fs_;

  // Forward simulation data (from EUDM)
  vec_E<vec_E<Vehicle>> forward_trajs_;
  std::vector<LateralBehavior> forward_behaviors_;
  vec_E<std::unordered_map<int, vec_E<Vehicle>>> surround_forward_trajs_;

  // Corridor data
  vec_E<common::SpatioTemporalSemanticCubeNd<2>> corridor_cubes_;
  CartesianCorridor cartesian_corridor_;
  CorridorConstraintFactory corridor_factory_;

  // Reference trajectory
  std::vector<FrenetState> reference_frenet_states_;
  std::vector<State> reference_cartesian_states_;

  // altro-cpp components
  std::shared_ptr<CartesianDynamics> dynamics_;
  std::vector<std::shared_ptr<altro::problem::DiscreteDynamics>> discrete_dynamics_;

  // Optimized trajectory
  CartesianTrajectory cartesian_traj_;
  IlqrFrenetTrajectory optimized_trajectory_;
  
  // LQR warm start trajectory (for visualization as reference)
  CartesianTrajectory lqr_warm_start_traj_;

  // Timing
  decimal_t stamp_ = 0.0;
  decimal_t time_origin_ = 0.0;
  decimal_t time_cost_ = 0.0;
};

}  // namespace ilqr
}  // namespace planning

#endif  // ILQR_PLANNER_ILQR_PLANNER_H_
