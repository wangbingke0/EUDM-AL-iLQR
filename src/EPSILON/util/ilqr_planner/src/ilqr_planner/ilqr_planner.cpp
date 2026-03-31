/**
 * @file ilqr_planner.cpp
 * @brief Implementation of iLQR-based trajectory planner using altro-cpp
 */

#include "ilqr_planner/ilqr_planner.h"
#include "ilqr_planner/ilqr_al_ilqr_solver.h"

#include <glog/logging.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <Eigen/Dense>

namespace planning {
namespace ilqr {

namespace {

double ClipScalar(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

}  // namespace

// ============================================================================
// IlqrFrenetTrajectory Implementation
// ============================================================================

double extend_min_velocity = 10.0;


void IlqrFrenetTrajectory::SetFromCartesianTrajectory(
    const CartesianTrajectory& traj,
    const common::StateTransformer& stf,
    double time_origin) {
  stf_ = stf;
  time_stamps_.clear();
  cartesian_states_.clear();
  frenet_states_.clear();

  for (size_t k = 0; k < traj.states.size(); ++k) {
    const auto& x = traj.states[k];
    double t = time_origin + traj.times[k];
    
    // Build Cartesian state
    common::State state;
    state.time_stamp = t;
    state.vec_position = Vec2f(x(CartesianDynamics::kPosX), x(CartesianDynamics::kPosY));
    state.angle = x(CartesianDynamics::kYaw);
    state.velocity = x(CartesianDynamics::kVel);
    state.acceleration = x(CartesianDynamics::kAcc);
    state.curvature = x(CartesianDynamics::kKappa);
    
    // Convert to Frenet state
    common::FrenetState fs;
    if (stf_.GetFrenetStateFromState(state, &fs) == kSuccess) {
      fs.time_stamp = t;
      time_stamps_.push_back(t);
      cartesian_states_.push_back(state);
      frenet_states_.push_back(fs);
    }
  }

  is_valid_ = !time_stamps_.empty();
}

ErrorType IlqrFrenetTrajectory::GetState(const decimal_t& t, 
                                          common::State* state) const {
  if (!is_valid_ || time_stamps_.empty()) {
    return kWrongStatus;
  }

  int idx = FindTimeIndex(t);
  if (idx < 0) {
    return kWrongStatus;
  }

  *state = InterpolateCartesianState(idx, t);
  return kSuccess;
}

ErrorType IlqrFrenetTrajectory::GetFrenetState(const decimal_t& t,
                                                common::FrenetState* fs) const {
  if (!is_valid_ || time_stamps_.empty()) {
    return kWrongStatus;
  }

  int idx = FindTimeIndex(t);
  if (idx < 0) {
    return kWrongStatus;
  }

  *fs = InterpolateFrenetState(idx, t);
  return kSuccess;
}

int IlqrFrenetTrajectory::FindTimeIndex(double t) const {
  if (t <= time_stamps_.front()) {
    return 0;
  }
  if (t >= time_stamps_.back()) {
    return static_cast<int>(time_stamps_.size()) - 2;
  }

  for (size_t i = 0; i < time_stamps_.size() - 1; ++i) {
    if (t >= time_stamps_[i] && t <= time_stamps_[i + 1]) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

common::State IlqrFrenetTrajectory::InterpolateCartesianState(int idx, double t) const {
  if (idx >= static_cast<int>(time_stamps_.size()) - 1) {
    return cartesian_states_.back();
  }

  double t0 = time_stamps_[idx];
  double t1 = time_stamps_[idx + 1];
  double alpha = (t - t0) / (t1 - t0);

  const auto& s0 = cartesian_states_[idx];
  const auto& s1 = cartesian_states_[idx + 1];

  common::State s;
  s.time_stamp = t;
  s.vec_position = s0.vec_position + alpha * (s1.vec_position - s0.vec_position);
  s.angle = s0.angle + alpha * normalize_angle(s1.angle - s0.angle);
  s.velocity = s0.velocity + alpha * (s1.velocity - s0.velocity);
  s.acceleration = s0.acceleration + alpha * (s1.acceleration - s0.acceleration);
  s.curvature = s0.curvature + alpha * (s1.curvature - s0.curvature);

  return s;
}

common::FrenetState IlqrFrenetTrajectory::InterpolateFrenetState(int idx, double t) const {
  if (idx >= static_cast<int>(time_stamps_.size()) - 1) {
    return frenet_states_.back();
  }

  double t0 = time_stamps_[idx];
  double t1 = time_stamps_[idx + 1];
  double alpha = (t - t0) / (t1 - t0);

  const auto& fs0 = frenet_states_[idx];
  const auto& fs1 = frenet_states_[idx + 1];

  common::FrenetState fs;
  fs.time_stamp = t;
  
  for (int i = 0; i < 3; ++i) {
    fs.vec_s[i] = fs0.vec_s[i] + alpha * (fs1.vec_s[i] - fs0.vec_s[i]);
    fs.vec_dt[i] = fs0.vec_dt[i] + alpha * (fs1.vec_dt[i] - fs0.vec_dt[i]);
  }

  return fs;
}

// ============================================================================
// IlqrPlanner Implementation
// ============================================================================

ErrorType IlqrPlanner::Init(const std::string config_path) {
  LOG(INFO) << "Initialized with altro-cpp AL-iLQR";
  
  // Initialize SSC map with configuration
  SscMap::Config map_cfg;
  map_cfg.map_size[0] = 200;
  map_cfg.map_size[1] = 20;
  map_cfg.map_size[2] = 60;
  map_cfg.map_resolution[0] = 0.5;
  map_cfg.map_resolution[1] = 0.2;
  map_cfg.map_resolution[2] = 0.1;
  map_cfg.s_back_len = 10.0;
  map_cfg.kMaxLongitudinalVel = cfg_.max_velocity;
  map_cfg.kMinLongitudinalVel = cfg_.min_velocity;
  map_cfg.kMaxLongitudinalAcc = cfg_.max_acceleration;
  map_cfg.kMaxLongitudinalDecel = -cfg_.min_acceleration;
  map_cfg.kMaxLateralVel = 5.0;
  map_cfg.kMaxLateralAcc = 3.0;
  map_cfg.kMaxNumOfGridAlongTime = 60;

  p_ssc_map_ = new SscMap(map_cfg);

  // Initialize dynamics model
  dynamics_ = std::make_shared<CartesianDynamics>();

  return kSuccess;
}

ErrorType IlqrPlanner::set_map_interface(SscPlannerMapItf* map_itf) {
  if (map_itf == nullptr) return kIllegalInput;
  map_itf_ = map_itf;
  map_valid_ = true;
  return kSuccess;
}

ErrorType IlqrPlanner::set_initial_state(const State& state) {
  initial_state_ = state;
  has_initial_state_ = true;
  return kSuccess;
}

ErrorType IlqrPlanner::RunOnce() {
  stamp_ = map_itf_->GetTimeStamp();
  LOG(WARNING) << "*** RUNONCE START: " << stamp_ << " ***";
  
  TicToc timer;
  timer.tic();

  // Get ego vehicle
  if (map_itf_->GetEgoVehicle(&ego_vehicle_) != kSuccess) {
    LOG(ERROR) << "Failed to get ego vehicle";
    return kWrongStatus;
  }

  // Set initial state
  if (!has_initial_state_) {
    initial_state_ = ego_vehicle_.state();
  }
  has_initial_state_ = false;

  // Get reference lane
  if (map_itf_->GetLocalReferenceLane(&nav_lane_local_) != kSuccess) {
    LOG(ERROR) << "Failed to get reference lane";
    return kWrongStatus;
  }
  stf_ = common::StateTransformer(nav_lane_local_);

  // Get initial Frenet state
  if (stf_.GetFrenetStateFromState(initial_state_, &initial_frenet_state_) != kSuccess) {
    LOG(ERROR) << "Failed to get initial Frenet state";
    return kWrongStatus;
  }

  // Get behavior
  if (map_itf_->GetEgoDiscretBehavior(&ego_behavior_) != kSuccess) {
    LOG(ERROR) << "Failed to get ego behavior";
    return kWrongStatus;
  }

  // Get forward trajectories for reference
  if (map_itf_->GetForwardTrajectories(&forward_behaviors_, &forward_trajs_,
                                        &surround_forward_trajs_) != kSuccess) {
    LOG(ERROR) << "Failed to get forward trajectories";
    return kWrongStatus;
  }

  // State transformation
  if (StateTransformForInputData() != kSuccess) {
    LOG(ERROR) << "Failed to transform states";
    return kWrongStatus;
  }

  // Build SSC map and corridor
  time_origin_ = initial_state_.time_stamp;
  p_ssc_map_->ResetSscMap(initial_frenet_state_);

  // Use the first behavior
  if (!forward_trajs_fs_.empty() && !surround_forward_trajs_fs_.empty()) {
    if (p_ssc_map_->ConstructSscMap(surround_forward_trajs_fs_[0], {}) != kSuccess) {
      LOG(WARNING) << "Failed to construct SSC map";
    }
    
    if (p_ssc_map_->ConstructCorridorUsingInitialTrajectory(
            p_ssc_map_->p_3d_grid(), forward_trajs_fs_[0]) != kSuccess) {
      LOG(ERROR) << "Failed to construct corridor";
      return kWrongStatus;
    }
  }

  // Get corridor cubes
  if (p_ssc_map_->GetFinalGlobalMetricCubesList() != kSuccess) {
    LOG(ERROR) << "Failed to get corridor cubes";
    return kWrongStatus;
  }

  auto corridors = p_ssc_map_->final_corridor_vec();
  if (!corridors.empty() && !corridors[0].empty()) {
    corridor_cubes_ = corridors[0];
  }

  // Convert corridor to Cartesian
  cartesian_corridor_.ConvertFromFrenetCorridor(corridor_cubes_, nav_lane_local_, 2.0);

  // Build reference trajectory from forward simulation
  reference_frenet_states_.clear();
  if (!forward_trajs_fs_.empty() && !forward_trajs_fs_[0].empty()) {
    for (const auto& fs_v : forward_trajs_fs_[0]) {
      reference_frenet_states_.push_back(fs_v.frenet_state);
    }
    LOG(INFO) << "Original reference trajectory from EUDM: " 
              << reference_frenet_states_.size() << " points";
  }

  // Convert reference to Cartesian (temporary, for sampling)
  std::vector<common::State> temp_cartesian_states;
  
  // IMPORTANT: Insert initial state as the first point to ensure coverage from t=0
  // EUDM trajectory starts from t=dt (e.g., 0.2s), not from current time
  // CRITICAL: Apply the same velocity/curvature corrections as x0 to avoid mismatch
  common::State initial_cartesian = initial_state_;
  initial_cartesian.time_stamp = time_origin_;
  
  // Apply same corrections as in RunALiLQROptimization to ensure x0 == x_ref[0]
  const double min_velocity_for_lqr = 0.5;
  if (initial_state_.velocity < min_velocity_for_lqr) {
    initial_cartesian.velocity = min_velocity_for_lqr;
    initial_cartesian.curvature = 0.0;  // Set curvature to zero when velocity is low
    LOG(INFO) << "Corrected initial state for reference: v=" 
              << std::fixed << std::setprecision(2) << initial_cartesian.velocity 
              << ", kappa=0.0 (original v=" << initial_state_.velocity << ")";
  }
  
  temp_cartesian_states.push_back(initial_cartesian);
  
  for (const auto& fs : reference_frenet_states_) {
    common::State state;
    if (stf_.GetStateFromFrenetState(fs, &state) == kSuccess) {
      state.time_stamp = fs.time_stamp;
      temp_cartesian_states.push_back(state);
    }
  }
  
  LOG(INFO) << "Reference trajectory after conversion: " 
            << temp_cartesian_states.size() << " points (with initial state)";
  
  // Debug: Check time range
  if (!temp_cartesian_states.empty()) {
    LOG(INFO) << "Reference time range: [" 
              << std::fixed << std::setprecision(2) 
              << temp_cartesian_states.front().time_stamp << ", "
              << temp_cartesian_states.back().time_stamp << "] seconds";
  }

  // Sample reference trajectory to fixed 50 points (5 seconds, 0.1s per point)
  // Using time-based interpolation instead of distance-based sampling
  if (SampleReferenceTrajectoryByTime(cfg_.num_knot_points, cfg_.dt, temp_cartesian_states) != kSuccess) {
    LOG(WARNING) << "Failed to sample reference trajectory, using original";
    // Fallback: use original trajectory
    reference_cartesian_states_ = temp_cartesian_states;
  }
  
  LOG(INFO) << "Reference trajectory after sampling: " 
            << reference_cartesian_states_.size() << " points (target: " 
            << cfg_.num_knot_points << " points)";
  
 
  if (!reference_cartesian_states_.empty()) {
    LOG(INFO) << "=== Sampled Reference Trajectory States ===";
    LOG(INFO) << "Total points: " << reference_cartesian_states_.size();
    // 打印参考线
    // for (size_t i = 0; i < reference_cartesian_states_.size(); ++i) {
    //   const auto& s = reference_cartesian_states_[i];
    //   LOG(INFO) << "Point[" << i << "]: x=" << std::fixed << std::setprecision(2) 
    //             << s.vec_position.x() << ", y=" << s.vec_position.y() << ", yaw=" << std::setprecision(3)
    //             << s.angle << ", v=" << std::setprecision(2) << s.velocity << ", a=" 
    //             << s.acceleration << ", kappa=" << std::setprecision(4) << s.curvature;
    // }

    // Check for invalid states
    int invalid_count = 0;
    for (size_t i = 0; i < reference_cartesian_states_.size(); ++i) {
      const auto& s = reference_cartesian_states_[i];
      if (!std::isfinite(s.vec_position.x()) || !std::isfinite(s.vec_position.y()) ||
          !std::isfinite(s.angle) || !std::isfinite(s.velocity) ||
          !std::isfinite(s.acceleration) || !std::isfinite(s.curvature)) {
        invalid_count++;
        if (invalid_count <= 5) {
          LOG(WARNING) << "WARNING: Invalid state at point[" << i << "]: x=" 
                       << std::fixed << std::setprecision(2) << s.vec_position.x() << ", y=" 
                       << s.vec_position.y() << ", yaw=" << std::setprecision(3) << s.angle 
                       << ", v=" << std::setprecision(2) << s.velocity << ", a=" 
                       << s.acceleration << ", kappa=" << std::setprecision(4) << s.curvature;
        }
      }
    }
    if (invalid_count > 0) {
      LOG(WARNING) << "WARNING: Found " << invalid_count 
                   << " invalid states in sampled reference trajectory!";
    }
    
    // Check for large jumps between consecutive points
    int large_jump_count = 0;
    for (size_t i = 1; i < reference_cartesian_states_.size(); ++i) {
      const auto& prev = reference_cartesian_states_[i - 1];
      const auto& curr = reference_cartesian_states_[i];
      double dx = curr.vec_position.x() - prev.vec_position.x();
      double dy = curr.vec_position.y() - prev.vec_position.y();
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist > 20.0) {  // Large jump threshold: 20 meters
        large_jump_count++;
        if (large_jump_count <= 5) {
          LOG(WARNING) << "WARNING: Large jump at point[" << i << "]: dist=" 
                       << std::fixed << std::setprecision(2) << dist << " m (from [" 
                       << prev.vec_position.x() << "," << prev.vec_position.y() << "] to [" 
                       << curr.vec_position.x() << "," << curr.vec_position.y() << "])";
        }
      }
    }
    if (large_jump_count > 0) {
      LOG(WARNING) << "WARNING: Found " << large_jump_count 
                   << " large jumps in sampled reference trajectory!";
    }
    LOG(INFO) << "=== End of Sampled Reference Trajectory Check ===";
  }

  // Run AL-iLQR optimization
  if (RunALiLQROptimization() != kSuccess) {
    LOG(ERROR) << "AL-iLQR optimization failed";
    return kWrongStatus;
  }

  time_cost_ = timer.toc();
  LOG(WARNING) << "*** RUNONCE FINISH: " << stamp_ 
               << " +" << time_cost_ << " ms ***";

  return kSuccess;
}

ErrorType IlqrPlanner::StateTransformForInputData() {
  fs_ego_vehicle_.frenet_state = initial_frenet_state_;
  fs_ego_vehicle_.vertices.clear();

  vec_E<Vec2f> ego_vertices;
  common::SemanticsUtils::GetVehicleVertices(ego_vehicle_.param(), 
                                              initial_state_, &ego_vertices);
  for (const auto& v : ego_vertices) {
    Vec2f fs_pt;
    stf_.GetFrenetPointFromPoint(v, &fs_pt);
    fs_ego_vehicle_.vertices.push_back(fs_pt);
  }

  ego_frenet_state_ = initial_frenet_state_;

  forward_trajs_fs_.clear();
  for (const auto& traj : forward_trajs_) {
    vec_E<common::FsVehicle> traj_fs;
    for (const auto& vehicle : traj) {
      common::FsVehicle fs_v;
      if (stf_.GetFrenetStateFromState(vehicle.state(), &fs_v.frenet_state) == kSuccess) {
        vec_E<Vec2f> vertices;
        common::SemanticsUtils::GetVehicleVertices(vehicle.param(), 
                                                    vehicle.state(), &vertices);
        for (const auto& v : vertices) {
          Vec2f fs_pt;
          stf_.GetFrenetPointFromPoint(v, &fs_pt);
          fs_v.vertices.push_back(fs_pt);
        }
        traj_fs.push_back(fs_v);
      }
    }
    forward_trajs_fs_.push_back(traj_fs);
  }

  surround_forward_trajs_fs_.clear();
  for (const auto& sur_traj_map : surround_forward_trajs_) {
    std::unordered_map<int, vec_E<common::FsVehicle>> sur_traj_fs_map;
    for (const auto& pair : sur_traj_map) {
      vec_E<common::FsVehicle> traj_fs;
      for (const auto& vehicle : pair.second) {
        common::FsVehicle fs_v;
        if (stf_.GetFrenetStateFromState(vehicle.state(), &fs_v.frenet_state) == kSuccess) {
          vec_E<Vec2f> vertices;
          common::SemanticsUtils::GetVehicleVertices(vehicle.param(),
                                                      vehicle.state(), &vertices);
          for (const auto& v : vertices) {
            Vec2f fs_pt;
            stf_.GetFrenetPointFromPoint(v, &fs_pt);
            fs_v.vertices.push_back(fs_pt);
          }
          traj_fs.push_back(fs_v);
        }
      }
      sur_traj_fs_map[pair.first] = traj_fs;
    }
    surround_forward_trajs_fs_.push_back(sur_traj_fs_map);
  }

  return kSuccess;
}

ErrorType IlqrPlanner::ConvertReferenceToCartesian() {
  reference_cartesian_states_.clear();
  
  size_t original_size = reference_frenet_states_.size();
  for (const auto& fs : reference_frenet_states_) {
    common::State state;
    if (stf_.GetStateFromFrenetState(fs, &state) == kSuccess) {
      state.time_stamp = fs.time_stamp;
      reference_cartesian_states_.push_back(state);
    }
  }
  
  if (reference_cartesian_states_.size() != original_size) {
    LOG(WARNING) << "Some Frenet states failed to convert: " 
                 << original_size << " -> " << reference_cartesian_states_.size();
  }
  
  return reference_cartesian_states_.empty() ? kWrongStatus : kSuccess;
}

ErrorType IlqrPlanner::SampleReferenceTrajectoryByTime(
    int num_points, double dt, const std::vector<common::State>& original_trajectory) {
  if (original_trajectory.empty()) {
    LOG(ERROR) << "Empty original trajectory for time-based sampling";
    return kWrongStatus;
  }
  
  // Get time range from original trajectory
  double t_start = time_origin_;
  double t_end = t_start + (num_points - 1) * dt;  // 5 seconds for 50 points at 0.1s interval
  
  LOG(INFO) << "Time-based sampling: " << num_points << " points, "
            << "dt=" << dt << "s, time range=[" << t_start << ", " << t_end << "]";
  
  // Check if original trajectory covers the required time range
  if (!original_trajectory.empty()) {
    double orig_t_start = original_trajectory.front().time_stamp;
    double orig_t_end = original_trajectory.back().time_stamp;
    LOG(INFO) << "Original trajectory time range: [" 
              << orig_t_start << ", " << orig_t_end << "], "
              << original_trajectory.size() << " points";
  }
  
  // Clear and resize reference trajectory
  reference_cartesian_states_.clear();
  reference_cartesian_states_.reserve(num_points);
  
  // Sample at fixed time intervals
  for (int k = 0; k < num_points; ++k) {
    double target_time = t_start + k * dt;
    
    // Find the two adjacent points in original trajectory for interpolation
    common::State sampled_state;
    
    if (InterpolateStateAtTime(original_trajectory, target_time, &sampled_state) != kSuccess) {
      LOG(WARNING) << "Failed to interpolate at time " << target_time;
      // Use the closest state as fallback
      if (target_time <= original_trajectory.front().time_stamp) {
        sampled_state = original_trajectory.front();
      } else {
        sampled_state = original_trajectory.back();
      }
      sampled_state.time_stamp = target_time;
    }
    
    reference_cartesian_states_.push_back(sampled_state);
  }
  
  LOG(INFO) << "Time-based sampling complete: " 
            << reference_cartesian_states_.size() << " points";
  
  // Debug: Print first few and last few sampled points
  if (!reference_cartesian_states_.empty()) {
    LOG(INFO) << "First 5 sampled points:";
    for (size_t i = 0; i < std::min(size_t(5), reference_cartesian_states_.size()); ++i) {
      const auto& s = reference_cartesian_states_[i];
      LOG(INFO) << "  Point[" << i << "]: t=" << std::fixed << std::setprecision(2)
                << s.time_stamp << ", x=" << s.vec_position.x() << ", y=" << s.vec_position.y()
                << ", yaw=" << std::setprecision(3) << s.angle
                << ", v=" << s.velocity << " m/s, kappa=" << std::setprecision(4) << s.curvature;
    }
    
    LOG(INFO) << "Last 5 sampled points:";
    size_t start_idx = reference_cartesian_states_.size() >= 5 ? 
                       reference_cartesian_states_.size() - 5 : 0;
    for (size_t i = start_idx; i < reference_cartesian_states_.size(); ++i) {
      const auto& s = reference_cartesian_states_[i];
      LOG(INFO) << "  Point[" << i << "]: t=" << std::fixed << std::setprecision(2)
                << s.time_stamp << ", x=" << s.vec_position.x() << ", y=" << s.vec_position.y()
                << ", yaw=" << std::setprecision(3) << s.angle
                << ", v=" << s.velocity << " m/s, kappa=" << std::setprecision(4) << s.curvature;
    }
  }
  
  return kSuccess;
}

ErrorType IlqrPlanner::InterpolateStateAtTime(
    const std::vector<common::State>& trajectory, double target_time, common::State* output) {
  if (trajectory.empty() || output == nullptr) {
    return kWrongStatus;
  }
  
  // If target time is before first point, return first state
  if (target_time <= trajectory.front().time_stamp) {
    *output = trajectory.front();
    output->time_stamp = target_time;
    return kSuccess;
  }
  
  // If target time is after last point, extrapolate from the last segment.
  // This avoids flattening the tail to repeated identical points.
  if (target_time >= trajectory.back().time_stamp) {
    const auto& s_last = trajectory.back();
    *output = s_last;
    output->time_stamp = target_time;

    if (trajectory.size() < 2) {
      return kSuccess;
    }

    const auto& s_prev = trajectory[trajectory.size() - 2];
    const double dt_seg = std::max(1e-3, s_last.time_stamp - s_prev.time_stamp);
    const double dt_extra = target_time - s_last.time_stamp;
    if (dt_extra <= 1e-6) {
      return kSuccess;
    }

    const double acc_last =
        ClipScalar(s_last.acceleration, cfg_.min_acceleration, cfg_.max_acceleration);
    const double acc_seg =
        ClipScalar((s_last.velocity - s_prev.velocity) / dt_seg, cfg_.min_acceleration,
                   cfg_.max_acceleration);
    const double acc_extrap = ClipScalar(0.7 * acc_last + 0.3 * acc_seg,
                                         cfg_.min_acceleration, cfg_.max_acceleration);

    const double kappa_last =
        ClipScalar(s_last.curvature, -cfg_.max_curvature, cfg_.max_curvature);
    const double dkappa_seg =
        ClipScalar((s_last.curvature - s_prev.curvature) / dt_seg, -cfg_.max_dkappa,
                   cfg_.max_dkappa);
    const double kappa_extrap =
        ClipScalar(kappa_last + dkappa_seg * dt_extra, -cfg_.max_curvature,
                   cfg_.max_curvature);
    const double kappa_avg = 0.5 * (kappa_last + kappa_extrap);

    const double v0 = ClipScalar(s_last.velocity, cfg_.min_velocity, cfg_.max_velocity);
    const double v1 =
        ClipScalar(v0 + acc_extrap * dt_extra, cfg_.min_velocity, cfg_.max_velocity);
    const double v_avg = 0.5 * (v0 + v1);

    const double yaw_rate = v_avg * kappa_avg;
    const double yaw_mid = s_last.angle + 0.5 * yaw_rate * dt_extra;

    output->vec_position.x() =
        s_last.vec_position.x() + v_avg * std::cos(yaw_mid) * dt_extra;
    output->vec_position.y() =
        s_last.vec_position.y() + v_avg * std::sin(yaw_mid) * dt_extra;
    output->angle = s_last.angle + yaw_rate * dt_extra;
    while (output->angle > M_PI) output->angle -= 2.0 * M_PI;
    while (output->angle < -M_PI) output->angle += 2.0 * M_PI;
    output->velocity = v1;
    output->acceleration = acc_extrap;
    output->curvature = kappa_extrap;
    return kSuccess;
  }
  
  // Find the two adjacent points for interpolation
  for (size_t i = 0; i < trajectory.size() - 1; ++i) {
    const auto& s0 = trajectory[i];
    const auto& s1 = trajectory[i + 1];
    
    if (target_time >= s0.time_stamp && target_time <= s1.time_stamp) {
      // Linear interpolation
      double dt = s1.time_stamp - s0.time_stamp;
      if (dt < 1e-6) {
        // Time difference too small, return first state
        *output = s0;
        output->time_stamp = target_time;
        return kSuccess;
      }
      
      double alpha = (target_time - s0.time_stamp) / dt;
      
      // Interpolate position
      output->vec_position.x() = s0.vec_position.x() + alpha * (s1.vec_position.x() - s0.vec_position.x());
      output->vec_position.y() = s0.vec_position.y() + alpha * (s1.vec_position.y() - s0.vec_position.y());
      
      // Interpolate angle (handle wrap-around)
      double angle_diff = s1.angle - s0.angle;
      while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
      while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
      output->angle = s0.angle + alpha * angle_diff;
      // Normalize to [-pi, pi]
      while (output->angle > M_PI) output->angle -= 2.0 * M_PI;
      while (output->angle < -M_PI) output->angle += 2.0 * M_PI;
      
      // Interpolate velocity, acceleration, curvature
      output->velocity = s0.velocity + alpha * (s1.velocity - s0.velocity);
      output->acceleration = s0.acceleration + alpha * (s1.acceleration - s0.acceleration);
      output->curvature = s0.curvature + alpha * (s1.curvature - s0.curvature);
      
      // Set time stamp
      output->time_stamp = target_time;
      
      return kSuccess;
    }
  }
  
  // Should not reach here
  return kWrongStatus;
}

ErrorType IlqrPlanner::SampleReferenceTrajectoryToFixedPoints(int num_points, double dt, double min_sampling_velocity,
                                                              const std::vector<common::State>* extended_trajectory) {
  // Use extended trajectory if provided, otherwise convert from Frenet states
  std::vector<common::State> temp_cartesian_states;
  if (extended_trajectory != nullptr && !extended_trajectory->empty()) {
    // Use the provided extended trajectory
    temp_cartesian_states = *extended_trajectory;
  } else {
    // Fallback: convert from Frenet states
    for (const auto& fs : reference_frenet_states_) {
      common::State state;
      if (stf_.GetStateFromFrenetState(fs, &state) == kSuccess) {
        state.time_stamp = fs.time_stamp;
        temp_cartesian_states.push_back(state);
      }
    }
  }
  
  if (temp_cartesian_states.empty()) {
    LOG(ERROR) << "No valid states to sample from";
    return kWrongStatus;
  }
  
  // Clear and resize reference trajectory to fixed number of points
  reference_cartesian_states_.clear();
  reference_cartesian_states_.reserve(num_points);
  
  // Start from initial state
  common::State current_state = initial_state_;
  double current_time = time_origin_;
  
  // First point: use initial state (ensure it matches exactly)
  current_state.time_stamp = current_time;
  
  // If initial velocity is too low, set a minimum velocity for LQR stability
  // This prevents issues with curvature dynamics (dyaw/dt = v * kappa) when v ≈ 0
  const double min_velocity_for_lqr = 0.5;  // Minimum velocity for LQR stability
  if (current_state.velocity < min_velocity_for_lqr) {
    LOG(WARNING) << "WARNING: Initial velocity " << std::fixed << std::setprecision(2)
                 << current_state.velocity << " < " << min_velocity_for_lqr 
                 << ", setting to " << min_velocity_for_lqr << " for LQR stability";
    current_state.velocity = min_velocity_for_lqr;
    // Also set curvature to zero when velocity is low to prevent spinning
    current_state.curvature = 0.0;
  }
  
  // Ensure the first point exactly matches initial_state_ to avoid mismatch in LQR
  reference_cartesian_states_.push_back(current_state);
  
  // Debug: Verify first point matches initial state
  LOG(INFO) << "Sampling: initial_state_ x=" << std::fixed << std::setprecision(2)
            << initial_state_.vec_position.x() << ", y=" << initial_state_.vec_position.y()
            << ", yaw=" << std::setprecision(3) << initial_state_.angle 
            << ", v=" << std::setprecision(2) << initial_state_.velocity;
  LOG(INFO) << "Sampling: first sampled point x=" << std::fixed << std::setprecision(2)
            << current_state.vec_position.x() << ", y=" << current_state.vec_position.y()
            << ", yaw=" << std::setprecision(3) << current_state.angle 
            << ", v=" << std::setprecision(2) << current_state.velocity;
  
  // Find starting point in original trajectory (closest to initial state)
  size_t current_traj_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < temp_cartesian_states.size(); ++i) {
    double dx = temp_cartesian_states[i].vec_position.x() - current_state.vec_position.x();
    double dy = temp_cartesian_states[i].vec_position.y() - current_state.vec_position.y();
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist) {
      min_dist = dist;
      current_traj_idx = i;
    }
  }
  
  // Use fixed sampling velocity to ensure uniform spacing
  // Determine initial sampling velocity: use max(initial_velocity, min_sampling_velocity)
  double fixed_sampling_velocity = std::max(initial_state_.velocity, min_sampling_velocity);
  
  // Sample remaining points
  for (int k = 1; k < num_points; ++k) {
    current_time += dt;
    
    // Use fixed sampling velocity to ensure uniform point spacing
    // This prevents trajectory segments from being long-short due to velocity changes
    double sampling_velocity = fixed_sampling_velocity;
    
    // Calculate distance to travel in this step
    double ds = sampling_velocity * dt;
    
    // Find the state in original trajectory that is 'ds' distance away
    // Start searching from current_traj_idx
    double segment_accumulated = 0.0;
    bool found = false;
    
    // Find the segment in original trajectory
    for (size_t i = current_traj_idx; i < temp_cartesian_states.size() - 1; ++i) {
      double dx = temp_cartesian_states[i + 1].vec_position.x() - temp_cartesian_states[i].vec_position.x();
      double dy = temp_cartesian_states[i + 1].vec_position.y() - temp_cartesian_states[i].vec_position.y();
      double segment_dist = std::sqrt(dx * dx + dy * dy);
      
      if (segment_accumulated + segment_dist >= ds) {
        // Interpolate between temp_cartesian_states[i] and temp_cartesian_states[i+1]
        double alpha = (ds - segment_accumulated) / segment_dist;
        
        // Interpolate position
        current_state.vec_position.x() = temp_cartesian_states[i].vec_position.x() + 
                                         alpha * (temp_cartesian_states[i + 1].vec_position.x() - 
                                                  temp_cartesian_states[i].vec_position.x());
        current_state.vec_position.y() = temp_cartesian_states[i].vec_position.y() + 
                                         alpha * (temp_cartesian_states[i + 1].vec_position.y() - 
                                                  temp_cartesian_states[i].vec_position.y());
        
        // Interpolate angle (handle wrap-around)
        double angle0 = temp_cartesian_states[i].angle;
        double angle1 = temp_cartesian_states[i + 1].angle;
        double angle_diff = angle1 - angle0;
        // Normalize angle difference to [-pi, pi]
        while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
        current_state.angle = angle0 + alpha * angle_diff;
        // Normalize angle to [-pi, pi]
        while (current_state.angle > M_PI) current_state.angle -= 2.0 * M_PI;
        while (current_state.angle < -M_PI) current_state.angle += 2.0 * M_PI;
        
        // Interpolate velocity, acceleration, curvature
        current_state.velocity = temp_cartesian_states[i].velocity + 
                                 alpha * (temp_cartesian_states[i + 1].velocity - temp_cartesian_states[i].velocity);
        current_state.acceleration = temp_cartesian_states[i].acceleration + 
                                    alpha * (temp_cartesian_states[i + 1].acceleration - temp_cartesian_states[i].acceleration);
        current_state.curvature = temp_cartesian_states[i].curvature + 
                                  alpha * (temp_cartesian_states[i + 1].curvature - temp_cartesian_states[i].curvature);
        
        // Ensure velocity is within bounds
        current_state.velocity = std::max(cfg_.min_velocity, 
                                          std::min(cfg_.max_velocity, current_state.velocity));
        
        // Update current trajectory index for next iteration
        current_traj_idx = i;
        found = true;
        break;
      }
      
      segment_accumulated += segment_dist;
    }
    
    // If we reached the end of original trajectory, extend it
    if (!found) {
      const auto& last_state = temp_cartesian_states.back();
      double remaining_ds = ds - segment_accumulated;
      
      // Extend along the direction of the last state
      double yaw = last_state.angle;
      current_state.vec_position.x() = last_state.vec_position.x() + remaining_ds * std::cos(yaw);
      current_state.vec_position.y() = last_state.vec_position.y() + remaining_ds * std::sin(yaw);
      current_state.angle = yaw;
      current_state.velocity = std::max(last_state.velocity, min_sampling_velocity);
      current_state.acceleration = last_state.acceleration;
      current_state.curvature = last_state.curvature;
      
      // Update index to last point
      current_traj_idx = temp_cartesian_states.size() - 1;
    }
    
    current_state.time_stamp = current_time;
    reference_cartesian_states_.push_back(current_state);
  }
  
  LOG(INFO) << "Sampled reference trajectory: " 
            << reference_cartesian_states_.size() << " points, "
            << "time horizon: " << (num_points - 1) * dt << " seconds";
  
  return kSuccess;
}

void IlqrPlanner::ExtendTrajectoryToMinimumLength(std::vector<common::State>* trajectory, double min_length_meters) {
  if (trajectory == nullptr || trajectory->empty()) {
    return;
  }
  
  // Calculate current trajectory length
  double total_length = 0.0;
  for (size_t i = 1; i < trajectory->size(); ++i) {
    const auto& prev = (*trajectory)[i - 1];
    const auto& curr = (*trajectory)[i];
    double dx = curr.vec_position.x() - prev.vec_position.x();
    double dy = curr.vec_position.y() - prev.vec_position.y();
    total_length += std::sqrt(dx * dx + dy * dy);
  }
  
  // If length is sufficient, return
  if (total_length >= min_length_meters) {
    LOG(INFO) << "Trajectory length: " << total_length 
              << " m (>= " << min_length_meters << " m), no extension needed";
    return;
  }
  
  LOG(INFO) << "Trajectory length: " << total_length 
            << " m, extending to " << min_length_meters << " m";
  
  // Get the last state
  const auto& last_state = trajectory->back();
  double remaining_length = min_length_meters - total_length;
  
  // Calculate extension parameters
  double dt = cfg_.dt;  // Use the same time step
  // Use the same minimum velocity as sampling (8.0 m/s) to ensure consistency
  // This prevents extended segments from having different spacing than sampled segments
  double v = std::max(std::max(last_state.velocity, extend_min_velocity), cfg_.min_velocity);
  double yaw = last_state.angle;
  double a = last_state.acceleration;
  double kappa = last_state.curvature;
  
  // Generate extension states
  double accumulated_length = 0.0;
  double t = last_state.time_stamp;
  common::State current_state = last_state;
  
  while (accumulated_length < remaining_length) {
    t += dt;
    
    // Update position based on velocity and yaw
    double ds = v * dt;  // Distance traveled in this step
    current_state.vec_position.x() += ds * std::cos(yaw);
    current_state.vec_position.y() += ds * std::sin(yaw);
    
    // Update yaw based on curvature and velocity: dyaw/dt = v * kappa
    // Handle low velocity case to prevent spinning
    if (v < 0.5) {
      kappa = 0.0;  // Set curvature to zero if velocity is too low
    }
    yaw += v * kappa * dt;
    // Normalize angle to [-pi, pi]
    while (yaw > M_PI) yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;
    current_state.angle = yaw;
    
    // Update velocity (keep acceleration constant for simplicity)
    v = std::max(cfg_.min_velocity, 
                 std::min(cfg_.max_velocity, v + a * dt));
    current_state.velocity = v;
    
    // Keep acceleration and curvature constant
    current_state.acceleration = a;
    current_state.curvature = kappa;
    current_state.time_stamp = t;
    
    trajectory->push_back(current_state);
    accumulated_length += ds;
    
    // Safety check: prevent infinite loop
    if (trajectory->size() > 1000) {
      LOG(WARNING) << "Trajectory extension reached max size";
      break;
    }
  }
  
  LOG(INFO) << "Extended trajectory by " 
            << accumulated_length << " m, new length: " 
            << (total_length + accumulated_length) << " m, new size: " 
            << trajectory->size() << " points";
  
  // Debug: Check extended trajectory states
  if (!trajectory->empty() && trajectory->size() > 1) {
    LOG(INFO) << "=== Extended Trajectory States Check ===";
    LOG(INFO) << "Extended trajectory size: " << trajectory->size();
    const auto& first = trajectory->front();
    const auto& last = trajectory->back();
    LOG(INFO) << "First point: x=" << std::fixed << std::setprecision(2)
              << first.vec_position.x() << ", y=" << first.vec_position.y() 
              << ", yaw=" << std::setprecision(3) << first.angle << ", v=" 
              << std::setprecision(2) << first.velocity << ", a=" << first.acceleration 
              << ", kappa=" << std::setprecision(4) << first.curvature;
    LOG(INFO) << "Last point: x=" << std::fixed << std::setprecision(2)
              << last.vec_position.x() << ", y=" << last.vec_position.y() 
              << ", yaw=" << std::setprecision(3) << last.angle << ", v=" 
              << std::setprecision(2) << last.velocity << ", a=" << last.acceleration 
              << ", kappa=" << std::setprecision(4) << last.curvature;
    
    // Check for invalid states
    int invalid_count = 0;
    for (size_t i = 0; i < trajectory->size(); ++i) {
      const auto& s = (*trajectory)[i];
      if (!std::isfinite(s.vec_position.x()) || !std::isfinite(s.vec_position.y()) ||
          !std::isfinite(s.angle) || !std::isfinite(s.velocity) ||
          !std::isfinite(s.acceleration) || !std::isfinite(s.curvature)) {
        invalid_count++;
        if (invalid_count <= 3) {
          LOG(WARNING) << "WARNING: Invalid extended state at point[" << i << "]";
        }
      }
    }
    if (invalid_count > 0) {
      LOG(WARNING) << "WARNING: Found " << invalid_count 
                   << " invalid states in extended trajectory!";
    }
    LOG(INFO) << "=== End of Extended Trajectory Check ===";
  }
}



ErrorType IlqrPlanner::RunALiLQROptimization() {
  IlqrAlIlqrSolver solver(this);
  return solver.Solve();
}

}  // namespace ilqr
}  // namespace planning
