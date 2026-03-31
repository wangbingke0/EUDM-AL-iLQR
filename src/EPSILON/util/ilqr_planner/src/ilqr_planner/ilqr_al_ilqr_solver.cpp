/**
 * @file ilqr_al_ilqr_solver.cpp
 * @brief AL-iLQR solver wrapper that consumes LQR warm-start
 */

#include "ilqr_planner/ilqr_al_ilqr_solver.h"

#include "ilqr_planner/cartesian_dynamics.h"
#include "ilqr_planner/ilqr_planner.h"

#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace planning {
namespace ilqr {

namespace {

double ClipScalar(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double AlignAngleNear(double angle, double reference) {
  while (angle - reference > M_PI) angle -= 2.0 * M_PI;
  while (angle - reference < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

class StateBoundConstraint
    : public altro::constraints::Constraint<altro::constraints::NegativeOrthant> {
 public:
  StateBoundConstraint(const std::vector<double>& lb,
                       const std::vector<double>& ub)
      : n_(static_cast<int>(lb.size())), lower_bound_(lb), upper_bound_(ub) {
    if (lb.size() != ub.size() || lb.empty()) {
      n_ = 0;
      return;
    }
    CollectFiniteIndices(lower_bound_, &index_lower_bound_);
    CollectFiniteIndices(upper_bound_, &index_upper_bound_);
  }

  std::string GetLabel() const override { return "State Bound"; }

  int StateDimension() const override { return n_; }

  int OutputDimension() const override {
    return static_cast<int>(index_lower_bound_.size() + index_upper_bound_.size());
  }

  void Evaluate(const altro::VectorXdRef& x, const altro::VectorXdRef& u,
                Eigen::Ref<altro::VectorXd> c) override {
    ALTRO_UNUSED(u);
    const int nx = std::min<int>(n_, x.size());
    int row = 0;
    for (size_t i = 0; i < index_lower_bound_.size(); ++i) {
      const int j = index_lower_bound_[i];
      if (j < nx) {
        c(row++) = lower_bound_[j] - x(j);
      }
    }
    for (size_t i = 0; i < index_upper_bound_.size(); ++i) {
      const int j = index_upper_bound_[i];
      if (j < nx) {
        c(row++) = x(j) - upper_bound_[j];
      }
    }
  }

  void Jacobian(const altro::VectorXdRef& x, const altro::VectorXdRef& u,
                Eigen::Ref<altro::MatrixXd> jac) override {
    ALTRO_UNUSED(u);
    jac.setZero();
    const int nx = std::min<int>(n_, x.size());
    int row = 0;
    for (size_t i = 0; i < index_lower_bound_.size(); ++i) {
      const int j = index_lower_bound_[i];
      if (j < nx) {
        jac(row++, j) = -1.0;
      }
    }
    for (size_t i = 0; i < index_upper_bound_.size(); ++i) {
      const int j = index_upper_bound_[i];
      if (j < nx) {
        jac(row++, j) = 1.0;
      }
    }
  }

 private:
  static void CollectFiniteIndices(const std::vector<double>& bound,
                                   std::vector<int>* index) {
    index->clear();
    for (size_t i = 0; i < bound.size(); ++i) {
      if (std::isfinite(bound[i])) {
        index->push_back(static_cast<int>(i));
      }
    }
  }

  int n_ = 0;
  std::vector<double> lower_bound_;
  std::vector<double> upper_bound_;
  std::vector<int> index_lower_bound_;
  std::vector<int> index_upper_bound_;
};

}  // namespace

IlqrAlIlqrSolver::IlqrAlIlqrSolver(IlqrPlanner* planner)
    : planner_(planner), warm_start_solver_(planner) {}

ErrorType IlqrAlIlqrSolver::Solve() {
  if (planner_ == nullptr) {
    return kIllegalInput;
  }

  const auto& cfg = planner_->cfg_;
  const int N = cfg.num_knot_points;
  const double dt = cfg.dt;

  LOG(INFO) << "Running AL-iLQR with N=" << N << ", dt=" << std::fixed
            << std::setprecision(3) << dt;

  // Create problem.
  altro::problem::Problem prob(N);

  // Set initial state.
  altro::VectorXd x0(kStateDim);

  const double min_velocity_for_lqr = std::max(cfg.min_velocity, 0.5);
  double initial_velocity = std::max(planner_->initial_state_.velocity,
                                     min_velocity_for_lqr);
  double initial_curvature = planner_->initial_state_.curvature;
  if (planner_->initial_state_.velocity < min_velocity_for_lqr) {
    initial_curvature = 0.0;
    LOG(WARNING) << "WARNING: Initial velocity " << std::fixed
                 << std::setprecision(2) << planner_->initial_state_.velocity
                 << " < " << min_velocity_for_lqr << ", using v="
                 << initial_velocity << ", kappa=0.0 for LQR";
  }

  x0 << planner_->initial_state_.vec_position.x(),
      planner_->initial_state_.vec_position.y(), planner_->initial_state_.angle,
      initial_velocity, planner_->initial_state_.acceleration,
      initial_curvature;
  prob.SetInitialState(x0);

  // Create discretized dynamics for each knot point.
  planner_->discrete_dynamics_.clear();
  for (int k = 0; k < N; ++k) {
    CartesianDynamics cont_dyn;
    auto disc_dyn =
        std::make_shared<altro::problem::DiscretizedModel<CartesianDynamics>>(
            cont_dyn);
    planner_->discrete_dynamics_.push_back(disc_dyn);
    prob.SetDynamics(disc_dyn, k);
  }

  // Create cost functions.
  altro::MatrixXd Q = altro::MatrixXd::Zero(kStateDim, kStateDim);
  Q(0, 0) = std::max(cfg.weight_x, 6.0);
  Q(1, 1) = std::max(cfg.weight_y, 10.0);
  Q(2, 2) = std::max(cfg.weight_yaw, 1.0);
  Q(3, 3) = cfg.weight_v;
  Q(4, 4) = cfg.weight_a;
  Q(5, 5) = std::max(cfg.weight_kappa, 2.0);

  altro::MatrixXd R = altro::MatrixXd::Zero(kControlDim, kControlDim);
  R(0, 0) = std::max(cfg.weight_jerk, 0.02);
  R(1, 1) = std::max(cfg.weight_dkappa, 1.2);

  altro::MatrixXd Qf = altro::MatrixXd::Zero(kStateDim, kStateDim);
  Qf(0, 0) = cfg.weight_terminal_x;
  Qf(1, 1) = cfg.weight_terminal_y;
  Qf(2, 2) = std::max(cfg.weight_terminal_yaw, 20.0);
  Qf(3, 3) = cfg.weight_terminal_v;
  Qf(4, 4) = std::max(cfg.weight_terminal_a, 2.0);
  Qf(5, 5) = std::max(cfg.weight_terminal_kappa, 8.0);

  auto get_reference = [&](int k, double yaw_anchor) -> altro::VectorXd {
    altro::VectorXd xref(kStateDim);
    if (!planner_->reference_cartesian_states_.empty()) {
      const int idx =
          std::min<int>(k, planner_->reference_cartesian_states_.size() - 1);
      const auto& ref = planner_->reference_cartesian_states_[idx];
      const double v_ref =
          ClipScalar(ref.velocity, min_velocity_for_lqr, cfg.max_velocity);
      const double a_ref =
          ClipScalar(ref.acceleration, cfg.min_acceleration, cfg.max_acceleration);
      double kappa_ref =
          ClipScalar(ref.curvature, -cfg.max_curvature, cfg.max_curvature);
      if (v_ref < 0.5) {
        kappa_ref = 0.0;
      }
      const double yaw_ref = AlignAngleNear(ref.angle, yaw_anchor);
      xref << ref.vec_position.x(), ref.vec_position.y(), yaw_ref, v_ref, a_ref,
          kappa_ref;
      return xref;
    }
    xref = x0;
    xref(2) = AlignAngleNear(xref(2), yaw_anchor);
    return xref;
  };

  std::vector<altro::VectorXd> xref_traj(
      N + 1, altro::VectorXd::Zero(kStateDim));
  xref_traj[0] = x0;
  xref_traj[0](3) =
      ClipScalar(xref_traj[0](3), min_velocity_for_lqr, cfg.max_velocity);
  xref_traj[0](4) =
      ClipScalar(xref_traj[0](4), cfg.min_acceleration, cfg.max_acceleration);
  xref_traj[0](5) =
      ClipScalar(xref_traj[0](5), -cfg.max_curvature, cfg.max_curvature);
  if (xref_traj[0](3) < 0.5) {
    xref_traj[0](5) = 0.0;
  }
  double yaw_anchor = xref_traj[0](2);
  for (int k = 1; k <= N; ++k) {
    xref_traj[k] = get_reference(k, yaw_anchor);
    yaw_anchor = xref_traj[k](2);
  }

  // Set stage costs with reference tracking.
  for (int k = 0; k < N; ++k) {
    altro::MatrixXd Qk = Q;
    altro::MatrixXd Rk = R;
    const double progress =
        (N > 0) ? static_cast<double>(k) / static_cast<double>(N) : 0.0;
    if (progress > 0.70) {
      const double alpha = ClipScalar((progress - 0.70) / 0.30, 0.0, 1.0);
      Qk(0, 0) *= (1.0 + 1.2 * alpha);
      Qk(1, 1) *= (1.0 + 1.2 * alpha);
      Qk(5, 5) *= (1.0 + 2.5 * alpha);
      Rk(0, 0) *= (1.0 + 0.4 * alpha);
      Rk(1, 1) *= (1.0 + 2.5 * alpha);
    }

    altro::VectorXd uref(kControlDim);
    const double jerk_ref = (xref_traj[k + 1](4) - xref_traj[k](4)) / dt;
    const double dkappa_ref = (xref_traj[k + 1](5) - xref_traj[k](5)) / dt;
    uref(0) = ClipScalar(jerk_ref, -cfg.max_jerk, cfg.max_jerk);
    uref(1) = ClipScalar(dkappa_ref, -cfg.max_dkappa, cfg.max_dkappa);

    auto cost = std::make_shared<altro::examples::QuadraticCost>(
        altro::examples::QuadraticCost::LQRCost(Qk, Rk, xref_traj[k], uref,
                                                false));
    prob.SetCostFunction(cost, k);
  }

  // Set terminal cost.
  altro::VectorXd terminal_uref = altro::VectorXd::Zero(kControlDim);
  auto terminal_cost = std::make_shared<altro::examples::QuadraticCost>(
      altro::examples::QuadraticCost::LQRCost(Qf, R, xref_traj[N],
                                              terminal_uref, true));
  prob.SetCostFunction(terminal_cost, N);

  // Add control bounds.
  std::vector<double> u_lb = {-cfg.max_jerk, -cfg.max_dkappa};
  std::vector<double> u_ub = {cfg.max_jerk, cfg.max_dkappa};
  for (int k = 0; k < N; ++k) {
    auto control_bound = std::make_shared<altro::examples::ControlBound>(u_lb, u_ub);
    prob.SetConstraint(control_bound, k);
  }

  // Add state bounds.
  std::vector<double> x_lb(kStateDim, -std::numeric_limits<double>::infinity());
  std::vector<double> x_ub(kStateDim, std::numeric_limits<double>::infinity());
  x_lb[CartesianDynamics::kVel] = cfg.min_velocity;
  x_ub[CartesianDynamics::kVel] = cfg.max_velocity;
  x_lb[CartesianDynamics::kAcc] = cfg.min_acceleration;
  x_ub[CartesianDynamics::kAcc] = cfg.max_acceleration;
  x_lb[CartesianDynamics::kKappa] = -cfg.max_curvature;
  x_ub[CartesianDynamics::kKappa] = cfg.max_curvature;
  for (int k = 1; k <= N; ++k) {
    auto state_bound = std::make_shared<StateBoundConstraint>(x_lb, x_ub);
    prob.SetConstraint(state_bound, k);
  }

  // Add corridor constraints.
  planner_->corridor_factory_.Initialize(planner_->corridor_cubes_,
                                         planner_->nav_lane_local_,
                                         planner_->time_origin_, dt, N);
  for (int k = 0; k < N; ++k) {
    auto corridor_con = planner_->corridor_factory_.GetConstraint(k);
    if (corridor_con) {
      prob.SetConstraint(corridor_con, k);
    }
  }

  // Create initial trajectory.
  auto traj = std::make_shared<altro::Trajectory<kStateDim, kControlDim>>(N);

  // Compute LQR warm start.
  LOG(INFO) << "Attempting LQR warm start...";
  if (warm_start_solver_.Compute(x0, Q, R, Qf, dt, N, traj.get()) != kSuccess) {
    LOG(WARNING) << "LQR warm start failed, using zero controls";
    for (int k = 0; k <= N; ++k) {
      double t = k * dt;
      altro::VectorXd x(kStateDim);
      if (k < static_cast<int>(planner_->reference_cartesian_states_.size())) {
        const auto& ref = planner_->reference_cartesian_states_[k];
        x << ref.vec_position.x(), ref.vec_position.y(), ref.angle,
            std::max(ref.velocity, cfg.min_velocity), ref.acceleration,
            ref.curvature;
      } else {
        x = x0;
        x(0) += planner_->initial_state_.velocity * std::cos(planner_->initial_state_.angle) * t;
        x(1) += planner_->initial_state_.velocity * std::sin(planner_->initial_state_.angle) * t;
      }
      traj->State(k) = x;
      if (k < N) {
        traj->Control(k).setZero();
        traj->SetTime(k, static_cast<float>(k * dt));
        traj->SetStep(k, static_cast<float>(dt));
      }
    }
  }

  // Create and configure solver.
  altro::augmented_lagrangian::AugmentedLagrangianiLQR<kStateDim, kControlDim>
      solver(prob);
  solver.SetTrajectory(traj);

  auto& opts = solver.GetOptions();
  opts.max_iterations_outer = cfg.max_outer_iterations;
  opts.max_iterations_inner = cfg.max_inner_iterations;
  opts.constraint_tolerance = cfg.constraint_tolerance;
  opts.cost_tolerance = cfg.cost_tolerance;
  opts.initial_penalty = cfg.initial_penalty;
  opts.verbose = altro::LogLevel::kSilent;

  // Solve.
  TicToc solve_timer;
  solve_timer.tic();
  solver.Solve();
  double solve_time = solve_timer.toc();
  LOG(INFO) << "AL-iLQR solve time: " << solve_time << " ms";

  auto status = solver.GetStatus();
  LOG(INFO) << "Solver status: " << static_cast<int>(status);

  auto apply_warm_start_fallback = [&](const std::string& reason) -> bool {
    if (planner_->lqr_warm_start_traj_.states.size() != static_cast<size_t>(N + 1) ||
        planner_->lqr_warm_start_traj_.controls.size() != static_cast<size_t>(N)) {
      LOG(WARNING) << "Cannot fallback to warm start (" << reason
                   << "): warm start trajectory unavailable.";
      return false;
    }
    planner_->cartesian_traj_ = planner_->lqr_warm_start_traj_;
    planner_->optimized_trajectory_.SetFromCartesianTrajectory(
        planner_->cartesian_traj_, planner_->stf_, planner_->time_origin_);
    LOG(WARNING) << "Using warm start trajectory as final output due to: " << reason;
    return planner_->optimized_trajectory_.IsValid();
  };

  if (status != altro::SolverStatus::kSolved) {
    LOG(WARNING) << "Solver did not converge! Status: " << static_cast<int>(status);
    if (apply_warm_start_fallback("solver did not converge")) {
      return kSuccess;
    }
  } else {
    LOG(INFO) << "Solver converged successfully";
  }

  auto stats = solver.GetStats();
  LOG(INFO) << "Solver iterations: outer=" << stats.iterations_outer
            << ", inner=" << stats.iterations_inner;
  if (!stats.cost.empty()) {
    LOG(INFO) << "Final cost: " << std::fixed << std::setprecision(6)
              << stats.cost.back();
  }

  // Extract optimized trajectory.
  planner_->cartesian_traj_.clear();
  planner_->cartesian_traj_.states.resize(N + 1);
  planner_->cartesian_traj_.controls.resize(N);
  planner_->cartesian_traj_.times.resize(N + 1);
  planner_->cartesian_traj_.dt = dt;

  bool trajectory_valid = true;
  double max_segment_dist = 0.0;
  double max_ref_deviation = 0.0;
  double max_ref_violation = 0.0;
  int worst_ref_index = -1;
  double worst_ref_observed = 0.0;
  double worst_ref_allowed = 0.0;
  const double ref_dev_base_threshold = 5.5;
  const double ref_dev_tail_threshold = 8.0;
  const double ref_dev_tail_start_ratio = 0.70;

  for (int k = 0; k <= N; ++k) {
    planner_->cartesian_traj_.states[k] = traj->State(k);
    planner_->cartesian_traj_.times[k] = k * dt;

    const auto& x = planner_->cartesian_traj_.states[k];
    for (int i = 0; i < kStateDim; ++i) {
      if (!std::isfinite(x(i))) {
        LOG(ERROR) << "Invalid state at k=" << k << ", dim=" << i << ", value=" << x(i);
        trajectory_valid = false;
      }
    }

    if (x(CartesianDynamics::kVel) < cfg.min_velocity - 1e-3 ||
        x(CartesianDynamics::kVel) > cfg.max_velocity + 1e-3 ||
        x(CartesianDynamics::kAcc) < cfg.min_acceleration - 1e-3 ||
        x(CartesianDynamics::kAcc) > cfg.max_acceleration + 1e-3 ||
        std::fabs(x(CartesianDynamics::kKappa)) > cfg.max_curvature + 1e-3) {
      trajectory_valid = false;
    }

    if (k > 0) {
      double dx = x(CartesianDynamics::kPosX) -
                  planner_->cartesian_traj_.states[k - 1](CartesianDynamics::kPosX);
      double dy = x(CartesianDynamics::kPosY) -
                  planner_->cartesian_traj_.states[k - 1](CartesianDynamics::kPosY);
      double dist = std::sqrt(dx * dx + dy * dy);
      max_segment_dist = std::max(max_segment_dist, dist);
      const double v_prev =
          std::max(planner_->cartesian_traj_.states[k - 1](CartesianDynamics::kVel),
                   min_velocity_for_lqr);
      const double max_reasonable_step = std::max(3.0, v_prev * dt * 3.0 + 0.8);
      if (!std::isfinite(dist) || dist > max_reasonable_step) {
        trajectory_valid = false;
      }
    }

    if (k < N) {
      planner_->cartesian_traj_.controls[k] = traj->Control(k);
    }

    if (k < static_cast<int>(planner_->reference_cartesian_states_.size())) {
      const auto& ref = planner_->reference_cartesian_states_[k];
      const double dx_ref = x(CartesianDynamics::kPosX) - ref.vec_position.x();
      const double dy_ref = x(CartesianDynamics::kPosY) - ref.vec_position.y();
      const double ref_dist = std::sqrt(dx_ref * dx_ref + dy_ref * dy_ref);
      if (std::isfinite(ref_dist)) {
        max_ref_deviation = std::max(max_ref_deviation, ref_dist);
        const double progress =
            (N > 0) ? static_cast<double>(k) / static_cast<double>(N) : 0.0;
        double allowed_ref_dist = ref_dev_base_threshold;
        if (progress > ref_dev_tail_start_ratio) {
          const double alpha =
              ClipScalar((progress - ref_dev_tail_start_ratio) /
                             std::max(1e-3, 1.0 - ref_dev_tail_start_ratio),
                         0.0, 1.0);
          allowed_ref_dist = ref_dev_base_threshold +
                             alpha * (ref_dev_tail_threshold - ref_dev_base_threshold);
        }
        const double ref_violation = ref_dist - allowed_ref_dist;
        if (ref_violation > max_ref_violation) {
          max_ref_violation = ref_violation;
          worst_ref_index = k;
          worst_ref_observed = ref_dist;
          worst_ref_allowed = allowed_ref_dist;
        }
      }
    }
  }

  if (!trajectory_valid || max_ref_violation > 0.0) {
    LOG(ERROR) << "Optimized trajectory quality check failed. valid="
               << (trajectory_valid ? "true" : "false")
               << ", max_segment_dist=" << std::fixed << std::setprecision(3)
               << max_segment_dist << ", max_ref_deviation=" << max_ref_deviation
               << ", max_ref_violation=" << max_ref_violation
               << ", worst_ref_k=" << worst_ref_index
               << ", worst_ref_observed=" << worst_ref_observed
               << ", worst_ref_allowed=" << worst_ref_allowed;
    if (apply_warm_start_fallback("optimized trajectory quality check failed")) {
      return kSuccess;
    }
    return kWrongStatus;
  }

  planner_->optimized_trajectory_.SetFromCartesianTrajectory(
      planner_->cartesian_traj_, planner_->stf_, planner_->time_origin_);

  LOG(INFO) << "Optimization complete. Final cost: " << solver.GetStats().cost.back();
  LOG(INFO) << "Optimization complete. Trajectory is "
            << (planner_->optimized_trajectory_.IsValid() ? "valid" : "invalid");

  return kSuccess;
}

}  // namespace ilqr
}  // namespace planning
