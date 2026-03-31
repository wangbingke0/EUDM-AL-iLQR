/**
 * @file ilqr_lqr_warm_start.cpp
 * @brief LQR warm-start generator for iLQR trajectory optimization
 */

#include "ilqr_planner/ilqr_lqr_warm_start.h"

#include "ilqr_planner/cartesian_dynamics.h"
#include "ilqr_planner/ilqr_planner.h"

#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <iomanip>
#include <vector>

#include <Eigen/Dense>

namespace planning {
namespace ilqr {

IlqrLqrWarmStart::IlqrLqrWarmStart(IlqrPlanner* planner) : planner_(planner) {}

ErrorType IlqrLqrWarmStart::Compute(
    const altro::VectorXd& x0,
    const altro::MatrixXd& Q,
    const altro::MatrixXd& R,
    const altro::MatrixXd& Qf,
    double dt,
    int N,
    altro::Trajectory<kStateDim, kControlDim>* traj) {
  if (planner_ == nullptr || traj == nullptr || N <= 0 || dt <= 1e-6) {
    return kIllegalInput;
  }

  LOG(INFO) << "Running LQR warm start with N=" << N << ", dt=" << dt;

  const auto& cfg = planner_->cfg_;
  const double min_velocity_for_lqr = std::max(cfg.min_velocity, 0.5);
  const double feedback_gain_scale = 0.08;
  const double feedback_gain_floor = 0.02;
  const double feedback_decay_time = 1.5;
  const double max_feedback_norm = 1.5;

  auto clip_scalar = [](double value, double low, double high) {
    return std::max(low, std::min(high, value));
  };

  auto build_reference_state = [&](int k) -> altro::VectorXd {
    altro::VectorXd xr(kStateDim);
    if (k < static_cast<int>(planner_->reference_cartesian_states_.size())) {
      const auto& ref = planner_->reference_cartesian_states_[k];
      double v_ref = clip_scalar(ref.velocity, min_velocity_for_lqr, cfg.max_velocity);
      double a_ref =
          clip_scalar(ref.acceleration, cfg.min_acceleration, cfg.max_acceleration);
      double kappa_ref =
          clip_scalar(ref.curvature, -cfg.max_curvature, cfg.max_curvature);
      if (v_ref < 0.5) {
        kappa_ref = 0.0;
      }
      xr << ref.vec_position.x(), ref.vec_position.y(), ref.angle, v_ref, a_ref,
          kappa_ref;
      return xr;
    }

    if (!planner_->reference_cartesian_states_.empty()) {
      const auto& ref = planner_->reference_cartesian_states_.back();
      double v_ref = clip_scalar(ref.velocity, min_velocity_for_lqr, cfg.max_velocity);
      double a_ref =
          clip_scalar(ref.acceleration, cfg.min_acceleration, cfg.max_acceleration);
      double kappa_ref =
          clip_scalar(ref.curvature, -cfg.max_curvature, cfg.max_curvature);
      if (v_ref < 0.5) {
        kappa_ref = 0.0;
      }
      xr << ref.vec_position.x(), ref.vec_position.y(), ref.angle, v_ref, a_ref,
          kappa_ref;
      return xr;
    }

    xr = x0;
    xr(0) = x0(0) + x0(3) * std::cos(x0(2)) * k * dt;
    xr(1) = x0(1) + x0(3) * std::sin(x0(2)) * k * dt;
    xr(3) = clip_scalar(xr(3), min_velocity_for_lqr, cfg.max_velocity);
    xr(4) = clip_scalar(xr(4), cfg.min_acceleration, cfg.max_acceleration);
    xr(5) = clip_scalar(xr(5), -cfg.max_curvature, cfg.max_curvature);
    if (xr(3) < 0.5) {
      xr(5) = 0.0;
    }
    return xr;
  };

  // Build reference states and feedforward controls from reference derivatives.
  std::vector<altro::VectorXd> x_ref(N + 1, altro::VectorXd::Zero(kStateDim));
  std::vector<altro::VectorXd> u_ref(N, altro::VectorXd::Zero(kControlDim));
  for (int k = 0; k <= N; ++k) {
    x_ref[k] = build_reference_state(k);
  }
  x_ref[0] = x0;
  x_ref[0](3) = clip_scalar(x_ref[0](3), min_velocity_for_lqr, cfg.max_velocity);
  x_ref[0](4) = clip_scalar(x_ref[0](4), cfg.min_acceleration, cfg.max_acceleration);
  x_ref[0](5) = clip_scalar(x_ref[0](5), -cfg.max_curvature, cfg.max_curvature);
  if (x_ref[0](3) < 0.5) {
    x_ref[0](5) = 0.0;
  }

  for (int k = 0; k < N; ++k) {
    const double jerk_ff = (x_ref[k + 1](4) - x_ref[k](4)) / dt;
    const double dkappa_ff = (x_ref[k + 1](5) - x_ref[k](5)) / dt;
    u_ref[k](0) = clip_scalar(jerk_ff, -cfg.max_jerk, cfg.max_jerk);
    u_ref[k](1) = clip_scalar(dkappa_ff, -cfg.max_dkappa, cfg.max_dkappa);
  }

  // Linearize dynamics around reference trajectory.
  std::vector<altro::MatrixXd> A(N);
  std::vector<altro::MatrixXd> B(N);
  std::vector<altro::MatrixXd> K(
      N, altro::MatrixXd::Zero(kControlDim, kStateDim));
  CartesianDynamics dynamics;

  for (int k = 0; k < N; ++k) {
    altro::MatrixXd jac(kStateDim, kStateDim + kControlDim);
    dynamics.Jacobian(x_ref[k], u_ref[k], k * dt, jac);
    A[k] = altro::MatrixXd::Identity(kStateDim, kStateDim) +
           dt * jac.block(0, 0, kStateDim, kStateDim);
    B[k] = dt * jac.block(0, kStateDim, kStateDim, kControlDim);
  }

  // Backward Riccati with damping for numerical robustness.
  altro::MatrixXd P = Qf;
  const altro::MatrixXd identity_u =
      altro::MatrixXd::Identity(kControlDim, kControlDim);
  for (int k = N - 1; k >= 0; --k) {
    const altro::MatrixXd R_BPB = R + B[k].transpose() * P * B[k];
    bool gain_ok = false;
    double reg = 1e-6;

    for (int attempt = 0; attempt < 6; ++attempt) {
      const altro::MatrixXd R_reg = R_BPB + reg * identity_u;
      Eigen::LDLT<altro::MatrixXd> ldlt(R_reg);
      if (ldlt.info() == Eigen::Success) {
        const altro::MatrixXd rhs = B[k].transpose() * P * A[k];
        altro::MatrixXd gain_candidate = -ldlt.solve(rhs);
        if (gain_candidate.allFinite()) {
          K[k] = gain_candidate;
          gain_ok = true;
          break;
        }
      }
      reg *= 10.0;
    }

    if (!gain_ok) {
      K[k].setZero();
    }

    const altro::MatrixXd A_BK = A[k] + B[k] * K[k];
    P = Q + A_BK.transpose() * P * A_BK + K[k].transpose() * R * K[k];
    if (!P.allFinite()) {
      P = Q;
      K[k].setZero();
    }
  }

  // Forward rollout with mild feedback around feedforward controls.
  altro::VectorXd x = x0;
  x(3) = clip_scalar(x(3), min_velocity_for_lqr, cfg.max_velocity);
  x(4) = clip_scalar(x(4), cfg.min_acceleration, cfg.max_acceleration);
  x(5) = clip_scalar(x(5), -cfg.max_curvature, cfg.max_curvature);
  if (x(3) < 0.5) {
    x(5) = 0.0;
  }
  traj->State(0) = x;

  double max_segment_dist = 0.0;
  bool warm_start_valid = true;
  for (int k = 0; k < N; ++k) {
    altro::VectorXd dx = x - x_ref[k];

    // Yaw residual wrap-around.
    while (dx(2) > M_PI) dx(2) -= 2.0 * M_PI;
    while (dx(2) < -M_PI) dx(2) += 2.0 * M_PI;

    // Component-wise clipping avoids one dimension dominating feedback.
    dx(0) = clip_scalar(dx(0), -3.0, 3.0);
    dx(1) = clip_scalar(dx(1), -1.8, 1.8);
    dx(2) = clip_scalar(dx(2), -0.20, 0.20);
    dx(3) = clip_scalar(dx(3), -2.0, 2.0);
    dx(4) = clip_scalar(dx(4), -1.5, 1.5);
    dx(5) = clip_scalar(dx(5), -0.12, 0.12);

    const double gain =
        feedback_gain_floor +
        feedback_gain_scale *
            std::exp(-static_cast<double>(k) * dt / feedback_decay_time);
    altro::VectorXd u = u_ref[k] + gain * K[k] * dx;
    if (!u.allFinite()) {
      u = u_ref[k];
    }

    // Limit deviation from feedforward to avoid aggressive corrections.
    const double du_jerk = clip_scalar(u(0) - u_ref[k](0), -1.0, 1.0);
    const double du_kappa = clip_scalar(u(1) - u_ref[k](1), -0.08, 0.08);
    u(0) = u_ref[k](0) + du_jerk;
    u(1) = u_ref[k](1) + du_kappa;

    // Global norm guard and hard box constraints.
    const double u_norm = u.norm();
    if (u_norm > max_feedback_norm) {
      u *= (max_feedback_norm / u_norm);
    }
    u(0) = clip_scalar(u(0), -cfg.max_jerk, cfg.max_jerk);
    u(1) = clip_scalar(u(1), -cfg.max_dkappa, cfg.max_dkappa);

    traj->Control(k) = u;
    traj->SetTime(k, static_cast<float>(k * dt));
    traj->SetStep(k, static_cast<float>(dt));

    // RK4 rollout.
    altro::VectorXd k1(kStateDim), k2(kStateDim), k3(kStateDim), k4(kStateDim);
    altro::VectorXd x_temp(kStateDim);
    dynamics.Evaluate(x, u, k * dt, k1);
    x_temp = x + 0.5 * dt * k1;
    dynamics.Evaluate(x_temp, u, k * dt + 0.5 * dt, k2);
    x_temp = x + 0.5 * dt * k2;
    dynamics.Evaluate(x_temp, u, k * dt + 0.5 * dt, k3);
    x_temp = x + dt * k3;
    dynamics.Evaluate(x_temp, u, k * dt + dt, k4);
    x = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    // State guards.
    while (x(2) > M_PI) x(2) -= 2.0 * M_PI;
    while (x(2) < -M_PI) x(2) += 2.0 * M_PI;
    x(3) = clip_scalar(x(3), cfg.min_velocity, cfg.max_velocity);
    x(4) = clip_scalar(x(4), cfg.min_acceleration, cfg.max_acceleration);
    x(5) = clip_scalar(x(5), -cfg.max_curvature, cfg.max_curvature);
    if (x(3) < 0.5) {
      x(5) = 0.0;
    }

    if (!x.allFinite()) {
      warm_start_valid = false;
      break;
    }

    const double dx_pos = x(0) - traj->State(k)(0);
    const double dy_pos = x(1) - traj->State(k)(1);
    const double step_dist = std::sqrt(dx_pos * dx_pos + dy_pos * dy_pos);
    max_segment_dist = std::max(max_segment_dist, step_dist);

    const double v_prev = std::max(traj->State(k)(3), min_velocity_for_lqr);
    const double max_reasonable_step = std::max(2.5, v_prev * dt * 2.5 + 0.5);
    if (!std::isfinite(step_dist) || step_dist > max_reasonable_step) {
      warm_start_valid = false;
      break;
    }

    traj->State(k + 1) = x;
  }

  // If rollout quality is poor, fall back to pure reference warm start.
  if (!warm_start_valid) {
    LOG(WARNING)
        << "LQR warm start rollout invalid, fallback to reference warm start.";
    for (int k = 0; k <= N; ++k) {
      traj->State(k) = x_ref[k];
      if (k < N) {
        traj->Control(k) = u_ref[k];
        traj->SetTime(k, static_cast<float>(k * dt));
        traj->SetStep(k, static_cast<float>(dt));
      }
    }
  }

  LOG(INFO) << "LQR warm start complete. max_step_dist=" << std::fixed
            << std::setprecision(3) << max_segment_dist
            << ", valid=" << (warm_start_valid ? "true" : "false");

  // Save LQR warm start trajectory for visualization.
  planner_->lqr_warm_start_traj_.clear();

  try {
    const auto& first_state = traj->State(0);
    const auto& last_state = traj->State(N);

    bool states_valid = true;
    for (int i = 0; i < kStateDim; ++i) {
      if (!std::isfinite(first_state(i)) || !std::isfinite(last_state(i))) {
        states_valid = false;
        break;
      }
    }

    if (!states_valid) {
      LOG(WARNING)
          << "LQR warm start trajectory has invalid states, skipping save";
      return kSuccess;
    }

    planner_->lqr_warm_start_traj_.states.resize(N + 1);
    planner_->lqr_warm_start_traj_.controls.resize(N);
    planner_->lqr_warm_start_traj_.times.resize(N + 1);
    planner_->lqr_warm_start_traj_.dt = dt;

    for (int k = 0; k <= N; ++k) {
      const auto& state_k = traj->State(k);

      bool state_valid = true;
      for (int i = 0; i < kStateDim; ++i) {
        if (!std::isfinite(state_k(i))) {
          state_valid = false;
          LOG(WARNING) << "Invalid state at k=" << k << ", dim=" << i;
          break;
        }
      }

      if (state_valid) {
        planner_->lqr_warm_start_traj_.states[k] = state_k;
        planner_->lqr_warm_start_traj_.times[k] = k * dt;
      } else {
        if (k > 0) {
          planner_->lqr_warm_start_traj_.states[k] =
              planner_->lqr_warm_start_traj_.states[k - 1];
        } else {
          planner_->lqr_warm_start_traj_.states[k] = x0;
        }
        planner_->lqr_warm_start_traj_.times[k] = k * dt;
      }

      if (k < N) {
        const auto& control_k = traj->Control(k);

        bool control_valid = true;
        for (int i = 0; i < kControlDim; ++i) {
          if (!std::isfinite(control_k(i))) {
            control_valid = false;
            LOG(WARNING) << "Invalid control at k=" << k << ", dim=" << i;
            break;
          }
        }

        if (control_valid) {
          planner_->lqr_warm_start_traj_.controls[k] = control_k;
        } else {
          planner_->lqr_warm_start_traj_.controls[k] =
              altro::VectorXd::Zero(kControlDim);
        }
      }
    }

    LOG(INFO) << "LQR warm start trajectory saved for visualization";
  } catch (const std::exception& e) {
    LOG(ERROR) << "Exception while saving LQR warm start trajectory: "
               << e.what();
    planner_->lqr_warm_start_traj_.clear();
  } catch (...) {
    LOG(ERROR) << "Unknown exception while saving LQR warm start trajectory";
    planner_->lqr_warm_start_traj_.clear();
  }

  return kSuccess;
}

}  // namespace ilqr
}  // namespace planning
