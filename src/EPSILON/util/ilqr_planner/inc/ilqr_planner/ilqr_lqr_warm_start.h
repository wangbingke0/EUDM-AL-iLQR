/**
 * @file ilqr_lqr_warm_start.h
 * @brief LQR warm-start generator for iLQR trajectory optimization
 */

#ifndef ILQR_PLANNER_ILQR_LQR_WARM_START_H_
#define ILQR_PLANNER_ILQR_LQR_WARM_START_H_

#include "common/basics/basics.h"

#include "altro/common/trajectory.hpp"

namespace planning {
namespace ilqr {

class IlqrPlanner;

class IlqrLqrWarmStart {
 public:
  static constexpr int kStateDim = 6;
  static constexpr int kControlDim = 2;

  explicit IlqrLqrWarmStart(IlqrPlanner* planner);

  ErrorType Compute(
      const altro::VectorXd& x0,
      const altro::MatrixXd& Q,
      const altro::MatrixXd& R,
      const altro::MatrixXd& Qf,
      double dt,
      int N,
      altro::Trajectory<kStateDim, kControlDim>* traj);

 private:
  IlqrPlanner* planner_ = nullptr;
};

}  // namespace ilqr
}  // namespace planning

#endif  // ILQR_PLANNER_ILQR_LQR_WARM_START_H_
