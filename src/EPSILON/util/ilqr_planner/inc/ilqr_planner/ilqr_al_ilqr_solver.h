/**
 * @file ilqr_al_ilqr_solver.h
 * @brief AL-iLQR solver wrapper that consumes LQR warm-start
 */

#ifndef ILQR_PLANNER_ILQR_AL_ILQR_SOLVER_H_
#define ILQR_PLANNER_ILQR_AL_ILQR_SOLVER_H_

#include "common/basics/basics.h"

#include "ilqr_planner/ilqr_lqr_warm_start.h"

namespace planning {
namespace ilqr {

class IlqrPlanner;

class IlqrAlIlqrSolver {
 public:
  static constexpr int kStateDim = 6;
  static constexpr int kControlDim = 2;

  explicit IlqrAlIlqrSolver(IlqrPlanner* planner);

  ErrorType Solve();

 private:
  IlqrPlanner* planner_ = nullptr;
  IlqrLqrWarmStart warm_start_solver_;
};

}  // namespace ilqr
}  // namespace planning

#endif  // ILQR_PLANNER_ILQR_AL_ILQR_SOLVER_H_
