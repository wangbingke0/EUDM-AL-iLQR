/**
 * @file cartesian_corridor_constraint.h
 * @brief Corridor constraint in Cartesian (x, y, t) coordinates
 * 
 * This class converts spatio-temporal corridor boundaries from Frenet coordinates
 * to Cartesian coordinates and enforces the trajectory to stay within the corridor.
 */

#ifndef ILQR_PLANNER_CARTESIAN_CORRIDOR_CONSTRAINT_H_
#define ILQR_PLANNER_CARTESIAN_CORRIDOR_CONSTRAINT_H_

#include <memory>
#include <vector>

#include "altro/constraints/constraint.hpp"
#include "altro/eigentypes.hpp"

#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/state_transformer.h"

namespace planning {
namespace ilqr {

/**
 * @brief A 2D convex polygon boundary in Cartesian coordinates
 */
struct CartesianPolygon {
  std::vector<Eigen::Vector2d> vertices;
  double t_lb;  // Time lower bound
  double t_ub;  // Time upper bound
  
  /**
   * @brief Check if a point is inside the polygon using cross product method
   * @param x X coordinate
   * @param y Y coordinate
   * @return Signed distance (negative if inside, positive if outside)
   */
  double SignedDistance(double x, double y) const;
  
  /**
   * @brief Get the gradient of signed distance with respect to (x, y)
   */
  void SignedDistanceGradient(double x, double y, double* grad_x, double* grad_y) const;
};

/**
 * @brief Corridor boundary in Cartesian coordinates (x, y, t)
 * 
 * Stores a sequence of convex polygons representing the drivable area
 * at different time slices.
 */
class CartesianCorridor {
 public:
  CartesianCorridor() = default;
  
  /**
   * @brief Convert from Frenet corridor cubes to Cartesian polygons
   * @param frenet_cubes Corridor cubes in Frenet (s, d, t) coordinates
   * @param lane Reference lane for coordinate transformation
   * @param sample_resolution Resolution for sampling points on corridor boundary
   */
  void ConvertFromFrenetCorridor(
      const vec_E<common::SpatioTemporalSemanticCubeNd<2>>& frenet_cubes,
      const common::Lane& lane,
      double sample_resolution = 1.0);
  
  /**
   * @brief Get the polygon containing a specific time
   * @param t Time to query
   * @return Pointer to the polygon, nullptr if not found
   */
  const CartesianPolygon* GetPolygonAtTime(double t) const;
  
  /**
   * @brief Get all polygons
   */
  const std::vector<CartesianPolygon>& GetPolygons() const { return polygons_; }
  
  bool IsEmpty() const { return polygons_.empty(); }
  
 private:
  std::vector<CartesianPolygon> polygons_;
};

/**
 * @brief Corridor constraint for trajectory optimization
 * 
 * Enforces that the vehicle position stays within the corridor bounds.
 * Uses inequality constraints: c(x) <= 0 means inside the corridor.
 */
class CartesianCorridorConstraint : public altro::constraints::Constraint<altro::constraints::Inequality> {
 public:
  using ConstraintType = altro::constraints::Inequality;
  
  CartesianCorridorConstraint() = default;
  
  /**
   * @brief Set the corridor from Frenet cubes
   */
  void SetFromFrenetCorridor(
      const vec_E<common::SpatioTemporalSemanticCubeNd<2>>& frenet_cubes,
      const common::Lane& lane,
      double time_origin = 0.0);
  
  /**
   * @brief Set the current time for evaluating constraints at a specific knot point
   */
  void SetCurrentTime(double t) { current_time_ = t; }
  
  std::string GetLabel() const override { return "Cartesian Corridor Constraint"; }
  
  int StateDimension() const override { return 6; }
  int ControlDimension() const override { return 2; }
  
  /**
   * @brief Output dimension: 4 constraints (left, right, front, back boundaries)
   */
  int OutputDimension() const override { return 4; }
  
  /**
   * @brief Evaluate corridor constraint
   * c <= 0 means the point is inside the corridor
   */
  void Evaluate(const altro::VectorXdRef& x, const altro::VectorXdRef& u,
                Eigen::Ref<altro::VectorXd> c) override;
  
  void Jacobian(const altro::VectorXdRef& x, const altro::VectorXdRef& u,
                Eigen::Ref<altro::MatrixXd> jac) override;

 private:
  CartesianCorridor corridor_;
  double current_time_ = 0.0;
  double time_origin_ = 0.0;
  
  // Cached Frenet corridor for simpler constraint evaluation
  const vec_E<common::SpatioTemporalSemanticCubeNd<2>>* frenet_cubes_ = nullptr;
  common::StateTransformer stf_;
  bool use_simple_box_ = true;  // Use simple box constraint instead of polygon
  
  // Simple box bounds (interpolated from corridor at current time)
  double x_min_, x_max_, y_min_, y_max_;
  
  void UpdateBoxBoundsAtTime(double t);
};

/**
 * @brief Factory to create corridor constraints for each knot point
 */
class CorridorConstraintFactory {
 public:
  CorridorConstraintFactory() = default;
  
  /**
   * @brief Initialize with Frenet corridor
   */
  void Initialize(
      const vec_E<common::SpatioTemporalSemanticCubeNd<2>>& frenet_cubes,
      const common::Lane& lane,
      double time_origin,
      double dt,
      int num_knots);
  
  /**
   * @brief Get constraint for a specific knot point
   */
  std::shared_ptr<CartesianCorridorConstraint> GetConstraint(int k);
  
  /**
   * @brief Get all constraints
   */
  const std::vector<std::shared_ptr<CartesianCorridorConstraint>>& GetConstraints() const {
    return constraints_;
  }

 private:
  std::vector<std::shared_ptr<CartesianCorridorConstraint>> constraints_;
  vec_E<common::SpatioTemporalSemanticCubeNd<2>> frenet_cubes_;
  common::Lane lane_;
  double time_origin_ = 0.0;
  double dt_ = 0.1;
};

}  // namespace ilqr
}  // namespace planning

#endif  // ILQR_PLANNER_CARTESIAN_CORRIDOR_CONSTRAINT_H_

