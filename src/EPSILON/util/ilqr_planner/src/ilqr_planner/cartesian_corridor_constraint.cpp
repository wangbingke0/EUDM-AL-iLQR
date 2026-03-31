/**
 * @file cartesian_corridor_constraint.cpp
 * @brief Implementation of corridor constraints in Cartesian coordinates
 */

#include "ilqr_planner/cartesian_corridor_constraint.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "altro/utils/utils.hpp"

namespace planning {
namespace ilqr {

// ============================================================================
// CartesianPolygon Implementation
// ============================================================================

double CartesianPolygon::SignedDistance(double x, double y) const {
  if (vertices.size() < 3) {
    return std::numeric_limits<double>::max();
  }
  
  // For a convex polygon, compute the minimum signed distance to all edges
  double min_dist = std::numeric_limits<double>::max();
  
  for (size_t i = 0; i < vertices.size(); ++i) {
    const auto& p1 = vertices[i];
    const auto& p2 = vertices[(i + 1) % vertices.size()];
    
    // Edge vector
    Eigen::Vector2d edge = p2 - p1;
    double edge_len = edge.norm();
    if (edge_len < 1e-6) continue;
    
    // Normal pointing inward (for counterclockwise winding)
    Eigen::Vector2d normal(-edge.y() / edge_len, edge.x() / edge_len);
    
    // Point relative to edge start
    Eigen::Vector2d point(x - p1.x(), y - p1.y());
    
    // Signed distance to this edge (negative if inside)
    double dist = point.dot(normal);
    min_dist = std::min(min_dist, dist);
  }
  
  return min_dist;  // Negative if inside all edges (inside polygon)
}

void CartesianPolygon::SignedDistanceGradient(double x, double y, 
                                               double* grad_x, double* grad_y) const {
  if (vertices.size() < 3) {
    *grad_x = 0.0;
    *grad_y = 0.0;
    return;
  }
  
  // Find the edge with minimum signed distance
  double min_dist = std::numeric_limits<double>::max();
  int min_idx = 0;
  
  for (size_t i = 0; i < vertices.size(); ++i) {
    const auto& p1 = vertices[i];
    const auto& p2 = vertices[(i + 1) % vertices.size()];
    
    Eigen::Vector2d edge = p2 - p1;
    double edge_len = edge.norm();
    if (edge_len < 1e-6) continue;
    
    Eigen::Vector2d normal(-edge.y() / edge_len, edge.x() / edge_len);
    Eigen::Vector2d point(x - p1.x(), y - p1.y());
    
    double dist = point.dot(normal);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  
  // Gradient is the normal of the closest edge
  const auto& p1 = vertices[min_idx];
  const auto& p2 = vertices[(min_idx + 1) % vertices.size()];
  Eigen::Vector2d edge = p2 - p1;
  double edge_len = edge.norm();
  
  *grad_x = -edge.y() / edge_len;
  *grad_y = edge.x() / edge_len;
}

// ============================================================================
// CartesianCorridor Implementation
// ============================================================================

void CartesianCorridor::ConvertFromFrenetCorridor(
    const vec_E<common::SpatioTemporalSemanticCubeNd<2>>& frenet_cubes,
    const common::Lane& lane,
    double sample_resolution) {
  polygons_.clear();
  
  if (frenet_cubes.empty() || !lane.IsValid()) {
    return;
  }
  
  common::StateTransformer stf(lane);
  
  for (const auto& cube : frenet_cubes) {
    CartesianPolygon polygon;
    polygon.t_lb = cube.t_lb;
    polygon.t_ub = cube.t_ub;
    
    double s_min = cube.p_lb[0];
    double s_max = cube.p_ub[0];
    double d_min = cube.p_lb[1];
    double d_max = cube.p_ub[1];
    
    // Sample points along the corridor boundary to create polygon vertices
    // We sample the four boundaries: left, front, right, back
    
    // Number of samples along each boundary
    int n_samples_s = std::max(2, static_cast<int>((s_max - s_min) / sample_resolution));
    int n_samples_d = std::max(2, static_cast<int>((d_max - d_min) / sample_resolution));
    
    // Bottom boundary (d = d_min, s from s_min to s_max)
    for (int i = 0; i <= n_samples_s; ++i) {
      double s = s_min + i * (s_max - s_min) / n_samples_s;
      double d = d_min;
      
      // Convert Frenet point to Cartesian
      Vecf<2> lane_pos;
      if (lane.GetPositionByArcLength(s, &lane_pos) == kSuccess) {
        Vecf<2> tangent;
        if (lane.GetTangentVectorByArcLength(s, &tangent) == kSuccess) {
          Vecf<2> normal(-tangent[1], tangent[0]);
          Eigen::Vector2d cart_point;
          cart_point.x() = lane_pos[0] + d * normal[0];
          cart_point.y() = lane_pos[1] + d * normal[1];
          polygon.vertices.push_back(cart_point);
        }
      }
    }
    
    // Right boundary (s = s_max, d from d_min to d_max)
    for (int i = 1; i <= n_samples_d; ++i) {
      double s = s_max;
      double d = d_min + i * (d_max - d_min) / n_samples_d;
      
      Vecf<2> lane_pos;
      if (lane.GetPositionByArcLength(s, &lane_pos) == kSuccess) {
        Vecf<2> tangent;
        if (lane.GetTangentVectorByArcLength(s, &tangent) == kSuccess) {
          Vecf<2> normal(-tangent[1], tangent[0]);
          Eigen::Vector2d cart_point;
          cart_point.x() = lane_pos[0] + d * normal[0];
          cart_point.y() = lane_pos[1] + d * normal[1];
          polygon.vertices.push_back(cart_point);
        }
      }
    }
    
    // Top boundary (d = d_max, s from s_max to s_min)
    for (int i = 1; i <= n_samples_s; ++i) {
      double s = s_max - i * (s_max - s_min) / n_samples_s;
      double d = d_max;
      
      Vecf<2> lane_pos;
      if (lane.GetPositionByArcLength(s, &lane_pos) == kSuccess) {
        Vecf<2> tangent;
        if (lane.GetTangentVectorByArcLength(s, &tangent) == kSuccess) {
          Vecf<2> normal(-tangent[1], tangent[0]);
          Eigen::Vector2d cart_point;
          cart_point.x() = lane_pos[0] + d * normal[0];
          cart_point.y() = lane_pos[1] + d * normal[1];
          polygon.vertices.push_back(cart_point);
        }
      }
    }
    
    // Left boundary (s = s_min, d from d_max to d_min) - close the polygon
    for (int i = 1; i < n_samples_d; ++i) {
      double s = s_min;
      double d = d_max - i * (d_max - d_min) / n_samples_d;
      
      Vecf<2> lane_pos;
      if (lane.GetPositionByArcLength(s, &lane_pos) == kSuccess) {
        Vecf<2> tangent;
        if (lane.GetTangentVectorByArcLength(s, &tangent) == kSuccess) {
          Vecf<2> normal(-tangent[1], tangent[0]);
          Eigen::Vector2d cart_point;
          cart_point.x() = lane_pos[0] + d * normal[0];
          cart_point.y() = lane_pos[1] + d * normal[1];
          polygon.vertices.push_back(cart_point);
        }
      }
    }
    
    if (polygon.vertices.size() >= 3) {
      polygons_.push_back(polygon);
    }
  }
}

const CartesianPolygon* CartesianCorridor::GetPolygonAtTime(double t) const {
  for (const auto& poly : polygons_) {
    if (t >= poly.t_lb && t <= poly.t_ub) {
      return &poly;
    }
  }
  return nullptr;
}

// ============================================================================
// CartesianCorridorConstraint Implementation
// ============================================================================

void CartesianCorridorConstraint::SetFromFrenetCorridor(
    const vec_E<common::SpatioTemporalSemanticCubeNd<2>>& frenet_cubes,
    const common::Lane& lane,
    double time_origin) {
  time_origin_ = time_origin;
  stf_ = common::StateTransformer(lane);
  
  // Store reference to cubes for simple box constraint
  // (We don't copy to avoid memory issues - caller must ensure lifetime)
  frenet_cubes_ = &frenet_cubes;
  
  // Also convert to Cartesian for advanced constraints
  corridor_.ConvertFromFrenetCorridor(frenet_cubes, lane, 2.0);
}

void CartesianCorridorConstraint::UpdateBoxBoundsAtTime(double t) {
  if (frenet_cubes_ == nullptr || frenet_cubes_->empty()) {
    x_min_ = -1000.0;
    x_max_ = 1000.0;
    y_min_ = -1000.0;
    y_max_ = 1000.0;
    return;
  }
  
  // Find the cube containing this time
  double query_time = time_origin_ + t;
  const common::SpatioTemporalSemanticCubeNd<2>* cube = nullptr;
  
  for (const auto& c : *frenet_cubes_) {
    if (query_time >= c.t_lb && query_time <= c.t_ub) {
      cube = &c;
      break;
    }
  }
  
  if (cube == nullptr) {
    // Use the last cube if time is beyond corridor
    cube = &frenet_cubes_->back();
  }
  
  // Get Cartesian polygon at this time
  const CartesianPolygon* poly = corridor_.GetPolygonAtTime(query_time);
  
  if (poly != nullptr && !poly->vertices.empty()) {
    // Compute bounding box of the polygon
    x_min_ = std::numeric_limits<double>::max();
    x_max_ = -std::numeric_limits<double>::max();
    y_min_ = std::numeric_limits<double>::max();
    y_max_ = -std::numeric_limits<double>::max();
    
    for (const auto& v : poly->vertices) {
      x_min_ = std::min(x_min_, v.x());
      x_max_ = std::max(x_max_, v.x());
      y_min_ = std::min(y_min_, v.y());
      y_max_ = std::max(y_max_, v.y());
    }
  } else {
    // Fallback: use large bounds
    x_min_ = -1000.0;
    x_max_ = 1000.0;
    y_min_ = -1000.0;
    y_max_ = 1000.0;
  }
}

void CartesianCorridorConstraint::Evaluate(const altro::VectorXdRef& x, 
                                            const altro::VectorXdRef& u,
                                            Eigen::Ref<altro::VectorXd> c) {
  ALTRO_UNUSED(u);
  
  double px = x(0);  // x position
  double py = x(1);  // y position
  
  UpdateBoxBoundsAtTime(current_time_);
  
  // Box constraints: all should be <= 0 when inside
  // c0: x - x_max <= 0  (right boundary)
  // c1: x_min - x <= 0  (left boundary)
  // c2: y - y_max <= 0  (top boundary)
  // c3: y_min - y <= 0  (bottom boundary)
  
  c(0) = px - x_max_;
  c(1) = x_min_ - px;
  c(2) = py - y_max_;
  c(3) = y_min_ - py;
}

void CartesianCorridorConstraint::Jacobian(const altro::VectorXdRef& x,
                                            const altro::VectorXdRef& u,
                                            Eigen::Ref<altro::MatrixXd> jac) {
  ALTRO_UNUSED(x);
  ALTRO_UNUSED(u);
  
  // Jacobian size: 4 x (n + m) = 4 x 8
  jac.setZero();
  
  // c0 = px - x_max: dc0/dx = 1
  jac(0, 0) = 1.0;
  
  // c1 = x_min - px: dc1/dx = -1
  jac(1, 0) = -1.0;
  
  // c2 = py - y_max: dc2/dy = 1
  jac(2, 1) = 1.0;
  
  // c3 = y_min - py: dc3/dy = -1
  jac(3, 1) = -1.0;
}

// ============================================================================
// CorridorConstraintFactory Implementation
// ============================================================================

void CorridorConstraintFactory::Initialize(
    const vec_E<common::SpatioTemporalSemanticCubeNd<2>>& frenet_cubes,
    const common::Lane& lane,
    double time_origin,
    double dt,
    int num_knots) {
  frenet_cubes_ = frenet_cubes;
  lane_ = lane;
  time_origin_ = time_origin;
  dt_ = dt;
  
  constraints_.clear();
  constraints_.reserve(num_knots);
  
  for (int k = 0; k < num_knots; ++k) {
    auto constraint = std::make_shared<CartesianCorridorConstraint>();
    constraint->SetFromFrenetCorridor(frenet_cubes_, lane_, time_origin_);
    constraint->SetCurrentTime(k * dt_);
    constraints_.push_back(constraint);
  }
}

std::shared_ptr<CartesianCorridorConstraint> CorridorConstraintFactory::GetConstraint(int k) {
  if (k >= 0 && k < static_cast<int>(constraints_.size())) {
    return constraints_[k];
  }
  return nullptr;
}

}  // namespace ilqr
}  // namespace planning

