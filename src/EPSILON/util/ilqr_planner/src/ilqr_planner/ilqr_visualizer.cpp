/**
 * @file ilqr_visualizer.cpp
 * @brief Implementation of iLQR planner visualization
 */

#include "ilqr_planner/ilqr_visualizer.h"

#include <glog/logging.h>
#include <cmath>
#include <unordered_map>
#include "common/visualization/common_visualization_util.h"

namespace planning {
namespace ilqr {

// ============================================================================
// Color Helpers
// ============================================================================

std_msgs::ColorRGBA IlqrVisualizer::Colors::Red() {
  std_msgs::ColorRGBA c;
  c.r = 1.0; c.g = 0.0; c.b = 0.0; c.a = 1.0;
  return c;
}

std_msgs::ColorRGBA IlqrVisualizer::Colors::Green() {
  std_msgs::ColorRGBA c;
  c.r = 0.0; c.g = 1.0; c.b = 0.0; c.a = 1.0;
  return c;
}

std_msgs::ColorRGBA IlqrVisualizer::Colors::Blue() {
  std_msgs::ColorRGBA c;
  c.r = 0.0; c.g = 0.0; c.b = 1.0; c.a = 1.0;
  return c;
}

std_msgs::ColorRGBA IlqrVisualizer::Colors::Yellow() {
  std_msgs::ColorRGBA c;
  c.r = 1.0; c.g = 1.0; c.b = 0.0; c.a = 1.0;
  return c;
}

std_msgs::ColorRGBA IlqrVisualizer::Colors::Cyan() {
  std_msgs::ColorRGBA c;
  c.r = 0.0; c.g = 1.0; c.b = 1.0; c.a = 1.0;
  return c;
}

std_msgs::ColorRGBA IlqrVisualizer::Colors::Magenta() {
  std_msgs::ColorRGBA c;
  c.r = 1.0; c.g = 0.0; c.b = 1.0; c.a = 1.0;
  return c;
}

std_msgs::ColorRGBA IlqrVisualizer::Colors::Orange() {
  std_msgs::ColorRGBA c;
  c.r = 1.0; c.g = 0.5; c.b = 0.0; c.a = 1.0;
  return c;
}

std_msgs::ColorRGBA IlqrVisualizer::Colors::White() {
  std_msgs::ColorRGBA c;
  c.r = 1.0; c.g = 1.0; c.b = 1.0; c.a = 1.0;
  return c;
}

std_msgs::ColorRGBA IlqrVisualizer::Colors::Gray() {
  std_msgs::ColorRGBA c;
  c.r = 0.5; c.g = 0.5; c.b = 0.5; c.a = 1.0;
  return c;
}

std_msgs::ColorRGBA IlqrVisualizer::Colors::Transparent(
    const std_msgs::ColorRGBA& color, double alpha) {
  std_msgs::ColorRGBA c = color;
  c.a = alpha;
  return c;
}

// ============================================================================
// IlqrVisualizer Implementation
// ============================================================================

IlqrVisualizer::IlqrVisualizer(ros::NodeHandle& nh, const std::string& frame_id,
                                 const std::string& topic_prefix)
    : nh_(nh), frame_id_(frame_id) {
  ref_lane_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      topic_prefix + "/reference_lane", 1);
  cartesian_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      topic_prefix + "/cartesian_trajectory", 1);
  lqr_warm_start_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      topic_prefix + "/lqr_warm_start_trajectory", 1);
  cartesian_corridor_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      topic_prefix + "/cartesian_corridor", 1);
  frenet_corridor_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      topic_prefix + "/frenet_corridor", 1);
  reference_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      topic_prefix + "/reference_trajectory", 1);
  ego_vehicle_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      topic_prefix + "/ego_vehicle", 1);
  all_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      topic_prefix + "/all_markers", 1);
}

visualization_msgs::Marker IlqrVisualizer::CreateLineStripMarker(
    int id, const std::string& ns, const std_msgs::ColorRGBA& color,
    double width, const ros::Time& stamp) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = width;
  marker.color = color;
  marker.pose.orientation.w = 1.0;
  return marker;
}

visualization_msgs::Marker IlqrVisualizer::CreatePolygonMarker(
    int id, const std::string& ns, const std_msgs::ColorRGBA& color,
    const ros::Time& stamp) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1;
  marker.color = color;
  marker.pose.orientation.w = 1.0;
  return marker;
}

visualization_msgs::Marker IlqrVisualizer::CreateCubeMarker(
    int id, const std::string& ns, const std_msgs::ColorRGBA& color,
    const geometry_msgs::Pose& pose, const geometry_msgs::Vector3& scale,
    const ros::Time& stamp) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  marker.scale = scale;
  marker.color = color;
  return marker;
}

visualization_msgs::Marker IlqrVisualizer::CreateSphereMarker(
    int id, const std::string& ns, const std_msgs::ColorRGBA& color,
    double radius, const ros::Time& stamp) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = marker.scale.y = marker.scale.z = radius * 2;
  marker.color = color;
  marker.pose.orientation.w = 1.0;
  return marker;
}

void IlqrVisualizer::Visualize(const IlqrPlanner& planner, const ros::Time& stamp) {
  // Visualize reference lane
  VisualizeReferenceLane(planner.reference_lane(), stamp);
  
  // Visualize LQR warm start trajectory (yellow, as reference for iLQR)
  VisualizeLQRWarmStartTrajectory(planner.GetLQRWarmStartTrajectory(), stamp);
  
  // Visualize Cartesian trajectory
  VisualizeCartesianTrajectory(planner.GetCartesianTrajectory(), stamp);
  
  // Visualize Cartesian corridor
  VisualizeCartesianCorridor(planner.GetCartesianCorridor(), stamp);
  
  // Visualize Frenet corridor
  VisualizeFrenetCorridor(planner.GetCorridorCubes(), 
                          planner.reference_lane(), stamp);
  
  // Visualize reference trajectory
  VisualizeReferenceTrajectory(planner.GetReferenceStates(), stamp);
  
  // Visualize ego vehicle
  VisualizeEgoVehicle(planner.ego_vehicle(), stamp);
}

void IlqrVisualizer::VisualizeReferenceLane(const common::Lane& lane,
                                             const ros::Time& stamp) {
  visualization_msgs::MarkerArray markers;
  
  if (!lane.IsValid()) {
    ref_lane_pub_.publish(markers);
    return;
  }
  
  auto marker = CreateLineStripMarker(0, "reference_lane", Colors::Gray(), 0.3, stamp);
  
  double s_begin = lane.begin();
  double s_end = lane.end();
  double ds = 1.0;
  
  for (double s = s_begin; s <= s_end; s += ds) {
    Vecf<2> pos;
    if (lane.GetPositionByArcLength(s, &pos) == kSuccess) {
      geometry_msgs::Point pt;
      pt.x = pos[0];
      pt.y = pos[1];
      pt.z = 0.0;
      marker.points.push_back(pt);
    }
  }
  
  markers.markers.push_back(marker);
  ref_lane_pub_.publish(markers);
}

void IlqrVisualizer::VisualizeCartesianTrajectory(const CartesianTrajectory& traj,
                                                   const ros::Time& stamp) {
  visualization_msgs::MarkerArray markers;
  
  if (traj.empty()) {
    cartesian_traj_pub_.publish(markers);
    return;
  }
  
  // Trajectory line
  auto line_marker = CreateLineStripMarker(0, "trajectory_line", 
                                           Colors::Green(), 0.2, stamp);
  
  for (size_t k = 0; k < traj.states.size(); ++k) {
    geometry_msgs::Point pt;
    pt.x = traj.states[k](CartesianDynamics::kPosX);
    pt.y = traj.states[k](CartesianDynamics::kPosY);
    pt.z = 0.1;
    line_marker.points.push_back(pt);
  }
  markers.markers.push_back(line_marker);
  
  // Trajectory points with velocity coloring
  for (size_t k = 0; k < traj.states.size(); k += 5) {
    auto sphere = CreateSphereMarker(k + 100, "trajectory_points",
                                     Colors::Cyan(), 0.2, stamp);
    sphere.pose.position.x = traj.states[k](CartesianDynamics::kPosX);
    sphere.pose.position.y = traj.states[k](CartesianDynamics::kPosY);
    sphere.pose.position.z = 0.1;
    
    // Color by velocity
    double v = traj.states[k](CartesianDynamics::kVel);
    double v_normalized = std::min(1.0, std::max(0.0, v / 20.0));
    sphere.color.r = v_normalized;
    sphere.color.g = 1.0 - v_normalized;
    sphere.color.b = 0.2;
    
    markers.markers.push_back(sphere);
  }
  
  // Vehicle orientations along trajectory
  for (size_t k = 0; k < traj.states.size(); k += 10) {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = frame_id_;
    arrow.header.stamp = stamp;
    arrow.ns = "trajectory_heading";
    arrow.id = k + 200;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    
    double x = traj.states[k](CartesianDynamics::kPosX);
    double y = traj.states[k](CartesianDynamics::kPosY);
    double yaw = traj.states[k](CartesianDynamics::kYaw);
    
    geometry_msgs::Point start, end;
    start.x = x;
    start.y = y;
    start.z = 0.15;
    end.x = x + 2.0 * std::cos(yaw);
    end.y = y + 2.0 * std::sin(yaw);
    end.z = 0.15;
    
    arrow.points.push_back(start);
    arrow.points.push_back(end);
    
    arrow.scale.x = 0.1;  // shaft diameter
    arrow.scale.y = 0.2;  // head diameter
    arrow.scale.z = 0.3;  // head length
    arrow.color = Colors::Yellow();
    
    markers.markers.push_back(arrow);
  }
  
  cartesian_traj_pub_.publish(markers);
}

void IlqrVisualizer::VisualizeLQRWarmStartTrajectory(const CartesianTrajectory& traj,
                                                      const ros::Time& stamp) {
  visualization_msgs::MarkerArray markers;
  
  if (traj.empty()) {
    lqr_warm_start_traj_pub_.publish(markers);
    return;
  }
  
  // Safety check: validate trajectory states before visualization
  if (traj.states.size() != traj.times.size()) {
    LOG(WARNING) << "[IlqrVisualizer] LQR warm start trajectory size mismatch, skipping visualization";
    lqr_warm_start_traj_pub_.publish(markers);
    return;
  }
  
  // Trajectory line (yellow, thicker to distinguish from optimized trajectory)
  auto line_marker = CreateLineStripMarker(0, "lqr_warm_start_line", 
                                           Colors::Yellow(), 0.2, stamp);
  
  // Validate and add points
  for (size_t k = 0; k < traj.states.size(); ++k) {
    const auto& state = traj.states[k];
    
    // Check if state is valid
    bool state_valid = true;
    for (int i = 0; i < 6; ++i) {  // kStateDim = 6
      if (!std::isfinite(state(i))) {
        state_valid = false;
        break;
      }
    }
    
    if (state_valid) {
      geometry_msgs::Point pt;
      pt.x = state(CartesianDynamics::kPosX);
      pt.y = state(CartesianDynamics::kPosY);
      pt.z = 0.12;  // Slightly higher than optimized trajectory (0.1)
      
      // Additional check: ensure position is finite
      if (std::isfinite(pt.x) && std::isfinite(pt.y)) {
        line_marker.points.push_back(pt);
      }
    }
  }
  
  // Only add line marker if it has valid points
  if (!line_marker.points.empty()) {
    markers.markers.push_back(line_marker);
  }
  
  // Trajectory points (yellow spheres, less frequent for clarity)
  for (size_t k = 0; k < traj.states.size(); k += 10) {
    const auto& state = traj.states[k];
    
    // Validate state
    bool state_valid = true;
    for (int i = 0; i < 6; ++i) {
      if (!std::isfinite(state(i))) {
        state_valid = false;
        break;
      }
    }
    
    if (state_valid) {
      double x = state(CartesianDynamics::kPosX);
      double y = state(CartesianDynamics::kPosY);
      
      if (std::isfinite(x) && std::isfinite(y)) {
        auto sphere = CreateSphereMarker(k + 1000, "lqr_warm_start_points",
                                         Colors::Yellow(), 0.2, stamp);
        sphere.pose.position.x = x;
        sphere.pose.position.y = y;
        sphere.pose.position.z = 0.12;
        markers.markers.push_back(sphere);
      }
    }
  }
  
  lqr_warm_start_traj_pub_.publish(markers);
}

void IlqrVisualizer::VisualizeCartesianCorridor(const CartesianCorridor& corridor,
                                                 const ros::Time& stamp) {
  visualization_msgs::MarkerArray markers;
  
  if (corridor.IsEmpty()) {
    cartesian_corridor_pub_.publish(markers);
    return;
  }
  
  const auto& polygons = corridor.GetPolygons();
  int id = 0;
  
  for (const auto& poly : polygons) {
    auto marker = CreatePolygonMarker(id++, "corridor_polygon",
                                      Colors::Transparent(Colors::Blue(), 0.3), stamp);
    
    for (const auto& v : poly.vertices) {
      geometry_msgs::Point pt;
      pt.x = v.x();
      pt.y = v.y();
      pt.z = 0.05;
      marker.points.push_back(pt);
    }
    
    // Close the polygon
    if (!poly.vertices.empty()) {
      geometry_msgs::Point pt;
      pt.x = poly.vertices.front().x();
      pt.y = poly.vertices.front().y();
      pt.z = 0.05;
      marker.points.push_back(pt);
    }
    
    markers.markers.push_back(marker);
  }
  
  cartesian_corridor_pub_.publish(markers);
}

void IlqrVisualizer::VisualizeFrenetCorridor(
    const vec_E<common::SpatioTemporalSemanticCubeNd<2>>& cubes,
    const common::Lane& lane,
    const ros::Time& stamp) {
  visualization_msgs::MarkerArray markers;
  
  if (cubes.empty() || !lane.IsValid()) {
    frenet_corridor_pub_.publish(markers);
    return;
  }
  
  int id = 0;
  for (const auto& cube : cubes) {
    double s_center = (cube.p_lb[0] + cube.p_ub[0]) / 2.0;
    double d_center = (cube.p_lb[1] + cube.p_ub[1]) / 2.0;
    
    // Get position on lane
    Vecf<2> lane_pos;
    Vecf<2> tangent;
    if (lane.GetPositionByArcLength(s_center, &lane_pos) != kSuccess ||
        lane.GetTangentVectorByArcLength(s_center, &tangent) != kSuccess) {
      continue;
    }
    
    Vecf<2> normal(-tangent[1], tangent[0]);
    
    double x = lane_pos[0] + d_center * normal[0];
    double y = lane_pos[1] + d_center * normal[1];
    double yaw = std::atan2(tangent[1], tangent[0]);
    
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.05;
    
    // Quaternion from yaw
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = std::sin(yaw / 2);
    pose.orientation.w = std::cos(yaw / 2);
    
    geometry_msgs::Vector3 scale;
    scale.x = cube.p_ub[0] - cube.p_lb[0];  // length in s direction
    scale.y = cube.p_ub[1] - cube.p_lb[1];  // width in d direction
    scale.z = 0.1;
    
    auto marker = CreateCubeMarker(id++, "frenet_corridor",
                                   Colors::Transparent(Colors::Cyan(), 0.2),
                                   pose, scale, stamp);
    markers.markers.push_back(marker);
  }
  
  frenet_corridor_pub_.publish(markers);
}

void IlqrVisualizer::VisualizeReferenceTrajectory(
    const std::vector<common::State>& states,
    const ros::Time& stamp) {
  visualization_msgs::MarkerArray markers;
  
  if (states.empty()) {
    reference_traj_pub_.publish(markers);
    return;
  }
  
  auto line_marker = CreateLineStripMarker(0, "reference_trajectory",
                                           Colors::Orange(), 0.15, stamp);
  
  for (const auto& state : states) {
    geometry_msgs::Point pt;
    pt.x = state.vec_position.x();
    pt.y = state.vec_position.y();
    pt.z = 0.08;
    line_marker.points.push_back(pt);
  }
  
  markers.markers.push_back(line_marker);
  reference_traj_pub_.publish(markers);
}

void IlqrVisualizer::VisualizeEgoVehicle(const common::Vehicle& vehicle,
                                          const ros::Time& stamp) {
  visualization_msgs::MarkerArray markers;
  
  geometry_msgs::Pose pose;
  pose.position.x = vehicle.state().vec_position.x();
  pose.position.y = vehicle.state().vec_position.y();
  pose.position.z = 0.2;
  
  double yaw = vehicle.state().angle;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = std::sin(yaw / 2);
  pose.orientation.w = std::cos(yaw / 2);
  
  geometry_msgs::Vector3 scale;
  scale.x = vehicle.param().length();
  scale.y = vehicle.param().width();
  scale.z = 1.5;
  
  auto marker = CreateCubeMarker(0, "ego_vehicle", Colors::Red(), pose, scale, stamp);
  markers.markers.push_back(marker);
  
  ego_vehicle_pub_.publish(markers);
}

}  // namespace ilqr
}  // namespace planning
