/**
 * @file ilqr_visualizer.h
 * @brief Visualization for iLQR planner in RViz
 */

#ifndef ILQR_PLANNER_ILQR_VISUALIZER_H_
#define ILQR_PLANNER_ILQR_VISUALIZER_H_

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/state.h"
#include "ilqr_planner/ilqr_planner.h"
#include "ilqr_planner/cartesian_corridor_constraint.h"

namespace planning {
namespace ilqr {

/**
 * @brief Visualization class for iLQR planner
 */
class IlqrVisualizer {
 public:
  IlqrVisualizer(ros::NodeHandle& nh, const std::string& frame_id = "map",
                 const std::string& topic_prefix = "ilqr_planner");
  ~IlqrVisualizer() = default;

  /**
   * @brief Visualize the iLQR planner state
   */
  void Visualize(const IlqrPlanner& planner, const ros::Time& stamp);

  /**
   * @brief Visualize reference lane
   */
  void VisualizeReferenceLane(const common::Lane& lane, const ros::Time& stamp);

  /**
   * @brief Visualize Cartesian trajectory
   */
  void VisualizeCartesianTrajectory(const CartesianTrajectory& traj, 
                                     const ros::Time& stamp);

  /**
   * @brief Visualize LQR warm start trajectory (yellow, as reference for iLQR)
   */
  void VisualizeLQRWarmStartTrajectory(const CartesianTrajectory& traj,
                                        const ros::Time& stamp);

  /**
   * @brief Visualize Cartesian corridor (polygons)
   */
  void VisualizeCartesianCorridor(const CartesianCorridor& corridor,
                                   const ros::Time& stamp);

  /**
   * @brief Visualize Frenet corridor (cubes in s-d-t space)
   */
  void VisualizeFrenetCorridor(
      const vec_E<common::SpatioTemporalSemanticCubeNd<2>>& cubes,
      const common::Lane& lane,
      const ros::Time& stamp);

  /**
   * @brief Visualize reference trajectory
   */
  void VisualizeReferenceTrajectory(const std::vector<common::State>& states,
                                     const ros::Time& stamp);

  /**
   * @brief Visualize ego vehicle
   */
  void VisualizeEgoVehicle(const common::Vehicle& vehicle, const ros::Time& stamp);

 private:
  ros::NodeHandle nh_;
  std::string frame_id_;

  // Publishers
  ros::Publisher ref_lane_pub_;
  ros::Publisher cartesian_traj_pub_;
  ros::Publisher lqr_warm_start_traj_pub_;
  ros::Publisher cartesian_corridor_pub_;
  ros::Publisher frenet_corridor_pub_;
  ros::Publisher reference_traj_pub_;
  ros::Publisher ego_vehicle_pub_;
  ros::Publisher all_markers_pub_;

  // Color definitions
  struct Colors {
    static std_msgs::ColorRGBA Red();
    static std_msgs::ColorRGBA Green();
    static std_msgs::ColorRGBA Blue();
    static std_msgs::ColorRGBA Yellow();
    static std_msgs::ColorRGBA Cyan();
    static std_msgs::ColorRGBA Magenta();
    static std_msgs::ColorRGBA Orange();
    static std_msgs::ColorRGBA White();
    static std_msgs::ColorRGBA Gray();
    static std_msgs::ColorRGBA Transparent(const std_msgs::ColorRGBA& color, double alpha);
  };

  visualization_msgs::Marker CreateLineStripMarker(
      int id, const std::string& ns, const std_msgs::ColorRGBA& color,
      double width, const ros::Time& stamp);

  visualization_msgs::Marker CreatePolygonMarker(
      int id, const std::string& ns, const std_msgs::ColorRGBA& color,
      const ros::Time& stamp);

  visualization_msgs::Marker CreateCubeMarker(
      int id, const std::string& ns, const std_msgs::ColorRGBA& color,
      const geometry_msgs::Pose& pose, const geometry_msgs::Vector3& scale,
      const ros::Time& stamp);

  visualization_msgs::Marker CreateSphereMarker(
      int id, const std::string& ns, const std_msgs::ColorRGBA& color,
      double radius, const ros::Time& stamp);
};

}  // namespace ilqr
}  // namespace planning

#endif  // ILQR_PLANNER_ILQR_VISUALIZER_H_
