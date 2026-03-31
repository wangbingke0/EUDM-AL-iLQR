/**
 * @file ilqr_server_ros.h
 * @brief ROS server for iLQR planner
 */

#ifndef ILQR_PLANNER_ILQR_SERVER_ROS_H_
#define ILQR_PLANNER_ILQR_SERVER_ROS_H_

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/state.h"
#include "common/trajectory/frenet_traj.h"
#include "common/visualization/common_visualization_util.h"

#include "vehicle_msgs/ControlSignal.h"
#include "vehicle_msgs/encoder.h"

#include "ilqr_planner/ilqr_planner.h"
#include "ilqr_planner/ilqr_visualizer.h"

#include "ssc_planner/map_adapter.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"

namespace planning {
namespace ilqr {

/**
 * @brief ROS server for iLQR planner
 */
class IlqrPlannerServer {
 public:
  using SemanticMapManager = semantic_map_manager::SemanticMapManager;
  using FrenetState = common::FrenetState;
  using State = common::State;
  using Vehicle = common::Vehicle;
  using Lane = common::Lane;

  struct Config {
    int planning_rate = 10;  // Hz
    bool enable_visualization = true;
  };

  IlqrPlannerServer(ros::NodeHandle& nh, int ego_id);
  IlqrPlannerServer(ros::NodeHandle& nh, double work_rate, int ego_id);
  ~IlqrPlannerServer();

  void Init(const std::string& config_path);
  void Start();
  void Stop();

  /**
   * @brief Set semantic map manager
   */
  void set_semantic_map_manager(SemanticMapManager* p_smm);

  /**
   * @brief Push semantic map to processing queue (for integration with behavior planner)
   */
  void PushSemanticMap(const SemanticMapManager& smm);

  /**
   * @brief Get the optimized trajectory
   */
  std::unique_ptr<common::FrenetTrajectory> trajectory() const {
    return planner_.trajectory();
  }

  /**
   * @brief Check if planning is ready
   */
  bool is_ready() const { return is_ready_; }

  /**
   * @brief Get reference to planner
   */
  const IlqrPlanner& planner() const { return planner_; }

 private:
  void PlanningThread();
  void PublishData();
  void VisualizationCallback(const ros::TimerEvent& event);
  void PublishDataCallback(const ros::TimerEvent& event);
  ErrorType FilterSingularityState(const vec_E<common::State>& hist, common::State* filter_state);

  ros::NodeHandle nh_;
  Config config_;
  int ego_id_;
  double work_rate_;

  // Thread management
  std::thread planning_thread_;
  bool is_running_ = false;
  bool is_ready_ = false;
  std::mutex mutex_;

  // Semantic map manager
  SemanticMapManager* p_smm_ = nullptr;
  SscPlannerAdapter map_adapter_;
  SemanticMapManager last_smm_;  ///< Last used semantic map for visualization

  // Planner
  IlqrPlanner planner_;
  IlqrPlannerConfig planner_config_;

  // Visualizer
  std::unique_ptr<IlqrVisualizer> visualizer_;
  semantic_map_manager::Visualizer* p_smm_vis_ = nullptr;
  ros::Timer visualization_timer_;
  ros::Timer publish_data_timer_;

  // Control signal publishing
  ros::Publisher ctrl_signal_pub_;
  ros::Publisher executing_traj_vis_pub_;
  std::unique_ptr<common::FrenetTrajectory> executing_traj_;
  vec_E<common::State> ctrl_state_hist_;
  bool use_sim_state_ = true;
  int last_trajmk_cnt_ = 0;

  // For PushSemanticMap support
  mutable std::mutex smm_mutex_;  ///< Mutex for semantic map access
  SemanticMapManager latest_smm_;  ///< Latest semantic map
  bool has_new_map_ = false;  ///< Flag for new map availability
};

}  // namespace ilqr
}  // namespace planning

#endif  // ILQR_PLANNER_ILQR_SERVER_ROS_H_
