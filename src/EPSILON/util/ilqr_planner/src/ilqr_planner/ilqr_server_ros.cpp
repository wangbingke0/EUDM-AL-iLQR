/**
 * @file ilqr_server_ros.cpp
 * @brief Implementation of ROS server for iLQR planner
 */

#include "ilqr_planner/ilqr_server_ros.h"

#include <glog/logging.h>
#include <glog/raw_logging.h>
#include <string>
#include <cstring>

namespace planning {
namespace ilqr {

IlqrPlannerServer::IlqrPlannerServer(ros::NodeHandle& nh, int ego_id)
    : nh_(nh), ego_id_(ego_id), work_rate_(20.0) {
  config_.planning_rate = 20;
  config_.enable_visualization = true;
  p_smm_vis_ = new semantic_map_manager::Visualizer(nh, ego_id);
}

IlqrPlannerServer::IlqrPlannerServer(ros::NodeHandle& nh, double work_rate, int ego_id)
    : nh_(nh), ego_id_(ego_id), work_rate_(work_rate) {
  config_.planning_rate = static_cast<int>(work_rate);
  config_.enable_visualization = true;
  p_smm_vis_ = new semantic_map_manager::Visualizer(nh, ego_id);
}

IlqrPlannerServer::~IlqrPlannerServer() {
  Stop();
  if (p_smm_vis_) delete p_smm_vis_;
}

void IlqrPlannerServer::Init(const std::string& config_path) {
  printf("[IlqrPlannerServer] Initializing with config: %s\n", config_path.c_str());
  
  // Re-enable glog for IlqrPlanner (EudmManager may have disabled it)
  // This ensures IlqrPlanner's LOG statements are visible
  FLAGS_logtostderr = 1;
  FLAGS_alsologtostderr = 0;
  FLAGS_colorlogtostderr = 1;
  FLAGS_minloglevel = 0;  // 0=INFO, allow INFO/WARNING/ERROR logs
  FLAGS_stderrthreshold = 0;  // Output INFO and above to stderr
  FLAGS_log_dir = "";
  
  // Set log destinations to stderr
  google::SetLogDestination(google::GLOG_INFO, "");
  google::SetLogDestination(google::GLOG_WARNING, "");
  google::SetLogDestination(google::GLOG_ERROR, "");
  google::SetLogDestination(google::GLOG_FATAL, "");
  
  // Configure planner
  planner_config_.time_horizon = 5.0;
  planner_config_.dt = 0.1;
  planner_config_.num_knot_points = 50;
  
  // Cost weights
  planner_config_.weight_x = 8.0;
  planner_config_.weight_y = 12.0;
  planner_config_.weight_yaw = 1.0;
  planner_config_.weight_v = 1.0;
  planner_config_.weight_a = 0.1;
  planner_config_.weight_kappa = 2.0;
  planner_config_.weight_jerk = 0.02;
  planner_config_.weight_dkappa = 1.2;
  
  // Terminal weights
  planner_config_.weight_terminal_x = 100.0;
  planner_config_.weight_terminal_y = 100.0;
  planner_config_.weight_terminal_yaw = 20.0;
  planner_config_.weight_terminal_v = 10.0;
  planner_config_.weight_terminal_a = 2.0;
  planner_config_.weight_terminal_kappa = 8.0;

  // Dynamic limits for smoother lateral behavior
  planner_config_.max_dkappa = 0.20;
  
  // Solver settings
  planner_config_.max_outer_iterations = 30;  // Increased from 20
  planner_config_.max_inner_iterations = 150;  // Increased from 100
  planner_config_.constraint_tolerance = 1e-3;  // Relaxed from 1e-4 for better convergence
  planner_config_.cost_tolerance = 1e-3;  // Relaxed from 1e-4
  
  planner_.set_config(planner_config_);
  planner_.Init(config_path);
  
  // Set up map interface
  planner_.set_map_interface(&map_adapter_);
  
  // Initialize control signal publisher
  nh_.param("use_sim_state", use_sim_state_, true);
  ctrl_signal_pub_ = nh_.advertise<vehicle_msgs::ControlSignal>("ctrl", 20);
  
  std::string exec_traj_topic = std::string("/vis/agent_") +
                                std::to_string(ego_id_) +
                                std::string("/ilqr/exec_traj");
  executing_traj_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(exec_traj_topic, 1);
  
  // Initialize visualizer
  if (config_.enable_visualization) {
    std::string vis_topic_prefix = std::string("/vis/agent_") +
                                   std::to_string(ego_id_) +
                                   std::string("/ilqr");
    visualizer_ = std::unique_ptr<IlqrVisualizer>(
        new IlqrVisualizer(nh_, "map", vis_topic_prefix));
    visualization_timer_ = nh_.createTimer(
        ros::Duration(0.05), &IlqrPlannerServer::VisualizationCallback, this);
  }
  
  // Data publish timer (same as SSC)
  publish_data_timer_ = nh_.createTimer(
      ros::Duration(1.0 / work_rate_), &IlqrPlannerServer::PublishDataCallback, this);
  
  printf("[IlqrPlannerServer] Initialization complete. work_rate=%.1f Hz\n", work_rate_);
}

void IlqrPlannerServer::set_semantic_map_manager(SemanticMapManager* p_smm) {
  p_smm_ = p_smm;
}

void IlqrPlannerServer::PushSemanticMap(const SemanticMapManager& smm) {
  std::lock_guard<std::mutex> lock(smm_mutex_);
  latest_smm_ = smm;
  has_new_map_ = true;
}

void IlqrPlannerServer::Start() {
  if (is_running_) {
    printf("[IlqrPlannerServer] Already running\n");
    return;
  }
  
  is_running_ = true;
  planning_thread_ = std::thread(&IlqrPlannerServer::PlanningThread, this);
  
  printf("[IlqrPlannerServer] Started planning thread at %.1f Hz\n", work_rate_);
}

void IlqrPlannerServer::Stop() {
  is_running_ = false;
  
  if (planning_thread_.joinable()) {
    planning_thread_.join();
  }
  
  printf("[IlqrPlannerServer] Stopped\n");
}

void IlqrPlannerServer::PlanningThread() {
  ros::Rate rate(work_rate_);
  
  printf("[IlqrPlannerServer] Planning thread running\n");
  
  while (ros::ok() && is_running_) {
    // Check if we have a new semantic map from PushSemanticMap
    SemanticMapManager current_smm;
    bool have_map = false;
    
    {
      std::lock_guard<std::mutex> lock(smm_mutex_);
      if (has_new_map_) {
        current_smm = latest_smm_;
        have_map = true;
      }
    }
    
    // If no map from callback, try p_smm_ pointer
    if (!have_map && p_smm_ != nullptr) {
      current_smm = *p_smm_;
      have_map = true;
    }
    
    if (!have_map) {
      rate.sleep();
      continue;
    }
    
    // Save for visualization
    last_smm_ = current_smm;
    
    // Create a snapshot of the semantic map for the adapter
    auto map_ptr = std::make_shared<SemanticMapManager>(current_smm);
    map_adapter_.set_map(map_ptr);
    
    // Run planning
    {
      std::lock_guard<std::mutex> lock(mutex_);
      
      auto start_time = ros::Time::now();
      if (planner_.RunOnce() == kSuccess) {
        is_ready_ = true;
        double time_cost = (ros::Time::now() - start_time).toSec() * 1000.0;
        printf("[IlqrPlannerServer] Planning succeeded in %.3f ms\n", time_cost);
        
        // Update executing trajectory
        executing_traj_ = planner_.trajectory();
      } else {
        printf("[IlqrPlannerServer] Planning failed\n");
      }
    }
    
    rate.sleep();
  }
}

void IlqrPlannerServer::PublishDataCallback(const ros::TimerEvent& event) {
  PublishData();
}

void IlqrPlannerServer::PublishData() {
  using common::VisualizationUtil;
  auto current_time = ros::Time::now().toSec();
  
  // SMM visualization
  if (p_smm_vis_ && has_new_map_) {
    p_smm_vis_->VisualizeDataWithStamp(ros::Time(current_time), last_smm_);
    p_smm_vis_->SendTfWithStamp(ros::Time(current_time), last_smm_);
  }
  
  // Trajectory feedback and control signal
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (executing_traj_ == nullptr || !executing_traj_->IsValid()) {
      return;
    }
    
    if (use_sim_state_) {
      printf("[IlqrPlannerServer] use_sim_state_: true.\n");
      
      decimal_t plan_horizon = 1.0 / work_rate_;
      int num_cycles = std::floor((current_time - executing_traj_->begin()) / plan_horizon);
      decimal_t ct = executing_traj_->begin() + num_cycles * plan_horizon;
      
      common::State state;
      if (executing_traj_->GetState(ct, &state) == kSuccess) {
        FilterSingularityState(ctrl_state_hist_, &state);
        ctrl_state_hist_.push_back(state);
        if (ctrl_state_hist_.size() > 100)
          ctrl_state_hist_.erase(ctrl_state_hist_.begin());
        
        vehicle_msgs::ControlSignal ctrl_msg;
        vehicle_msgs::Encoder::GetRosControlSignalFromControlSignal(
            common::VehicleControlSignal(state), ros::Time(ct),
            std::string("map"), &ctrl_msg);
        ctrl_signal_pub_.publish(ctrl_msg);
        
        printf("[IlqrPlannerServer] Published ctrl signal: t=%.2f, x=%.2f, y=%.2f, v=%.2f\n",
               ct, state.vec_position[0], state.vec_position[1], state.velocity);
      } else {
        printf("[IlqrPlannerServer] Cannot evaluate state at %lf with begin %lf.\n",
               ct, executing_traj_->begin());
      }
    }
    
    // Trajectory visualization
    {
      auto color = common::cmap["cyan"];
      visualization_msgs::MarkerArray traj_mk_arr;
      VisualizationUtil::GetMarkerArrayByTrajectory(
          *executing_traj_, 0.1, Vecf<3>(0.3, 0.3, 0.3), color, 0.5,
          &traj_mk_arr);
      
      int num_traj_mks = static_cast<int>(traj_mk_arr.markers.size());
      VisualizationUtil::FillHeaderIdInMarkerArray(
          ros::Time(current_time), std::string("map"), last_trajmk_cnt_,
          &traj_mk_arr);
      last_trajmk_cnt_ = num_traj_mks;
      executing_traj_vis_pub_.publish(traj_mk_arr);
    }
  }
}

ErrorType IlqrPlannerServer::FilterSingularityState(
    const vec_E<common::State>& hist, common::State* filter_state) {
  if (hist.empty()) {
    return kWrongStatus;
  }
  decimal_t duration = filter_state->time_stamp - hist.back().time_stamp;
  decimal_t wheel_base = 2.85;
  decimal_t max_steer = M_PI / 4.0;
  decimal_t singular_velocity = kBigEPS;
  decimal_t max_orientation_rate = tan(max_steer) / wheel_base * singular_velocity;
  decimal_t max_orientation_change = max_orientation_rate * duration;

  if (fabs(filter_state->velocity) < singular_velocity &&
      fabs(normalize_angle(filter_state->angle - hist.back().angle)) >
          max_orientation_change) {
    printf("[IlqrPlannerServer] Detect singularity velocity %lf angle (%lf, %lf).\n",
           filter_state->velocity, hist.back().angle, filter_state->angle);
    filter_state->angle = hist.back().angle;
    printf("[IlqrPlannerServer] Filter angle to %lf.\n", hist.back().angle);
  }
  return kSuccess;
}

void IlqrPlannerServer::VisualizationCallback(const ros::TimerEvent& event) {
  if (!is_ready_ || visualizer_ == nullptr) {
    return;
  }
  
  std::lock_guard<std::mutex> lock(mutex_);
  visualizer_->Visualize(planner_, ros::Time::now());
}

}  // namespace ilqr
}  // namespace planning
