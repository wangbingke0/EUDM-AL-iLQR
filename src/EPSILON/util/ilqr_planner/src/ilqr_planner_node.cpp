/**
 * @file ilqr_planner_node.cpp
 * @brief ROS node for iLQR planner
 */

#include <ros/ros.h>
#include <glog/logging.h>

#include "ilqr_planner/ilqr_server_ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ilqr_planner_node");
  ros::NodeHandle nh("~");
  
  // Note: If running via test_ilqr_with_eudm, glog is initialized by EudmManager::Init()
  // Log level settings are handled in IlqrPlannerServer::Init()
  // Only initialize glog here if running ilqr_planner_node directly
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  
  // Parameters
  int ego_id;
  double work_rate;
  std::string config_path;
  
  nh.param<int>("ego_id", ego_id, 0);
  nh.param<double>("work_rate", work_rate, 10.0);
  nh.param<std::string>("config_path", config_path, "");
  
  LOG(INFO) << "===============================================";
  LOG(INFO) << "[IlqrPlannerNode] Starting with:";
  LOG(INFO) << "  ego_id: " << ego_id;
  LOG(INFO) << "  work_rate: " << work_rate << " Hz";
  LOG(INFO) << "  config_path: " << config_path;
  LOG(INFO) << "===============================================";
  
  // Create and initialize server
  planning::ilqr::IlqrPlannerServer server(nh, work_rate, ego_id);
  server.Init(config_path);
  
  // Note: The semantic map manager should be set externally
  // This is typically done by the planning_integrated node
  
  // Start planning
  // server.Start();
  
  ros::spin();
  
  return 0;
}

