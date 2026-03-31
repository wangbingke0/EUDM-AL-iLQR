/**
 * @file test_ilqr_with_eudm.cc
 * @author HKUST Aerial Robotics Group
 * @brief test iLQR planner with eudm behavior planner
 * @version 0.1
 * @date 2024-12
 * @copyright Copyright (c) 2024
 */
#include <glog/logging.h>
#include <ros/ros.h>
#include <stdlib.h>

#include <chrono>
#include <iostream>

#include "eudm_planner/eudm_server_ros.h"
#include "semantic_map_manager/data_renderer.h"
#include "semantic_map_manager/ros_adapter.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"
#include "ilqr_planner/ilqr_server_ros.h"

DECLARE_BACKWARD;
double ilqr_planner_work_rate = 20.0;
double bp_work_rate = 20.0;

planning::ilqr::IlqrPlannerServer* p_ilqr_server_{nullptr};
planning::EudmPlannerServer* p_bp_server_{nullptr};

int BehaviorUpdateCallback(
    const semantic_map_manager::SemanticMapManager& smm) {
  if (p_ilqr_server_) p_ilqr_server_->PushSemanticMap(smm);
  return 0;
}

int SemanticMapUpdateCallback(
    const semantic_map_manager::SemanticMapManager& smm) {
  if (p_bp_server_) p_bp_server_->PushSemanticMap(smm);
  return 0;
}

int main(int argc, char** argv) {
  // Note: glog will be initialized by EudmManager::Init(), so we don't initialize it here
  // to avoid "InitGoogleLogging() called twice" error
  
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  int ego_id;
  if (!nh.getParam("ego_id", ego_id)) {
    ROS_ERROR("Failed to get param %d", ego_id);
    assert(false);
  }
  std::string agent_config_path;
  if (!nh.getParam("agent_config_path", agent_config_path)) {
    ROS_ERROR("Failed to get param agent_config_path %s",
              agent_config_path.c_str());
    assert(false);
  }

  std::string bp_config_path;
  if (!nh.getParam("bp_config_path", bp_config_path)) {
    ROS_ERROR("Failed to get param bp_config_path %s", bp_config_path.c_str());
    assert(false);
  }

  std::string ilqr_config_path;
  if (!nh.getParam("ilqr_config_path", ilqr_config_path)) {
    ROS_WARN("Failed to get param ilqr_config_path, using default config");
    ilqr_config_path = "";  // Use default config
  }

  semantic_map_manager::SemanticMapManager semantic_map_manager(
      ego_id, agent_config_path);
  semantic_map_manager::RosAdapter smm_ros_adapter(nh, &semantic_map_manager);
  smm_ros_adapter.BindMapUpdateCallback(SemanticMapUpdateCallback);

  double desired_vel;
  nh.param("desired_vel", desired_vel, 6.0);
  
  // Declare behavior planner
  p_bp_server_ = new planning::EudmPlannerServer(nh, bp_work_rate, ego_id);
  p_bp_server_->set_user_desired_velocity(desired_vel);
  p_bp_server_->BindBehaviorUpdateCallback(BehaviorUpdateCallback);

  // Declare iLQR planner
  p_ilqr_server_ =
      new planning::ilqr::IlqrPlannerServer(nh, ilqr_planner_work_rate, ego_id);

  p_bp_server_->Init(bp_config_path);
  p_ilqr_server_->Init(ilqr_config_path);
  smm_ros_adapter.Init();

  p_bp_server_->Start();
  p_ilqr_server_->Start();

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

