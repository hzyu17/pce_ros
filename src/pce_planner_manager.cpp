/**
 * @file pce_planner_manager.cpp
 * @brief Plugin interface for PCE planner in MoveIt
 */

#include "pce_planner_manager.h"
#include <pluginlib/class_list_macros.hpp>

namespace pce_ros
{

PCEPlannerManager::PCEPlannerManager() 
  : planning_interface::PlannerManager() 
{
}

bool PCEPlannerManager::initialize(const moveit::core::RobotModelConstPtr& model,
                                   const std::string& ns)
{
  robot_model_ = model;
  ns_ = ns;
  
  ros::NodeHandle nh(ns);
  
  if (!PCEPlanner::getConfigData(nh, config_, "pce"))
  {
    ROS_ERROR("PCEPlannerManager: Failed to load configuration from parameter server");
    return false;
  }
  
  // Create persistent visualizer
  VisualizationConfig viz_config;
  XmlRpc::XmlRpcValue pce_config;
  if (nh.getParam("pce", pce_config) && pce_config.hasMember("visualization"))
  {
    viz_config = PCEVisualization::loadConfig(pce_config["visualization"]);
  }
  
  ros::NodeHandle global_nh;
  visualizer_ = std::make_shared<PCEVisualization>(viz_config, global_nh);
  
  ROS_INFO("PCEPlannerManager initialized for %zu planning groups", config_.size());
  
  return true;
}


void PCEPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  algs.clear();
  algs.push_back("PCE");
}

planning_interface::PlanningContextPtr PCEPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
  ROS_INFO("=== PCEPlannerManager::getPlanningContext ===");
  
  if (req.group_name.empty())
  {
    ROS_ERROR("PCEPlannerManager: No planning group specified");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return planning_interface::PlanningContextPtr();
  }

  auto config_it = config_.find(req.group_name);
  if (config_it == config_.end())
  {
    ROS_ERROR("PCEPlannerManager: No configuration found for group '%s'", req.group_name.c_str());
    
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return planning_interface::PlanningContextPtr();
  }


  // Add this to see what's in the config
  ROS_INFO("Found config for group '%s'", req.group_name.c_str());
  const XmlRpc::XmlRpcValue& group_config = config_it->second;
  ROS_INFO("Config type: %d", group_config.getType());
  if (group_config.hasMember("pce_planner"))
  {
    ROS_INFO("Has pce_planner section");
  }
  else
  {
    ROS_WARN("No pce_planner section found!");
  }

  try
  {
    auto planner = std::make_shared<PCEPlanner>(
        req.group_name,
        config_it->second,
        robot_model_,
        visualizer_
    );

    if (!planner->canServiceRequest(req))
    {
      ROS_ERROR("PCEPlannerManager: Cannot service request for group '%s'", req.group_name.c_str());
      error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
      return planning_interface::PlanningContextPtr();
    }

    planner->setPlanningScene(planning_scene);
    planner->setMotionPlanRequest(req);

    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return planner;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("PCEPlannerManager: Failed to create planner: %s", e.what());
    error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return planning_interface::PlanningContextPtr();
  }
}


bool PCEPlannerManager::canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const
{
  return config_.find(req.group_name) != config_.end();
}

}  // namespace pce_ros

PLUGINLIB_EXPORT_CLASS(pce_ros::PCEPlannerManager, planning_interface::PlannerManager)