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
  ROS_INFO("=== PCEPlannerManager::initialize ===");
  ROS_INFO("  Namespace: '%s'", ns.c_str());
  
  robot_model_ = model;
  ns_ = ns;
  
  ros::NodeHandle nh(ns);
  ROS_INFO("  NodeHandle namespace: %s", nh.getNamespace().c_str());
  
  std::string param_name = "pce";
  ROS_INFO("  Looking for config at: %s/%s", nh.getNamespace().c_str(), param_name.c_str());
  
  std::vector<std::string> planning_groups;
  if (nh.getParam("pce/planning_groups", planning_groups))
  {
    ROS_INFO("  ✓ Found planning_groups with %zu entries:", planning_groups.size());
    for (const auto& group : planning_groups)
    {
      ROS_INFO("    - %s", group.c_str());
    }
  }
  else
  {
    ROS_ERROR("  ✗ Could not find pce/planning_groups!");
  }
  
  if (!PCEPlanner::getConfigData(nh, config_, param_name))
  {
    ROS_ERROR("PCEPlannerManager: Failed to load configuration");
    return false;
  }
  
  ROS_INFO("PCEPlannerManager: Initialized with %zu planning group configurations", config_.size());
  
  // Create persistent visualizer
  VisualizationConfig viz_config;
  viz_config.enable_collision_spheres = true;
  viz_config.enable_trajectory = true;
  viz_config.collision_spheres_topic = "/pce/collision_spheres";
  viz_config.trajectory_topic = "/pce/trajectory";
  viz_config.distance_field_topic = "/pce/distance_field";
  
  ros::NodeHandle global_nh;
  visualizer_ = std::make_shared<PCEVisualization>(viz_config, global_nh);
  
  ROS_INFO("PCEPlannerManager: Created persistent visualizer");
  
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