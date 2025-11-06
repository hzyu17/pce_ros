/**
 * @file pce_planner_manager.cpp
 * @brief Plugin interface for PCE planner in MoveIt
 */

#include "pce_planner_manager.h"
#include <pluginlib/class_list_macros.hpp>
#include <cstdlib>

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


  // Reload YAML file into parameter server
  ros::NodeHandle nh(ns_);
  std::string yaml_file;
  if (nh.getParam("pce_config_file", yaml_file))
  {
    ROS_INFO("Reloading YAML: %s -> %s", yaml_file.c_str(), ns_.c_str());
    std::string cmd = "rosparam load " + yaml_file + " " + ns_;
    int result = system(cmd.c_str());
    if (result == 0)
    {
      ROS_INFO("YAML reloaded successfully");
      // Give ROS a moment to update
      ros::Duration(0.1).sleep();
    }
    else
    {
      ROS_WARN("Failed to reload YAML (exit code: %d)", result);
    }
  }
  else
  {
    ROS_WARN("No pce_config_file parameter set, using cached config");
  }


  // Read fresh config from parameter server
  std::map<std::string, XmlRpc::XmlRpcValue> fresh_config;
  if (!PCEPlanner::getConfigData(nh, fresh_config, "pce"))
  {
    ROS_ERROR("Failed to read configuration from parameter server");
    error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return planning_interface::PlanningContextPtr();
  }

  auto config_it = fresh_config.find(req.group_name);
  if (config_it == fresh_config.end())
  {
    ROS_ERROR("No configuration for group '%s'", req.group_name.c_str());
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return planning_interface::PlanningContextPtr();
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
      ROS_ERROR("Cannot service request for group '%s'", req.group_name.c_str());
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
    ROS_ERROR("Failed to create planner: %s", e.what());
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