/**
 * @file ngd_planner_manager.cpp
 * @brief NGD planner manager - similar to PCE manager
 */
#include "ngd_planner_manager.h"
#include <pluginlib/class_list_macros.hpp>
#include <cstdlib>

namespace pce_ros
{

NGDPlannerManager::NGDPlannerManager() 
  : planning_interface::PlannerManager() 
{
}

bool NGDPlannerManager::initialize(const moveit::core::RobotModelConstPtr& model,
                                   const std::string& ns)
{
  robot_model_ = model;
  ns_ = ns;
  
  ros::NodeHandle nh(ns);
  
  // Load NGD configuration
  if (!NGDPlanner::getConfigData(nh, config_, "ngd"))
  {
    ROS_ERROR("NGDPlannerManager: Failed to load configuration from parameter server");
    return false;
  }
  
  // Create persistent visualizer (reuse PCEVisualization!)
  VisualizationConfig viz_config;
  XmlRpc::XmlRpcValue ngd_config;
  if (nh.getParam("ngd", ngd_config) && ngd_config.hasMember("visualization"))
  {
    viz_config = PCEVisualization::loadConfig(ngd_config["visualization"]);
  }
  
  ros::NodeHandle global_nh;
  visualizer_ = std::make_shared<PCEVisualization>(viz_config, global_nh);
  
  ROS_INFO("NGDPlannerManager initialized for %zu planning groups", config_.size());
  
  return true;
}

void NGDPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  algs.clear();
  algs.push_back("NGD");
}

planning_interface::PlanningContextPtr NGDPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
  ROS_INFO("=== NGDPlannerManager::getPlanningContext ===");
  
  if (req.group_name.empty())
  {
    ROS_ERROR("NGDPlannerManager: No planning group specified");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return planning_interface::PlanningContextPtr();
  }

  // Reload YAML file (optional - same pattern as PCE)
  ros::NodeHandle nh(ns_);
  std::string yaml_file;
  if (nh.getParam("ngd_config_file", yaml_file))
  {
    std::string cmd = "rosparam load " + yaml_file + " " + ns_;
    int result = system(cmd.c_str());
    if (result == 0)
    {
      ROS_INFO("YAML reloaded successfully");
      ros::Duration(0.1).sleep();
    }
    else
    {
      ROS_WARN("Failed to reload YAML (exit code: %d)", result);
    }
  }

  // Read fresh config from parameter server
  std::map<std::string, XmlRpc::XmlRpcValue> fresh_config;
  if (!NGDPlanner::getConfigData(nh, fresh_config, "ngd"))
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
    // Create NGD planner (similar pattern to PCE)
    auto planner = std::make_shared<NGDPlanner>(
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

bool NGDPlannerManager::canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const
{
  return config_.find(req.group_name) != config_.end();
}

}  // namespace pce_ros

PLUGINLIB_EXPORT_CLASS(pce_ros::NGDPlannerManager, planning_interface::PlannerManager)
