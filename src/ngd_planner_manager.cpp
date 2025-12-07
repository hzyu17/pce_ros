/**
 * @file ngd_planner_manager.cpp
 * @brief NGD planner manager - similar to PCE manager
 */
#include "ngd_planner_manager.h"
#include <pluginlib/class_list_macros.hpp>
#include <cstdlib>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace pce_ros
{

NGDPlannerManager::NGDPlannerManager() 
  : planning_interface::PlannerManager() 
{
}

namespace {
rclcpp::Logger getLogger()
{
    return moveit::getLogger("ngd_planner_manager");
}
}

bool NGDPlannerManager::initialize(const moveit::core::RobotModelConstPtr& model,
                                   const rclcpp::Node::SharedPtr& node,
                                   const std::string& ns)
{
  robot_model_ = model;
  ns_ = ns;
  node_ = node;
  
  // Load NGD configuration
  if (!NGDPlanner::getConfigData(node_, config_, "ngd"))
  {
    RCLCPP_ERROR(getLogger(), "NGDPlannerManager: Failed to load configuration from parameter server");
    return false;
  }
  
  // Create persistent visualizer (reuse NGDVisualization!)
  VisualizationConfig viz_config;
  if (node_ -> has_parameter(ns_ + ".ngd.visualization"))
  {
    viz_config = PCEVisualization::loadConfig(node_, ns_);
  }

  visualizer_ = std::make_shared<PCEVisualization>(viz_config, node_);
  
  RCLCPP_INFO(getLogger(), "NGDPlannerManager initialized for %zu planning groups", config_.size());
  
  return true;
}

void NGDPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  algs.clear();
  algs.push_back("NGD");
}

planning_interface::PlanningContextPtr NGDPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const moveit_msgs::msg::MotionPlanRequest& req,
    moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  RCLCPP_INFO(getLogger(), "=== NGDPlannerManager::getPlanningContext ===");
  
  if (req.group_name.empty())
  {
    RCLCPP_ERROR(getLogger(), "NGDPlannerManager: No planning group specified");
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
    return planning_interface::PlanningContextPtr();
  }

  std::string config_dict = ament_index_cpp::get_package_share_directory("pce_ros") + "/config/ngd_planning.yaml";
  RCLCPP_INFO(getLogger(), "Load config: %s", config_dict.c_str());

  // Read fresh config from parameter server
  if (!NGDPlanner::getConfigData(config_dict, config_))
  {
    RCLCPP_ERROR(getLogger(), "NGDPlannerManager: Failed to reload configuration data from parameter server");
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return planning_interface::PlanningContextPtr();
  }
  
  auto config_it = config_.find(req.group_name);
  if (config_it == config_.end())
  {
    RCLCPP_ERROR(getLogger(), "No configuration for group '%s'", req.group_name.c_str());
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
    return planning_interface::PlanningContextPtr();
  }

  try
  {
    // Create NGD planner (similar pattern to PCE)
    auto planner = std::make_shared<NGDPlanner>(
        req.group_name,
        config_it->second,
        node_,
        robot_model_,
        visualizer_
    );

    if (!planner->canServiceRequest(req))
    {
      RCLCPP_ERROR(getLogger(), "Cannot service request for group '%s'", req.group_name.c_str());
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      return planning_interface::PlanningContextPtr();
    }

    planner->setPlanningScene(planning_scene);
    planner->setMotionPlanRequest(req);

    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return planner;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(getLogger(), "Failed to create planner: %s", e.what());
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return planning_interface::PlanningContextPtr();
  }
}

bool NGDPlannerManager::canServiceRequest(const moveit_msgs::msg::MotionPlanRequest& req) const
{
  return !req.group_name.empty() && (config_.find(req.group_name) != config_.end());
}

}  // namespace pce_ros

PLUGINLIB_EXPORT_CLASS(pce_ros::NGDPlannerManager, planning_interface::PlannerManager)
