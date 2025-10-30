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
                                   const rclcpp::Node::SharedPtr& node,
                                   const std::string& ns)
{
  RCLCPP_INFO(node->get_logger(), "=== PCEPlannerManager::initialize ===");
  RCLCPP_INFO(node->get_logger(), "  Namespace: '%s'", ns.c_str());
  
  robot_model_ = model;
  ns_ = ns;
  node_ = node;
  
  std::vector<std::string> planning_groups;

  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = "List of planning groups for PCE planner";
  
  node_->declare_parameter<std::vector<std::string>>("pce.planning_groups", std::vector<std::string>{}, descriptor);
  node_->declare_parameter<std::vector<std::string>>("pce/planning_groups", std::vector<std::string>{}, descriptor);

  bool got = node_->get_parameter("pce.planning_groups", planning_groups);
  if (!got) {
    got = node_->get_parameter("pce/planning_groups", planning_groups);
  }

  if (got)
  {
    RCLCPP_INFO(node_->get_logger(), "  ✓ Found planning_groups with %zu entries:", planning_groups.size());
    for (const auto& group : planning_groups)
    {
      RCLCPP_INFO(node_->get_logger(), "    - %s", group.c_str());
      config_[group] = true;
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "  ✗ Could not find parameter 'pce.planning_groups' (or 'pce/planning_groups')");
  }

  RCLCPP_INFO(node_->get_logger(), "PCEPlannerManager: Initialized with %zu planning group configurations", config_.size());

  // Create persistent visualizer
  VisualizationConfig viz_config;
  viz_config.enable_collision_spheres = true;
  viz_config.enable_trajectory = true;
  viz_config.collision_spheres_topic = "/pce/collision_spheres";
  viz_config.trajectory_topic = "/pce/trajectory";
  viz_config.distance_field_topic = "/pce/distance_field";
  
  visualizer_ = std::make_shared<PCEVisualization>(viz_config, node_);

  RCLCPP_INFO(node_->get_logger(), "PCEPlannerManager: Created persistent visualizer");

  return true;
}

void PCEPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  algs.clear();
  algs.push_back("PCE");
}

planning_interface::PlanningContextPtr PCEPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const moveit_msgs::msg::MotionPlanRequest& req,
    moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  RCLCPP_INFO(node_->get_logger(), "=== PCEPlannerManager::getPlanningContext ===");

  if (req.group_name.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "PCEPlannerManager: No planning group specified");
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
    return planning_interface::PlanningContextPtr();
  }

  auto config_it = config_.find(req.group_name);
  if (config_it == config_.end())
  {
    RCLCPP_ERROR(node_->get_logger(), "PCEPlannerManager: No configuration found for group '%s'", req.group_name.c_str());
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
    return planning_interface::PlanningContextPtr();
  }


  // Add this to see what's in the config
  RCLCPP_INFO(node_->get_logger(), "Found config for group '%s'", req.group_name.c_str());

  try
  {
    std::map<std::string, std::string> planner_config;
    auto planner = std::make_shared<PCEPlanner>(
      req.group_name, planner_config, robot_model_, node_, visualizer_);

    if (!planner->canServiceRequest(req))
    {
      RCLCPP_ERROR(node_->get_logger(), "PCEPlannerManager: Cannot service request for group '%s'", req.group_name.c_str());
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
    RCLCPP_ERROR(node_->get_logger(), "PCEPlannerManager: Failed to create planner: %s", e.what());
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return planning_interface::PlanningContextPtr();
  }
}


bool PCEPlannerManager::canServiceRequest(const moveit_msgs::msg::MotionPlanRequest& req) const
{
  return !req.group_name.empty() && (config_.find(req.group_name) != config_.end());
}

}  // namespace pce_ros

PLUGINLIB_EXPORT_CLASS(pce_ros::PCEPlannerManager, planning_interface::PlannerManager)