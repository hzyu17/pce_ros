#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_interface/planning_response.hpp>
#include "pce_planner.h"
#include "visualizer.h"

namespace pce_ros
{

class PCEPlannerManager : public planning_interface::PlannerManager
{
public:
  PCEPlannerManager();
  
  bool initialize(const moveit::core::RobotModelConstPtr& model,
                  const rclcpp::Node::SharedPtr& node,
                  const std::string& ns) override;
  
  std::string getDescription() const override { return "PCE"; }
  
  void getPlanningAlgorithms(std::vector<std::string>& algs) const override;
  
  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr& planning_scene,
      const moveit_msgs::msg::MotionPlanRequest& req,
      moveit_msgs::msg::MoveItErrorCodes& error_code) const override;
  
  bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override;

protected:
  std::string ns_;
  moveit::core::RobotModelConstPtr robot_model_;
  rclcpp::Node::SharedPtr node_;
  std::map<std::string, PCEConfig> config_;
  
  // Persistent visualizer
  std::shared_ptr<PCEVisualization> visualizer_;
  
  friend class PCEPlanner;
};


template<typename T>
T getParam(const rclcpp::Node::SharedPtr& node,
          const std::string& name,
          const T& default_value)
{
  if (!node->has_parameter(name))
  {
  node->declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).get_value<T>();
}


bool getConfigData(const rclcpp::Node::SharedPtr& node,
                  std::map<std::string, PCEConfig>& config,
                  const std::vector<std::string>& planning_groups,
                  const std::string& param)
{
  for (const auto& group : planning_groups)
  {
    PCEConfig group_config;
    std::string base = param + "." + group;
    std::string group_param = base + ".pce_planner";

    group_config.num_iterations = getParam<int>(node, group_param + ".num_iterations", 15);
    group_config.num_samples = getParam<int>(node, group_param + ".num_samples", 3000);
    group_config.temperature = getParam<double>(node, group_param + ".temperature", 1.5);
    group_config.eta = getParam<double>(node, group_param + ".eta", 1.0);
    group_config.num_discretization = getParam<int>(node, group_param + ".num_discretization", 20);
    group_config.total_time = getParam<double>(node, group_param + ".total_time", 5.0);
    group_config.node_collision_radius = getParam<double>(node, group_param + ".node_collision_radius", 0.1);
    // task collision parameters
    group_config.collision_clearance = getParam<double>(node, base + ".collision_clearance", 0.05);
    group_config.collision_threshold = getParam<double>(node, base + ".collision_threshold", 0.07);

    // node -> declare_parameter(group_param + ".num_iterations", 15);
    // node -> declare_parameter(group_param + ".num_samples", 3000);
    // node -> declare_parameter(group_param + ".temperature", 1.5);
    // node -> declare_parameter(group_param + ".eta", 1.0);
    // node -> declare_parameter(group_param + ".num_discretization", 20);
    // node -> declare_parameter(group_param + ".total_time", 5.0);
    // node -> declare_parameter(group_param + ".node_collision_radius", 0.1);
    // node -> declare_parameter(base + ".collision_clearance", 0.05);
    // node -> declare_parameter(base + ".collision_threshold", 0.07);

    // group_config.num_iterations = node -> get_parameter(group_param + ".num_iterations").as_int();
    // group_config.num_samples = node -> get_parameter(group_param + ".num_samples").as_int();
    // group_config.temperature = node -> get_parameter(group_param + ".temperature").as_double();
    // group_config.eta = node -> get_parameter(group_param + ".eta").as_double();
    // group_config.num_discretization = node -> get_parameter(group_param + ".num_discretization").as_int();
    // group_config.total_time = node -> get_parameter(group_param + ".total_time").as_double();
    // group_config.node_collision_radius = node -> get_parameter(group_param + ".node_collision_radius").as_double();

    // // task collision parameters
    // group_config.collision_clearance = node -> get_parameter(base + ".collision_clearance").as_double();
    // group_config.collision_threshold = node -> get_parameter(base + ".collision_threshold").as_double();

    // push into map
    config[group] = group_config;

    RCLCPP_INFO(node->get_logger(), "Loaded PCE config for group '%s'", group.c_str());
  }

  return true;
}

} // namespace pce_ros
