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

  ~PCEPlannerManager()
  {
    visualizer_.reset();
    config_.clear();
  }
  
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

  // TODO: Use setPlannerConfigurations to load configs from MoveIt
protected:
  std::string ns_;
  moveit::core::RobotModelConstPtr robot_model_;
  rclcpp::Node::SharedPtr node_;
  mutable std::map<std::string, PCEConfig> config_;
  
  // Persistent visualizer
  std::shared_ptr<PCEVisualization> visualizer_;
  
  friend class PCEPlanner;
};


} // namespace pce_ros
