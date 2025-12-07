/**
 * @file ngd_planner_manager.h
 * @brief NGD planner manager - reuses PCE infrastructure
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_interface/planning_response.hpp>
#include "ngd_planner.h"
#include "visualizer.h"

namespace pce_ros
{

/**
 * @class NGDPlannerManager
 * @brief MoveIt PlannerManager for NGD - reuses PCE infrastructure
 */
class NGDPlannerManager : public planning_interface::PlannerManager
{
public:
  NGDPlannerManager();
  
  ~NGDPlannerManager()
  {
    visualizer_.reset();
    config_.clear();
  }
  
  bool initialize(const moveit::core::RobotModelConstPtr& model,
                  const rclcpp::Node::SharedPtr& node,
                  const std::string& ns) override;
  
  std::string getDescription() const override { return "NGD"; }
  
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
  mutable std::map<std::string, NGDConfig> config_;
  
  // Shared visualizer (reuse PCEVisualization!)
  std::shared_ptr<PCEVisualization> visualizer_;
  
  friend class NGDPlanner;
};

} // namespace pce_ros