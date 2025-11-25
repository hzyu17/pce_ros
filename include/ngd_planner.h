/**
 * @file ngd_planner.h
 * @brief NGD planner for MoveIt - reuses PCEOptimizationTask infrastructure
 */
#pragma once

#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <pce/NGDMotionPlanner.h>
#include "pce_optimization_task.h"
#include <rclcpp/rclcpp.hpp>

namespace pce_ros
{
/**
 * @class NGDPlanner
 * @brief MoveIt2 PlanningContext implementation for NGD
 * 
 * This class wraps NGDMotionPlanner and reuses the existing
 * PCEOptimizationTask for collision checking and visualization.
 */
class NGDPlanner : public planning_interface::PlanningContext
{
public:
  /**
   * @brief Constructor
   * @param group Planning group name
   * @param config NGD configuration parameters
   * @param model Robot model
   * @param node ROS2 node for logging
   * @param visualizer Optional visualizer for trajectory visualization
   */
  NGDPlanner(const std::string& group,
             const NGDConfig& config,
             const rclcpp::Node::SharedPtr& node,
             const moveit::core::RobotModelConstPtr& model,
             std::shared_ptr<PCEVisualization> visualizer = nullptr);
  
  virtual ~NGDPlanner();

  // PlanningContext interface
  void solve(planning_interface::MotionPlanResponse& res) override;
  void solve(planning_interface::MotionPlanDetailedResponse& res) override;
  bool terminate() override;
  void clear() override;

  /**
   * @brief Check if this planner can service the request
   */
  bool canServiceRequest(const moveit_msgs::msg::MotionPlanRequest& req) const;

  /**
   * @brief Load configuration from ROS parameter server
   * @param node ROS2 node shared pointer
   * @param config Output: configuration map by group name
   * @param param Parameter name (default: "ngd")
   * @return true if successful
   */
  static bool getConfigData(const rclcpp::Node::SharedPtr& node,
                            std::map<std::string, NGDConfig>& config,
                            const std::string& param = "ngd");

  static bool getConfigData(const std::string& yaml_dict,
                            std::map<std::string, NGDConfig>& config);

  void setMotionPlanRequest(const planning_interface::MotionPlanRequest& req);
  void setPlanningScene(const planning_scene::PlanningSceneConstPtr& scene);

protected:
  void setup();
  bool getStartAndGoal(Eigen::VectorXd& start, Eigen::VectorXd& goal);
  bool ngdTrajectoryToJointTrajectory(const Trajectory& pce_traj,
                                      trajectory_msgs::msg::JointTrajectory& joint_traj);

protected:
  // NGD components
  std::shared_ptr<NGDMotionPlanner> ngd_planner_;
  PCEOptimizationTaskPtr optimization_task_;

  std::string group_name_;
  
  // ROS2 node for logging
  rclcpp::Node::SharedPtr node_;

  // Configuration from parameters
  NGDConfig ngd_config_;
  
  // Robot model
  moveit::core::RobotModelConstPtr robot_model_;
  
  // ROS
  std::shared_ptr<PCEVisualization> visualizer_;
};

} // namespace pce_ros