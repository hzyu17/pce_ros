/**
 * @file pce_planner.h
 * @brief PCE planner for MoveIt2
 */
#pragma once

#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <pce/PCEMotionPlanner.h>
#include "pce_optimization_task.h"
#include <rclcpp/rclcpp.hpp>

namespace pce_ros
{

/**
 * @class PCEPlanner
 * @brief MoveIt2 PlanningContext implementation for PCE
 * 
 * This class:
 * - Implements MoveIt2's planning_interface::PlanningContext
 * - Wraps your ProximalCrossEntropyMotionPlanner
 * - Converts between MoveIt2 messages and PCE data structures
 */
class PCEPlanner : public planning_interface::PlanningContext
{
public:
  /**
   * @brief Constructor
   * @param group Planning group name
   * @param config PCE configuration parameters
   * @param model Robot model
   * @param node ROS2 node for logging
   * @param visualizer Optional visualizer for trajectory visualization
   */
  PCEPlanner(const std::string& group,
             const PCEConfig& config,
             const rclcpp::Node::SharedPtr& node,
             const moveit::core::RobotModelConstPtr& model,
             std::shared_ptr<PCEVisualization> visualizer = nullptr);
  
  virtual ~PCEPlanner();

  /**
   * @brief Solve the motion planning problem
   * @param res Output: contains the solved trajectory
   * ROS2 MoveIt returns void, result is set in the response structure
   */
  void solve(planning_interface::MotionPlanResponse& res) override;
  
  /**
   * @brief Solve with detailed response
   * @param res Output: detailed response with trajectory
   */
  void solve(planning_interface::MotionPlanDetailedResponse& res) override;
  
  /**
   * @brief Request early termination of planning
   * @return true if termination was successful
   */
  bool terminate() override;
  
  /**
   * @brief Clear previous planning results and reset state
   */
  void clear() override;

  /**
   * @brief Check if this planner can service the request
   */
  bool canServiceRequest(const moveit_msgs::msg::MotionPlanRequest& req) const;

  /**
   * @brief Load configuration from ROS parameter server
   * @param node ROS2 node shared pointer
   * @param config Output: configuration map by group name
   * @param param Parameter name (default: "pce")
   * @return true if successful
   */
  static bool getConfigData(const rclcpp::Node::SharedPtr& node,
                            std::map<std::string, PCEConfig>& config,
                            const std::string& param = "pce");

  static bool getConfigData(const std::string& yaml_dict,
                            std::map<std::string, PCEConfig>& config);
  
  void setVisualizer(std::shared_ptr<PCEVisualization> viz) {
    visualizer_ = viz;
  }


  void setMotionPlanRequest(const planning_interface::MotionPlanRequest& req);

  void setPlanningScene(const planning_scene::PlanningSceneConstPtr& scene);

protected:
  /**
   * @brief Initialize planner components and load configuration
   */
  void setup();
  
  /**
   * @brief Extract start and goal states from motion plan request
   * @param start Output: start joint positions
   * @param goal Output: goal joint positions
   * @return true if extraction was successful
   */
  bool getStartAndGoal(Eigen::VectorXd& start, Eigen::VectorXd& goal);
  
  /**
   * @brief Convert PCE Trajectory to MoveIt2 JointTrajectory
   * @param pce_traj Input: PCE trajectory
   * @param joint_traj Output: MoveIt2 joint trajectory
   * @return true if conversion was successful
   */
  bool pceTrajectoryToJointTrajectory(const Trajectory& pce_traj,
                                      trajectory_msgs::msg::JointTrajectory& joint_traj);

protected:
  // Core PCE components
  std::shared_ptr<ProximalCrossEntropyMotionPlanner> pce_planner_;
  PCEOptimizationTaskPtr pce_task_;

  std::string group_name_;

  // ROS2 node for logging
  rclcpp::Node::SharedPtr node_;
  
  // Configuration from parameters
  PCEConfig pce_config_;
  
  // Robot model from MoveIt2
  moveit::core::RobotModelConstPtr robot_model_;

    // Planning scene for collision checking
  planning_scene::PlanningSceneConstPtr planning_scene_;

  // Visualizer for trajectory display
  std::shared_ptr<PCEVisualization> visualizer_;
};

} // namespace pce_ros