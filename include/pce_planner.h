/**
 * @file pce_planner.h
 * @brief PCE planner for MoveIt
 */
#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <pce/PCEMotionPlanner.h>
#include "pce_optimization_task.h"
#include <ros/ros.h>

namespace pce_ros
{

/**
 * @class PCEPlanner
 * @brief MoveIt PlanningContext implementation for PCE
 * 
 * This class:
 * - Implements MoveIt's planning_interface::PlanningContext
 * - Wraps your ProximalCrossEntropyMotionPlanner
 * - Converts between MoveIt messages and PCE data structures
 */
class PCEPlanner : public planning_interface::PlanningContext
{
public:
  /**
   * @brief Constructor
   * @param group Planning group name
   * @param config PCE configuration from YAML
   * @param model Robot model
   */
  PCEPlanner(const std::string& group,
             const XmlRpc::XmlRpcValue& config,
             const moveit::core::RobotModelConstPtr& model,
             std::shared_ptr<PCEVisualization> visualizer = nullptr);
  
  virtual ~PCEPlanner();

  /**
   * @brief Solve the motion planning problem
   * @param res Output: contains the solved trajectory
   * @return true if planning succeeded
   */
  bool solve(planning_interface::MotionPlanResponse& res) override;
  
  /**
   * @brief Solve with detailed response
   */
  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;
  
  /**
   * @brief Request early termination
   */
  bool terminate() override;
  
  /**
   * @brief Clear previous results
   */
  void clear() override;
  
  /**
   * @brief Check if this planner can service the request
   */
  bool canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const;
  
  /**
   * @brief Load configuration from ROS parameter server
   * @param nh Node handle
   * @param config Output: configuration map by group name
   * @param param Parameter name (default: "pce")
   * @return true if successful
   */
  static bool getConfigData(ros::NodeHandle& nh,
                            std::map<std::string, XmlRpc::XmlRpcValue>& config,
                            std::string param = "pce");

  void setVisualizer(std::shared_ptr<PCEVisualization> viz) {
    visualizer_ = viz;
  }


  void setMotionPlanRequest(const planning_interface::MotionPlanRequest& req);

  void setPlanningScene(const planning_scene::PlanningSceneConstPtr& scene);

protected:
  // Setup
  void setup();
  
  // Convert between MoveIt and PCE representations
  bool getStartAndGoal(Eigen::VectorXd& start, Eigen::VectorXd& goal);
  bool parametersToJointTrajectory(const Eigen::MatrixXd& parameters,
                                   trajectory_msgs::JointTrajectory& traj);
  
  // Convert your Trajectory type to MoveIt trajectory
  bool pceTrajectoryToJointTrajectory(const Trajectory& pce_traj,
                                      trajectory_msgs::JointTrajectory& joint_traj);

protected:
  // PCE components
  std::shared_ptr<ProximalCrossEntropyMotionPlanner> pce_planner_;
  PCEOptimizationTaskPtr pce_task_;

  std::string group_name_;
  
  // Configuration
  XmlRpc::XmlRpcValue config_;
  PCEConfig pce_config_;
  
  // Robot model
  moveit::core::RobotModelConstPtr robot_model_;
  
  // ROS
  ros::NodeHandlePtr nh_;

  std::shared_ptr<PCEVisualization> visualizer_;
};

} // namespace pce_ros