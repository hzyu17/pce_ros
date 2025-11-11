/**
 * @file ngd_planner.h
 * @brief NGD planner for MoveIt - reuses PCEOptimizationTask infrastructure
 */
#pragma once
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <pce/NGDMotionPlanner.h>
#include "pce_optimization_task.h"
#include "visualizer.h"
#include <ros/ros.h>

namespace pce_ros
{
/**
 * @class NGDPlanner
 * @brief MoveIt PlanningContext implementation for NGD
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
   * @param config NGD configuration from YAML
   * @param model Robot model
   * @param visualizer Shared visualizer (optional)
   */
  NGDPlanner(const std::string& group,
             const XmlRpc::XmlRpcValue& config,
             const moveit::core::RobotModelConstPtr& model,
             std::shared_ptr<PCEVisualization> visualizer = nullptr);
  
  virtual ~NGDPlanner();

  // PlanningContext interface
  bool solve(planning_interface::MotionPlanResponse& res) override;
  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;
  bool terminate() override;
  void clear() override;

  /**
   * @brief Check if this planner can service the request
   */
  bool canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const;

  /**
   * @brief Load configuration from ROS parameter server
   * @param nh Node handle
   * @param config Output: configuration map by group name
   * @param param Parameter name (default: "ngd")
   * @return true if successful
   */
  static bool getConfigData(ros::NodeHandle& nh,
                            std::map<std::string, XmlRpc::XmlRpcValue>& config,
                            std::string param = "ngd");

  void setMotionPlanRequest(const planning_interface::MotionPlanRequest& req);
  void setPlanningScene(const planning_scene::PlanningSceneConstPtr& scene);

protected:
  void setup();
  bool getStartAndGoal(Eigen::VectorXd& start, Eigen::VectorXd& goal);
  bool ngdTrajectoryToJointTrajectory(const Trajectory& ngd_traj,
                                      trajectory_msgs::JointTrajectory& joint_traj);

protected:
  // NGD components
  std::shared_ptr<NGDMotionPlanner> ngd_planner_;
  PCEOptimizationTaskPtr optimization_task_;
  std::string group_name_;
  
  // Configuration
  XmlRpc::XmlRpcValue config_;
  NGDConfig ngd_config_;
  
  // Robot model
  moveit::core::RobotModelConstPtr robot_model_;
  
  // ROS
  ros::NodeHandlePtr nh_;
  std::shared_ptr<PCEVisualization> visualizer_;
};

} // namespace pce_ros