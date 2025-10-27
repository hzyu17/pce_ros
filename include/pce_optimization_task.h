/**
 * @file pce_optimization_task.h
 * @brief PCE optimization task for MoveIt integration
 */
#pragma once

#include <ros/ros.h>
#include <memory>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <pce/PCEMotionPlanner.h>
#include <pce/Trajectory.h>
#include <pce/task.h>
#include <pluginlib/class_loader.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <eigen_stl_containers/eigen_stl_containers.h>
#include <visualization_msgs/MarkerArray.h>
#include "visualizer.h"
#include <moveit/collision_detection/collision_tools.h>


// Forward declarations for plugin types (you can adapt STOMP's or create your own)
namespace pce_ros
{

// Plugin loaders (reuse STOMP's plugins or create PCE-specific ones)
namespace cost_functions {
  class PCECostFunction;
  typedef std::shared_ptr<PCECostFunction> PCECostFunctionPtr;
}

namespace filters {
  class PCEFilter;
  typedef std::shared_ptr<PCEFilter> PCEFilterPtr;
}

/**
 * @class PCEOptimizationTask
 * @brief Loads and manages PCE plugins during planning
 * 
 * This class:
 * - Implements your pce::Task interface
 * - Manages MoveIt planning scene and robot model
 * - Loads cost function plugins
 * - Handles trajectory filtering for constraints
 */
class PCEOptimizationTask : public pce::Task
{
public:
  /**
   * @brief Constructor
   * @param robot_model_ptr Pointer to MoveIt robot model
   * @param group_name Planning group name
   * @param config Plugin configuration (YAML/XmlRpc)
   */
  PCEOptimizationTask(
      moveit::core::RobotModelConstPtr robot_model_ptr,
      const std::string& group_name,
      const XmlRpc::XmlRpcValue& config);
  
  virtual ~PCEOptimizationTask();


  void setVisualizer(std::shared_ptr<PCEVisualization> viz)
  {
    visualizer_ = viz;
    ROS_INFO("PCEOptimizationTask: Visualizer set");
  }

  /**
   * @brief Set the motion planning request
   * @param planning_scene Planning scene with collision world
   * @param req Motion planning request
   * @param error_code Output error code
   * @return true if successful
   */
  bool setMotionPlanRequest(
      const planning_scene::PlanningSceneConstPtr& planning_scene,
      const moveit_msgs::MotionPlanRequest& req,
      moveit_msgs::MoveItErrorCodes& error_code);

  // Implement pce::Task interface
  float computeCollisionCost(const Trajectory& trajectory) const override;

  bool filterTrajectory(Trajectory& trajectory, int iteration_number) override;
  
  void postIteration(int iteration_number, float cost, 
                     const Trajectory& trajectory) override;
  
  void done(bool success, int total_iterations, 
            float final_cost, const Trajectory& trajectory) override;
  
  void initialize(size_t num_dimensions, const PathNode& start,
                  const PathNode& goal, size_t num_nodes, 
                  float total_time) override;

  // Set planning scene and initialize distance field
  void setPlanningScene(const planning_scene::PlanningSceneConstPtr& scene);

protected:
  // Robot environment
  std::string group_name_;
  moveit::core::RobotModelConstPtr robot_model_ptr_;
  planning_scene::PlanningSceneConstPtr planning_scene_ptr_;

  // Distance field collision environment (like CHOMP uses)
  planning_scene::PlanningScenePtr planning_scene_;  
  
  // Direct access to distance field
  std::shared_ptr<distance_field::PropagationDistanceField> distance_field_;
  
  // Motion plan request
  moveit_msgs::MotionPlanRequest plan_request_;

  std::shared_ptr<PCEVisualization> visualizer_;

  float collision_clearance_ = 0.05f;  // Safety margin (epsilon)
  float collision_threshold_ = 0.07f;   // Max distance to consider
  
  // Plugin management
  std::vector<cost_functions::PCECostFunctionPtr> cost_functions_;
  std::vector<filters::PCEFilterPtr> filters_;
  
  // Helper: Convert Trajectory to joint states for collision checking
  bool trajectoryToRobotState(const Trajectory& trajectory, size_t timestep, moveit::core::RobotState& state) const;
  
  std::vector<Eigen::Vector3d> getSphereLocations(const moveit::core::RobotState& state) const;
  
  float getObstacleCost(double distance) const;

  // Collision cost computation based on simple collision checking query
  float computeCollisionCostSimple(const Trajectory& trajectory) const;


  // Distance field setup (CHOMP approach)
  void createDistanceFieldFromPlanningScene();
  void addCollisionObjectsToDistanceField();
  void samplePointsFromShape(
      const shapes::ShapeConstPtr& shape,
      const Eigen::Isometry3d& pose,
      double resolution,
      EigenSTL::vector_Vector3d& points);
  
  // Get signed distance at a point
  double getDistanceAtPoint(const Eigen::Vector3d& point) const;

};

typedef std::shared_ptr<PCEOptimizationTask> PCEOptimizationTaskPtr;

} // namespace pce_ros