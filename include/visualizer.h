/**
 * @file pce_visualization.h
 * @brief Visualization utilities for PCE trajectory optimization
 * 
 * This utility class provides RViz visualization for:
 * - Collision checking spheres on robot body
 * - Trajectory paths during optimization
 * - Distance field information
 * 
 * Can be used by any motion planner that works with trajectories.
 */
#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <pce/Trajectory.h>
#include <Eigen/Core>
#include <vector>
#include <memory>

namespace pce_ros
{

/**
 * @brief Configuration for visualization
 */
struct VisualizationConfig
{
  bool enable_collision_spheres = true;
  bool enable_trajectory = true;
  bool enable_distance_field = false;
  
  float collision_clearance = 0.05f;  // For color coding
  
  std::string collision_spheres_topic = "/pce/collision_spheres";
  std::string trajectory_topic = "/pce/trajectory";
  std::string distance_field_topic = "/pce/distance_field";
  
  // Visualization parameters
  double sphere_size = 0.04;           // Size of collision spheres
  double waypoint_size = 0.02;         // Size of trajectory waypoints
  double line_width = 0.01;            // Width of trajectory line
  double marker_lifetime = 0.5;        // How long markers persist
  size_t trajectory_decimation = 10;   // Show every Nth waypoint
};

/**
 * @brief Utility class for visualizing trajectory optimization
 */
class PCEVisualization
{
public:
  /**
   * @brief Constructor
   * @param config Visualization configuration
   * @param nh ROS node handle
   */
  explicit PCEVisualization(const VisualizationConfig& config = VisualizationConfig(),
                            ros::NodeHandle nh = ros::NodeHandle());

  /**
   * @brief Load visualization configuration from ROS parameters
   * @param config_value XmlRpc value containing visualization config
   * @return VisualizationConfig with loaded parameters
   */
  static VisualizationConfig loadConfig(const XmlRpc::XmlRpcValue& config_value);
  
  /**
   * @brief Visualize collision checking spheres on robot body
   * @param trajectory The trajectory to visualize
   * @param robot_model Robot kinematic model
   * @param group_name Planning group name
   * @param distance_field Optional distance field for color coding
   */
  void visualizeCollisionSpheres(
      const Trajectory& trajectory,
      const moveit::core::RobotModelConstPtr& robot_model,
      const std::string& group_name,
      const distance_field::DistanceFieldConstPtr& distance_field = nullptr) const;
  
  /**
   * @brief Visualize trajectory path in workspace
   * @param trajectory The trajectory to visualize
   * @param robot_model Robot kinematic model
   * @param group_name Planning group name
   * @param iteration Current iteration number
   */
  void visualizeTrajectory(
      const Trajectory& trajectory,
      const moveit::core::RobotModelConstPtr& robot_model,
      const std::string& group_name,
      int iteration = 0) const;
  
  /**
   * @brief Visualize distance field as point cloud
   * @param distance_field The distance field to visualize
   * @param frame_id Reference frame
   */
  void visualizeDistanceField(
      const std::shared_ptr<distance_field::PropagationDistanceField>& distance_field,
      const std::string& frame_id) const;
  
  /**
   * @brief Clear all visualization markers
   */
  void clearAllMarkers() const;
  
  /**
   * @brief Enable/disable specific visualization types
   */
  void setEnableCollisionSpheres(bool enable) { config_.enable_collision_spheres = enable; }
  void setEnableTrajectory(bool enable) { config_.enable_trajectory = enable; }
  void setEnableDistanceField(bool enable) { config_.enable_distance_field = enable; }
  
  /**
   * @brief Get current configuration
   */
  const VisualizationConfig& getConfig() const { return config_; }

private:
  VisualizationConfig config_;
  ros::NodeHandle nh_;
  
  mutable ros::Publisher collision_marker_pub_;
  mutable ros::Publisher trajectory_marker_pub_;
  mutable ros::Publisher distance_field_pub_;
  
  /**
   * @brief Get sphere locations on robot body for collision checking
   * @param state Robot state
   * @param robot_model Robot model
   * @param group_name Planning group
   * @return Vector of 3D points on robot surface
   */
  std::vector<Eigen::Vector3d> getSphereLocations(
      const moveit::core::RobotState& state,
      const moveit::core::RobotModelConstPtr& robot_model,
      const std::string& group_name) const;
  
  /**
   * @brief Convert trajectory node to robot state
   */
  bool trajectoryToRobotState(
      const Trajectory& trajectory,
      size_t timestep,
      moveit::core::RobotState& state,
      const moveit::core::RobotModelConstPtr& robot_model,
      const std::string& group_name) const;
  
  /**
   * @brief Get color based on distance to obstacles
   * @param distance Signed distance (negative = collision)
   * @return RGBA color
   */
  std_msgs::ColorRGBA getColorFromDistance(double distance) const;
  
  /**
   * @brief Get gradient color for trajectory visualization
   * @param ratio Value from 0 to 1
   * @return RGBA color
   */
  std_msgs::ColorRGBA getGradientColor(float ratio) const;
};

} // namespace pce_ros
