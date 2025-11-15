// include/pce_utils.h
namespace pce_ros {
namespace utils {
  
  std::vector<Eigen::Vector3d> getSphereLocations(
      const moveit::core::RobotState& state,
      const moveit::core::RobotModelConstPtr& robot_model,
      const std::string& group_name);
  
  bool trajectoryToRobotState(
      const Trajectory& trajectory,
      size_t timestep,
      moveit::core::RobotState& state,
      const moveit::core::RobotModelConstPtr& robot_model,
      const std::string& group_name);

} // namespace utils
} // namespace pce_ros