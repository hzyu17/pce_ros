#include "pce_planner.h"
#include <moveit/robot_state/conversions.h>

namespace pce_ros
{

PCEPlanner::PCEPlanner(
    const std::string& group,
    const XmlRpc::XmlRpcValue& config,
    const moveit::core::RobotModelConstPtr& model,
    std::shared_ptr<PCEVisualization> visualizer)
  : planning_interface::PlanningContext("PCE", group)
  , config_(config)
  , robot_model_(model)
  , group_name_(group)
  , visualizer_(visualizer)
{
  ROS_INFO("=== PCEPlanner constructor ===");
  ROS_INFO("  Group name: '%s'", group.c_str());

  if (visualizer_)
  {
    ROS_INFO("  Visualizer provided in constructor");
  }
  else
  {
    ROS_WARN("  No visualizer provided in constructor");
  }

  setup();
}

PCEPlanner::~PCEPlanner()
{
}

void PCEPlanner::setup()
{ 

  ROS_INFO("=== PCEPlanner::setup() ===");
  ROS_INFO("  Group name: '%s'", group_name_.c_str());

  // Create optimization task
  pce_task_ = std::make_shared<PCEOptimizationTask>(
      robot_model_, 
      group_name_,
      config_
  ); 

  ROS_INFO("PCEOptimizationTask created");

  // Pass visualizer to task
  if (visualizer_)
  {
    ROS_INFO("Passing visualizer to PCEOptimizationTask");
    pce_task_->setVisualizer(visualizer_);
  }
  else
  {
    ROS_WARN("No visualizer available for PCEOptimizationTask");
  }
  
  // Create PCE planner
  pce_planner_ = std::make_shared<ProximalCrossEntropyMotionPlanner>(pce_task_);
  
  ROS_INFO("PCEMotionPlanner created with task");

  // Load PCE configuration from YAML
  // You'll need to convert XmlRpc to your PCEConfig format
  // For now, use defaults or parse manually
  
  pce_config_.num_samples = 3000;
  pce_config_.num_iterations = 10;
  pce_config_.temperature = 1.5f;
  pce_config_.eta = 1.0f;
  
  // TODO: Parse config_ to populate pce_config_
  
  ROS_INFO("PCEPlanner setup complete for group '%s'", getGroupName().c_str());
}

bool PCEPlanner::solve(planning_interface::MotionPlanResponse& res)
{
  ROS_INFO("=== PCEPlanner::solve() CALLED ===");

  ros::WallTime start_time = ros::WallTime::now();
  
  // Get start and goal from MoveIt request
  Eigen::VectorXd start, goal;
  if (!getStartAndGoal(start, goal))
  {
    ROS_ERROR("Failed to get start and goal");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }

  ROS_INFO("Got start and goal:");
  ROS_INFO("  Start size: %d", (int)start.size());
  ROS_INFO("  Goal size: %d", (int)goal.size());
  
  // Set planning request in task
  moveit_msgs::MoveItErrorCodes error_code;
  if (!pce_task_->setMotionPlanRequest(getPlanningScene(), request_, error_code))
  {
    ROS_ERROR("Failed to set motion plan request");
    res.error_code_ = error_code;
    return false;
  }

  ROS_INFO("Motion plan request set in task");
  
  // Populate PCE config with start and goal
  pce_config_.num_dimensions = start.size();
  pce_config_.num_discretization = 20;  // Or from YAML config
  pce_config_.total_time = 5.0f;        // Or from request/config
  pce_config_.node_collision_radius = 0.1f;

  ROS_INFO("PCE config:");
  ROS_INFO("  num_dimensions: %d", pce_config_.num_dimensions);
  ROS_INFO("  num_discretization: %d", pce_config_.num_discretization);
  ROS_INFO("  total_time: %.2f", pce_config_.total_time);
  
  // Convert start/goal from Eigen::VectorXd to std::vector<float>
  pce_config_.start_position.resize(start.size());
  pce_config_.goal_position.resize(goal.size());
  
  for (int i = 0; i < start.size(); ++i)
  {
    pce_config_.start_position[i] = static_cast<float>(start[i]);
    pce_config_.goal_position[i] = static_cast<float>(goal[i]);
  }
  
  ROS_INFO("Calling pce_planner_->initialize()...");
  // Initialize planner (this sets up start_node_, goal_node_, and trajectory)
  if (!pce_planner_->initialize(pce_config_))
  {
    ROS_ERROR("Failed to initialize PCE planner");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return false;
  }
  ROS_INFO("PCE planner initialized successfully");
  
  ROS_INFO("Calling pce_planner_->solve()...");
  // Now solve (no parameters needed - everything was set in initialize)
  if (!pce_planner_->solve())
  {
    ROS_ERROR("PCE optimization failed");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }
  ROS_INFO("PCE solve() returned TRUE");

  // Get the optimized trajectory
  const Trajectory& pce_traj = pce_planner_->getCurrentTrajectory();
  
  ROS_INFO("Got trajectory with %zu nodes", pce_traj.nodes.size());
  // Convert result to MoveIt trajectory
  trajectory_msgs::JointTrajectory joint_traj;
  if (!pceTrajectoryToJointTrajectory(pce_traj, joint_traj))
  {
    ROS_ERROR("Failed to convert trajectory");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return false;
  }
  ROS_INFO("Converted to JointTrajectory with %zu points", joint_traj.points.size());
  
  // Populate response
  res.trajectory_.reset(new robot_trajectory::RobotTrajectory(robot_model_, getGroupName()));
  res.trajectory_->setRobotTrajectoryMsg(getPlanningScene()->getCurrentState(), joint_traj);
  
  res.planning_time_ = (ros::WallTime::now() - start_time).toSec();
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  
  ROS_INFO("PCE planning succeeded in %.3f seconds", res.planning_time_);
  
  return true;
}


bool PCEPlanner::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  planning_interface::MotionPlanResponse simple_res;
  if (solve(simple_res))
  {
    res.trajectory_.push_back(simple_res.trajectory_);
    res.processing_time_.push_back(simple_res.planning_time_);
    res.description_.push_back("PCE");
    res.error_code_ = simple_res.error_code_;
    return true;
  }
  return false;
}

bool PCEPlanner::terminate()
{
  // PCE doesn't have built-in cancellation yet
  // You could add an atomic flag to ProximalCrossEntropyMotionPlanner
  return false;
}

void PCEPlanner::clear()
{
  // Reset planner state
}

bool PCEPlanner::canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const
{
  // Check if request is compatible
  return req.group_name == getGroupName();
}

bool PCEPlanner::getStartAndGoal(Eigen::VectorXd& start, Eigen::VectorXd& goal)
{
  const moveit::core::JointModelGroup* jmg = 
      robot_model_->getJointModelGroup(getGroupName());
  
  if (!jmg)
    return false;
  
  // Get start state
  moveit::core::RobotState start_state(robot_model_);
  moveit::core::robotStateMsgToRobotState(request_.start_state, start_state);
  
  std::vector<double> start_positions;
  start_state.copyJointGroupPositions(jmg, start_positions);
  
  start = Eigen::Map<Eigen::VectorXd>(start_positions.data(), start_positions.size());
  
  // Get goal state (simplified - assumes joint space goal)
  if (!request_.goal_constraints.empty() &&
      !request_.goal_constraints[0].joint_constraints.empty())
  {
    goal.resize(start.size());
    for (const auto& jc : request_.goal_constraints[0].joint_constraints)
    {
      auto idx = jmg->getVariableGroupIndex(jc.joint_name);
      if (idx < goal.size())
      {
        goal[idx] = jc.position;
      }
    }
    return true;
  }
  
  return false;
}

bool PCEPlanner::pceTrajectoryToJointTrajectory(
    const Trajectory& pce_traj,
    trajectory_msgs::JointTrajectory& joint_traj)
{
  const moveit::core::JointModelGroup* jmg = 
      robot_model_->getJointModelGroup(getGroupName());
  
  if (!jmg)
    return false;
  
  joint_traj.joint_names = jmg->getVariableNames();
  joint_traj.points.resize(pce_traj.nodes.size());
  
  for (size_t i = 0; i < pce_traj.nodes.size(); ++i)
  {
    // Convert Eigen::VectorXf to std::vector<double>
    const auto& node_pos = pce_traj.nodes[i].position;
    joint_traj.points[i].positions.resize(node_pos.size());
    for (int j = 0; j < node_pos.size(); ++j)
    {
      joint_traj.points[i].positions[j] = static_cast<double>(node_pos[j]);
    }
    
    // Compute time for this waypoint (evenly distributed)
    double time = (pce_traj.total_time * i) / (pce_traj.nodes.size() - 1);
    joint_traj.points[i].time_from_start = ros::Duration(time);
  }
  
  return true;
}

bool PCEPlanner::getConfigData(ros::NodeHandle& nh,
                               std::map<std::string, XmlRpc::XmlRpcValue>& config,
                               std::string param)
{
  // Similar to STOMP's implementation
  XmlRpc::XmlRpcValue pce_config;
  if (!nh.getParam(param, pce_config))
  {
    ROS_ERROR("Could not find '%s' parameter", param.c_str());
    return false;
  }
  
  if (pce_config.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR("'%s' parameter is not a struct", param.c_str());
    return false;
  }
  
  for (auto& pair : pce_config)
  {
    config[pair.first] = pair.second;
  }
  
  return true;
}

} // namespace pce_ros