#include "pce_planner.h"
#include <moveit/robot_state/conversions.h>

namespace pce_ros
{

PCEPlanner::PCEPlanner(
    const std::string& group,
    const std::map<std::string, std::string>& config,
    const moveit::core::RobotModelConstPtr& model,
    const rclcpp::Node::SharedPtr& node,
    std::shared_ptr<PCEVisualization> visualizer)
  : planning_interface::PlanningContext("PCE", group)
  , robot_model_(model)
  , group_name_(group)
  , config_(config)
  , node_(node)
  , visualizer_(visualizer)
{
  RCLCPP_INFO(node_->get_logger(), "=== PCEPlanner Constructor ===");
  RCLCPP_INFO(node_->get_logger(), "  Planning group: '%s'", group.c_str());

  if (visualizer_)
  {
    RCLCPP_INFO(node_->get_logger(), "  Visualizer provided and initialized");
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "  No visualizer provided - visualization disabled");
  }

  setup();
}

PCEPlanner::~PCEPlanner()
{
}

void PCEPlanner::setup()
{ 
  RCLCPP_INFO(node_->get_logger(), "=== PCEPlanner::setup() ===");
  RCLCPP_INFO(node_->get_logger(), "  Initializing for planning group: '%s'", group_name_.c_str());

  // Create optimization task
  pce_task_ = std::make_shared<PCEOptimizationTask>(
      robot_model_, 
      group_name_,
      node_
  ); 

  // Pass visualizer to task
  if (visualizer_)
  {
    pce_task_->setVisualizer(visualizer_);
    RCLCPP_INFO(node_->get_logger(), "  Visualizer connected to optimization task");
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "  No visualizer available for PCEOptimizationTask");
  }
  
  // Create PCE planner
  pce_planner_ = std::make_shared<ProximalCrossEntropyMotionPlanner>(pce_task_);

  RCLCPP_INFO(node_->get_logger(), "  PCEMotionPlanner created with task");

  // Load PCE configuration from parameters
  // Helper lambda to get parameter with default value
  auto getParam = [this](const std::string& key, auto default_val) -> decltype(default_val) {
    if (config_.find(key) != config_.end())
    {
      if constexpr (std::is_same_v<decltype(default_val), int>)
      {
        return std::stoi(config_.at(key));
      }
      else if constexpr (std::is_same_v<decltype(default_val), double> || 
                         std::is_same_v<decltype(default_val), float>)
      {
        return std::stod(config_.at(key));
      }
      else
      {
        return config_.at(key);
      }
    }
    return default_val;
  };

  // Load num_iterations with validation
  pce_config_.num_iterations = getParam("num_iterations", 15);
  RCLCPP_INFO(node_->get_logger(), "  num_iterations: %zu", pce_config_.num_iterations);
  
  // Load num_samples with validation
  pce_config_.num_samples = getParam("num_samples", 3000);
  RCLCPP_INFO(node_->get_logger(), "  num_samples: %zu", pce_config_.num_samples);

  // Load temperature with validation
  pce_config_.temperature = static_cast<float>(getParam("temperature", 1.5));
  RCLCPP_INFO(node_->get_logger(), "  temperature: %.3f", pce_config_.temperature);

  // Load eta (learning rate) with validation
  pce_config_.eta = static_cast<float>(getParam("eta", 1.0));
  RCLCPP_INFO(node_->get_logger(), "  eta: %.3f", pce_config_.eta);
  
  // Load num_discretization with validation
  pce_config_.num_discretization = getParam("num_discretization", 20);
  RCLCPP_INFO(node_->get_logger(), "  num_discretization: %zu", pce_config_.num_discretization);

  // Load total_time with validation
  pce_config_.total_time = static_cast<float>(getParam("total_time", 5.0));
  RCLCPP_INFO(node_->get_logger(), "  total_time: %.2f", pce_config_.total_time);

  // Load node_collision_radius with validation
  pce_config_.node_collision_radius = static_cast<float>(getParam("node_collision_radius", 0.1));
  RCLCPP_INFO(node_->get_logger(), "  node_collision_radius: %.3f", pce_config_.node_collision_radius);

  RCLCPP_INFO(node_->get_logger(), "=== PCE Configuration Summary ===");
  RCLCPP_INFO(node_->get_logger(), "  num_discretization:     %zu", pce_config_.num_discretization);
  RCLCPP_INFO(node_->get_logger(), "  total_time:             %.2f s", pce_config_.total_time);
  RCLCPP_INFO(node_->get_logger(), "  num_samples:            %zu", pce_config_.num_samples);
  RCLCPP_INFO(node_->get_logger(), "  num_iterations:         %zu", pce_config_.num_iterations);
  RCLCPP_INFO(node_->get_logger(), "  eta:                    %.3f", pce_config_.eta);
  RCLCPP_INFO(node_->get_logger(), "  temperature:            %.3f", pce_config_.temperature);
  RCLCPP_INFO(node_->get_logger(), "  node_collision_radius:  %.3f m", pce_config_.node_collision_radius);
  RCLCPP_INFO(node_->get_logger(), "PCEPlanner setup complete for group '%s'", getGroupName().c_str());
}

void PCEPlanner::setVisualizer(std::shared_ptr<PCEVisualization> viz)
{
  visualizer_ = viz;
  
  if (pce_task_ && visualizer_)
  {
    pce_task_->setVisualizer(visualizer_);
    RCLCPP_INFO(node_->get_logger(), "Visualizer set for PCEPlanner and task");
  }
}

void PCEPlanner::setPlanningScene(const planning_scene::PlanningSceneConstPtr& scene)
{
  if (!scene)
  {
    RCLCPP_WARN(node_->get_logger(), "Attempted to set null planning scene");
    return;
  }
  
  // Store the planning scene
  planning_scene_ = scene;
  
  // Pass to optimization task if it exists
  if (pce_task_)
  {
    pce_task_->setPlanningScene(scene);
  }

  RCLCPP_INFO(node_->get_logger(), "PCEPlanner::setPlanningScene complete");
}

void PCEPlanner::solve(planning_interface::MotionPlanResponse& res)
{
  RCLCPP_INFO(node_->get_logger(), "=== PCEPlanner::solve() CALLED ===");

  auto start_time = node_->now();
  
  // Validate planning scene
  if (!planning_scene_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Planning scene not set");
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return;
  }
  
  // Get start and goal from MoveIt2 request
  Eigen::VectorXd start, goal;
  if (!getStartAndGoal(start, goal))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to extract start and goal states");
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return;
  }
  
  // Log start and goal states
  std::stringstream ss_start, ss_goal;
  ss_start << "[";
  ss_goal << "[";
  for (int i = 0; i < start.size(); ++i)
  {
    ss_start << start[i];
    ss_goal << goal[i];
    if (i < start.size() - 1)
    {
      ss_start << ", ";
      ss_goal << ", ";
    }
  }
  ss_start << "]";
  ss_goal << "]";
  
  RCLCPP_INFO(node_->get_logger(), "Start state: %s", ss_start.str().c_str());
  RCLCPP_INFO(node_->get_logger(), "Goal state:  %s", ss_goal.str().c_str());
  
  // Set planning request in task
  moveit_msgs::msg::MoveItErrorCodes error_code;
  if (!pce_task_->setMotionPlanRequest(getPlanningScene(), request_, error_code))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to set motion plan request");
    res.error_code = error_code;
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Motion plan request set in task");
  
  // Populate PCE config with start and goal
  pce_config_.num_dimensions = start.size();

  // Convert start/goal from Eigen::VectorXd to std::vector<float>
  pce_config_.start_position.resize(start.size());
  pce_config_.goal_position.resize(goal.size());
  
  for (int i = 0; i < start.size(); ++i)
  {
    pce_config_.start_position[i] = static_cast<float>(start[i]);
    pce_config_.goal_position[i] = static_cast<float>(goal[i]);
  }
  
  RCLCPP_INFO(node_->get_logger(), "Initializing PCE planner with %zu dimensions...", 
              pce_config_.num_dimensions);
  
  // Initialize planner (sets up start_node_, goal_node_, and trajectory)
  if (!pce_planner_->initialize(pce_config_))
  {
    RCLCPP_ERROR(node_->get_logger(), "PCE planner initialization failed");
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return;
  }
  
  RCLCPP_INFO(node_->get_logger(), "PCE planner initialized successfully");
  
  RCLCPP_INFO(node_->get_logger(), "Starting PCE optimization...");
  
  // Solve the planning problem
  if (!pce_planner_->solve())
  {
    RCLCPP_ERROR(node_->get_logger(), "PCE optimization failed to find valid solution");
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return;
  }
  
  RCLCPP_INFO(node_->get_logger(), "PCE optimization completed successfully");

  // Get the optimized trajectory
  const Trajectory& pce_traj = pce_planner_->getCurrentTrajectory();
  
  RCLCPP_INFO(node_->get_logger(), "Retrieved trajectory with %zu waypoints", pce_traj.nodes.size());
  
  // Convert PCE trajectory to MoveIt2 joint trajectory
  trajectory_msgs::msg::JointTrajectory joint_traj;
  if (!pceTrajectoryToJointTrajectory(pce_traj, joint_traj))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to convert PCE trajectory to joint trajectory");
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return;
  }
  
  RCLCPP_INFO(node_->get_logger(), "Converted to JointTrajectory with %zu points", 
              joint_traj.points.size());
  
  // Populate response
  res.trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, getGroupName());
  res.trajectory->setRobotTrajectoryMsg(getPlanningScene()->getCurrentState(), joint_traj);
  
  auto end_time = node_->now();
  res.planning_time = (end_time - start_time).seconds();
  res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  
  RCLCPP_INFO(node_->get_logger(), "PCE planning succeeded in %.3f seconds", res.planning_time);
  
  return;
}

void PCEPlanner::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  planning_interface::MotionPlanResponse simple_res;
  solve(simple_res);
  if (simple_res.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    res.trajectory.push_back(simple_res.trajectory);
    res.processing_time.push_back(simple_res.planning_time);
    res.description.push_back("PCE");
  }
  res.error_code = simple_res.error_code;
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

bool PCEPlanner::canServiceRequest(const moveit_msgs::msg::MotionPlanRequest& req) const
{
  // Check if request is compatible
  return req.group_name == getGroupName();
}

bool PCEPlanner::getStartAndGoal(Eigen::VectorXd& start, Eigen::VectorXd& goal)
{
  const moveit::core::JointModelGroup* jmg = 
      robot_model_->getJointModelGroup(getGroupName());
  
  if (!jmg)
  {
    RCLCPP_ERROR(node_->get_logger(), "Joint model group '%s' not found", getGroupName().c_str());
    return false;
  }
  
  // Get start state
  moveit::core::RobotState start_state(robot_model_);
  moveit::core::robotStateMsgToRobotState(request_.start_state, start_state);
  
  std::vector<double> start_positions;
  start_state.copyJointGroupPositions(jmg, start_positions);
  
  start = Eigen::Map<Eigen::VectorXd>(start_positions.data(), start_positions.size());
  
  // Get goal state (simplified - assumes joint space goal)
  const auto& goal_constraints = request_.goal_constraints;
  if (!goal_constraints.empty() && !goal_constraints[0].joint_constraints.empty())
  {
    goal.resize(start.size());
    goal.setZero();
    
    for (const auto& jc : goal_constraints[0].joint_constraints)
    {
      int idx = jmg->getVariableGroupIndex(jc.joint_name);
      if (idx >= 0 && idx < goal.size())
      {
        goal[idx] = jc.position;
      }
    }
    return true;
  }
  
  RCLCPP_ERROR(node_->get_logger(), "No valid goal constraints found");
  return false;
}

bool PCEPlanner::pceTrajectoryToJointTrajectory(
    const Trajectory& pce_traj,
    trajectory_msgs::msg::JointTrajectory& joint_traj)
{
  const moveit::core::JointModelGroup* jmg = 
      robot_model_->getJointModelGroup(getGroupName());
  
  if (!jmg)
  {
    RCLCPP_ERROR(node_->get_logger(), "Joint model group not found");
    return false;
  }
  
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
    joint_traj.points[i].time_from_start = rclcpp::Duration::from_seconds(time);
  }
  
  return true;
}

} // namespace pce_ros