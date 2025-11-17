#include "pce_planner.h"
#include <yaml-cpp/yaml.h>
#include <moveit/robot_state/conversions.hpp>

namespace pce_ros
{

namespace {
rclcpp::Logger getLogger()
{
    return moveit::getLogger("pce_planner");
}
}

PCEPlanner::PCEPlanner(
    const std::string& group,
    const PCEConfig& config,
    const rclcpp::Node::SharedPtr& node,
    const moveit::core::RobotModelConstPtr& model,
    std::shared_ptr<PCEVisualization> visualizer)
  : planning_interface::PlanningContext("PCE", group)
  , robot_model_(model)
  , group_name_(group)
  , pce_config_(config)
  , visualizer_(visualizer)
  , node_(node)
{
  setup();  // All logging happens in setup()
}


PCEPlanner::~PCEPlanner()
{
}

void PCEPlanner::setup()
{ 
  // Create optimization task
  pce_task_ = std::make_shared<PCEOptimizationTask>(
      robot_model_, 
      group_name_,
      pce_config_,
      node_
  ); 

  // Pass visualizer to task
  if (visualizer_)
  {
    pce_task_->setVisualizer(visualizer_);
    RCLCPP_INFO(getLogger(), "  Visualizer connected to optimization task");
  }
  else
    RCLCPP_WARN(getLogger(), "  No visualizer available for PCEOptimizationTask");
  
  // Create PCE planner
  pce_planner_ = std::make_shared<ProximalCrossEntropyMotionPlanner>(pce_task_);

  // ====================================================================
  // CONSOLIDATED CONFIGURATION SUMMARY - ASCII version for compatibility
  // ====================================================================
  RCLCPP_INFO(getLogger(), "+============================================================+");
  RCLCPP_INFO(getLogger(), "|           PCE PLANNER CONFIGURATION                        |");
  RCLCPP_INFO(getLogger(), "+============================================================+");
  RCLCPP_INFO(getLogger(), "| Planning Group: %-42s |", group_name_.c_str());
  RCLCPP_INFO(getLogger(), "| Visualizer:     %-42s |", visualizer_ ? "Enabled" : "Disabled");
  RCLCPP_INFO(getLogger(), "+============================================================+");
  RCLCPP_INFO(getLogger(), "| OPTIMIZATION PARAMETERS                                    |");
  RCLCPP_INFO(getLogger(), "+------------------------------------------------------------+");
  RCLCPP_INFO(getLogger(), "|   Samples per iteration:  %-32d |", pce_config_.num_samples);
  RCLCPP_INFO(getLogger(), "|   Max iterations:         %-32d |", pce_config_.num_iterations);
  RCLCPP_INFO(getLogger(), "|   Temperature:            %-32.3f |", pce_config_.temperature);
  RCLCPP_INFO(getLogger(), "|   Proximal step size (eta):      %-32.3f |", pce_config_.eta);
  RCLCPP_INFO(getLogger(), "+------------------------------------------------------------+");
  RCLCPP_INFO(getLogger(), "| TRAJECTORY PARAMETERS                                      |");
  RCLCPP_INFO(getLogger(), "+------------------------------------------------------------+");
  RCLCPP_INFO(getLogger(), "|   Waypoints:              %-32d |", pce_config_.num_discretization);
  RCLCPP_INFO(getLogger(), "|   Total time:             %-32.2f |", pce_config_.total_time);
  RCLCPP_INFO(getLogger(), "|   Node collision radius:  %-32.3f |", pce_config_.node_collision_radius);
  RCLCPP_INFO(getLogger(), "+------------------------------------------------------------+");
  RCLCPP_INFO(getLogger(), "| COLLISION PARAMETERS                                       |");
  RCLCPP_INFO(getLogger(), "+------------------------------------------------------------+");
  RCLCPP_INFO(getLogger(), "|   Collision clearance:    %-32.3f |", pce_task_->getCollisionClearance());
  RCLCPP_INFO(getLogger(), "|   Collision threshold:    %-32.3f |", pce_task_->getCollisionThreshold());
  RCLCPP_INFO(getLogger(), "|   Sigma obs (weight):     %-32.3f |", pce_task_->getSigmaObs()); 
  RCLCPP_INFO(getLogger(), "|   Sphere overlap ratio:   %-32.3f |", pce_task_->getSphereOverlapRatio());
  RCLCPP_INFO(getLogger(), "+============================================================+");
}


void PCEPlanner::setVisualizer(std::shared_ptr<PCEVisualization> viz)
{
  visualizer_ = viz;
  
  if (pce_task_ && visualizer_)
  {
    pce_task_->setVisualizer(visualizer_);
    RCLCPP_INFO(getLogger(), "Visualizer set for PCEPlanner and task");
  }
}

void PCEPlanner::setPlanningScene(const planning_scene::PlanningSceneConstPtr& scene)
{  
  // Store the planning scene
  planning_scene_ = scene;
  
  // Pass to optimization task if it exists
  if (pce_task_)
  {
    pce_task_->setPlanningScene(scene);
  }

  RCLCPP_INFO(getLogger(), "PCEPlanner::setPlanningScene complete");
}

void PCEPlanner::solve(planning_interface::MotionPlanResponse& res)
{
  RCLCPP_INFO(getLogger(), "=== PCEPlanner::solve() CALLED ===");

  auto start_time = node_->now();
  
  // Validate planning scene
  if (!planning_scene_)
  {
    RCLCPP_ERROR(getLogger(), "Planning scene not set");
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return;
  }
  
  // Get start and goal from MoveIt2 request
  Eigen::VectorXd start, goal;
  if (!getStartAndGoal(start, goal))
  {
    RCLCPP_ERROR(getLogger(), "Failed to extract start and goal states");
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
  
  RCLCPP_INFO(getLogger(), "Start state: %s", ss_start.str().c_str());
  RCLCPP_INFO(getLogger(), "Goal state:  %s", ss_goal.str().c_str());
  
  // Set planning request in task
  moveit_msgs::msg::MoveItErrorCodes error_code;
  if (!pce_task_->setMotionPlanRequest(getPlanningScene(), request_, error_code))
  {
    RCLCPP_ERROR(getLogger(), "Failed to set motion plan request");
    res.error_code = error_code;
    return;
  }

  RCLCPP_INFO(getLogger(), "Motion plan request set in task");
  
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
  
  RCLCPP_INFO(getLogger(), "Initializing PCE planner with %zu dimensions...", 
              pce_config_.num_dimensions);
  
  // Initialize planner (sets up start_node_, goal_node_, and trajectory)
  if (!pce_planner_->initialize(pce_config_))
  {
    RCLCPP_ERROR(getLogger(), "PCE planner initialization failed");
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return;
  }
  
  RCLCPP_INFO(getLogger(), "PCE planner initialized successfully");
  RCLCPP_INFO(getLogger(), "Starting PCE optimization...");
  
  // Solve the planning problem
  if (!pce_planner_->solve())
  {
    RCLCPP_ERROR(getLogger(), "PCE optimization failed to find valid solution");
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return;
  }
  
  RCLCPP_INFO(getLogger(), "PCE optimization completed successfully");

  // Get the optimized trajectory
  const Trajectory& pce_traj = pce_planner_->getCurrentTrajectory();
  
  RCLCPP_INFO(getLogger(), "Retrieved trajectory with %zu waypoints", pce_traj.nodes.size());

  // Convert PCE trajectory to MoveIt2 joint trajectory
  trajectory_msgs::msg::JointTrajectory joint_traj;
  if (!pceTrajectoryToJointTrajectory(pce_traj, joint_traj))
  {
    RCLCPP_ERROR(getLogger(), "Failed to convert PCE trajectory to joint trajectory");
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return;
  }
  
  RCLCPP_INFO(getLogger(), "Converted to JointTrajectory with %zu points", joint_traj.points.size());
    
  try
  {
    if (!res.trajectory)
      res.trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, getGroupName());

    moveit::core::RobotState start_state(robot_model_);
    moveit::core::robotStateMsgToRobotState(request_.start_state, start_state);
    res.trajectory->setRobotTrajectoryMsg(start_state, joint_traj);
    
    RCLCPP_INFO(getLogger(), "Trajectory message set successfully");
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(getLogger(), "Exception while setting trajectory: %s", e.what());
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return;
  }
  
  res.planning_time = (node_->now() - start_time).seconds();
  res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  
  RCLCPP_INFO(getLogger(), "PCE planning succeeded in %.3f seconds", res.planning_time);
  
  return;
}

void PCEPlanner::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  planning_interface::MotionPlanResponse simple_res;
  solve(simple_res);
  if (simple_res.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
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
    RCLCPP_ERROR(getLogger(), "Joint model group '%s' not found", getGroupName().c_str());
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
  
  RCLCPP_ERROR(getLogger(), "No valid goal constraints found");
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
    RCLCPP_ERROR(getLogger(), "Joint model group not found");
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


bool PCEPlanner::getConfigData(const rclcpp::Node::SharedPtr& node,
                  std::map<std::string, PCEConfig>& config,
                  const std::string& param)
{
  std::vector<std::string> planning_groups;
  const std::string param_group = param + ".pce.planning_groups";
  planning_groups = getParam<std::vector<std::string>>(node, param_group, {});
  if (planning_groups.empty()){
    RCLCPP_ERROR(getLogger(), "  ✗ Could not find parameter 'pce.planning_groups'");
    return false;
  }

  RCLCPP_INFO(getLogger(), "  ✓ Found planning_groups with %zu entries:", planning_groups.size());
  for (const auto& group : planning_groups)
    RCLCPP_INFO(getLogger(), "    - %s", group.c_str());

  for (const auto& group : planning_groups)
  {
    PCEConfig group_config;
    std::string base = param + ".pce." + group;
    std::string group_param = base + ".pce_planner";

    // TODO: Try get_parameter_or function
    group_config.num_iterations = getParam<int>(node, group_param + ".num_iterations", 15);
    group_config.num_samples = getParam<int>(node, group_param + ".num_samples", 3000);
    group_config.temperature = getParam<double>(node, group_param + ".temperature", 1.5);
    group_config.eta = getParam<double>(node, group_param + ".eta", 1.0);
    group_config.num_discretization = getParam<int>(node, group_param + ".num_discretization", 20);
    group_config.total_time = getParam<double>(node, group_param + ".total_time", 5.0);
    group_config.node_collision_radius = getParam<double>(node, group_param + ".node_collision_radius", 0.1);

    // task collision parameters
    // TODO: Delete these since already read in PCEOptimizationTask
    group_config.collision_clearance = getParam<double>(node, base + ".collision_clearance", 0.05);
    group_config.collision_threshold = getParam<double>(node, base + ".collision_threshold", 0.07);

    // push into map
    config[group] = group_config;

    RCLCPP_INFO(getLogger(), "Loaded PCE config for group '%s'", group.c_str());
  }

  return true;
}


bool PCEPlanner::getConfigData(const std::string& yaml_dict,
                std::map<std::string, PCEConfig>& config)
{
  try{
    YAML::Node root = YAML::LoadFile(yaml_dict);
    YAML::Node pce_node = root["pce"];
    for (auto& [group_name, group_config] : config)
    {
      if (!pce_node[group_name])
        continue;
      
      YAML::Node planner_node = pce_node[group_name]["pce_planner"];
      if (planner_node["num_iterations"])
        group_config.num_iterations = planner_node["num_iterations"].as<int>();
      if (planner_node["num_samples"])
        group_config.num_samples = planner_node["num_samples"].as<int>();
      if (planner_node["temperature"])
        group_config.temperature = planner_node["temperature"].as<double>();
      if (planner_node["eta"])
        group_config.eta = planner_node["eta"].as<double>();
      if (planner_node["num_discretization"])
        group_config.num_discretization = planner_node["num_discretization"].as<int>();
      if (planner_node["total_time"])
        group_config.total_time = planner_node["total_time"].as<double>();
      if (planner_node["node_collision_radius"])
        group_config.node_collision_radius = planner_node["node_collision_radius"].as<double>();
    }
    return true;
  }
  catch (const YAML::Exception& e)
  {
    RCLCPP_ERROR(getLogger(), "Failed to load YAML config: %s", e.what());
    return false;
  }
}

} // namespace pce_ros