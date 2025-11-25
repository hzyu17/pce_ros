/**
 * @file ngd_planner.cpp
 * @brief NGD planner implementation - matches PCE planner structure
 */
#include "ngd_planner.h"
#include <yaml-cpp/yaml.h>
#include <moveit/robot_state/conversions.hpp>

namespace pce_ros
{

namespace {
rclcpp::Logger getLogger()
{
    return moveit::getLogger("ngd_planner");
}
}

NGDPlanner::NGDPlanner(
    const std::string& group,
    const NGDConfig& config,
    const rclcpp::Node::SharedPtr& node,
    const moveit::core::RobotModelConstPtr& model,
    std::shared_ptr<PCEVisualization> visualizer)
  : planning_interface::PlanningContext("ngd_planner", group)
  , group_name_(group)
  , ngd_config_(config)
  , robot_model_(model)
  , node_(node)
  , visualizer_(visualizer)
{
  setup();
}

NGDPlanner::~NGDPlanner()
{
  ngd_planner_.reset();
  optimization_task_.reset();
  visualizer_.reset();
}

void NGDPlanner::setup()
{ 
  // Create optimization task
  optimization_task_ = std::make_shared<PCEOptimizationTask>(
      robot_model_, 
      group_name_,
      node_
  ); 

  // Pass visualizer to task
  if (visualizer_)
  {
    optimization_task_->setVisualizer(visualizer_);
  }
  
  // Create PCE planner
  ngd_planner_ = std::make_shared<NGDMotionPlanner>(optimization_task_);

  // ====================================================================
  // CONSOLIDATED CONFIGURATION SUMMARY - ASCII version for compatibility
  // ====================================================================
  RCLCPP_INFO(getLogger(), "+============================================================+");
  RCLCPP_INFO(getLogger(), "|           NGD PLANNER CONFIGURATION                        |");
  RCLCPP_INFO(getLogger(), "+============================================================+");
  RCLCPP_INFO(getLogger(), "| Planning Group: %-42s |", group_name_.c_str());
  RCLCPP_INFO(getLogger(), "| Visualizer:     %-42s |", visualizer_ ? "Enabled" : "Disabled");
  RCLCPP_INFO(getLogger(), "+============================================================+");
  RCLCPP_INFO(getLogger(), "| OPTIMIZATION PARAMETERS                                    |");
  RCLCPP_INFO(getLogger(), "+------------------------------------------------------------+");
  RCLCPP_INFO(getLogger(), "|   Samples per iteration:  %-32d |", ngd_config_.num_samples);
  RCLCPP_INFO(getLogger(), "|   Max iterations:         %-32d |", ngd_config_.num_iterations);
  RCLCPP_INFO(getLogger(), "|   Temperature:            %-32.3f |", ngd_config_.temperature);
  RCLCPP_INFO(getLogger(), "+------------------------------------------------------------+");
  RCLCPP_INFO(getLogger(), "| TRAJECTORY PARAMETERS                                      |");
  RCLCPP_INFO(getLogger(), "+------------------------------------------------------------+");
  RCLCPP_INFO(getLogger(), "|   Waypoints:              %-32d |", ngd_config_.num_discretization);
  RCLCPP_INFO(getLogger(), "|   Total time:             %-32.2f |", ngd_config_.total_time);
  RCLCPP_INFO(getLogger(), "|   Node collision radius:  %-32.3f |", ngd_config_.node_collision_radius);
  RCLCPP_INFO(getLogger(), "+------------------------------------------------------------+");
  RCLCPP_INFO(getLogger(), "| COLLISION PARAMETERS                                       |");
  RCLCPP_INFO(getLogger(), "+------------------------------------------------------------+");
  RCLCPP_INFO(getLogger(), "|   Collision clearance:    %-32.3f |", optimization_task_->getCollisionClearance());
  RCLCPP_INFO(getLogger(), "|   Collision threshold:    %-32.3f |", optimization_task_->getCollisionThreshold());
  RCLCPP_INFO(getLogger(), "|   Sigma obs (weight):     %-32.3f |", optimization_task_->getSigmaObs()); 
  RCLCPP_INFO(getLogger(), "|   Sphere overlap ratio:   %-32.3f |", optimization_task_->getSphereOverlapRatio());
  RCLCPP_INFO(getLogger(), "+============================================================+");
}


void NGDPlanner::solve(planning_interface::MotionPlanResponse& res)
{
  RCLCPP_INFO(getLogger(), "=== NGDPlanner::solve() CALLED ===");

  auto start_time = node_->now();

  if (!planning_scene_)
  {
    RCLCPP_ERROR(getLogger(), "Planning scene not set");
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return;
  }
  
  // Get start and goal from MoveIt request
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
 
  // Set motion plan request in task
  moveit_msgs::msg::MoveItErrorCodes error_code;
  if (!optimization_task_->setMotionPlanRequest(getPlanningScene(), request_, error_code))
  {
    RCLCPP_ERROR(getLogger(), "Failed to set motion plan request");
    res.error_code = error_code;
    return;
  }
  
  // Set task in NGD planner
  ngd_planner_->setTask(optimization_task_);
  
  // Populate NGD config with start and goal
  ngd_config_.num_dimensions = start.size();

  // Convert start/goal from Eigen::VectorXd to std::vector<float>
  ngd_config_.start_position.resize(start.size());
  ngd_config_.goal_position.resize(goal.size());
  
  for (int i = 0; i < start.size(); ++i)
  {
    ngd_config_.start_position[i] = static_cast<float>(start[i]);
    ngd_config_.goal_position[i] = static_cast<float>(goal[i]);
  }
    
  try
  {
    if (!ngd_planner_->initialize(ngd_config_))
    {
      RCLCPP_ERROR(getLogger(), "Failed to initialize NGD planner");
      res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return;
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(getLogger(), "Exception in ngd_planner_->initialize: %s", e.what());
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return;
  }
  catch (...)
  {
    RCLCPP_ERROR(getLogger(), "Unknown exception in ngd_planner_->initialize");
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return;
  }

  RCLCPP_INFO(getLogger(), "========================================================");
  RCLCPP_INFO(getLogger(), "COLLISION SPHERE PREVIEW");
  RCLCPP_INFO(getLogger(), "========================================================");
  RCLCPP_INFO(getLogger(), "Visualizing collision checking spheres for initial trajectory...");
  
  // Get the initial trajectory (use copy, not reference)
  Trajectory initial_traj = ngd_planner_->getCurrentTrajectory();
  
  // Compute collision cost (which caches sphere locations)
  float initial_cost = optimization_task_->computeCollisionCostSimple(initial_traj);
  RCLCPP_INFO(getLogger(), "Initial trajectory collision cost: %.4f", initial_cost);
  
  // Visualize the collision spheres
  if (!visualizer_) {
      RCLCPP_WARN(getLogger(), "Visualizer is NULL - markers will not be published!");
  } else {
      RCLCPP_INFO(getLogger(), "Visualizer is active, publishing markers...");

      visualizer_->visualizeCollisionSpheres(
        initial_traj,
        optimization_task_->getCachedSphereLocations(),
        robot_model_,
        group_name_,
        optimization_task_->getCollisionClearance(),
        nullptr
    );
    
    visualizer_->visualizeTrajectory(
        initial_traj,
        robot_model_,
        group_name_,
        0
    );

    RCLCPP_INFO(getLogger(), "--------------------------------------------------------");
    RCLCPP_INFO(getLogger(), "Check RViz to see the collision checking spheres.");
    RCLCPP_INFO(getLogger(), "Total spheres per waypoint: %zu", 
            optimization_task_->getCachedSphereLocations().empty() ? 0 : 
            optimization_task_->getCachedSphereLocations()[0].size());
    RCLCPP_INFO(getLogger(), "Total waypoints: %zu", initial_traj.nodes.size());
    RCLCPP_INFO(getLogger(), "--------------------------------------------------------");
    RCLCPP_WARN(getLogger(), "Starting optimization in 5 seconds...");
    
    // Keep visualizing for 5 seconds
    if (visualizer_)
    {
      visualizer_->visualizeCollisionSpheres(
          initial_traj,
          optimization_task_->getCachedSphereLocations(),
          robot_model_,
          group_name_,
          optimization_task_->getCollisionClearance(),
          nullptr
      );
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 2 Hz = 500ms

  }
  
  RCLCPP_INFO(getLogger(), "Starting optimization...");
  RCLCPP_INFO(getLogger(), "========================================================\n");
  
  // Run optimization
  if (!ngd_planner_->solve())
  {
    RCLCPP_ERROR(getLogger(), "NGD optimization failed");
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return;
  }

  // Get the optimized trajectory
  Trajectory ngd_traj = ngd_planner_->getCurrentTrajectory();
  
  RCLCPP_INFO(getLogger(), "Got trajectory with %zu nodes", ngd_traj.nodes.size());
  
  // Convert result to MoveIt trajectory
  trajectory_msgs::msg::JointTrajectory joint_traj;
  if (!ngdTrajectoryToJointTrajectory(ngd_traj, joint_traj))
  {
    RCLCPP_ERROR(getLogger(), "Failed to convert trajectory");
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return;
  }
  RCLCPP_INFO(getLogger(), "Converted to JointTrajectory with %zu points", joint_traj.points.size());
  
  // Populate response
  res.trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, getGroupName());
  res.trajectory->setRobotTrajectoryMsg(getPlanningScene()->getCurrentState(), joint_traj);
  
  res.planning_time = (node_->now() - start_time).seconds();
  res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  
  RCLCPP_INFO(getLogger(), "NGD planning succeeded in %.3f seconds", res.planning_time);
  
  return;
}

void NGDPlanner::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  planning_interface::MotionPlanResponse simple_res;
  solve(simple_res);
  if (simple_res.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    res.trajectory.push_back(simple_res.trajectory);
    res.processing_time.push_back(simple_res.planning_time);
    res.description.push_back("NGD solution");
  }
  res.error_code = simple_res.error_code;
}

bool NGDPlanner::terminate()
{
  return false;
}

void NGDPlanner::clear()
{
  ngd_planner_.reset();
  optimization_task_.reset();
  setup();
}

bool NGDPlanner::canServiceRequest(const moveit_msgs::msg::MotionPlanRequest& req) const
{
  if (req.group_name != group_name_)
    return false;
  
  if (req.start_state.joint_state.name.empty() || req.goal_constraints.empty())
    return false;
  
  return true;
}

void NGDPlanner::setMotionPlanRequest(const planning_interface::MotionPlanRequest& req)
{
  request_ = req;
}

void NGDPlanner::setPlanningScene(const planning_scene::PlanningSceneConstPtr& scene)
{
  planning_scene_ = scene;

  if (optimization_task_)
  {
    optimization_task_->setPlanningScene(scene);
  }
  
  RCLCPP_INFO(getLogger(), "NGDPlanner::setPlanningScene complete");
}

bool NGDPlanner::getStartAndGoal(Eigen::VectorXd& start, Eigen::VectorXd& goal)
{
  const moveit::core::JointModelGroup* jmg = 
    robot_model_->getJointModelGroup(group_name_);
  
  if (!jmg)
  {
    RCLCPP_ERROR(getLogger(), "Joint model group '%s' not found", group_name_.c_str());
    return false;
  }

  // Get start state
  moveit::core::RobotState start_state(robot_model_);
  moveit::core::robotStateMsgToRobotState(request_.start_state, start_state);
  
  std::vector<double> start_positions;
  start_state.copyJointGroupPositions(jmg, start_positions);
  
  start = Eigen::VectorXd::Map(start_positions.data(), start_positions.size());

  // Get goal state from goal constraints
  if (request_.goal_constraints.empty() || 
      request_.goal_constraints[0].joint_constraints.empty())
  {
    RCLCPP_ERROR(getLogger(), "No joint constraints in goal");
    return false;
  }

  const auto& joint_constraints = request_.goal_constraints[0].joint_constraints;
  std::vector<double> goal_positions(start_positions.size());
  
  // Initialize with start positions
  goal_positions = start_positions;
  
  for (size_t i = 0; i < joint_constraints.size(); ++i)
  {
    const std::string& joint_name = joint_constraints[i].joint_name;
    int joint_index = -1;
    
    const std::vector<std::string>& joint_names = jmg->getVariableNames();
    for (size_t j = 0; j < joint_names.size(); ++j)
    {
      if (joint_names[j] == joint_name)
      {
        joint_index = j;
        break;
      }
    }
    
    if (joint_index >= 0)
    {
      goal_positions[joint_index] = joint_constraints[i].position;
    }
  }
  
  goal = Eigen::VectorXd::Map(goal_positions.data(), goal_positions.size());
  
  return true;
}

bool NGDPlanner::ngdTrajectoryToJointTrajectory(
    const Trajectory& ngd_traj,
    trajectory_msgs::msg::JointTrajectory& joint_traj)
{
  const moveit::core::JointModelGroup* jmg = 
      robot_model_->getJointModelGroup(getGroupName());
  
  if (!jmg)
    return false;
  
  joint_traj.joint_names = jmg->getVariableNames();
  joint_traj.points.resize(ngd_traj.nodes.size());
  
  for (size_t i = 0; i < ngd_traj.nodes.size(); ++i)
  {
    // Convert Eigen::VectorXf to std::vector<double>
    const auto& node_pos = ngd_traj.nodes[i].position;
    joint_traj.points[i].positions.resize(node_pos.size());
    for (int j = 0; j < node_pos.size(); ++j)
    {
      joint_traj.points[i].positions[j] = static_cast<double>(node_pos[j]);
    }
    
    // Compute time for this waypoint (evenly distributed)
    double time = (ngd_traj.total_time * i) / (ngd_traj.nodes.size() - 1);
    joint_traj.points[i].time_from_start = rclcpp::Duration::from_seconds(time);
  }
  
  return true;
}


bool NGDPlanner::getConfigData(const rclcpp::Node::SharedPtr& node,
                  std::map<std::string, NGDConfig>& config,
                  const std::string& param)
{
  std::vector<std::string> planning_groups;
  const std::string param_group = param + ".ngd.planning_groups";
  planning_groups = getParam<std::vector<std::string>>(node, param_group, {});
  if (planning_groups.empty()){
    RCLCPP_ERROR(getLogger(), "  ✗ Could not find parameter 'ngd.planning_groups'");
    return false;
  }

  RCLCPP_INFO(getLogger(), "  ✓ Found planning_groups with %zu entries:", planning_groups.size());
  for (const auto& group : planning_groups)
    RCLCPP_INFO(getLogger(), "    - %s", group.c_str());

  for (const auto& group : planning_groups)
  {
    NGDConfig group_config;
    std::string base = param + ".ngd." + group;
    std::string group_param = base + ".ngd_planner";

    // TODO: Try get_parameter_or function
    group_config.num_iterations = getParam<int>(node, group_param + ".num_iterations", 15);
    group_config.num_samples = getParam<int>(node, group_param + ".num_samples", 3000);
    group_config.temperature = getParam<double>(node, group_param + ".temperature", 1.5);
    group_config.learning_rate = getParam<double>(node, group_param + ".learning_rate", 0.0001);
    group_config.num_discretization = getParam<int>(node, group_param + ".num_discretization", 20);
    group_config.total_time = getParam<double>(node, group_param + ".total_time", 5.0);
    group_config.node_collision_radius = getParam<double>(node, group_param + ".node_collision_radius", 0.1);

    // push into map
    config[group] = group_config;

    RCLCPP_INFO(getLogger(), "Loaded NGD config for group '%s'", group.c_str());
  }

  return true;
}


bool NGDPlanner::getConfigData(const std::string& yaml_dict,
                std::map<std::string, NGDConfig>& config)
{
  try{
    YAML::Node root = YAML::LoadFile(yaml_dict);
    YAML::Node ngd_node = root["ngd"];
    for (auto& [group_name, group_config] : config)
    {
      if (!ngd_node[group_name])
        continue;
      
      YAML::Node planner_node = ngd_node[group_name]["ngd_planner"];
      if (planner_node["num_iterations"])
        group_config.num_iterations = planner_node["num_iterations"].as<int>();
      if (planner_node["num_samples"])
        group_config.num_samples = planner_node["num_samples"].as<int>();
      if (planner_node["temperature"])
        group_config.temperature = planner_node["temperature"].as<double>();
      if (planner_node["learning_rate"])
        group_config.learning_rate = planner_node["learning_rate"].as<double>();
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