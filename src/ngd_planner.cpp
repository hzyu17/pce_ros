/**
 * @file ngd_planner.cpp
 * @brief NGD planner implementation - matches PCE planner structure
 */
#include "ngd_planner.h"
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>

namespace pce_ros
{

NGDPlanner::NGDPlanner(const std::string& group,
                       const XmlRpc::XmlRpcValue& config,
                       const moveit::core::RobotModelConstPtr& model,
                       std::shared_ptr<PCEVisualization> visualizer)
  : planning_interface::PlanningContext("ngd_planner", group)
  , group_name_(group)
  , config_(config)
  , robot_model_(model)
  , visualizer_(visualizer)
{
  nh_.reset(new ros::NodeHandle("~"));
  setup();
}

NGDPlanner::~NGDPlanner()
{
}

void NGDPlanner::setup()
{ 
  // Create optimization task
  optimization_task_ = std::make_shared<PCEOptimizationTask>(
      robot_model_, 
      group_name_,
      config_
  ); 

  // Pass visualizer to task
  if (visualizer_)
  {
    optimization_task_->setVisualizer(visualizer_);
  }
  
  // Create PCE planner
  ngd_planner_ = std::make_shared<NGDMotionPlanner>(optimization_task_);

  // Load PCE configuration from YAML (with defaults)
  if (config_.hasMember("ngd_planner"))
  {
    XmlRpc::XmlRpcValue& ngd_planner = config_["ngd_planner"];

    if (ngd_planner.hasMember("num_iterations"))
    {
      ngd_config_.num_iterations = static_cast<int>(ngd_planner["num_iterations"]);
    }
    else
    {
      ngd_config_.num_iterations = 15;
    }
    
    if (ngd_planner.hasMember("num_samples"))
    {
      ngd_config_.num_samples = static_cast<int>(ngd_planner["num_samples"]);
    }
    else
    {
      ngd_config_.num_samples = 3000;
    }

    if (ngd_planner.hasMember("temperature"))
    {
      ngd_config_.temperature = static_cast<double>(ngd_planner["temperature"]);
    }
    else
    {
      ngd_config_.temperature = 1.5f;
    }
    
    if (ngd_planner.hasMember("num_discretization"))
    {
      ngd_config_.num_discretization = static_cast<int>(ngd_planner["num_discretization"]);
    }
    else
    {
      ngd_config_.num_discretization = 20;
    }

    if (ngd_planner.hasMember("total_time"))
    {
      ngd_config_.total_time = static_cast<double>(ngd_planner["total_time"]);
    }
    else
    {
      ngd_config_.total_time = 5.0f;
    }

    if (ngd_planner.hasMember("node_collision_radius"))
    {
      ngd_config_.node_collision_radius = static_cast<double>(ngd_planner["node_collision_radius"]);
    }
    else
    {
      ngd_config_.node_collision_radius = 0.1f;
    }
  }
  else
  {
    ROS_WARN("No ngd_planner section found in config, using all defaults");
    ngd_config_.num_iterations = 15;
    ngd_config_.num_samples = 3000;
    ngd_config_.temperature = 1.5f;
    ngd_config_.num_discretization = 20;
    ngd_config_.total_time = 5.0f;
    ngd_config_.node_collision_radius = 0.1f;
  }

  // ====================================================================
  // CONSOLIDATED CONFIGURATION SUMMARY - ASCII version for compatibility
  // ====================================================================
  ROS_INFO("+============================================================+");
  ROS_INFO("|           NGD PLANNER CONFIGURATION                        |");
  ROS_INFO("+============================================================+");
  ROS_INFO("| Planning Group: %-42s |", group_name_.c_str());
  ROS_INFO("| Visualizer:     %-42s |", visualizer_ ? "Enabled" : "Disabled");
  ROS_INFO("+============================================================+");
  ROS_INFO("| OPTIMIZATION PARAMETERS                                    |");
  ROS_INFO("+------------------------------------------------------------+");
  ROS_INFO("|   Samples per iteration:  %-32d |", ngd_config_.num_samples);
  ROS_INFO("|   Max iterations:         %-32d |", ngd_config_.num_iterations);
  ROS_INFO("|   Temperature:            %-32.3f |", ngd_config_.temperature);
  ROS_INFO("+------------------------------------------------------------+");
  ROS_INFO("| TRAJECTORY PARAMETERS                                      |");
  ROS_INFO("+------------------------------------------------------------+");
  ROS_INFO("|   Waypoints:              %-32d |", ngd_config_.num_discretization);
  ROS_INFO("|   Total time:             %-32.2f |", ngd_config_.total_time);
  ROS_INFO("|   Node collision radius:  %-32.3f |", ngd_config_.node_collision_radius);
  ROS_INFO("+------------------------------------------------------------+");
  ROS_INFO("| COLLISION PARAMETERS                                       |");
  ROS_INFO("+------------------------------------------------------------+");
  ROS_INFO("|   Collision clearance:    %-32.3f |", optimization_task_->getCollisionClearance());
  ROS_INFO("|   Collision threshold:    %-32.3f |", optimization_task_->getCollisionThreshold());
  ROS_INFO("|   Sigma obs (weight):     %-32.3f |", optimization_task_->getSigmaObs()); 
  ROS_INFO("|   Sphere overlap ratio:   %-32.3f |", optimization_task_->getSphereOverlapRatio());
  ROS_INFO("+============================================================+");
}


bool NGDPlanner::solve(planning_interface::MotionPlanResponse& res)
{
  ROS_INFO("=== NGDPlanner::solve() CALLED ===");

  ros::WallTime start_time = ros::WallTime::now();
  
  // Get start and goal from MoveIt request
  Eigen::VectorXd start, goal;
  if (!getStartAndGoal(start, goal))
  {
    ROS_ERROR("Failed to get start and goal");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }
  
  // Create optimization task - REUSE PCEOptimizationTask!
  optimization_task_ = std::make_shared<PCEOptimizationTask>(
    robot_model_, group_name_, config_);
  
  // Set planning scene
  optimization_task_->setPlanningScene(planning_scene_);
  
  // Set visualizer if available
  if (visualizer_)
  {
    optimization_task_->setVisualizer(visualizer_);
  }
  
  // Set motion plan request in task
  moveit_msgs::MoveItErrorCodes error_code;
  if (!optimization_task_->setMotionPlanRequest(getPlanningScene(), request_, error_code))
  {
    ROS_ERROR("Failed to set motion plan request");
    res.error_code_ = error_code;
    return false;
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
      ROS_ERROR("Failed to initialize NGD planner");
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return false;
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Exception in ngd_planner_->initialize: %s", e.what());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return false;
  }
  catch (...)
  {
    ROS_ERROR("Unknown exception in ngd_planner_->initialize");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return false;
  }

  ROS_INFO("========================================================");
  ROS_INFO("COLLISION SPHERE PREVIEW");
  ROS_INFO("========================================================");
  ROS_INFO("Visualizing collision checking spheres for initial trajectory...");
  
  // Get the initial trajectory (use copy, not reference)
  Trajectory initial_traj = ngd_planner_->getCurrentTrajectory();
  
  // Compute collision cost (which caches sphere locations)
  float initial_cost = optimization_task_->computeCollisionCost(initial_traj);
  ROS_INFO("Initial trajectory collision cost: %.4f", initial_cost);
  
  // Visualize the collision spheres
  if (!visualizer_) {
      ROS_WARN("Visualizer is NULL - markers will not be published!");
  } else {
      ROS_INFO("Visualizer is active, publishing markers...");

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

    ROS_INFO("--------------------------------------------------------");
    ROS_INFO("Check RViz to see the collision checking spheres.");
    ROS_INFO("Total spheres per waypoint: %zu", 
            optimization_task_->getCachedSphereLocations().empty() ? 0 : 
            optimization_task_->getCachedSphereLocations()[0].size());
    ROS_INFO("Total waypoints: %zu", initial_traj.nodes.size());
    ROS_INFO("--------------------------------------------------------");
    ROS_WARN("Starting optimization in 5 seconds...");
    
    // Keep visualizing for 5 seconds
    ros::Rate rate(2);  // 2 Hz
    for (int i = 0; i < 10; ++i)
    {
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
      ros::spinOnce();
      rate.sleep();
    }
  }
  
  ROS_INFO("Starting optimization...");
  ROS_INFO("========================================================\n");
  
  // Run optimization
  if (!ngd_planner_->solve())
  {
    ROS_ERROR("NGD optimization failed");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }

  // Get the optimized trajectory
  Trajectory ngd_traj = ngd_planner_->getCurrentTrajectory();
  
  ROS_INFO("Got trajectory with %zu nodes", ngd_traj.nodes.size());
  
  // Convert result to MoveIt trajectory
  trajectory_msgs::JointTrajectory joint_traj;
  if (!ngdTrajectoryToJointTrajectory(ngd_traj, joint_traj))
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
  
  ROS_INFO("NGD planning succeeded in %.3f seconds", res.planning_time_);
  
  return true;
}

bool NGDPlanner::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  planning_interface::MotionPlanResponse simple_res;
  bool success = solve(simple_res);
  
  res.trajectory_.push_back(simple_res.trajectory_);
  res.processing_time_.push_back(simple_res.planning_time_);
  res.description_.push_back("NGD solution");
  res.error_code_ = simple_res.error_code_;
  
  return success;
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

bool NGDPlanner::canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const
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
}

bool NGDPlanner::getStartAndGoal(Eigen::VectorXd& start, Eigen::VectorXd& goal)
{
  const moveit::core::JointModelGroup* jmg = 
    robot_model_->getJointModelGroup(group_name_);
  
  if (!jmg)
  {
    ROS_ERROR("Joint model group '%s' not found", group_name_.c_str());
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
    ROS_ERROR("No joint constraints in goal");
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
    trajectory_msgs::JointTrajectory& joint_traj)
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
    joint_traj.points[i].time_from_start = ros::Duration(time);
  }
  
  return true;
}


bool NGDPlanner::getConfigData(ros::NodeHandle& nh,
                               std::map<std::string, XmlRpc::XmlRpcValue>& config,
                               std::string param)
{
  XmlRpc::XmlRpcValue ngd_config;
  if (!nh.getParam(param, ngd_config))
  {
    ROS_ERROR("Failed to load NGD configuration from parameter '%s'", param.c_str());
    return false;
  }

  if (ngd_config.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR("NGD configuration must be a struct");
    return false;
  }

  for (auto it = ngd_config.begin(); it != ngd_config.end(); ++it)
  {
    const std::string& group_name = it->first;
    config[group_name] = it->second;
  }

  return true;
}

} // namespace pce_ros