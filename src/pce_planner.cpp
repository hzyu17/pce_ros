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
  setup();  // All logging happens in setup()
}


PCEPlanner::~PCEPlanner()
{
  pce_planner_.reset();
  pce_task_.reset();
  visualizer_.reset();
}

void PCEPlanner::setup()
{ 
  // Create optimization task
  pce_task_ = std::make_shared<PCEOptimizationTask>(
      robot_model_, 
      group_name_,
      config_
  ); 

  // Pass visualizer to task
  if (visualizer_)
  {
    pce_task_->setVisualizer(visualizer_);
  }
  
  // Create PCE planner
  pce_planner_ = std::make_shared<ProximalCrossEntropyMotionPlanner>(pce_task_);

  // Load PCE configuration from YAML (with defaults)
  if (config_.hasMember("pce_planner"))
  {
    XmlRpc::XmlRpcValue& pce_planner = config_["pce_planner"];

    if (pce_planner.hasMember("num_iterations"))
    {
      pce_config_.num_iterations = static_cast<int>(pce_planner["num_iterations"]);
    }
    else
    {
      pce_config_.num_iterations = 15;
    }
    
    if (pce_planner.hasMember("num_samples"))
    {
      pce_config_.num_samples = static_cast<int>(pce_planner["num_samples"]);
    }
    else
    {
      pce_config_.num_samples = 3000;
    }

    if (pce_planner.hasMember("temperature"))
    {
      pce_config_.temperature = static_cast<double>(pce_planner["temperature"]);
    }
    else
    {
      pce_config_.temperature = 1.5f;
    }

    if (pce_planner.hasMember("eta"))
    {
      pce_config_.eta = static_cast<double>(pce_planner["eta"]);
    }
    else
    {
      pce_config_.eta = 1.0f;
    }
    
    if (pce_planner.hasMember("num_discretization"))
    {
      pce_config_.num_discretization = static_cast<int>(pce_planner["num_discretization"]);
    }
    else
    {
      pce_config_.num_discretization = 20;
    }

    if (pce_planner.hasMember("total_time"))
    {
      pce_config_.total_time = static_cast<double>(pce_planner["total_time"]);
    }
    else
    {
      pce_config_.total_time = 5.0f;
    }

    if (pce_planner.hasMember("node_collision_radius"))
    {
      pce_config_.node_collision_radius = static_cast<double>(pce_planner["node_collision_radius"]);
    }
    else
    {
      pce_config_.node_collision_radius = 0.1f;
    }
  }
  else
  {
    ROS_WARN("No pce_planner section found in config, using all defaults");
    pce_config_.num_iterations = 15;
    pce_config_.num_samples = 3000;
    pce_config_.temperature = 1.5f;
    pce_config_.eta = 1.0f;
    pce_config_.num_discretization = 20;
    pce_config_.total_time = 5.0f;
    pce_config_.node_collision_radius = 0.1f;
  }

  // ====================================================================
  // CONSOLIDATED CONFIGURATION SUMMARY - ASCII version for compatibility
  // ====================================================================
  ROS_INFO("+============================================================+");
  ROS_INFO("|           PCE PLANNER CONFIGURATION                        |");
  ROS_INFO("+============================================================+");
  ROS_INFO("| Planning Group: %-42s |", group_name_.c_str());
  ROS_INFO("| Visualizer:     %-42s |", visualizer_ ? "Enabled" : "Disabled");
  ROS_INFO("+============================================================+");
  ROS_INFO("| OPTIMIZATION PARAMETERS                                    |");
  ROS_INFO("+------------------------------------------------------------+");
  ROS_INFO("|   Samples per iteration:  %-32d |", pce_config_.num_samples);
  ROS_INFO("|   Max iterations:         %-32d |", pce_config_.num_iterations);
  ROS_INFO("|   Temperature:            %-32.3f |", pce_config_.temperature);
  ROS_INFO("|   Proximal step size (eta):      %-32.3f |", pce_config_.eta);
  ROS_INFO("+------------------------------------------------------------+");
  ROS_INFO("| TRAJECTORY PARAMETERS                                      |");
  ROS_INFO("+------------------------------------------------------------+");
  ROS_INFO("|   Waypoints:              %-32d |", pce_config_.num_discretization);
  ROS_INFO("|   Total time:             %-32.2f |", pce_config_.total_time);
  ROS_INFO("|   Node collision radius:  %-32.3f |", pce_config_.node_collision_radius);
  ROS_INFO("+------------------------------------------------------------+");
  ROS_INFO("| COLLISION PARAMETERS                                       |");
  ROS_INFO("+------------------------------------------------------------+");
  ROS_INFO("|   Collision clearance:    %-32.3f |", pce_task_->getCollisionClearance());
  ROS_INFO("|   Collision threshold:    %-32.3f |", pce_task_->getCollisionThreshold());
  ROS_INFO("|   Sigma obs (weight):     %-32.3f |", pce_task_->getSigmaObs()); 
  ROS_INFO("|   Sphere overlap ratio:   %-32.3f |", pce_task_->getSphereOverlapRatio());
  ROS_INFO("+============================================================+");
}


void PCEPlanner::setMotionPlanRequest(const planning_interface::MotionPlanRequest& req)
{
  request_ = req;
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
  
  ROS_INFO("PCEPlanner::setPlanningScene complete");
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
  
  // Set planning request in task
  moveit_msgs::MoveItErrorCodes error_code;
  if (!pce_task_->setMotionPlanRequest(getPlanningScene(), request_, error_code))
  {
    ROS_ERROR("Failed to set motion plan request");
    res.error_code_ = error_code;
    return false;
  }
  
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
    
  try
  {
    if (!pce_planner_->initialize(pce_config_))
    {
      ROS_ERROR("Failed to initialize PCE planner");
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return false;
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Exception in pce_planner_->initialize: %s", e.what());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return false;
  }
  catch (...)
  {
    ROS_ERROR("Unknown exception in pce_planner_->initialize");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return false;
  }

  ROS_INFO("========================================================");
  ROS_INFO("COLLISION SPHERE PREVIEW");
  ROS_INFO("========================================================");
  ROS_INFO("Visualizing collision checking spheres for initial trajectory...");
  
  // Get the initial trajectory (use copy, not reference)
  Trajectory initial_traj = pce_planner_->getCurrentTrajectory();
  
  // Compute collision cost (which caches sphere locations)
  float initial_cost = pce_task_->computeCollisionCost(initial_traj);
  ROS_INFO("Initial trajectory collision cost: %.4f", initial_cost);
  
  // Visualize the collision spheres
  if (!visualizer_) {
      ROS_WARN("Visualizer is NULL - markers will not be published!");
  } else {
      ROS_INFO("Visualizer is active, publishing markers...");

      visualizer_->visualizeCollisionSpheres(
        initial_traj,
        pce_task_->getCachedSphereLocations(),
        robot_model_,
        group_name_,
        pce_task_->getCollisionClearance(),
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
            pce_task_->getCachedSphereLocations().empty() ? 0 : 
            pce_task_->getCachedSphereLocations()[0].size());
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
            pce_task_->getCachedSphereLocations(),
            robot_model_,
            group_name_,
            pce_task_->getCollisionClearance(),
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
  if (!pce_planner_->solve())
  {
    ROS_ERROR("PCE optimization failed");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }

  // Get the optimized trajectory
  Trajectory pce_traj = pce_planner_->getCurrentTrajectory();
  
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
  std::string group = getGroupName();

  if (!robot_model_)
  {
    ROS_ERROR("DEBUG: robot_model_ is NULL!");
    return false;
  }

  const moveit::core::JointModelGroup* jmg = 
      robot_model_->getJointModelGroup(group);
  
  if (!jmg)
  {
    ROS_ERROR("DEBUG: JointModelGroup is NULL for '%s'!", group.c_str());
    return false;
  }
  
  // Get start state
  moveit::core::RobotState start_state(robot_model_);
  
  try
  {
    moveit::core::robotStateMsgToRobotState(request_.start_state, start_state);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("DEBUG: Exception in robotStateMsgToRobotState: %s", e.what());
    return false;
  }
  std::vector<double> start_positions;
  start_state.copyJointGroupPositions(jmg, start_positions);
  
  start = Eigen::Map<Eigen::VectorXd>(start_positions.data(), start_positions.size());
  
  if (!request_.goal_constraints.empty())
  {
    
    if (!request_.goal_constraints[0].joint_constraints.empty())
    {
      goal.resize(start.size());
      
      for (size_t i = 0; i < request_.goal_constraints[0].joint_constraints.size(); ++i)
      {
        const auto& jc = request_.goal_constraints[0].joint_constraints[i];
        auto idx = jmg->getVariableGroupIndex(jc.joint_name);
        
        if (idx < goal.size())
        {
          goal[idx] = jc.position;
        }
        else
        {
          ROS_WARN("DEBUG: Index %zu out of bounds (size=%ld)", idx, goal.size());
        }
      }
      return true;
    }
  }
  
  ROS_WARN("DEBUG: No valid goal constraints found!");
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