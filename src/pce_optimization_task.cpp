#include "pce_optimization_task.h"


namespace pce_ros
{

PCEOptimizationTask::PCEOptimizationTask(
    moveit::core::RobotModelConstPtr robot_model_ptr,
    const std::string& group_name,
    const XmlRpc::XmlRpcValue& config)
  : robot_model_ptr_(robot_model_ptr)
  , group_name_(group_name)
{
  // TODO: Load cost function plugins from config (similar to STOMP)
  // For now, you can start with a simple built-in cost function
  
  ROS_INFO("PCEOptimizationTask initialized for group '%s'", group_name_.c_str());

  // Create visualization utility
  VisualizationConfig viz_config;
  
  if (config.hasMember("visualize_collision_spheres"))
  {
    viz_config.enable_collision_spheres = static_cast<bool>(config["visualize_collision_spheres"]);
  }
  if (config.hasMember("visualize_trajectory"))
  {
    viz_config.enable_trajectory = static_cast<bool>(config["visualize_trajectory"]);
  }
  
  viz_config.collision_clearance = collision_clearance_;
  
  ROS_INFO("Creating visualizer...");
  visualizer_ = std::make_unique<PCEVisualization>(viz_config);
  ROS_INFO("Visualizer created successfully");
  
  ROS_INFO("PCEOptimizationTask initialized");

}

PCEOptimizationTask::~PCEOptimizationTask()
{
}

void PCEOptimizationTask::createDistanceField()
{
  if (!planning_scene_ptr_)
  {
    ROS_ERROR("Cannot create distance field without planning scene");
    return;
  }
  
  // Define workspace bounds
  double size_x = 3.0;
  double size_y = 3.0;
  double size_z = 3.0;
  double resolution = 0.02;  // 2cm resolution
  double origin_x = -1.5;
  double origin_y = -1.5;
  double origin_z = -1.5;
  
  // Create distance field
  distance_field_.reset(new distance_field::PropagationDistanceField(
      size_x, size_y, size_z,
      resolution,
      origin_x, origin_y, origin_z,
      0.4  // max propagation distance
  ));
  
  // Get collision objects
  std::vector<std::string> object_ids;
  object_ids = planning_scene_ptr_->getWorld()->getObjectIds();
  
  ROS_INFO("Found %zu collision objects", object_ids.size());
  
  // Use Eigen-aligned vector
  EigenSTL::vector_Vector3d obstacle_points;
  
  for (const std::string& object_id : object_ids)
  {
    collision_detection::World::ObjectConstPtr object = 
        planning_scene_ptr_->getWorld()->getObject(object_id);
    
    if (!object)
    {
      continue;
    }
    
    // Process each shape in the object
    for (size_t i = 0; i < object->shapes_.size(); ++i)
    {
      const shapes::ShapeConstPtr& shape = object->shapes_[i];
      const Eigen::Isometry3d& pose = object->shape_poses_[i];
      
      if (!shape)
      {
        continue;
      }
      
      // Create body using the correct API
      std::unique_ptr<bodies::Body> body;
      
      switch (shape->type)
      {
        case shapes::BOX:
          body.reset(new bodies::Box(shape.get()));
          break;
        case shapes::SPHERE:
          body.reset(new bodies::Sphere(shape.get()));
          break;
        case shapes::CYLINDER:
          body.reset(new bodies::Cylinder(shape.get()));
          break;
        case shapes::MESH:
          body.reset(new bodies::ConvexMesh(shape.get()));
          break;
        default:
          ROS_WARN("Unsupported shape type: %d", shape->type);
          continue;
      }
      
      if (!body)
      {
        continue;
      }
      
      body->setPose(pose);
      
      // Get bounding sphere
      bodies::BoundingSphere bsphere;
      body->computeBoundingSphere(bsphere);
      
      // Sample points within bounding sphere
      double min_x = bsphere.center.x() - bsphere.radius;
      double max_x = bsphere.center.x() + bsphere.radius;
      double min_y = bsphere.center.y() - bsphere.radius;
      double max_y = bsphere.center.y() + bsphere.radius;
      double min_z = bsphere.center.z() - bsphere.radius;
      double max_z = bsphere.center.z() + bsphere.radius;
      
      for (double x = min_x; x <= max_x; x += resolution)
      {
        for (double y = min_y; y <= max_y; y += resolution)
        {
          for (double z = min_z; z <= max_z; z += resolution)
          {
            Eigen::Vector3d point(x, y, z);
            if (body->containsPoint(point))
            {
              obstacle_points.push_back(point);
            }
          }
        }
      }
    }
  }
  
  // Add points to distance field
  if (!obstacle_points.empty())
  {
    distance_field_->addPointsToField(obstacle_points);
    ROS_INFO("Distance field created with %zu obstacle voxels", obstacle_points.size());
  }
  else
  {
    ROS_WARN("No obstacle points found - distance field is empty");
  }
}


bool PCEOptimizationTask::setMotionPlanRequest(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code)
{
  planning_scene_ptr_ = planning_scene;
  plan_request_ = req;
  
  // Create/update distance field
  createDistanceField();
  
  return true;
}


// CHOMP-style obstacle cost function
float PCEOptimizationTask::getObstacleCost(double distance) const
{
  const double epsilon = collision_clearance_;
  
  if (distance < 0.0)
  {
    // Inside obstacle: linear penalty + constant
    return -distance + 0.5 * epsilon;
  }
  else if (distance < epsilon)
  {
    // Within safety margin: quadratic penalty
    double diff = distance - epsilon;
    return 0.5 * (1.0 / epsilon) * diff * diff;
  }
  else
  {
    // Safe distance: no cost
    return 0.0;
  }
}


std::vector<Eigen::Vector3d> PCEOptimizationTask::getSphereLocations(
    const moveit::core::RobotState& state) const
{
  std::vector<Eigen::Vector3d> sphere_locations;
  
  const moveit::core::JointModelGroup* jmg = 
      robot_model_ptr_->getJointModelGroup(group_name_);
  
  if (!jmg)
  {
    return sphere_locations;
  }
  
  const std::vector<const moveit::core::LinkModel*>& links = jmg->getLinkModels();
  
  for (const auto* link : links)
  {
    const Eigen::Isometry3d& link_transform = state.getGlobalLinkTransform(link);
    
    // Get collision shapes for this link
    const std::vector<shapes::ShapeConstPtr>& shapes = link->getShapes();
    const EigenSTL::vector_Isometry3d& shape_poses = link->getCollisionOriginTransforms();
    
    for (size_t s = 0; s < shapes.size(); ++s)
    {
      const shapes::ShapeConstPtr& shape = shapes[s];
      Eigen::Isometry3d shape_transform = link_transform * shape_poses[s];
      
      // Sample points based on shape type
      if (shape->type == shapes::CYLINDER)
      {
        const shapes::Cylinder* cylinder = 
            static_cast<const shapes::Cylinder*>(shape.get());
        
        double radius = cylinder->radius;
        double length = cylinder->length;
        
        // Sample points along cylinder axis
        int num_samples = std::max(3, static_cast<int>(length / 0.05)); // Every 5cm
        
        for (int i = 0; i < num_samples; ++i)
        {
          double t = static_cast<double>(i) / (num_samples - 1);
          double z = -length/2 + t * length;
          
          Eigen::Vector3d local_point(0, 0, z);
          Eigen::Vector3d world_point = shape_transform * local_point;
          sphere_locations.push_back(world_point);
        }
      }
      else if (shape->type == shapes::SPHERE)
      {
        const shapes::Sphere* sphere = 
            static_cast<const shapes::Sphere*>(shape.get());
        
        Eigen::Vector3d center = shape_transform.translation();
        sphere_locations.push_back(center);
      }
      else if (shape->type == shapes::BOX)
      {
        const shapes::Box* box = static_cast<const shapes::Box*>(shape.get());
        
        // Sample corners and center
        double dx = box->size[0] / 2;
        double dy = box->size[1] / 2;
        double dz = box->size[2] / 2;
        
        std::vector<Eigen::Vector3d> local_points = {
          Eigen::Vector3d(0, 0, 0),      // Center
          Eigen::Vector3d(dx, dy, dz),
          Eigen::Vector3d(dx, dy, -dz),
          Eigen::Vector3d(dx, -dy, dz),
          Eigen::Vector3d(dx, -dy, -dz),
          Eigen::Vector3d(-dx, dy, dz),
          Eigen::Vector3d(-dx, dy, -dz),
          Eigen::Vector3d(-dx, -dy, dz),
          Eigen::Vector3d(-dx, -dy, -dz)
        };
        
        for (const auto& local_pt : local_points)
        {
          Eigen::Vector3d world_pt = shape_transform * local_pt;
          sphere_locations.push_back(world_pt);
        }
      }
    }
  }
  
  return sphere_locations;
}


float PCEOptimizationTask::computeCollisionCostSimple(const Trajectory& trajectory) const
{
  if (!planning_scene_ptr_)
  {
    ROS_ERROR("No planning scene set!");
    return std::numeric_limits<float>::infinity();
  }
  
  float total_cost = 0.0f;
  collision_detection::CollisionRequest collision_request;
  collision_request.group_name = group_name_;
  collision_detection::CollisionResult collision_result;
  
  // Check each waypoint for collisions
  for (size_t i = 0; i < trajectory.nodes.size(); ++i)
  {
    moveit::core::RobotState state(robot_model_ptr_);
    
    // Convert trajectory node to robot state
    if (!trajectoryToRobotState(trajectory, i, state))
    {
      return std::numeric_limits<float>::infinity();
    }
    
    // Check collision
    collision_result.clear();
    planning_scene_ptr_->checkCollision(collision_request, collision_result, state);
    
    if (collision_result.collision)
    {
      // Penalize collision (you can make this more sophisticated)
      total_cost += 1000.0f;
    }
    
    // Optional: Check distance to obstacles for smooth costs
    // planning_scene_ptr_->distanceToCollision(state);
  }
  
  return total_cost;
}


float PCEOptimizationTask::computeCollisionCost(const Trajectory& trajectory) const
{
  if (!distance_field_)
  {
    ROS_WARN_THROTTLE(1.0, "Distance field not initialized, using fallback collision checking");
    // Fallback to simple collision checking
    return computeCollisionCostSimple(trajectory);
  }
  
  float total_cost = 0.0f;
  
  // Iterate over all waypoints in trajectory
  for (size_t i = 0; i < trajectory.nodes.size(); ++i)
  {
    moveit::core::RobotState state(robot_model_ptr_);
    
    if (!trajectoryToRobotState(trajectory, i, state))
    {
      return std::numeric_limits<float>::infinity();
    }
    
    // Get sphere locations on robot body
    std::vector<Eigen::Vector3d> sphere_locations = getSphereLocations(state);
    
    // Accumulate cost from all body points
    float waypoint_cost = 0.0f;
    
    for (const Eigen::Vector3d& point : sphere_locations)
    {
      // Query signed distance from distance field
      double distance = distance_field_->getDistance(point.x(), point.y(), point.z());
      
      // Apply CHOMP cost function
      float point_cost = getObstacleCost(distance);
      waypoint_cost += point_cost;
    }
    
    total_cost += waypoint_cost;
  }
  
  return total_cost;
}


bool PCEOptimizationTask::filterTrajectory(Trajectory& trajectory, int iteration_number)
{
  bool filtered = false;
  
  // Apply joint limit clamping
  const auto* joint_model_group = robot_model_ptr_->getJointModelGroup(group_name_);
  if (!joint_model_group)
  {
    return false;
  }
  
  // Get the active joint models - THIS LINE WAS MISSING
  const std::vector<const moveit::core::JointModel*>& joint_models = 
      joint_model_group->getActiveJointModels();
  
  // Check that dimensions match
  if (trajectory.nodes.empty() || 
      joint_models.size() != static_cast<size_t>(trajectory.nodes[0].position.size()))
  {
    ROS_ERROR("Trajectory dimensions don't match joint model group");
    return false;
  }
  
  // Iterate over each waypoint in the trajectory
  for (auto& node : trajectory.nodes)
  {
    // Iterate over each joint
    for (size_t j = 0; j < joint_models.size(); ++j)
    {
      const auto* joint_model = joint_models[j];
      
      // Get the variable bounds for this joint
      const moveit::core::JointModel::Bounds& bounds = joint_model->getVariableBounds();
      
      // Check if the joint has bounds
      if (bounds.empty())
        continue;
      
      // Clamp to the first variable's bounds (for revolute/prismatic joints)
      const moveit::core::VariableBounds& var_bounds = bounds[0];
      
      if (var_bounds.position_bounded_)
      {
        if (node.position[j] < var_bounds.min_position_)
        {
          node.position[j] = static_cast<float>(var_bounds.min_position_);
          filtered = true;
        }
        else if (node.position[j] > var_bounds.max_position_)
        {
          node.position[j] = static_cast<float>(var_bounds.max_position_);
          filtered = true;
        }
      }
    }
  }
  
  if (filtered)
  {
    ROS_DEBUG("PCE: Joint limits applied at iteration %d", iteration_number);
  }
  
  return filtered;
}


void PCEOptimizationTask::postIteration(int iteration_number, float cost, 
                                        const Trajectory& trajectory)
{
  ROS_INFO("PCE Iteration %d: cost = %.4f", iteration_number, cost);  // This should print
  
  // Check if visualizer exists
  if (!visualizer_)
  {
    ROS_ERROR("Visualizer is NULL!");
    return;
  }
  
  ROS_INFO("Calling visualization...");
  
  // Visualize current trajectory
  visualizer_->visualizeCollisionSpheres(trajectory, robot_model_ptr_, group_name_, distance_field_);
  visualizer_->visualizeTrajectory(trajectory, robot_model_ptr_, group_name_, iteration_number);
  
  ROS_INFO("Visualization called successfully");
  
  ros::Duration(0.05).sleep();
  ros::spinOnce();
}


void PCEOptimizationTask::done(bool success, int total_iterations,
                               float final_cost, const Trajectory& trajectory)
{
  ROS_INFO("PCE optimization %s after %d iterations (cost: %.4f)",
           success ? "succeeded" : "failed", total_iterations, final_cost);
  
  if (success)
  {
    visualizer_->visualizeCollisionSpheres(trajectory, robot_model_ptr_, group_name_, distance_field_);
    visualizer_->visualizeTrajectory(trajectory, robot_model_ptr_, group_name_, total_iterations);
  }
}

void PCEOptimizationTask::initialize(size_t num_dimensions, const PathNode& start,
                                     const PathNode& goal, size_t num_nodes,
                                     float total_time)
{
  ROS_DEBUG("PCE task initialized: %zu dimensions, %zu nodes", 
            num_dimensions, num_nodes);
}

bool PCEOptimizationTask::trajectoryToRobotState(
    const Trajectory& trajectory, size_t timestep,
    moveit::core::RobotState& state) const
{
  const auto* joint_model_group = robot_model_ptr_->getJointModelGroup(group_name_);
  if (!joint_model_group)
  {
    ROS_ERROR("Joint model group '%s' not found", group_name_.c_str());
    return false;
  }
  
  // Get variable names (not joint names - variables include all DOFs)
  const std::vector<std::string>& variable_names = joint_model_group->getVariableNames();
  const Eigen::VectorXf& positions_float = trajectory.nodes[timestep].position;  // VectorXf
  
  if (variable_names.size() != static_cast<size_t>(positions_float.size()))
  {
    ROS_ERROR("Dimension mismatch: trajectory has %d DOFs, group has %lu variables",
              positions_float.size(), variable_names.size());
    return false;
  }
  
  // Convert Eigen::VectorXf to std::vector<double> explicitly
  std::vector<double> positions(positions_float.size());
  for (Eigen::Index i = 0; i < positions_float.size(); ++i)
  {
    positions[i] = static_cast<double>(positions_float[i]);
  }
  
  // Set joint group positions
  state.setJointGroupPositions(joint_model_group, positions);
  state.update();
  
  return true;
}

} // namespace pce_ros