#include "pce_optimization_task.h"
#include <cmath>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

namespace pce_ros
{

namespace {
rclcpp::Logger getLogger()
{
  return moveit::getLogger("pce_optimization_task");
}
}

PCEOptimizationTask::PCEOptimizationTask(
    const moveit::core::RobotModelConstPtr& robot_model_ptr,
    const std::string& group_name,
    const PCEConfig& config,
    const rclcpp::Node::SharedPtr& node)
  : robot_model_ptr_(robot_model_ptr)
  , group_name_(group_name)
  , node_(node)
{
  // Load parameters from config
  collision_clearance_ = config.collision_clearance;
  collision_threshold_ = config.collision_threshold;
}

void PCEOptimizationTask::setPlanningScene(
    const planning_scene::PlanningSceneConstPtr& scene)
{
  if (!scene)
  {
    RCLCPP_ERROR(getLogger(), "Planning scene is NULL!");
    return;
  }

  // Create diff
  planning_scene_ = scene->diff();
  
  // Create distance field using CHOMP's approach
  createDistanceFieldFromPlanningScene();
}

void PCEOptimizationTask::createDistanceFieldFromPlanningScene()
{  
  // Distance field parameters (same as CHOMP uses)
  double size_x = 3.0;
  double size_y = 3.0;
  double size_z = 4.0;
  double resolution = 0.02;
  double origin_x = -1.5;
  double origin_y = -1.5;
  double origin_z = -2.0;
  double max_distance = 1.0;
  
  distance_field_ = std::make_shared<distance_field::PropagationDistanceField>(
      size_x, size_y, size_z,
      resolution,
      origin_x, origin_y, origin_z,
      max_distance
  );
    
  addCollisionObjectsToDistanceField();
  
  RCLCPP_INFO(getLogger(), "Distance field created [%.1f x %.1f x %.1f, res=%.3f]", 
           size_x, size_y, size_z, resolution);
}


void PCEOptimizationTask::addCollisionObjectsToDistanceField()
{
  if (!distance_field_ || !planning_scene_)
  {
    RCLCPP_ERROR(getLogger(), "Distance field or planning scene is NULL!");
    return;
  }
  
  const collision_detection::WorldConstPtr& world = planning_scene_->getWorld();
  
  if (!world)
  {
    RCLCPP_WARN(getLogger(), "World is NULL!");
    return;
  }
  
  std::vector<std::string> object_ids = world->getObjectIds();
  RCLCPP_INFO(getLogger(), "Adding %zu collision objects to distance field:", object_ids.size());
  
  int total_points = 0;
  
  for (const auto& id : object_ids)
  {
    collision_detection::World::ObjectConstPtr obj = world->getObject(id);
    if (!obj)
    {
      RCLCPP_WARN(getLogger(), "  Object '%s' is NULL!", id.c_str());
      continue;
    }
        
    // Try to get object pose from collision object message
    // We need to query the planning scene for the collision object message
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    planning_scene_->getCollisionObjectMsgs(collision_objects);
    
    Eigen::Isometry3d object_pose = Eigen::Isometry3d::Identity();
    bool found_object_pose = false;
    
    for (const auto& co_msg : collision_objects)
    {
      if (co_msg.id == id)
      {
        // Found the message - get the object-level pose
        tf2::fromMsg(co_msg.pose, object_pose);
        found_object_pose = true;
        break;
      }
    }
    
    if (!found_object_pose)
    {
      RCLCPP_WARN(getLogger(), "    Could not find collision object message for '%s', using identity", id.c_str());
    }
    
    // Process shapes
    for (size_t i = 0; i < obj->shapes_.size(); ++i)
    {
      const shapes::ShapeConstPtr& shape = obj->shapes_[i];
      const Eigen::Isometry3d& shape_pose_relative = obj->shape_poses_[i];
      
      // COMPOSE: world_pose = object_pose * shape_pose_relative
      Eigen::Isometry3d shape_pose_world = object_pose * shape_pose_relative;
      
      RCLCPP_INFO(getLogger(), "    Shape %zu world pose:", i);
      RCLCPP_INFO(getLogger(), "      Position: [%.3f, %.3f, %.3f]",
               shape_pose_world.translation().x(),
               shape_pose_world.translation().y(),
               shape_pose_world.translation().z());
      
      EigenSTL::vector_Vector3d points;
      samplePointsFromShape(shape, shape_pose_world, distance_field_->getResolution(), points);
      
      if (!points.empty())
      {
        distance_field_->addPointsToField(points);
        total_points += points.size();
        RCLCPP_INFO(getLogger(), "    Added %zu points", points.size());
      }
    }
  }
  
  RCLCPP_INFO(getLogger(), "Distance field populated with %d total points", total_points);
  
}


void PCEOptimizationTask::samplePointsFromShape(
    const shapes::ShapeConstPtr& shape,
    const Eigen::Isometry3d& pose,
    double resolution,
    EigenSTL::vector_Vector3d& points)
{
  points.clear();
  
  if (!shape)
  {
    return;
  }
  
  if (shape->type == shapes::BOX)
  {
    const shapes::Box* box = static_cast<const shapes::Box*>(shape.get());
    
    // Sample points throughout the box volume
    int nx = std::max(5, static_cast<int>(std::ceil(box->size[0] / resolution)));
    int ny = std::max(5, static_cast<int>(std::ceil(box->size[1] / resolution)));
    int nz = std::max(5, static_cast<int>(std::ceil(box->size[2] / resolution)));
    
    for (int ix = 0; ix < nx; ++ix)
    {
      for (int iy = 0; iy < ny; ++iy)
      {
        for (int iz = 0; iz < nz; ++iz)
        {
          double x = -box->size[0]/2.0 + (box->size[0] * ix) / (nx - 1);
          double y = -box->size[1]/2.0 + (box->size[1] * iy) / (ny - 1);
          double z = -box->size[2]/2.0 + (box->size[2] * iz) / (nz - 1);
          
          Eigen::Vector3d local_pt(x, y, z);
          Eigen::Vector3d world_pt = pose * local_pt;
          points.push_back(world_pt);
        }
      }
    }
    
    RCLCPP_INFO(getLogger(), "    Box [%.2f x %.2f x %.2f] at [%.2f, %.2f, %.2f]: %d points",
             box->size[0], box->size[1], box->size[2],
             pose.translation().x(), pose.translation().y(), pose.translation().z(),
             static_cast<int>(points.size()));
  }
  else if (shape->type == shapes::SPHERE)
  {
    const shapes::Sphere* sphere = static_cast<const shapes::Sphere*>(shape.get());
    
    double radius = sphere->radius;
    int n_phi = std::max(5, static_cast<int>(std::ceil(radius / resolution)));
    int n_theta = n_phi * 2;
    
    for (int ip = 0; ip < n_phi; ++ip)
    {
      for (int it = 0; it < n_theta; ++it)
      {
        double phi = M_PI * ip / (n_phi - 1);
        double theta = 2.0 * M_PI * it / n_theta;
        
        // Sample throughout volume, not just surface
        for (int ir = 0; ir <= 3; ++ir)
        {
          double r = radius * ir / 3.0;
          double x = r * sin(phi) * cos(theta);
          double y = r * sin(phi) * sin(theta);
          double z = r * cos(phi);
          
          Eigen::Vector3d local_pt(x, y, z);
          Eigen::Vector3d world_pt = pose * local_pt;
          points.push_back(world_pt);
        }
      }
    }
    
    RCLCPP_INFO(getLogger(), "    Sphere radius %.2f at [%.2f, %.2f, %.2f]: %d points",
             radius,
             pose.translation().x(), pose.translation().y(), pose.translation().z(),
             static_cast<int>(points.size()));
  }
  else if (shape->type == shapes::CYLINDER)
  {
    const shapes::Cylinder* cylinder = static_cast<const shapes::Cylinder*>(shape.get());
    
    double radius = cylinder->radius;
    double length = cylinder->length;

    int n_r = std::max(3, static_cast<int>(std::ceil(radius / resolution)));
    int n_theta = std::max(8, static_cast<int>(std::ceil(2.0 * M_PI * radius / resolution)));
    int n_z = std::max(3, static_cast<int>(std::ceil(length / resolution)));

    for (int ir = 0; ir < n_r; ++ir)
    {
      double r = radius * ir / (n_r - 1);
      for (int it = 0; it < n_theta; ++it)
      {
        double theta = 2.0 * M_PI * it / n_theta;
        for (int iz = 0; iz < n_z; ++iz)
        {
          double x = r * cos(theta);
          double y = r * sin(theta);
          double z = -length/2.0 + length * iz / (n_z - 1);
          
          Eigen::Vector3d local_pt(x, y, z);
          Eigen::Vector3d world_pt = pose * local_pt;
          points.push_back(world_pt);
        }
      }
    }
    
    RCLCPP_INFO(getLogger(), "    Cylinder r=%.2f, h=%.2f at [%.2f, %.2f, %.2f]: %d points",
             radius, length,
             pose.translation().x(), pose.translation().y(), pose.translation().z(),
             static_cast<int>(points.size()));
  }
  else
  {
    RCLCPP_WARN(getLogger(), "    Unsupported shape type: %d", static_cast<int>(shape->type));
  }
}


double PCEOptimizationTask::getDistanceAtPoint(const Eigen::Vector3d& point) const
{
  if (!distance_field_)
  {
    RCLCPP_ERROR_THROTTLE(getLogger(), *node_->get_clock(), 1000, "Distance field is NULL!");
    return 1000.0;  // Large distance
  }
  
  return distance_field_->getDistance(point.x(), point.y(), point.z());
}

float PCEOptimizationTask::getObstacleCost(double distance) const
{
  // CHOMP's cost function
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


bool PCEOptimizationTask::setMotionPlanRequest(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const moveit_msgs::msg::MotionPlanRequest& req,
    moveit_msgs::msg::MoveItErrorCodes& error_code)
{
  plan_request_ = req;
  
  setPlanningScene(planning_scene);
  
  return true;
}


std::vector<Eigen::Vector3d> PCEOptimizationTask::getSphereLocations(
    const moveit::core::RobotState& state) const
{
  std::vector<Eigen::Vector3d> sphere_locations;
  
  const moveit::core::JointModelGroup* jmg =
      robot_model_ptr_->getJointModelGroup(group_name_);
  
  if (!jmg)
  {
    RCLCPP_ERROR_ONCE(getLogger(), "Joint model group '%s' not found!", group_name_.c_str());
    return sphere_locations;
  }
  
  const std::vector<const moveit::core::LinkModel*>& links = jmg->getLinkModels();
  
  RCLCPP_INFO_ONCE(getLogger(), "Group '%s' has %zu links", group_name_.c_str(), links.size());
  
  for (const auto* link : links)
  {
    const Eigen::Isometry3d& link_transform = state.getGlobalLinkTransform(link);
    
    // Get collision shapes for this link
    const std::vector<shapes::ShapeConstPtr>& shapes = link->getShapes();
    const EigenSTL::vector_Isometry3d& shape_poses = link->getCollisionOriginTransforms();
    
    RCLCPP_INFO_ONCE(getLogger(), "  Link '%s' has %zu shapes", link->getName().c_str(), shapes.size());
    
    if (shapes.empty())
    {
      // No shapes - add link origin as fallback
      sphere_locations.push_back(link_transform.translation());
      RCLCPP_INFO_ONCE(getLogger(), "    Added link origin as fallback");
      continue;
    }
    
    for (size_t s = 0; s < shapes.size(); ++s)
    {
      const shapes::ShapeConstPtr& shape = shapes[s];
      Eigen::Isometry3d shape_transform = link_transform * shape_poses[s];
      
      RCLCPP_INFO_ONCE(getLogger(), "    Shape %zu type: %d", s, static_cast<int>(shape->type));
      
      // Sample points based on shape type
      if (shape->type == shapes::CYLINDER)
      {
        const shapes::Cylinder* cylinder =
            static_cast<const shapes::Cylinder*>(shape.get());
        double radius = cylinder->radius;
        double length = cylinder->length;
        
        // Sample points along cylinder axis
        int num_samples = std::max(3, static_cast<int>(length / 0.05));
        for (int i = 0; i < num_samples; ++i)
        {
          double t = static_cast<double>(i) / (num_samples - 1);
          double z = -length/2 + t * length;
          Eigen::Vector3d local_point(0, 0, z);
          Eigen::Vector3d world_point = shape_transform * local_point;
          sphere_locations.push_back(world_point);
        }
        RCLCPP_INFO_ONCE(getLogger(), "      Added %d cylinder samples", num_samples);
      }
      else if (shape->type == shapes::SPHERE)
      {
        const shapes::Sphere* sphere =
            static_cast<const shapes::Sphere*>(shape.get());
        Eigen::Vector3d center = shape_transform.translation();
        sphere_locations.push_back(center);
        RCLCPP_INFO_ONCE(getLogger(), "      Added sphere center");
      }
      else if (shape->type == shapes::BOX)
      {
        const shapes::Box* box = static_cast<const shapes::Box*>(shape.get());
        
        // Sample corners and center
        double dx = box->size[0] / 2;
        double dy = box->size[1] / 2;
        double dz = box->size[2] / 2;
        
        std::vector<Eigen::Vector3d> local_points = {
          Eigen::Vector3d(0, 0, 0), // Center
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
        RCLCPP_INFO_ONCE(getLogger(), "      Added %zu box samples", local_points.size());
      }
      else if (shape->type == shapes::MESH)
      {
        // Mesh - just add the mesh origin for now
        sphere_locations.push_back(shape_transform.translation());
        RCLCPP_INFO_ONCE(getLogger(), "      Added mesh origin (type MESH)");
      }
      else
      {
        // Unknown shape - add origin
        sphere_locations.push_back(shape_transform.translation());
        RCLCPP_INFO_ONCE(getLogger(), "      Added origin for unknown shape type %d", static_cast<int>(shape->type));
      }
    }
  }
  
  RCLCPP_INFO_ONCE(getLogger(), "Generated %zu total sphere locations", sphere_locations.size());
  
  return sphere_locations;
}


float PCEOptimizationTask::computeCollisionCostSimple(const Trajectory& trajectory) const
{
  if (!planning_scene_)  // NEW - using planning_scene_
  {
    RCLCPP_ERROR(getLogger(), "No planning scene set!");
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
    planning_scene_->checkCollision(collision_request, collision_result, state);  // NEW
    
    if (collision_result.collision)
    {
      // Penalize collision (you can make this more sophisticated)
      total_cost += 1000.0f;
    }
    
  }
  
  return total_cost;
}


float PCEOptimizationTask::computeCollisionCost(const Trajectory& trajectory) const
{
  RCLCPP_INFO_ONCE(getLogger(), "=== computeCollisionCost CALLED ===");
  
  if (!distance_field_)
  {
    RCLCPP_WARN_THROTTLE(getLogger(), *node_->get_clock(), 1000, "Distance field not available in computeCollisionCost!");
    return 0.0f;
  }
  
  RCLCPP_INFO_ONCE(getLogger(), "Distance field is available, computing collision cost...");
  
  float total_cost = 0.0f;
  int collision_count = 0;
  int near_collision_count = 0;
  
  for (size_t i = 0; i < trajectory.nodes.size(); ++i)
  {
    moveit::core::RobotState state(robot_model_ptr_);
    if (!trajectoryToRobotState(trajectory, i, state))
    {
      return std::numeric_limits<float>::infinity();
    }
    
    // Get sphere locations on robot
    std::vector<Eigen::Vector3d> sphere_locations = getSphereLocations(state);
    
    if (i == 0)
    {
      RCLCPP_INFO_ONCE(getLogger(), "Waypoint 0 has %zu sphere locations", sphere_locations.size());
      if (!sphere_locations.empty())
      {
        RCLCPP_INFO_ONCE(getLogger(), "  First sphere at [%.3f, %.3f, %.3f]",
                     sphere_locations[0].x(),
                     sphere_locations[0].y(),
                     sphere_locations[0].z());
      }
    }
    
    for (const Eigen::Vector3d& point : sphere_locations)
    {
      // Query signed distance from distance field
      double distance = getDistanceAtPoint(point);
      
      // Apply CHOMP cost function
      float point_cost = getObstacleCost(distance);
      
      if (point_cost > 0.0f)
      {
        if (distance < 0.0)
        {
          collision_count++;
        }
        else
        {
          near_collision_count++;
        }
      }
      
      total_cost += point_cost;
    }
  }
  
  RCLCPP_INFO_ONCE(getLogger(), "computeCollisionCost returning: %.4f", total_cost);
  
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
    RCLCPP_ERROR(getLogger(), "Trajectory dimensions don't match joint model group");
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
    RCLCPP_DEBUG(getLogger(), "PCE: Joint limits applied at iteration %d", iteration_number);
  }
  
  return filtered;
}


void PCEOptimizationTask::postIteration(int iteration_number, float cost, 
                                        const Trajectory& trajectory)
{
  RCLCPP_INFO(getLogger(), "PCE Iteration %d: cost = %.4f", iteration_number, cost);  // This should print
  
  // Check if visualizer exists
  if (!visualizer_)
  {
    RCLCPP_ERROR(getLogger(), "Visualizer is NULL!");
    return;
  }
  
  RCLCPP_INFO(getLogger(), "Calling visualization...");
  
  // Visualize current trajectory
  visualizer_->visualizeCollisionSpheres(trajectory, robot_model_ptr_, group_name_, distance_field_);
  visualizer_->visualizeTrajectory(trajectory, robot_model_ptr_, group_name_, iteration_number);
  
  RCLCPP_INFO(getLogger(), "Visualization called successfully");
  
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
}


void PCEOptimizationTask::done(bool success, int total_iterations,
                               float final_cost, const Trajectory& trajectory)
{
  RCLCPP_INFO(getLogger(), "PCE optimization %s after %d iterations (cost: %.4f)",
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
  RCLCPP_DEBUG(getLogger(), "PCE task initialized: %zu dimensions, %zu nodes", 
            num_dimensions, num_nodes);
}

bool PCEOptimizationTask::trajectoryToRobotState(
    const Trajectory& trajectory, size_t timestep,
    moveit::core::RobotState& state) const
{
  const auto* joint_model_group = robot_model_ptr_->getJointModelGroup(group_name_);
  if (!joint_model_group)
  {
    RCLCPP_ERROR(getLogger(), "Joint model group '%s' not found", group_name_.c_str());
    return false;
  }
  
  // Get variable names (not joint names - variables include all DOFs)
  const std::vector<std::string>& variable_names = joint_model_group->getVariableNames();
  const Eigen::VectorXf& positions_float = trajectory.nodes[timestep].position;  // VectorXf
  
  if (variable_names.size() != static_cast<size_t>(positions_float.size()))
  {
    RCLCPP_ERROR(getLogger(), "Dimension mismatch: trajectory has %zu DOFs, group has %zu variables",
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