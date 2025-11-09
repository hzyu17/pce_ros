#include "pce_optimization_task.h"
#include <cmath>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit_msgs/CollisionObject.h>
#include <omp.h>


namespace pce_ros
{

PCEOptimizationTask::PCEOptimizationTask(
    moveit::core::RobotModelConstPtr robot_model_ptr,
    const std::string& group_name,
    const XmlRpc::XmlRpcValue& config)
  : robot_model_ptr_(robot_model_ptr)
  , group_name_(group_name)
#ifdef USE_GPU
  , use_gpu_(false)
#endif
{
  // Load collision parameters from config
  if (config.hasMember("collision_clearance"))
  {
    collision_clearance_ = static_cast<double>(config["collision_clearance"]);
  }
  else
  {
    ROS_INFO("  Using default collision_clearance: %.3f", collision_clearance_);
  }
  
  if (config.hasMember("collision_threshold"))
  {
    collision_threshold_ = static_cast<double>(config["collision_threshold"]);
  }
  else
  {
    ROS_INFO("  Using default collision_threshold: %.3f", collision_threshold_);
  }

  if (config.hasMember("sigma_obs"))
  {
    sigma_obs_ = static_cast<double>(config["sigma_obs"]);
  }
  if (config.hasMember("sphere_overlap_ratio"))
  {
    sphere_overlap_ratio_ = static_cast<double>(config["sphere_overlap_ratio"]);
    // Clamp to valid range [0, 1]
    if (sphere_overlap_ratio_ < 0.0f) sphere_overlap_ratio_ = 0.0f;
    if (sphere_overlap_ratio_ > 1.0f) sphere_overlap_ratio_ = 1.0f;
  }
  else
  {
    ROS_INFO("  Using default sigma_obs: %.3f", sigma_obs_);
  }

  #ifdef USE_GPU
  // Check if GPU is requested
  if (config.hasMember("use_gpu"))
  {
    use_gpu_ = static_cast<bool>(config["use_gpu"]);
    
    if (use_gpu_)
    {
      // Initialize GPU bridge
      std::string urdf_path = "/path/to/robot.urdf";  // TODO: Get from robot model
      std::string base_link = robot_model_ptr_->getRootLinkName();
      std::string ee_link = "panda_hand";  // TODO: Get from config or robot model
      
      try
      {
        gpu_bridge_ = std::make_unique<GPUCollisionBridge>(
            urdf_path,
            base_link,
            ee_link,
            collision_clearance_
        );
        
        if (!gpu_bridge_->isAvailable())
        {
          ROS_WARN("GPU not available, falling back to CPU");
          use_gpu_ = false;
        }
        else
        {
          ROS_INFO("GPU acceleration enabled: %s", gpu_bridge_->getDeviceName().c_str());
        }
      }
      catch (const std::exception& e)
      {
        ROS_ERROR("Failed to initialize GPU bridge: %s", e.what());
        use_gpu_ = false;
      }
    }
  }
#endif

  
}

PCEOptimizationTask::~PCEOptimizationTask()
{
}

void PCEOptimizationTask::setPlanningScene(
    const planning_scene::PlanningSceneConstPtr& scene)
{
  
  if (!scene)
  {
    ROS_ERROR("Planning scene is NULL!");
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
      max_distance,
      true  // propagate - this is critical!
  );
    
  addCollisionObjectsToDistanceField();
  
  ROS_INFO("Distance field created [%.1f x %.1f x %.1f, res=%.3f]", 
           size_x, size_y, size_z, resolution);
}


void PCEOptimizationTask::addCollisionObjectsToDistanceField()
{  
  
  if (!distance_field_ || !planning_scene_)
  {
    ROS_ERROR("Distance field or planning scene is NULL!");
    return;
  }
  
  const collision_detection::WorldConstPtr& world = planning_scene_->getWorld();
  if (!world)
  {
    ROS_WARN("World is NULL!");
    return;
  }
  
  std::vector<std::string> object_ids = world->getObjectIds();
  ROS_INFO("Adding %zu collision objects to distance field:", object_ids.size());
  
  if (object_ids.empty())
  {
    ROS_WARN("No collision objects in world!");
    return;
  }
  
  // Cache resolution
  double resolution = distance_field_->getResolution();
  
  // CRITICAL FIX: Use std::vector (default allocator) to avoid alignment issues
  std::vector<Eigen::Vector3d> all_points;
  all_points.reserve(10000);
  
  int total_points = 0;
  
  for (const auto& id : object_ids)
  {
    
    collision_detection::World::ObjectConstPtr obj = world->getObject(id);
    if (!obj) continue;
        
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    planning_scene_->getCollisionObjectMsgs(collision_objects);
    
    Eigen::Isometry3d object_pose = Eigen::Isometry3d::Identity();
    for (const auto& co_msg : collision_objects)
    {
      if (co_msg.id == id)
      {
        tf2::fromMsg(co_msg.pose, object_pose);
        break;
      }
    }
    
    for (size_t i = 0; i < obj->shapes_.size(); ++i)
    {
      const shapes::ShapeConstPtr& shape = obj->shapes_[i];
      if (!shape) continue;
      
      const Eigen::Isometry3d& shape_pose_relative = obj->shape_poses_[i];
      Eigen::Isometry3d shape_pose_world = object_pose * shape_pose_relative;
            
      // Call helper that uses std::vector instead of EigenSTL
      std::vector<Eigen::Vector3d> shape_points;
      samplePointsFromShape(shape, shape_pose_world, resolution, shape_points);
            
      if (!shape_points.empty())
      {
        all_points.insert(all_points.end(), 
                         shape_points.begin(), 
                         shape_points.end());
        total_points += shape_points.size();
      }

    }
    
  }
    
  if (!all_points.empty())
  {    
    // Convert ONLY at the end for the API call
    EigenSTL::vector_Vector3d df_points;
    df_points.reserve(all_points.size());
    
    for (const auto& pt : all_points)
    {
      df_points.push_back(pt);
    }
        
    try
    {
      distance_field_->addPointsToField(df_points);
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("Exception: %s", e.what());
    }
  }
  
}


void PCEOptimizationTask::samplePointsFromShape(
    const shapes::ShapeConstPtr& shape,
    const Eigen::Isometry3d& pose,
    double resolution,
    std::vector<Eigen::Vector3d>& points)
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
    int nx = std::max(5, (int)std::ceil(box->size[0] / resolution));
    int ny = std::max(5, (int)std::ceil(box->size[1] / resolution));
    int nz = std::max(5, (int)std::ceil(box->size[2] / resolution));
    
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
    
    ROS_INFO("    Box [%.2f x %.2f x %.2f] at [%.2f, %.2f, %.2f]: %d points",
             box->size[0], box->size[1], box->size[2],
             pose.translation().x(), pose.translation().y(), pose.translation().z(),
             (int)points.size());
  }
  else if (shape->type == shapes::SPHERE)
  {
    const shapes::Sphere* sphere = static_cast<const shapes::Sphere*>(shape.get());
    
    double radius = sphere->radius;
    int n_phi = std::max(5, (int)std::ceil(radius / resolution));
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
    
    ROS_INFO("    Sphere radius %.2f at [%.2f, %.2f, %.2f]: %d points",
             radius,
             pose.translation().x(), pose.translation().y(), pose.translation().z(),
             (int)points.size());
  }
  else if (shape->type == shapes::CYLINDER)
  {
    const shapes::Cylinder* cylinder = static_cast<const shapes::Cylinder*>(shape.get());
    
    double radius = cylinder->radius;
    double length = cylinder->length;
    
    int n_r = std::max(3, (int)std::ceil(radius / resolution));
    int n_theta = std::max(8, (int)std::ceil(2.0 * M_PI * radius / resolution));
    int n_z = std::max(3, (int)std::ceil(length / resolution));
    
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
    
    ROS_INFO("    Cylinder r=%.2f, h=%.2f at [%.2f, %.2f, %.2f]: %d points",
             radius, length,
             pose.translation().x(), pose.translation().y(), pose.translation().z(),
             (int)points.size());
  }
  else
  {
    ROS_WARN("    Unsupported shape type: %d", (int)shape->type);
  }
}


double PCEOptimizationTask::getDistanceAtPoint(const Eigen::Vector3d& point) const
{
  if (!distance_field_)
  {
    ROS_ERROR_THROTTLE(1.0, "Distance field is NULL!");
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
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code)
{
  
  plan_request_ = req;
  
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
    ROS_ERROR("DEBUG: Joint model group '%s' not found!", group_name_.c_str());
    return sphere_locations;
  }
    
  const std::vector<const moveit::core::LinkModel*>& links = jmg->getLinkModels();
  
  sphere_locations.reserve(links.size() * 10);
  
  for (size_t link_idx = 0; link_idx < links.size(); ++link_idx)
  {
    const auto* link = links[link_idx];
    
    if (!link)
    {
      ROS_ERROR("DEBUG: Link is NULL, skipping");
      continue;
    }
    
    const Eigen::Isometry3d& link_transform = state.getGlobalLinkTransform(link);
    const std::vector<shapes::ShapeConstPtr>& shapes = link->getShapes();    
    const EigenSTL::vector_Isometry3d& shape_poses = link->getCollisionOriginTransforms();
    
    if (shapes.size() != shape_poses.size())
    {
      sphere_locations.push_back(link_transform.translation());
      continue;
    }
    
    if (shapes.empty())
    {
      ROS_WARN_ONCE("No shapes for link '%s', using origin", link->getName().c_str());
      sphere_locations.push_back(link_transform.translation());
      continue;
    }
    
    for (size_t s = 0; s < shapes.size(); ++s)
    {
      
      const shapes::ShapeConstPtr& shape = shapes[s];
      
      if (!shape)
      {
        ROS_ERROR("DEBUG: Shape %zu is NULL for link '%s'", s, link->getName().c_str());
        continue;
      }
            
      Eigen::Isometry3d shape_transform = link_transform * shape_poses[s];
      
      if (shape->type == shapes::CYLINDER)
      {
        const shapes::Cylinder* cylinder =
            static_cast<const shapes::Cylinder*>(shape.get());
        
        if (!cylinder || cylinder->length <= 0.0)
        {
          ROS_ERROR("DEBUG: Invalid cylinder");
          sphere_locations.push_back(shape_transform.translation());
          continue;
        }
        
        double length = cylinder->length;
        int num_samples = 3;
        
        for (int i = 0; i < num_samples; ++i)
        {
          double t = (num_samples > 1) ? static_cast<double>(i) / (num_samples - 1) : 0.5;
          double z = -length/2 + t * length;
          Eigen::Vector3d local_point(0, 0, z);
          sphere_locations.push_back(shape_transform * local_point);
        }
      }
      else if (shape->type == shapes::SPHERE)
      {
        sphere_locations.push_back(shape_transform.translation());
      }
      else if (shape->type == shapes::BOX)
      {
        const shapes::Box* box = static_cast<const shapes::Box*>(shape.get());
        
        if (!box || box->size[0] <= 0.0 || box->size[1] <= 0.0 || box->size[2] <= 0.0)
        {
          ROS_ERROR("DEBUG: Invalid box");
          sphere_locations.push_back(shape_transform.translation());
          continue;
        }
        
        double dx = box->size[0] / 2;
        double dy = box->size[1] / 2;
        double dz = box->size[2] / 2;
        
        std::vector<Eigen::Vector3d> local_points = {
          Eigen::Vector3d(0, 0, 0),
          Eigen::Vector3d(dx, 0, 0),
          Eigen::Vector3d(-dx, 0, 0),
          Eigen::Vector3d(0, dy, 0),
          Eigen::Vector3d(0, 0, dz),
        };
        
        for (const auto& local_pt : local_points)
        {
          sphere_locations.push_back(shape_transform * local_pt);
        }
      }
      else if (shape->type == shapes::MESH)
      {
        const shapes::Mesh* mesh = static_cast<const shapes::Mesh*>(shape.get());
        
        if (!mesh || mesh->vertex_count == 0 || !mesh->vertices)
        {
          sphere_locations.push_back(shape_transform.translation());
          continue;
        }
        
        if (mesh->vertex_count > 100000)
        {
          sphere_locations.push_back(shape_transform.translation());
          continue;
        }
        
        // Compute bounding box
        double x_min = 1e9, x_max = -1e9;
        double y_min = 1e9, y_max = -1e9;
        double z_min = 1e9, z_max = -1e9;
        bool valid = false;
        
        for (unsigned int i = 0; i < mesh->vertex_count; ++i)
        {          
          unsigned int idx = 3 * i;
          if (idx + 2 >= mesh->vertex_count * 3)
          {
            break;
          }
          
          double vx = mesh->vertices[idx + 0];
          double vy = mesh->vertices[idx + 1];
          double vz = mesh->vertices[idx + 2];
          
          if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz))
          {
            continue;
          }
          
          x_min = std::min(x_min, vx);
          x_max = std::max(x_max, vx);
          y_min = std::min(y_min, vy);
          y_max = std::max(y_max, vy);
          z_min = std::min(z_min, vz);
          z_max = std::max(z_max, vz);
          valid = true;
        }
        
        if (!valid)
        {
          sphere_locations.push_back(shape_transform.translation());
          continue;
        }
        
        // Sample points throughout the bounding box
        double size_x = x_max - x_min;
        double size_y = y_max - y_min;
        double size_z = z_max - z_min;
        
        // Determine sampling based on sphere overlap ratio
        double spacing = collision_clearance_ * (1.0 - sphere_overlap_ratio_);
        spacing = std::max(spacing, 0.03); // Minimum spacing
        
        int nx = std::max(1, std::min(4, (int)std::ceil(size_x / spacing)));
        int ny = std::max(1, std::min(4, (int)std::ceil(size_y / spacing)));
        int nz = std::max(1, std::min(4, (int)std::ceil(size_z / spacing)));
        
        // Limit total points
        int max_points = 20;
        if (nx * ny * nz > max_points)
        {
          double scale = std::pow(max_points / (double)(nx * ny * nz), 1.0/3.0);
          nx = std::max(2, (int)(nx * scale));
          ny = std::max(2, (int)(ny * scale));
          nz = std::max(2, (int)(nz * scale));
        }
        
        // Sample grid throughout bounding box
        for (int ix = 0; ix < nx; ++ix)
        {
          for (int iy = 0; iy < ny; ++iy)
          {
            for (int iz = 0; iz < nz; ++iz)
            {
              double tx = (nx > 1) ? (double)ix / (nx - 1) : 0.5;
              double ty = (ny > 1) ? (double)iy / (ny - 1) : 0.5;
              double tz = (nz > 1) ? (double)iz / (nz - 1) : 0.5;
              
              Eigen::Vector3d local_point(
                x_min + tx * size_x,
                y_min + ty * size_y,
                z_min + tz * size_z
              );
              
              sphere_locations.push_back(shape_transform * local_point);
            }
          }
        }
      }
      else
      {
        ROS_ERROR("DEBUG: Unknown shape type %d", (int)shape->type);
        sphere_locations.push_back(shape_transform.translation());
      }
    }
    
  }
    
  return sphere_locations;
}


float PCEOptimizationTask::computeCollisionCostSimple(const Trajectory& trajectory) const
{
  if (!planning_scene_) 
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
  if (!distance_field_)
  {
    ROS_WARN_THROTTLE(5.0, "Distance field not available!");
    return 0.0f;
  }


  // Set OpenMP threads ONLY when actually using parallel code
  static bool omp_initialized = false;
  if (!omp_initialized)
  {
    int num_threads = omp_get_max_threads();
    omp_set_num_threads(num_threads);
    ROS_INFO("OpenMP initialized with %d threads for collision checking", num_threads);
    omp_initialized = true;
  }
  
  const size_t num_waypoints = trajectory.nodes.size();
  
  // Resize OUTSIDE parallel region (not thread-safe operation)
  cached_sphere_locations_.resize(num_waypoints);
  
  const float MAX_ACCEPTABLE_COST = 5000.0f;
  
  // Use atomic for early termination flag
  std::atomic<bool> exceeded_threshold{false};
  
  // Use reduction for sum (OpenMP handles synchronization)
  float sum_squared_costs = 0.0f;
  
  // Use proper OpenMP reduction pattern
  // OPTIMIZATION: Use guided scheduling for better load balancing
  #pragma omp parallel for reduction(+:sum_squared_costs) schedule(guided, 2)
  for (size_t i = 0; i < num_waypoints; ++i)
  {
    if (exceeded_threshold.load(std::memory_order_relaxed))
    {
      continue;
    }
    
    // Thread-local RobotState
    moveit::core::RobotState state(robot_model_ptr_);
    if (!trajectoryToRobotState(trajectory, i, state))
    {
      continue;
    }
    
    // Get sphere locations
    std::vector<Eigen::Vector3d> sphere_locations = getSphereLocations(state);
    
    // OPTIMIZATION: Early exit if too many spheres (something wrong)
    if (sphere_locations.size() > 200)
    {
      ROS_WARN_THROTTLE(5.0, "Too many collision spheres (%zu), limiting to 200", 
                        sphere_locations.size());
      sphere_locations.resize(200);
    }
    
    cached_sphere_locations_[i] = std::move(sphere_locations);
    
    // Compute cost for this waypoint
    float waypoint_cost = 0.0f;
    
    for (const Eigen::Vector3d& point : cached_sphere_locations_[i])
    {
      double distance = getDistanceAtPoint(point);
      
      // OPTIMIZATION: Skip if very far from obstacles
      if (distance > collision_threshold_)
      {
        continue;
      }
      
      float point_cost = getObstacleCost(distance);
      waypoint_cost += point_cost * point_cost;
    }
    
    sum_squared_costs += waypoint_cost;
    
    // Early termination check
    if (waypoint_cost * sigma_obs_ > MAX_ACCEPTABLE_COST / num_waypoints)
    {
      exceeded_threshold.store(true, std::memory_order_relaxed);
    }
  }

  // Early termination check
  if (exceeded_threshold.load())
  {
    return MAX_ACCEPTABLE_COST;
  }
  
  float total_cost = sum_squared_costs * sigma_obs_;
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
  // Use throttled logging to reduce overhead
  ROS_INFO_THROTTLE(1.0, "PCE Iteration %d: cost = %.4f", iteration_number, cost);
  
  // Check if visualizer exists
  if (!visualizer_)
  {
    ROS_ERROR("Visualizer is NULL!");
    return;
  }
  
  ROS_INFO("Calling visualization...");
  
  // Visualize current trajectory
  if (visualizer_)
  {
    visualizer_->visualizeCollisionSpheres(trajectory, cached_sphere_locations_, robot_model_ptr_, group_name_, collision_clearance_, distance_field_);
    visualizer_->visualizeTrajectory(trajectory, robot_model_ptr_, group_name_, iteration_number);
  }
  else
  {
    ROS_ERROR_THROTTLE(5.0, "Visualizer is NULL!");
  }
  
  ROS_INFO("Visualization called successfully");
  
  // ros::Duration(0.05).sleep();
  ros::spinOnce();
}


void PCEOptimizationTask::done(bool success, int total_iterations,
                               float final_cost, const Trajectory& trajectory)
{
  ROS_INFO("PCE optimization %s after %d iterations (cost: %.4f)",
           success ? "succeeded" : "failed", total_iterations, final_cost);
  
  if (success)
  {
    visualizer_->visualizeCollisionSpheres(trajectory, cached_sphere_locations_, robot_model_ptr_, group_name_, collision_clearance_, distance_field_);
    visualizer_->visualizeTrajectory(trajectory, robot_model_ptr_, group_name_, total_iterations);
  }
}

void PCEOptimizationTask::initialize(size_t num_dimensions, const PathNode& start,
                                     const PathNode& goal, size_t num_nodes,
                                     float total_time)
{
  // Clear cached data
  cached_sphere_locations_.clear();

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
    return false;
  }
  
  // OPTIMIZATION: Static vector to avoid repeated allocations
  static thread_local std::vector<double> positions;
  positions.resize(positions_float.size());
  
  for (Eigen::Index i = 0; i < positions_float.size(); ++i)
  {
    positions[i] = static_cast<double>(positions_float[i]);
  }
  
  // Set joint group positions
  state.setJointGroupPositions(joint_model_group, positions);
  state.update();
  
  return true;
}


std::vector<float> PCEOptimizationTask::computeCollisionCost(
    const std::vector<Trajectory>& trajectories) const 
{    
    std::vector<float> costs;

    // For multiple trajectories, use batch processing
    #ifdef USE_GPU
    const size_t GPU_BATCH_THRESHOLD = 100;
    
    ROS_INFO("GPU Support: COMPILED IN (USE_GPU defined)");
    ROS_INFO("  use_gpu_ = %s", use_gpu_ ? "true" : "false");
    ROS_INFO("  gpu_bridge_ = %s", gpu_bridge_ ? "valid" : "null");
    ROS_INFO("  batch size = %zu (threshold = %zu)", trajectories.size(), GPU_BATCH_THRESHOLD);
    
    if (use_gpu_ && gpu_bridge_ && trajectories.size() >= GPU_BATCH_THRESHOLD)
    {
        ROS_INFO(">>> ATTEMPTING GPU PATH <<<");
        
        auto start = std::chrono::high_resolution_clock::now();
        
        bool success = gpu_bridge_->checkBatch(trajectories, costs);
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        if (success)
        {
            ROS_INFO(">>> GPU PATH SUCCESS: %zu samples in %ld ms <<<", 
                     trajectories.size(), duration.count());
            
            for (auto& cost : costs)
            {
                cost *= sigma_obs_;
            }
            
            return costs;
        }
        else
        {
            ROS_WARN(">>> GPU PATH FAILED, FALLING BACK TO CPU <<<");
        }
    }
    else
    {
        ROS_INFO(">>> USING CPU PATH <<<");
        if (!use_gpu_) ROS_INFO("  Reason: use_gpu_ is false");
        if (!gpu_bridge_) ROS_INFO("  Reason: gpu_bridge_ is null");
        if (trajectories.size() < GPU_BATCH_THRESHOLD) 
            ROS_INFO("  Reason: batch size %zu < threshold %zu", 
                     trajectories.size(), GPU_BATCH_THRESHOLD);
    }
    #else
      ROS_INFO(">>> USING CPU PATH (GPU NOT COMPILED) <<<");
    #endif
    
    // CPU PATH - TEMPORARILY DISABLE PARALLELIZATION
    ROS_DEBUG_THROTTLE(5.0, "Using sequential CPU for %zu trajectories", trajectories.size());
    
    auto start = std::chrono::high_resolution_clock::now();
    
    costs.resize(trajectories.size());
    
    // SEQUENTIAL (no OpenMP)
    for (size_t i = 0; i < trajectories.size(); ++i)
    {
        costs[i] = computeCollisionCost(trajectories[i]);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    ROS_INFO_THROTTLE(5.0, "CPU: %zu samples in %ld ms (%.1f samples/sec)",
                     trajectories.size(), duration.count(),
                     trajectories.size() / (duration.count() / 1000.0));
    
    return costs;
}


} // namespace pce_ros