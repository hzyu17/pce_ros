#include "visualizer.h"
#include <geometric_shapes/shapes.h>

namespace pce_ros
{

PCEVisualization::PCEVisualization(const VisualizationConfig& config, ros::NodeHandle nh)
  : config_(config)
  , nh_(nh)
{
  ROS_ERROR("========================================");
  ROS_ERROR("PCEVisualization constructor START");
  ROS_ERROR("  NodeHandle namespace: %s", nh_.getNamespace().c_str());
  ROS_ERROR("  collision_spheres_topic: %s", config_.collision_spheres_topic.c_str());
  ROS_ERROR("========================================");

  // Create publishers
  collision_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      config_.collision_spheres_topic, 10, true);

  ROS_ERROR("Collision marker publisher created!");
  ROS_ERROR("  Topic name: %s", collision_marker_pub_.getTopic().c_str());
  ROS_ERROR("  Is latched: %s", collision_marker_pub_.isLatched() ? "yes" : "no");
  
  trajectory_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      config_.trajectory_topic, 10, true);

  ROS_ERROR("Trajectory marker publisher created!");
  ROS_ERROR("  Topic name: %s", trajectory_marker_pub_.getTopic().c_str());
  
  distance_field_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      config_.distance_field_topic, 10);

  ROS_ERROR("Distance field publisher created!");
  ROS_ERROR("  Topic name: %s", distance_field_pub_.getTopic().c_str());
  
  ROS_INFO("PCE Visualization publishers created:");
  ROS_INFO("  - %s", config_.collision_spheres_topic.c_str());
  ROS_INFO("  - %s", config_.trajectory_topic.c_str());
  
  // Wait for publishers to be ready
  ros::Duration(0.5).sleep();

  ROS_ERROR("========================================");
  ROS_ERROR("PCEVisualization constructor END");
  ROS_ERROR("========================================");
  
  ROS_INFO("PCE Visualization initialized (collision_spheres=%s, trajectory=%s)",
           config_.enable_collision_spheres ? "enabled" : "disabled",
           config_.enable_trajectory ? "enabled" : "disabled");
}

std::vector<Eigen::Vector3d> PCEVisualization::getSphereLocations(
    const moveit::core::RobotState& state,
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::string& group_name) const
{
  std::vector<Eigen::Vector3d> sphere_locations;
  
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group_name);
  if (!jmg)
  {
    return sphere_locations;
  }
  
  const std::vector<const moveit::core::LinkModel*>& links = jmg->getLinkModels();
  
  for (const auto* link : links)
  {
    const Eigen::Isometry3d& link_transform = state.getGlobalLinkTransform(link);
    const std::vector<shapes::ShapeConstPtr>& shapes = link->getShapes();
    const EigenSTL::vector_Isometry3d& shape_poses = link->getCollisionOriginTransforms();
    
    for (size_t s = 0; s < shapes.size(); ++s)
    {
      const shapes::ShapeConstPtr& shape = shapes[s];
      Eigen::Isometry3d shape_transform = link_transform * shape_poses[s];
      
      if (shape->type == shapes::CYLINDER)
      {
        const shapes::Cylinder* cylinder = static_cast<const shapes::Cylinder*>(shape.get());
        double length = cylinder->length;
        int num_samples = std::max(3, static_cast<int>(length / 0.05));
        
        for (int i = 0; i < num_samples; ++i)
        {
          double t = static_cast<double>(i) / (num_samples - 1);
          double z = -length/2.0 + t * length;
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
        double dx = box->size[0] / 2.0;
        double dy = box->size[1] / 2.0;
        double dz = box->size[2] / 2.0;
        
        std::vector<Eigen::Vector3d> local_points = {
          Eigen::Vector3d(0, 0, 0),
          Eigen::Vector3d(dx, dy, dz), Eigen::Vector3d(dx, dy, -dz),
          Eigen::Vector3d(dx, -dy, dz), Eigen::Vector3d(dx, -dy, -dz),
          Eigen::Vector3d(-dx, dy, dz), Eigen::Vector3d(-dx, dy, -dz),
          Eigen::Vector3d(-dx, -dy, dz), Eigen::Vector3d(-dx, -dy, -dz)
        };
        
        for (const auto& local_pt : local_points)
        {
          sphere_locations.push_back(shape_transform * local_pt);
        }
      }
      else
      {
        sphere_locations.push_back(shape_transform.translation());
      }
    }
  }
  
  return sphere_locations;
}

bool PCEVisualization::trajectoryToRobotState(
    const Trajectory& trajectory,
    size_t timestep,
    moveit::core::RobotState& state,
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::string& group_name) const
{
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group_name);
  if (!jmg)
  {
    return false;
  }
  
  const std::vector<std::string>& variable_names = jmg->getVariableNames();
  const Eigen::VectorXf& positions_float = trajectory.nodes[timestep].position;
  
  if (variable_names.size() != static_cast<size_t>(positions_float.size()))
  {
    return false;
  }
  
  std::vector<double> positions(positions_float.size());
  for (Eigen::Index i = 0; i < positions_float.size(); ++i)
  {
    positions[i] = static_cast<double>(positions_float[i]);
  }
  
  state.setJointGroupPositions(jmg, positions);
  state.update();
  
  return true;
}

std_msgs::ColorRGBA PCEVisualization::getColorFromDistance(double distance) const
{
  std_msgs::ColorRGBA color;
  color.a = 0.8;
  
  if (distance < 0.0)
  {
    // Red - collision
    color.r = 1.0;
    color.g = 0.0;
    color.b = 0.0;
  }
  else if (distance < config_.collision_clearance)
  {
    // Yellow to green gradient
    float ratio = distance / config_.collision_clearance;
    color.r = 1.0 - ratio;
    color.g = 1.0;
    color.b = 0.0;
  }
  else
  {
    // Green - safe
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;
  }
  
  return color;
}

std_msgs::ColorRGBA PCEVisualization::getGradientColor(float ratio) const
{
  std_msgs::ColorRGBA color;
  color.r = 0.5 * ratio;
  color.g = 0.0;
  color.b = 1.0;
  color.a = 0.8;
  return color;
}

void PCEVisualization::visualizeCollisionSpheres(
    const Trajectory& trajectory,
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::string& group_name,
    const std::shared_ptr<distance_field::PropagationDistanceField>& distance_field) const
{

  ROS_INFO("visualizeCollisionSpheres called");
  ROS_INFO("  enabled: %s", config_.enable_collision_spheres ? "true" : "false");
  ROS_INFO("  subscribers: %d", collision_marker_pub_.getNumSubscribers());
  
  if (!config_.enable_collision_spheres)
  {
    ROS_WARN("Collision spheres visualization is disabled in config");
    return;
  }
  
  if (collision_marker_pub_.getNumSubscribers() == 0)
  {
    ROS_WARN("No subscribers to /pce/collision_spheres");
    return;
  }

  if (!config_.enable_collision_spheres || collision_marker_pub_.getNumSubscribers() == 0)
  {
    return;
  }
  
  visualization_msgs::MarkerArray marker_array;
  int marker_id = 0;
  
  size_t step = std::max(1ul, trajectory.nodes.size() / config_.trajectory_decimation);
  
  for (size_t i = 0; i < trajectory.nodes.size(); i += step)
  {
    moveit::core::RobotState state(robot_model);
    if (!trajectoryToRobotState(trajectory, i, state, robot_model, group_name))
    {
      continue;
    }
    
    std::vector<Eigen::Vector3d> sphere_locations = getSphereLocations(state, robot_model, group_name);
    
    for (const Eigen::Vector3d& point : sphere_locations)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = robot_model->getRootLinkName();
      marker.header.stamp = ros::Time::now();
      marker.ns = "collision_spheres";
      marker.id = marker_id++;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      
      marker.pose.position.x = point.x();
      marker.pose.position.y = point.y();
      marker.pose.position.z = point.z();
      marker.pose.orientation.w = 1.0;
      
      marker.scale.x = config_.sphere_size;
      marker.scale.y = config_.sphere_size;
      marker.scale.z = config_.sphere_size;
      
      if (distance_field)
      {
        double distance = distance_field->getDistance(point.x(), point.y(), point.z());
        marker.color = getColorFromDistance(distance);
      }
      else
      {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;
      }
      
      marker.lifetime = ros::Duration(config_.marker_lifetime);
      marker_array.markers.push_back(marker);
    }
  }
  
  ROS_INFO("Publishing %zu markers", marker_array.markers.size());
  collision_marker_pub_.publish(marker_array);
  ROS_INFO("Markers published");

}


void PCEVisualization::visualizeTrajectory(
    const Trajectory& trajectory,
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::string& group_name,
    int iteration) const
{
  if (!config_.enable_trajectory || trajectory_marker_pub_.getNumSubscribers() == 0)
  {
    return;
  }
  
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group_name);
  if (!jmg)
  {
    return;
  }
  
  visualization_msgs::MarkerArray marker_array;
  
  // Line strip for trajectory
  visualization_msgs::Marker line_marker;
  line_marker.header.frame_id = robot_model->getRootLinkName();
  line_marker.header.stamp = ros::Time::now();
  line_marker.ns = "trajectory_path";
  line_marker.id = 0;
  line_marker.type = visualization_msgs::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::Marker::ADD;
  line_marker.scale.x = config_.line_width;
  line_marker.color = getGradientColor(0.5f);
  line_marker.lifetime = ros::Duration(config_.marker_lifetime);
  
  const std::string& ee_link = jmg->getLinkModelNames().back();
  
  for (size_t i = 0; i < trajectory.nodes.size(); ++i)
  {
    moveit::core::RobotState state(robot_model);
    if (!trajectoryToRobotState(trajectory, i, state, robot_model, group_name))
    {
      continue;
    }
    
    const Eigen::Isometry3d& ee_pose = state.getGlobalLinkTransform(ee_link);
    
    geometry_msgs::Point p;
    p.x = ee_pose.translation().x();
    p.y = ee_pose.translation().y();
    p.z = ee_pose.translation().z();
    line_marker.points.push_back(p);
    
    // Waypoint spheres
    visualization_msgs::Marker waypoint_marker;
    waypoint_marker.header = line_marker.header;
    waypoint_marker.ns = "trajectory_waypoints";
    waypoint_marker.id = i;
    waypoint_marker.type = visualization_msgs::Marker::SPHERE;
    waypoint_marker.action = visualization_msgs::Marker::ADD;
    waypoint_marker.pose.position = p;
    waypoint_marker.pose.orientation.w = 1.0;
    waypoint_marker.scale.x = config_.waypoint_size;
    waypoint_marker.scale.y = config_.waypoint_size;
    waypoint_marker.scale.z = config_.waypoint_size;
    waypoint_marker.color = getGradientColor(static_cast<float>(i) / (trajectory.nodes.size() - 1));
    waypoint_marker.lifetime = line_marker.lifetime;
    
    marker_array.markers.push_back(waypoint_marker);
  }
  
  marker_array.markers.push_back(line_marker);
  
  // Iteration text
  visualization_msgs::Marker text_marker;
  text_marker.header = line_marker.header;
  text_marker.ns = "iteration_text";
  text_marker.id = 0;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.pose.position.x = 0.5;
  text_marker.pose.position.y = 0.0;
  text_marker.pose.position.z = 0.8;
  text_marker.pose.orientation.w = 1.0;
  text_marker.scale.z = 0.1;
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;
  text_marker.text = "PCE Iteration: " + std::to_string(iteration);
  text_marker.lifetime = line_marker.lifetime;
  
  marker_array.markers.push_back(text_marker);
  
  trajectory_marker_pub_.publish(marker_array);
}

void PCEVisualization::clearAllMarkers() const
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker delete_marker;
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);
  
  collision_marker_pub_.publish(marker_array);
  trajectory_marker_pub_.publish(marker_array);
  distance_field_pub_.publish(marker_array);
}

} // namespace pce_ros