#include "visualizer.h"
#include <geometric_shapes/shapes.h>

namespace pce_ros
{
VisualizationConfig PCEVisualization::loadConfig(const rclcpp::Node::SharedPtr& node, const std::string& ns)
{
  VisualizationConfig config;
  std::string vis_params = ns + ".pce.visualization";

  config.enable_collision_spheres = getParam<bool>(node, vis_params + ".enable_collision_spheres", true);
  config.enable_trajectory = getParam<bool>(node, vis_params + ".enable_trajectory", true);
  config.enable_distance_field = getParam<bool>(node, vis_params + ".enable_distance_field", false);
  config.waypoint_size = getParam<double>(node, vis_params + ".waypoint_size", 0.02);
  config.line_width = getParam<double>(node, vis_params + ".line_width", 0.01);
  config.marker_lifetime = getParam<double>(node, vis_params + ".marker_lifetime", 0.5);
  config.trajectory_decimation = getParam<int>(node, vis_params + ".trajectory_decimation", 10);
  config.collision_clearance = getParam<double>(node, vis_params + ".collision_clearance", 0.05);
  config.collision_spheres_topic = getParam<std::string>(node, vis_params + ".collision_spheres_topic", "/pce/collision_spheres");
  config.trajectory_topic = getParam<std::string>(node, vis_params + ".trajectory_topic", "/pce/trajectory");
  config.distance_field_topic = getParam<std::string>(node, vis_params + ".distance_field_topic", "/pce/distance_field");

  RCLCPP_INFO(node->get_logger(), "Loaded visualization config:");
  RCLCPP_INFO(node->get_logger(), "  enable_collision_spheres: %s", config.enable_collision_spheres ? "true" : "false");
  RCLCPP_INFO(node->get_logger(), "  enable_trajectory: %s", config.enable_trajectory ? "true" : "false");
  RCLCPP_INFO(node->get_logger(), "  collision_clearance (used for sphere size): %.3f", config.collision_clearance);
  RCLCPP_INFO(node->get_logger(), "  collision_spheres_topic: %s", config.collision_spheres_topic.c_str());

  return config;
}

PCEVisualization::PCEVisualization(const VisualizationConfig& config, const rclcpp::Node::SharedPtr& node)
  : config_(config)
  , node_(node)
{
  // Create QoS profile
  rclcpp::QoS qos(10);
  qos.transient_local();  // Latched equivalent in ROS2
  
  // Create publishers
  collision_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      config_.collision_spheres_topic, qos);

  trajectory_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      config_.trajectory_topic, qos);
  
  distance_field_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      config_.distance_field_topic, qos);

  RCLCPP_ERROR(node_->get_logger(), "Distance field publisher created!");
  RCLCPP_ERROR(node_->get_logger(), "  Topic name: %s", 
               distance_field_pub_->get_topic_name());
  
  RCLCPP_INFO(node_->get_logger(), "PCE Visualization publishers created:");
  RCLCPP_INFO(node_->get_logger(), "  - %s", config_.collision_spheres_topic.c_str());
  RCLCPP_INFO(node_->get_logger(), "  - %s", config_.trajectory_topic.c_str());

  
  // Wait for publishers to be ready
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(node_->get_logger(), "PCE Visualization ready (collision_spheres=%s, trajectory=%s)",
              config_.enable_collision_spheres ? "ON" : "OFF",
              config_.enable_trajectory ? "ON" : "OFF");
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

std_msgs::msg::ColorRGBA PCEVisualization::getColorFromDistance(
    double distance, 
    double collision_clearance) const  // ← ADD PARAMETER
{
  std_msgs::msg::ColorRGBA color;
  color.a = 0.8;
  
  if (distance < 0.0)
  {
    color.r = 1.0; color.g = 0.0; color.b = 0.0;  // Red - collision
  }
  else if (distance < collision_clearance)
  {
    float ratio = distance / collision_clearance;
    color.r = 1.0 - ratio;
    color.g = 1.0;
    color.b = 0.0;  // Yellow to green gradient
  }
  else
  {
    color.r = 0.0; color.g = 1.0; color.b = 0.0;  // Green - safe
  }
  
  return color;
}

std_msgs::msg::ColorRGBA PCEVisualization::getGradientColor(float ratio) const
{
  std_msgs::msg::ColorRGBA color;
  color.r = 0.5 * ratio;
  color.g = 0.0;
  color.b = 1.0;
  color.a = 0.8;
  return color;
}

void PCEVisualization::visualizeCollisionSpheres(
    const Trajectory& trajectory,
    const std::vector<std::vector<Eigen::Vector3d>>& sphere_locations_per_waypoint,
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::string& group_name,
    double collision_clearance,
    const distance_field::DistanceFieldConstPtr& distance_field) const
{
  RCLCPP_INFO(node_->get_logger(), "visualizeCollisionSpheres called");
  RCLCPP_INFO(node_->get_logger(), "  enabled: %s", 
              config_.enable_collision_spheres ? "true" : "false");
  RCLCPP_INFO(node_->get_logger(), "  subscribers: %zu", 
              collision_marker_pub_->get_subscription_count());
  
  
  if (!config_.enable_collision_spheres || collision_marker_pub_->get_subscription_count() == 0)
  {
    return;
  }
  
  visualization_msgs::msg::MarkerArray marker_array;
  int marker_id = 0;
  
  size_t step = std::max(1ul, trajectory.nodes.size() / config_.trajectory_decimation);
  
  for (size_t i = 0; i < trajectory.nodes.size(); i += step)
  {
    // Use pre-computed sphere locations
    if (i >= sphere_locations_per_waypoint.size())
      continue;
      
    const std::vector<Eigen::Vector3d>& sphere_locations = sphere_locations_per_waypoint[i];
    
    for (const Eigen::Vector3d& point : sphere_locations)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = robot_model->getRootLinkName();
      marker.header.stamp = node_->now();
      marker.ns = "collision_spheres";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      
      marker.pose.position.x = point.x();
      marker.pose.position.y = point.y();
      marker.pose.position.z = point.z();
      marker.pose.orientation.w = 1.0;
      
      marker.scale.x = collision_clearance * 2.0;  // Diameter
      marker.scale.y = collision_clearance * 2.0;
      marker.scale.z = collision_clearance * 2.0;
      
      if (distance_field)
      {
        double distance = distance_field->getDistance(point.x(), point.y(), point.z());
        marker.color = getColorFromDistance(distance, collision_clearance);
      }
      else
      {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;
      }
      
      marker.lifetime = rclcpp::Duration::from_seconds(config_.marker_lifetime);
      marker_array.markers.push_back(marker);
    }
  }
  
  RCLCPP_INFO(node_->get_logger(), "Publishing %zu markers", marker_array.markers.size());
  collision_marker_pub_->publish(marker_array);
  RCLCPP_INFO(node_->get_logger(), "Markers published");
}

void PCEVisualization::visualizeTrajectory(
    const Trajectory& trajectory,
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::string& group_name,
    int iteration) const
{
  if (!config_.enable_trajectory || trajectory_marker_pub_->get_subscription_count() == 0)
  {
    return;
  }
  
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group_name);
  if (!jmg)
  {
    return;
  }
  
  visualization_msgs::msg::MarkerArray marker_array;
  
  // Line strip for trajectory
  visualization_msgs::msg::Marker line_marker;
  line_marker.header.frame_id = robot_model->getRootLinkName();
  line_marker.header.stamp = node_->now();
  line_marker.ns = "trajectory_path";
  line_marker.id = 0;
  line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::msg::Marker::ADD;
  line_marker.scale.x = config_.line_width;
  line_marker.color = getGradientColor(0.5f);
  line_marker.lifetime = rclcpp::Duration::from_seconds(config_.marker_lifetime);
  
  const std::string& ee_link = jmg->getLinkModelNames().back();
  
  for (size_t i = 0; i < trajectory.nodes.size(); ++i)
  {
    moveit::core::RobotState state(robot_model);
    if (!trajectoryToRobotState(trajectory, i, state, robot_model, group_name))
    {
      continue;
    }
    
    const Eigen::Isometry3d& ee_pose = state.getGlobalLinkTransform(ee_link);
    
    geometry_msgs::msg::Point p;
    p.x = ee_pose.translation().x();
    p.y = ee_pose.translation().y();
    p.z = ee_pose.translation().z();
    line_marker.points.push_back(p);
    
    // Waypoint spheres
    visualization_msgs::msg::Marker waypoint_marker;
    waypoint_marker.header = line_marker.header;
    waypoint_marker.ns = "trajectory_waypoints";
    waypoint_marker.id = i;
    waypoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
    waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
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
  visualization_msgs::msg::Marker text_marker;
  text_marker.header = line_marker.header;
  text_marker.ns = "iteration_text";
  text_marker.id = 0;
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::msg::Marker::ADD;
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
  
  trajectory_marker_pub_->publish(marker_array);
}

void PCEVisualization::clearAllMarkers() const
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);
  
  collision_marker_pub_->publish(marker_array);
  trajectory_marker_pub_->publish(marker_array);
  distance_field_pub_->publish(marker_array);
}

} // namespace pce_ros