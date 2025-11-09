#include "gpu_collision_bridge.h"
#include <ros/package.h>

#ifdef USE_GPU
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/embed.h>

namespace py = pybind11;
#endif

namespace pce_ros
{

GPUCollisionBridge::GPUCollisionBridge(
    const std::string& urdf_path,
    const std::string& base_link,
    const std::string& ee_link,
    float collision_clearance)
  : gpu_available_(false)
  , collision_clearance_(collision_clearance)
  , device_name_("None")
{
#ifdef USE_GPU
  try
  {
    ROS_INFO("Initializing GPU collision bridge...");
    
    // Initialize Python interpreter
    py_guard_ = std::make_unique<py::scoped_interpreter>();
    
    // Add scripts directory to Python path
    py::module sys = py::module::import("sys");
    py::list path = sys.attr("path");
    std::string scripts_path = ros::package::getPath("pce_ros") + "/scripts";
    path.append(scripts_path);
    
    // Import GPU collision checker
    py::module gpu_module = py::module::import("gpu_collision_checker");
    
    // Create checker instance
    py_checker_ = std::make_unique<py::object>(
      gpu_module.attr("GPUCollisionChecker")(
        urdf_path,
        base_link,
        ee_link,
        collision_clearance
      )
    );
    
    // Get device name
    py::module torch = py::module::import("torch");
    if (torch.attr("cuda").attr("is_available")().cast<bool>())
    {
      device_name_ = torch.attr("cuda").attr("get_device_name")(0).cast<std::string>();
    }
    
    gpu_available_ = true;
    ROS_INFO("GPU collision bridge initialized successfully!");
    ROS_INFO("  Device: %s", device_name_.c_str());
  }
  catch (const std::exception& e)
  {
    ROS_WARN("Failed to initialize GPU collision bridge: %s", e.what());
    ROS_WARN("GPU acceleration disabled. Continuing with CPU...");
    gpu_available_ = false;
  }
#else
  ROS_INFO("GPU support not compiled. Use -DUSE_GPU=ON to enable.");
#endif
}

GPUCollisionBridge::~GPUCollisionBridge()
{
  py_checker_.reset();
  py_guard_.reset();
}

void GPUCollisionBridge::trajectoriesToArray(
    const std::vector<Trajectory>& trajectories,
    std::vector<float>& flat_data,
    size_t& num_samples,
    size_t& num_waypoints,
    size_t& num_joints)
{
  if (trajectories.empty())
  {
    num_samples = num_waypoints = num_joints = 0;
    return;
  }
  
  num_samples = trajectories.size();
  num_waypoints = trajectories[0].nodes.size();
  num_joints = trajectories[0].nodes[0].position.size();
  
  // Flatten: [sample][waypoint][joint]
  flat_data.reserve(num_samples * num_waypoints * num_joints);
  
  for (const auto& traj : trajectories)
  {
    for (const auto& node : traj.nodes)
    {
      for (int j = 0; j < node.position.size(); ++j)
      {
        flat_data.push_back(node.position[j]);
      }
    }
  }
}

bool GPUCollisionBridge::checkBatch(
    const std::vector<Trajectory>& trajectories,
    std::vector<float>& costs)
{
#ifdef USE_GPU
  if (!gpu_available_ || trajectories.empty())
  {
    return false;
  }
  
  try
  {
    // Convert trajectories to flat array
    std::vector<float> flat_data;
    size_t num_samples, num_waypoints, num_joints;
    trajectoriesToArray(trajectories, flat_data, num_samples, num_waypoints, num_joints);
    
    // Create numpy array
    py::array_t<float> traj_array({
      static_cast<py::ssize_t>(num_samples),
      static_cast<py::ssize_t>(num_waypoints),
      static_cast<py::ssize_t>(num_joints)
    }, flat_data.data());
    
    // Call GPU collision checker
    py::array_t<float> costs_array = 
      py_checker_->attr("check_trajectory_batch")(traj_array);
    
    // Convert results back
    auto costs_buf = costs_array.unchecked<1>();
    costs.resize(num_samples);
    for (size_t i = 0; i < num_samples; ++i)
    {
      costs[i] = costs_buf(i);
    }
    
    return true;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_THROTTLE(5.0, "GPU collision check failed: %s", e.what());
    return false;
  }
#else
  return false;
#endif
}

void GPUCollisionBridge::updateWorld(
    const std::map<std::string, std::map<std::string, std::vector<double>>>& obstacles)
{
#ifdef USE_GPU
  if (!gpu_available_) return;
  
  try
  {
    // Convert to Python dict format
    py::dict py_obstacles;
    
    for (const auto& [name, props] : obstacles)
    {
      py::dict obj_dict;
      
      for (const auto& [key, values] : props)
      {
        if (key == "type")
        {
          obj_dict["type"] = py::str("cuboid");  // Simplified: assume cuboid
        }
        else
        {
          py::list py_list;
          for (double val : values)
          {
            py_list.append(val);
          }
          obj_dict[py::str(key)] = py_list;
        }
      }
      
      py_obstacles[py::str(name)] = obj_dict;
    }
    
    // Update GPU world
    py_checker_->attr("update_world")(py_obstacles);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Failed to update GPU world: %s", e.what());
  }
#endif
}

} // namespace pce_ros
