#pragma once

#include <ros/ros.h>
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <pce/Trajectory.h>

// Forward declare to avoid Python headers in main code
namespace pybind11 { class object; class scoped_interpreter; }

namespace pce_ros
{

/**
 * @brief GPU collision checking bridge via CuRobo
 * 
 * Provides batch collision checking using NVIDIA GPU acceleration.
 * Falls back gracefully to CPU if GPU unavailable.
 */
class GPUCollisionBridge
{
public:
  /**
   * @brief Initialize GPU collision checker
   * @param urdf_path Path to robot URDF file
   * @param base_link Robot base link name
   * @param ee_link End effector link name
   * @param collision_clearance Collision safety margin
   */
  GPUCollisionBridge(const std::string& urdf_path,
                     const std::string& base_link,
                     const std::string& ee_link,
                     float collision_clearance);
  
  ~GPUCollisionBridge();
  
  /**
   * @brief Check if GPU is available and initialized
   */
  bool isAvailable() const { return gpu_available_; }
  
  /**
   * @brief Get GPU device name
   */
  std::string getDeviceName() const { return device_name_; }
  
  /**
   * @brief Batch collision check on GPU
   * @param trajectories Vector of trajectories to evaluate
   * @param costs Output vector of costs (one per trajectory)
   * @return true if successful, false if GPU unavailable
   */
  bool checkBatch(const std::vector<Trajectory>& trajectories,
                  std::vector<float>& costs);
  
  /**
   * @brief Update obstacle world
   * @param obstacles Map of obstacle name to properties
   */
  void updateWorld(const std::map<std::string, std::map<std::string, std::vector<double>>>& obstacles);

private:
  bool gpu_available_;
  std::string device_name_;
  float collision_clearance_;
  
  std::unique_ptr<pybind11::scoped_interpreter> py_guard_;
  std::unique_ptr<pybind11::object> py_checker_;
  
  // Helper to flatten trajectories to numpy-compatible format
  void trajectoriesToArray(const std::vector<Trajectory>& trajectories,
                          std::vector<float>& flat_data,
                          size_t& num_samples,
                          size_t& num_waypoints,
                          size_t& num_joints);
};

} // namespace pce_ros