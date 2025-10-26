/**
 * @file pce_planner_manager.cpp
 * @brief Plugin interface for PCE planner in MoveIt
 *
 * This file implements the PlannerManager interface that allows
 * MoveIt to discover and use the PCE planner as a plugin.
 */

#include "pce_planner.h"
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_response.h>
#include <class_loader/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace pce_ros
{

/**
 * @brief MoveIt PlannerManager implementation for PCE
 * 
 * This class serves as the entry point for MoveIt to create
 * PCE planning contexts for different planning groups.
 */
class PCEPlannerManager : public planning_interface::PlannerManager
{
public:
  PCEPlannerManager() : planning_interface::PlannerManager() {}

  virtual ~PCEPlannerManager() {}

  /**
   * @brief Initialize the planner manager with robot model and namespace
   */
  virtual bool initialize(const moveit::core::RobotModelConstPtr& model,
                        const std::string& ns) override
  {
    ROS_INFO("=== PCEPlannerManager::initialize ===");
    ROS_INFO("  Namespace: '%s'", ns.c_str());
    
    robot_model_ = model;
    ns_ = ns;
    
    // CRITICAL: Use the passed namespace, not private namespace
    // MoveIt passes "move_group" as the namespace
    ros::NodeHandle nh(ns);  // Changed from ros::NodeHandle("~")
    
    ROS_INFO("  NodeHandle namespace: %s", nh.getNamespace().c_str());
    
    // Parameters are at: /move_group/pce/panda_arm/pce_planner/...
    std::string param_name = "pce";
    
    ROS_INFO("  Looking for config at: %s/%s", nh.getNamespace().c_str(), param_name.c_str());
    
    // Test if we can access the parameters
    std::vector<std::string> planning_groups;
    if (nh.getParam("pce/planning_groups", planning_groups))
    {
      ROS_INFO("  ✓ Found planning_groups with %zu entries:", planning_groups.size());
      for (const auto& group : planning_groups)
      {
        ROS_INFO("    - %s", group.c_str());
      }
    }
    else
    {
      ROS_ERROR("  ✗ Could not find pce/planning_groups!");
    }
    
    // Load configuration for all planning groups
    if (!PCEPlanner::getConfigData(nh, config_, param_name))
    {
      ROS_ERROR("PCEPlannerManager: Failed to load configuration from parameter '%s'",
                param_name.c_str());
      return false;
    }
    
    ROS_INFO("PCEPlannerManager: Initialized with %zu planning group configurations",
            config_.size());
    
    return true;
  }

  /**
   * @brief Describe the planner (algorithm name and parameters)
   */
  virtual std::string getDescription() const override 
  { 
    return "Proximal Cross-Entropy Method"; 
  }

  /**
   * @brief Get available planning algorithms
   * PCE only has one algorithm variant
   */
  virtual void getPlanningAlgorithms(std::vector<std::string>& algs) const override
  {
    algs.clear();
    algs.push_back("PCE");
  }

  /**
   * @brief Create a planning context for a specific request
   * This is called by MoveIt when a planning request is received
   */
  virtual planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr& planning_scene,
      const planning_interface::MotionPlanRequest& req,
      moveit_msgs::MoveItErrorCodes& error_code) const override
  {
    // Validate request
    if (req.group_name.empty())
    {
      ROS_ERROR("PCEPlannerManager: No planning group specified");
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      return planning_interface::PlanningContextPtr();
    }

    // Check if we have configuration for this group
    auto config_it = config_.find(req.group_name);
    if (config_it == config_.end())
    {
      ROS_ERROR("PCEPlannerManager: No configuration found for group '%s'", 
                req.group_name.c_str());
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      return planning_interface::PlanningContextPtr();
    }

    // Create PCE planner for this group
    try
    {
      auto planner = std::make_shared<PCEPlanner>(
          req.group_name, 
          config_it->second, 
          robot_model_);

      // Check if planner can service this request
      if (!planner->canServiceRequest(req))
      {
        ROS_ERROR("PCEPlannerManager: Cannot service request for group '%s'", 
                  req.group_name.c_str());
        error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return planning_interface::PlanningContextPtr();
      }

      // Set up the planning context
      planner->setPlanningScene(planning_scene);
      planner->setMotionPlanRequest(req);

      error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      return planner;
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("PCEPlannerManager: Failed to create planner: %s", e.what());
      error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
      return planning_interface::PlanningContextPtr();
    }
  }


  virtual bool canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const override
  {
    ROS_INFO("=== PCEPlannerManager::canServiceRequest ===");
    ROS_INFO("  Requested group: %s", req.group_name.c_str());
    
    // Check if this group has a configuration
    // config_ should have been populated in initialize()
    if (config_.find(req.group_name) != config_.end())
    {
      ROS_INFO("  ✓ Group '%s' found in config", req.group_name.c_str());
      return true;
    }
    
    ROS_ERROR("  ✗ Group '%s' NOT found in config", req.group_name.c_str());
    ROS_ERROR("  Available groups:");
    for (const auto& entry : config_)
    {
      ROS_ERROR("    - %s", entry.first.c_str());
    }
    
    return false;
  }

  
protected:
  moveit::core::RobotModelConstPtr robot_model_;
  std::map<std::string, XmlRpc::XmlRpcValue> config_;
  std::string ns_;
};

} /* namespace pce_ros */

// Register this planner as a MoveIt plugin
PLUGINLIB_EXPORT_CLASS(pce_ros::PCEPlannerManager, 
                       planning_interface::PlannerManager);
