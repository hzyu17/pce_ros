#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include "pce_planner.h"

namespace pce_ros
{

class PCEPlannerManager : public planning_interface::PlannerManager
{
public:
  PCEPlannerManager();
  
  bool initialize(const moveit::core::RobotModelConstPtr& model,
                  const std::string& ns) override;
  
  std::string getDescription() const override { return "PCE"; }
  
  void getPlanningAlgorithms(std::vector<std::string>& algs) const override;
  
  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr& planning_scene,
      const planning_interface::MotionPlanRequest& req,
      moveit_msgs::MoveItErrorCodes& error_code) const override;
  
  bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override;

protected:
  moveit::core::RobotModelConstPtr robot_model_;
  std::map<std::string, XmlRpc::XmlRpcValue> config_;
};

} // namespace pce_ros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(pce_ros::PCEPlannerManager, planning_interface::PlannerManager)