#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
import geometry_msgs.msg

def main():
    # Initialize WITHOUT creating new nodes
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pce_planning_test', anonymous=True)
    
    # Wait for move_group to be ready
    rospy.loginfo("Waiting for move_group node...")
    try:
        rospy.wait_for_service('/move_group/plan_kinematic_path', timeout=10.0)
    except rospy.ROSException:
        rospy.logerr("move_group service not available!")
        return
    
    rospy.loginfo("move_group is ready")
    
    # Check what plugin is ACTUALLY running
    plugin = rospy.get_param('/move_group/planning_plugin', 'NOT_SET')
    rospy.loginfo(f"Active planning plugin: {plugin}")
    
    if plugin != 'pce_ros/PCEPlannerManager':
        rospy.logerr("ERROR: PCE is not the active planning plugin!")
        rospy.logerr("The move_group node must be launched with PCE already set.")
        rospy.logerr("Please restart with correct launch file.")
        return
    
    # Now create interfaces (connects to existing move_group)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("panda_arm")
    
    print("\n" + "="*60)
    print("Panda PCE Planning Test")
    print("="*60)
    print(f"Planning frame: {group.get_planning_frame()}")
    print(f"End effector: {group.get_end_effector_link()}")
    print(f"Active plugin: {plugin}")
    print("="*60 + "\n")
    
    # Rest of your script...