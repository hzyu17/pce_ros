#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
import geometry_msgs.msg

def main():
    print("Starting PCE planning test...")
    
    # Initialize
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pce_planning_test', anonymous=True)
    
    print("Node initialized")
    
    # Wait for move_group to be ready
    print("Waiting for move_group service...")
    try:
        rospy.wait_for_service('/move_group/plan_kinematic_path', timeout=5.0)
        print("✓ move_group service is available")
    except rospy.ROSException:
        print("✗ move_group service not available!")
        print("Make sure pce_standalone.launch is running")
        return
    
    # Check plugin
    plugin = rospy.get_param('/move_group/planning_plugin', 'NOT_SET')
    print(f"\nActive planning plugin: {plugin}")
    
    if plugin != 'pce_ros/PCEPlannerManager':
        print("ERROR: PCE is not the active planning plugin!")
        print("Expected: pce_ros/PCEPlannerManager")
        print(f"Got: {plugin}")
        return
    
    print("✓ PCE is active\n")
    
    # Create interfaces
    print("Creating MoveIt interfaces...")
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("panda_arm")
    
    print("✓ Interfaces created\n")
    
    print("="*60)
    print("Panda PCE Planning Test")
    print("="*60)
    print(f"Planning frame: {group.get_planning_frame()}")
    print(f"End effector: {group.get_end_effector_link()}")
    print(f"Planning plugin: {plugin}")
    print("="*60 + "\n")
    
    # Add obstacles
    print("Setting up planning scene with obstacles...")
    rospy.sleep(1.0)
    
    # Table
    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = "panda_link0"
    table_pose.pose.position.x = 0.5
    table_pose.pose.position.y = 0.0
    table_pose.pose.position.z = -0.05
    table_pose.pose.orientation.w = 1.0
    scene.add_box("table", table_pose, size=(0.6, 1.0, 0.1))
    print("✓ Table added")
    
    # Obstacle
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_link0"
    box_pose.pose.position.x = 0.4
    box_pose.pose.position.y = 0.3
    box_pose.pose.position.z = 0.2
    box_pose.pose.orientation.w = 1.0
    scene.add_box("obstacle", box_pose, size=(0.1, 0.1, 0.4))
    print("✓ Obstacle added\n")
    
    rospy.sleep(1.0)
    
    # Plan to a pose target
    print("Planning with PCE to pose target...")
    
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = -0.2
    pose_goal.position.z = 0.4
    
    group.set_pose_target(pose_goal)
    group.set_planning_time(15.0)
    
    print("Calling group.plan()...")
    print("Watch RViz and Terminal 1 for PCE planning logs!\n")
    
    plan = group.plan()
    
    print("\n" + "="*60)
    if plan[0]:
        print("✓ PCE planning SUCCEEDED!")
        print(f"  Planning time: {plan[2]:.3f} seconds")
        print(f"  Trajectory points: {len(plan[1].joint_trajectory.points)}")
        print("="*60 + "\n")
        
        user_input = input("Execute trajectory? (y/n): ")
        if user_input.lower() == 'y':
            group.execute(plan[1], wait=True)
            print("Execution complete!")
    else:
        print("✗ PCE planning FAILED!")
        print("="*60 + "\n")
    
    # Cleanup
    print("Cleaning up...")
    scene.remove_world_object("table")
    scene.remove_world_object("obstacle")
    moveit_commander.roscpp_shutdown()
    print("Done!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Interrupted by user")
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()