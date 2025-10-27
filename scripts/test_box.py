#!/usr/bin/env python
import rospy
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

rospy.init_node('test_collision_object')
pub = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)
rospy.sleep(1.0)

# Create Box_1
box = CollisionObject()
box.header.frame_id = "panda_link0"
box.id = "Box_1"
box.operation = CollisionObject.ADD

# CRITICAL: Set the object-level pose (not primitive pose!)
box.pose.position.x = 0.34
box.pose.position.y = 0.27
box.pose.position.z = 0.58
box.pose.orientation.w = 1.0

# Box shape with IDENTITY primitive pose (relative to object pose)
primitive = SolidPrimitive()
primitive.type = SolidPrimitive.BOX
primitive.dimensions = [0.2, 0.2, 0.2]
box.primitives.append(primitive)

# Primitive pose is IDENTITY (at object's location)
primitive_pose = Pose()
primitive_pose.orientation.w = 1.0  # Identity orientation
box.primitive_poses.append(primitive_pose)

print("Publishing Box_1 at [0.34, 0.27, 0.58]...")
pub.publish(box)
rospy.sleep(0.5)
pub.publish(box)
print("Done!")
