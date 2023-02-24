#!/usr/bin/python3

# This example shows how to read periodically the joint angles and the position of the robot.
# Use Ctrl+C to stop the program 

import rospy
from sensor_msgs.msg import JointState
from robot_movement_interface.msg import EulerFrame

def cb_joint_states(msg):
    print("Joint positions in radians: " + str(msg.position))

def cb_tool_frame(msg):
    print("Robot position as Euler Intrinsic ZYX frame (m, radians): " + str(msg))

if __name__ == "__main__":
    rospy.init_node('ros_example_read_robot_position')

    # ROS Subscribe to the joint_states topic
    sub_joint_states = rospy.Subscriber("/joint_states", JointState, cb_joint_states)

    # ROS Subscribe to the tool_frame topic
    sub_tool_frame = rospy.Subscriber("/tool_frame", EulerFrame, cb_tool_frame)
    
    rospy.spin()


    
