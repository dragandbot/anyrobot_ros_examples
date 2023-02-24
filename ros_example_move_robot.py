#!/usr/bin/python3

# This example moves the robot periodically while reading the joint angles and the cartesian position of the robot.
# Use Ctrl+C to stop the program. The robot will finish the currently running movement.

import rospy
from sensor_msgs.msg import JointState
from robot_movement_interface.msg import CommandList, Command, Result, EulerFrame

pub_command_list = None

def cb_joint_states(msg):
    print("Joint positions in radians: " + str(msg.position))

def cb_tool_frame(msg):
    print("Robot position as Euler Intrinsic ZYX frame (m, radians): " + str(msg))

def cb_command_result(msg):
    global pub_command_list

    # Publish a new move command if the result of the preceeding command is Success
    if msg.result_code == Result.SUCCESS:
        pub_command_list.publish(create_move_command())

# Returns a joint motion command with two waypoints (two joint positions)
def create_move_command():
    cmd_list = CommandList()

    cmd = Command()
    cmd.command_type = "PTP"
    cmd.pose_type = "JOINTS"
    cmd.pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    cmd.velocity_type = "RAD/S"
    cmd.velocity = [0.1]
    cmd.acceleration_type = "RAD/S^2"
    cmd.acceleration = [1.0]
    cmd.blending_type = "%"
    cmd.blending = [0.0]
    cmd_list.commands.append(cmd)

    cmd = Command()
    cmd.command_type = "PTP"
    cmd.pose_type = "JOINTS"
    cmd.pose = [1.0, 1.0, 1.0, -1.0, -1.0, -1.0]
    cmd.velocity_type = "RAD/S"
    cmd.velocity = [0.1]
    cmd.acceleration_type = "RAD/S^2"
    cmd.acceleration = [1.0]
    cmd.blending_type = "%"
    cmd.blending = [0.0]
    cmd_list.commands.append(cmd)

    return cmd_list

if __name__ == "__main__":
    rospy.init_node('ros_example_read_robot_position')

    # Subscribe to the joint_states topic
    sub_joint_states = rospy.Subscriber("/joint_states", JointState, cb_joint_states)

    # Subscribe to the tool_frame topic
    sub_tool_frame = rospy.Subscriber("/tool_frame", EulerFrame, cb_tool_frame)

    # Publisher to the command_list topic
    pub_command_list = rospy.Publisher("/command_list", CommandList, queue_size=1)

    # Subscribe to the command_result topic
    sub_command_result = rospy.Subscriber("/command_result", Result, cb_command_result)

    rospy.sleep(1.0)

    # Publish the first command
    pub_command_list.publish(create_move_command())
    
    rospy.spin()


    
