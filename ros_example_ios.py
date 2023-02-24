#!/usr/bin/python3

# Assumed a Simbot is running (drag&bot Robot Simulator)

# This example shows how to get and set digital IOs with ROS Service Clients
# Digital Output will be read, then set the inverted value and then read again
# Use Ctrl+C to stop the program

import rospy
from robot_movement_interface.srv import *
from robot_movement_interface.msg import *

if __name__ == "__main__":
    ####################################################################
    # Request the original value of the digital io
    rospy.wait_for_service('/robot/get_digital_ios', 1.0) # Wait until the ROS service is active
    srv_get_value = rospy.ServiceProxy('/robot/get_digital_ios', GetDigitalIOs) # Create the ROS service client

    request1 = GetDigitalIOsRequest()

    io_id = IOID()
    io_id.group_id = "DO"
    io_id.pin_number = 1
    request1.ios.append(io_id)

    response1 = srv_get_value(request1)

    assert(len(response1.ios) > 0)

    io_value = bool(response1.ios[0].value)
    print("Digital Output original value: " + str(io_value))
    
    ####################################################################
    # Invert the original value
    new_io_value = not io_value

    ####################################################################
    # Set the inverted value to the digital io
    rospy.wait_for_service('/robot/set_digital_ios', 1.0) # Wait until the ROS service is active
    srv_set_value = rospy.ServiceProxy('/robot/set_digital_ios', SetDigitalIOs) # Create the ROS service client

    request2 = SetDigitalIOsRequest()

    digital_io = DigitalIOValue()
    digital_io.id.group_id = "DO"
    digital_io.id.pin_number = 1
    digital_io.value = new_io_value
    request2.ios.append(digital_io)

    response2 = srv_set_value(request2)

    ####################################################################
    # Request the new value of the digital io
    # The service client from the first get request is used again
    request1 = GetDigitalIOsRequest()

    io_id = IOID()
    io_id.group_id = "DO"
    io_id.pin_number = 1
    request1.ios.append(io_id)

    response1 = srv_get_value(request1)

    assert(len(response1.ios) > 0)

    io_value = bool(response1.ios[0].value)
    print("Digital Output new value: " + str(io_value))


    
