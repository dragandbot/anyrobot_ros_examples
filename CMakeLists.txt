cmake_minimum_required(VERSION 3.0.2)
project(anyrobot_ros_examples)

find_package(catkin REQUIRED COMPONENTS
  rospy
  robot_movement_interface
  sensor_msgs
)

catkin_package( CATKIN_DEPENDS )