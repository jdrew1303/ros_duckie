cmake_minimum_required(VERSION 2.8.3)
project(cv_lane_follower)

find_package(catkin REQUIRED COMPONENTS
  rospy
  ackermann_msgs
)

catkin_python_setup()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})