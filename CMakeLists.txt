cmake_minimum_required(VERSION 3.0.2)
project(fake_cartesian_state_sim)

find_package(catkin REQUIRED COMPONENTS
    tf
    rospy
    geometry_msgs
    #cartesian_state_msgs
)

catkin_package()

include_directories()

