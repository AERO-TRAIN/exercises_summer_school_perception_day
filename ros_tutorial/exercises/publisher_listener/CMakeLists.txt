cmake_minimum_required(VERSION 2.8.3)
project(publisher_listener)

add_definitions(-std=c++17)


find_package(catkin REQUIRED COMPONENTS rospy )

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    )
