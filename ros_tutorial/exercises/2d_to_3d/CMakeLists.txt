cmake_minimum_required(VERSION 3.0.2)
project(2d_to_3d)

# necessary beceause of the parameters
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

set(dependencies
  roscpp
  rospy
  roslib
  std_msgs
)

