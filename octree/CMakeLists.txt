cmake_minimum_required(VERSION 2.8.9)
project(Octree)


  set(CMAKE_CXX_FLAGS "-O0 -W -Wall -std=c++11")


#for debugging it is nice to use ros, though this can be deleted if one is to remove the includes in the parsing files.

#find_package(catkin REQUIRED COMPONENTS
#  roscpp
#  rospy
#  std_msgs
#  tf
#  roslib
#)

file(GLOB INCLUDES "include/*.h" "eigen/*.h" "eigen/src/*/*.h")
include_directories(./include eigen eigen/src/*/*  ${catkin_INCLUDE_DIRS})
file(GLOB SOURCES "src/*.cpp" "include/*.h" "eigen/*.h" "eigen/src/*/*.h")

add_executable(Octree ${SOURCES})

target_link_libraries(Octree
  ${catkin_LIBRARIES}
)
