cmake_minimum_required(VERSION 3.0.2)
project(fail_safe)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  roscpp
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES waypoint_specification
  CATKIN_DEPENDS actionlib geometry_msgs move_base_msgs roscpp tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fail_safe_scan src/fail_safe_scan.cpp)
target_link_libraries(fail_safe_scan ${catkin_LIBRARIES})