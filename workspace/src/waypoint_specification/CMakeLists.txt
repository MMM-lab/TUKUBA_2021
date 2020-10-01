cmake_minimum_required(VERSION 3.0.2)
project(waypoint_specification)

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

add_executable(waypoint_specification_node src/waypoint_specification_node.cpp)
target_link_libraries(waypoint_specification_node ${catkin_LIBRARIES})

add_executable(save_odometry src/save_odometry.cpp)
target_link_libraries(save_odometry ${catkin_LIBRARIES})