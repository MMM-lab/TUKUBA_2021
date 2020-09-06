cmake_minimum_required(VERSION 3.0.2)
project(encoder_to_odometry)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES encoder_to_odometry
  CATKIN_DEPENDS nav_msgs roscpp std_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(encoder_to_odometry_node src/encoder_to_odometry_node.cpp)
target_link_libraries(encoder_to_odometry_node ${catkin_LIBRARIES})