cmake_minimum_required(VERSION 3.0.2)
project(rotary_encoder)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

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

add_executable(cmdvel_cmdmotorspeed src/cmdvel_cmdmotorspeed.cpp)
target_link_libraries(cmdvel_cmdmotorspeed ${catkin_LIBRARIES})

add_executable(odometry_publisher src/odometry_publisher.cpp)
target_link_libraries(odometry_publisher ${catkin_LIBRARIES})

#add_executable(Test src/Test.cpp)
#target_link_libraries(Test ${catkin_LIBRARIES})

#add_executable(Test2 src/Test2.cpp)
#target_link_libraries(Test2 ${catkin_LIBRARIES})

add_executable(Test4 src/Test4.cpp)
target_link_libraries(Test4 ${catkin_LIBRARIES})
