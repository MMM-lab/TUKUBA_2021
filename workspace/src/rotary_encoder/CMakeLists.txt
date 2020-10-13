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

add_executable(simple_publish_PWM_and_odom src/simple_publish_PWM_and_odom.cpp)
target_link_libraries(simple_publish_PWM_and_odom ${catkin_LIBRARIES})

add_executable(Test src/Test.cpp)
target_link_libraries(Test ${catkin_LIBRARIES})

#add_executable(Test2 src/Test2.cpp)
#target_link_libraries(Test2 ${catkin_LIBRARIES})

add_executable(Test4 src/Test4.cpp)
target_link_libraries(Test4 ${catkin_LIBRARIES})

#add_executable(serial_test_node src/serial_test.cpp)
#add_dependencies(serial_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(serial_test_node ${catkin_LIBRARIES})

add_executable(serial_test_node src/serial_test_v1.cpp)
#add_dependencies(serial_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(serial_test_node ${catkin_LIBRARIES})

add_executable(simple_cmd_vel_publish src/simple_cmd_vel_publish.cpp)
target_link_libraries(simple_cmd_vel_publish ${catkin_LIBRARIES})

add_executable(simple_cmd_Int_publish src/simple_cmd_Int_publish.cpp)
target_link_libraries(simple_cmd_Int_publish ${catkin_LIBRARIES})

add_executable(simple_odom_publisher src/simple_odom_publisher.cpp)
target_link_libraries(simple_odom_publisher ${catkin_LIBRARIES})