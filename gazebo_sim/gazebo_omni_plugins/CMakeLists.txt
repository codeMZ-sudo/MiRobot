cmake_minimum_required(VERSION 3.1.4)
project(gazebo_omni_plugins)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  geometry_msgs
  joint_state_controller
  robot_state_publisher
  roscpp
  std_msgs
  tf
  message_generation
  dynamic_reconfigure
)

find_package(gazebo)

add_message_files(
  FILES
  gazebo_omni_plugins_vel.msg
  gazebo_omni_plugins_pid.msg
)

generate_dynamic_reconfigure_options(
  cfg/gazebo_ros_omni_pid.cfg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_omni_plugins
  CATKIN_DEPENDS
    nav_msgs
    geometry_msgs
    joint_state_controller
    robot_state_publisher
    roscpp
    std_msgs
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(gazebo_ros_omni_drive SHARED src/gazebo_ros_omni_drive.cpp)
target_link_libraries(gazebo_ros_omni_drive ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(gazebo_ros_omni_drive gazebo_omni_plugins_gencfg gazebo_omni_plugins_generate_messages_cpp)

add_library(gazebo_ros_omni_drive_ac SHARED src/gazebo_ros_omni_drive_ac.cpp)
target_link_libraries(gazebo_ros_omni_drive_ac ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(gazebo_ros_omni_drive_ac gazebo_omni_plugins_gencfg gazebo_omni_plugins_generate_messages_cpp)

add_library(gazebo_ros_omni_pid_drive SHARED src/gazebo_ros_omni_pid_drive.cpp src/gazebo_ros_omni_pid.cpp)
target_link_libraries(gazebo_ros_omni_pid_drive ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(gazebo_ros_omni_pid_drive gazebo_omni_plugins_gencfg gazebo_omni_plugins_generate_messages_cpp)

add_executable(gazebo_omni_plugin_pid src/gazebo_ros_omni_pid_param.cpp)
target_link_libraries(gazebo_omni_plugin_pid ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(gazebo_omni_plugin_pid gazebo_omni_plugins_gencfg gazebo_omni_plugins_generate_messages_cpp)

install(TARGETS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
