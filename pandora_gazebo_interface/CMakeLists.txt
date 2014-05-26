cmake_minimum_required(VERSION 2.8.3)
project(pandora_gazebo_interface)
find_package(
  catkin
  REQUIRED COMPONENTS
  roscpp
  roslint
  gazebo_ros
  gazebo_plugins
  gazebo_ros_control
  control_toolbox
  pandora_xmega_hardware_interface
  pandora_arm_hardware_interface
  pluginlib
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    gazebo_interface
  CATKIN_DEPENDS
    roscpp
    roslint
    gazebo_ros
    gazebo_plugins
    gazebo_ros_control
    control_toolbox
    pandora_xmega_hardware_interface
    pandora_arm_hardware_interface
    pluginlib
#  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS})

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS})

set(CMAKE_BUILD_TYPE Debug)

add_library(
  gazebo_interface
  src/gazebo_interface.cpp)
target_link_libraries(
  gazebo_interface
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES})
