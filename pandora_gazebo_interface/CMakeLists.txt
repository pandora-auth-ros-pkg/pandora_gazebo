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
  pandora_imu_hardware_interface
  pandora_xmega_hardware_interface
  pandora_arm_hardware_interface
  pandora_sensor_msgs
  pluginlib
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    gazebo_interface
  CATKIN_DEPENDS
    roscpp
    gazebo_ros
    gazebo_plugins
    gazebo_ros_control
    control_toolbox
    pandora_imu_hardware_interface
    pandora_xmega_hardware_interface
    pandora_arm_hardware_interface
    pandora_sensor_msgs
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
  ${PROJECT_NAME}_gears_gazebo_interface
  src/gears_gazebo_interface.cpp)
target_link_libraries(
  ${PROJECT_NAME}_gears_gazebo_interface
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES})

add_library(
  ${PROJECT_NAME}_woody_gazebo_interface
  src/woody_gazebo_interface.cpp)
target_link_libraries(
  ${PROJECT_NAME}_woody_gazebo_interface
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES})

# To check lint : catkin_make roslint_pandora_gazebo_interface
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use, -runtime/references")
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
    pandora_gazebo_interface/include/*.h
    pandora_gazebo_interface/src/*.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
