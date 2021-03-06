cmake_minimum_required(VERSION 2.8.3)
project(pandora_gazebo_interface)

# Enable C++11 support, in order to link with gazebo6
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

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
  pandora_monstertruck_hardware_interface
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

add_library(
  ${PROJECT_NAME}_gio_gazebo_interface
  src/gio_gazebo_interface.cpp)
target_link_libraries(
  ${PROJECT_NAME}_gio_gazebo_interface
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES})

add_library(
  ${PROJECT_NAME}_monstertruck_gazebo_interface
  src/monstertruck_gazebo_interface.cpp)
target_link_libraries(
  ${PROJECT_NAME}_monstertruck_gazebo_interface
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
