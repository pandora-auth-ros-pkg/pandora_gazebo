cmake_minimum_required(VERSION 2.8.3)
project(pandora_gazebo_plugins)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
      roscpp
      tf
      std_msgs
      geometry_msgs
      sensor_msgs
      gazebo_ros
      gazebo_plugins
      pandora_sensor_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} )

generate_dynamic_reconfigure_options(
  cfg/Differential.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    tf
    std_msgs
    geometry_msgs
    sensor_msgs
    gazebo_ros
    gazebo_plugins
    pandora_sensor_msgs
#  DEPENDS system_lib
  LIBRARIES
    motion_plugin
    pandora_sonar_plugin
    pandora_imu_stabilizer_plugin
    pandora_thermal_plugin
    pandora_co2_plugin
    pandora_microphone_plugin
    pandora_differential_plugin
    pandora_p3d_plugin
)

add_library(${PROJECT_NAME}_pandora_sonar_plugin src/pandora_sonar_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_pandora_sonar_plugin 
    gazebo_ros_camera_utils 
    RayPlugin 
    ${GAZEBO_LIBRARIES} 
    ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_pandora_imu_stabilizer_plugin src/pandora_imu_stabilizer_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_pandora_imu_stabilizer_plugin 
    ${GAZEBO_LIBRARIES} 
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES}
)

add_library(${PROJECT_NAME}_pandora_differential_plugin src/pandora_differential_plugin.cpp)
add_dependencies(${PROJECT_NAME}_pandora_differential_plugin pandora_gazebo_plugins_gencfg)
target_link_libraries(${PROJECT_NAME}_pandora_differential_plugin 
    ${GAZEBO_LIBRARIES} 
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES}
)

add_library(${PROJECT_NAME}_pandora_p3d_plugin src/pandora_p3d_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_pandora_p3d_plugin 
    ${GAZEBO_LIBRARIES} 
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES}
)

add_library(${PROJECT_NAME}_pandora_thermal_plugin src/pandora_thermal_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_pandora_thermal_plugin
    CameraPlugin
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_pandora_co2_plugin src/pandora_co2_plugin.cpp)
add_dependencies(${PROJECT_NAME}_pandora_co2_plugin pandora_sensor_msgs_gencpp)
target_link_libraries(${PROJECT_NAME}_pandora_co2_plugin
    CameraPlugin
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_pandora_microphone_plugin src/pandora_microphone_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_pandora_microphone_plugin
    CameraPlugin
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
)
