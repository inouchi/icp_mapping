cmake_minimum_required(VERSION 2.8.3)
project(ssh_icp_mapping)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#SET(CMAKE_BUILD_TYPE Debug) # NOTE: Debug mode causes segfault because of PCL when C++11 is used.
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo

find_package(PCL 1.7 REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  message_generation
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GenerateModel.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ssh_icp_mapping
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(icp_mapper_node
  src/main.cpp
  src/ICPMapper.cpp
)
add_dependencies(icp_mapper_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PCL_DEFINITIONS}
  ${PROJECT_NAME}_generate_massages_cpp
)
target_link_libraries(icp_mapper_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)