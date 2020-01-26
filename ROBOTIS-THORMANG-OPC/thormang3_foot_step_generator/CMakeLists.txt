################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(thormang3_foot_step_generator)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  robotis_controller_msgs
  thormang3_walking_module_msgs
  cmake_modules
  message_generation
)

find_package(Eigen3 REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
  FILES
  FootStepCommand.msg
  Step2D.msg
  Step2DArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    roslib
    std_msgs
    geometry_msgs
    robotis_controller_msgs
    thormang3_walking_module_msgs
    cmake_modules
    message_runtime
  DEPENDS EIGEN3
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(thormang3_foot_step_generator_node
   src/robotis_foot_step_generator.cpp
   src/message_callback.cpp
   src/main.cpp
)

add_dependencies(thormang3_foot_step_generator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(thormang3_foot_step_generator_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

################################################################################
# Install
################################################################################
install(TARGETS thormang3_foot_step_generator_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################
# Test
################################################################################
