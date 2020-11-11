cmake_minimum_required(VERSION 2.8.3)
project(interbotix_moveit_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  interbotix_moveit
  message_generation
  moveit_commander
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  rospy
  std_msgs
  tf
  calibration
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  MoveItPlan.srv
  # Capture.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS geometry_msgs interbotix_moveit message_runtime moveit_commander moveit_ros_planning_interface moveit_visual_tools roscpp rospy std_msgs tf calibration
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## Specify libraries to link a library or executable target against
## Add cmake target dependencies of the executable
add_executable(moveit_interface_node src/moveit_interface_node.cpp src/moveit_interface.cpp)
target_link_libraries(moveit_interface_node ${catkin_LIBRARIES})
add_dependencies(moveit_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

set(DESKTOP_APP_DIR "$ENV{HOME}/.local/share/applications")
set(DESKTOP_ICON_DIR ${DESKTOP_APP_DIR})
set(DESKTOP_APP_SUFFIX " (local)")
message(${CMAKE_CURRENT_BINARY_DIR})
message(${CMAKE_BINARY_DIR})
# Create directory for the desktop file/icon
install(DIRECTORY DESTINATION
  ${DESKTOP_APP_DIR}
)
install(DIRECTORY DESTINATION
  ${DESKTOP_ICON_DIR}
)

configure_file(
  scripts/arm_exec.bash.in
  scripts/arm_exec.bash
  @ONLY
)

# Install bash scripts
install(
  PROGRAMS
  ${CMAKE_CURRENT_BINARY_DIR}/scripts/arm_exec.bash
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)
message(${CATKIN_PACKAGE_BIN_DESTINATION})

configure_file(
  desktop/arm_control.desktop.in
  desktop/arm_control.desktop
)

install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/desktop/arm_control.desktop
    DESTINATION
    ${DESKTOP_APP_DIR}
)

install(
  FILES
  desktop/arm_control.png
  DESTINATION
  ${DESKTOP_ICON_DIR}
)
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/moveit_python_interface
  scripts/location_capture_moveit
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY
  launch
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)