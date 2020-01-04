cmake_minimum_required(VERSION 2.8.3)
project(premaidai_controller)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  controller_manager
  message_generation
  controller_manager
)

find_package(Boost REQUIRED COMPONENTS thread)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

## Generate services in the 'srv' folder
add_service_files(
  FILES
  MotionRequest.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  sensor_msgs
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(premaidai_controller
  src/command.cc
  src/serial_port.cc
  src/ros_bridge.cc
  src/robot.cc
)

target_link_libraries(premaidai_controller
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(premaidai_controller_node
  src/ros_bridge_node.cc
)

target_link_libraries(premaidai_controller_node
  premaidai_controller
  ${catkin_LIBRARIES}
)

add_executable(premaidai_driver_node
  src/robot.cc
  src/ros_hw_interface_node.cc
)

target_link_libraries(premaidai_driver_node
  premaidai_controller
  ${catkin_LIBRARIES}
)

add_executable(premaidai_driver_node
  src/ros_hw_interface_node.cc
)

target_link_libraries(premaidai_driver_node
  premaidai_ros_bridge
  ${catkin_LIBRARIES}
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES premaidai_bridge
  # CATKIN_DEPENDS other_catkin_pkg
  # DEPENDS system_lib
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
