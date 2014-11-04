cmake_minimum_required(VERSION 2.8.3)
project(cyberglove)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  rospy
  std_msgs
  sensor_msgs
  diagnostic_msgs
  genmsg
  sr_cyberglove_config
  cereal_port
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED system filesystem date_time thread)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Calibration.srv
  Start.srv
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
  diagnostic_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cyberglove
CATKIN_DEPENDS roslib roscpp rospy std_msgs sensor_msgs diagnostic_msgs genmsg sr_cyberglove_config cereal_port message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp library
# add_library(cyberglove
#   src/${PROJECT_NAME}/cyberglove.cpp
# )

## Declare a cpp executable
add_executable(cyberglove
  src/cyberglove_publisher.cpp
  src/cyberglove_node.cpp
  src/serial_glove.cpp
  src/xml_calibration_parser.cpp
  src/cyberglove_service.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(cyberglove
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(cyberglove
  cereal_port
  tinyxml
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS cyberglove
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest) 
  add_rostest_gtest(test_cyberglove
    test/test_calibration.test
    test/test_calibration.cpp
    src/xml_calibration_parser.cpp
  )
  target_link_libraries(test_cyberglove
    tinyxml
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  
endif()

