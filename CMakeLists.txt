cmake_minimum_required(VERSION 2.8.3)
project(task_sim_nimbus_bridge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  ar_track_alvar_msgs
  cmake_modules
  eigen_conversions
  geometry_msgs
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_srvs
  task_sim
  tf
)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen REQUIRED)

add_message_files(
  FILES
  BoundingBox.msg
)

add_service_files(
  FILES
  Classify.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES task_sim
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(
  robot_executor_node
  nodes/robot_executor_node.cpp
  src/robot_executor.cpp
)
add_executable(
  state_calculator_node
  nodes/state_calculator_node.cpp
  src/state_calculator.cpp
  src/bounding_box_calculator.cpp
)
add_executable(
  controller_node
  nodes/controller_node.cpp
  src/controller.cpp
)

add_dependencies(
  robot_executor_node
  task_sim_generate_messages_cpp
)
add_dependencies(
  state_calculator_node
  task_sim_generate_messages_cpp
  task_sim_nimbus_bridge_generate_messages_cpp
)
add_dependencies(
  controller_node
  task_sim_generate_messages_cpp
)

target_link_libraries(
  robot_executor_node
  ${catkin_LIBRARIES}
)
target_link_libraries(
  state_calculator_node
  ${boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${EIGEN_INCLUDE_DIRS}
)
target_link_libraries(
  controller_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_task_sim.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
