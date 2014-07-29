cmake_minimum_required(VERSION 2.8.3)
project(ggp_robot)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
  find_package(catkin REQUIRED COMPONENTS
      pcl_conversions
      pcl_ros
      roscpp
      rospy
      sensor_msgs
      image_transport
      std_msgs
      cv_bridge
      visualization_msgs
      )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV2 REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

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
      LIBRARIES ggp_robot
      CATKIN_DEPENDS
        pcl_conversions
        pcl_ros
        sensor_msgs
        cv_bridge
        roscpp
        rospy
        std_msgs
        image_transport
        visualization_msgs
      )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

## Declare a cpp library
# add_library(ggp_robot
#   src/${PROJECT_NAME}/ggp_robot.cpp
# )

## Declare a cpp executable
# add_executable(ggp_robot_node src/ggp_robot_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ggp_robot_node ggp_robot_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ggp_robot_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS ggp_robot ggp_robot_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ggp_robot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

  add_executable(pclview_rgbd src/pclview_rgbd.cpp)
target_link_libraries(pclview_rgbd ${catkin_LIBRARIES})
target_link_libraries(pclview_rgbd ${PCL_LIBRARIES} libvtkFiltering.so)


  add_executable(cloudview_rgbd src/cloudview_rgbd.cpp)
target_link_libraries(cloudview_rgbd ${catkin_LIBRARIES})

  add_executable(process_cloud src/process_cloud.cpp)
target_link_libraries(process_cloud ${catkin_LIBRARIES})

  add_executable(cluster_extraction src/cluster_extraction.cpp)
target_link_libraries(cluster_extraction ${catkin_LIBRARIES})
target_link_libraries(cluster_extraction ${PCL_LIBRARIES})

  add_executable(orb_test src/orb_test.cpp)
target_link_libraries(orb_test ${catkin_LIBRARIES})
  find_package(OpenCV REQUIRED)
  include_directories(${OpenCV_INCLUDE_CIRS})
  target_link_libraries(orb_test ${OpenCV_LIBRARIES})
  
  add_executable(orb_test2 src/orb_test2.cpp)
target_link_libraries(orb_test2 ${catkin_LIBRARIES})
  find_package(OpenCV REQUIRED)
  include_directories(${OpenCV_INCLUDE_CIRS})
  target_link_libraries(orb_test2 ${OpenCV_LIBRARIES})

  add_executable(feature_test src/feature_test.cpp)
target_link_libraries(feature_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

  add_executable(chessboard_test src/chessboard_test.cpp)
target_link_libraries(chessboard_test ${catkin_LIBRARIES})
