cmake_minimum_required(VERSION 2.8.3)
project(ggp_robot)


# Include file FindOpenCV2 for OpenCV2 support
set(CMAKE_MODULE_PATH 
  ${CMAKE_MODULE_PATH}
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules
)


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


catkin_package(
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


# --------------------------------------------------- #
# specify header file locations
# --------------------------------------------------- #
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)


# --------------------------------------------------- #
# my own libraries
# --------------------------------------------------- #
add_library(ggp_robot_libs

  # library source files
  src/libs/boards/board.cpp
  src/libs/boards/boardpoint.cpp
  src/libs/boards/chessboard1.cpp
  src/libs/cameras/camera.cpp
  src/libs/cameras/xtion.cpp
  src/libs/boardRec/board_recognition.cpp
  src/libs/boardRec/chessboardrec1.cpp
  src/libs/stateRec/state_recognition.cpp
  src/libs/stateRec/chessstaterec1.cpp

  # header-only libraries and templates 
  include/ggp_robot/libs/tools/factories.h
  include/ggp_robot/libs/tools/factoryclass.h
  include/ggp_robot/libs/tools/debug.h
  include/ggp_robot/libs/tools/keylistener.h
)
target_link_libraries(ggp_robot_libs ${catkin_LIBRARIES})


# --------------------------------------------------- #
# executables (ros nodes)
# --------------------------------------------------- #
#add_executable(pclview_rgbd src/pclview_rgbd.cpp)
#target_link_libraries(pclview_rgbd ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkFiltering.so)
#
#add_executable(cloudview_rgbd src/cloudview_rgbd.cpp)
#target_link_libraries(cloudview_rgbd ${catkin_LIBRARIES})
#
#add_executable(process_cloud src/process_cloud.cpp)
#target_link_libraries(process_cloud ${catkin_LIBRARIES})
#
#add_executable(cluster_extraction src/cluster_extraction.cpp)
#target_link_libraries(cluster_extraction ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#
#add_executable(orb_test src/orb_test.cpp)
#target_link_libraries(orb_test ${catkin_LIBRARIES})
#
#add_executable(orb_test2 src/orb_test2.cpp)
#target_link_libraries(orb_test2 ${catkin_LIBRARIES})
#
#add_executable(feature_test src/feature_test.cpp)
#target_link_libraries(feature_test ${catkin_LIBRARIES})
#
#add_executable(chessboard_test src/chessboard_test.cpp)
#target_link_libraries(chessboard_test ${catkin_LIBRARIES})
#
#add_executable(board_recognition src/board_recognition.cpp)
#target_link_libraries(board_recognition ${catkin_LIBRARIES} ggp_robot_libs)

add_executable(vision_controller_node src/vision_controller_node.cpp)
target_link_libraries(vision_controller_node ${catkin_LIBRARIES} ggp_robot_libs)

