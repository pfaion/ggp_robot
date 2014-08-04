cmake_minimum_required(VERSION 2.8.3)
project(ggp_robot)





set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)





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





# (pfaion) apparently I do not need this...
#find_package(OpenCV2 REQUIRED)
#find_package(OpenCV REQUIRED)





catkin_package(
#      INCLUDE_DIRS include
#      LIBRARIES ggp_robot_libs
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
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})




# --------------------------------------------------- #
# my own libraries
# --------------------------------------------------- #
add_library(ggp_robot_libs
  src/libs/board.cpp
  src/libs/board_old.cpp
  src/libs/chessboard.cpp
  # Factories are templates and therefore header-only.
  include/ggp_robot/libs/factories.h
)
target_link_libraries(ggp_robot_libs ${catkin_LIBRARIES})




# --------------------------------------------------- #
# executables (ros nodes)
# --------------------------------------------------- #

add_executable(pclview_rgbd src/pclview_rgbd.cpp)
target_link_libraries(pclview_rgbd ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkFiltering.so)

add_executable(cloudview_rgbd src/cloudview_rgbd.cpp)
target_link_libraries(cloudview_rgbd ${catkin_LIBRARIES})

add_executable(process_cloud src/process_cloud.cpp)
target_link_libraries(process_cloud ${catkin_LIBRARIES})

add_executable(cluster_extraction src/cluster_extraction.cpp)
target_link_libraries(cluster_extraction ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(orb_test src/orb_test.cpp)
target_link_libraries(orb_test ${catkin_LIBRARIES})

add_executable(orb_test2 src/orb_test2.cpp)
target_link_libraries(orb_test2 ${catkin_LIBRARIES})

add_executable(feature_test src/feature_test.cpp)
target_link_libraries(feature_test ${catkin_LIBRARIES})

add_executable(chessboard_test src/chessboard_test.cpp)
target_link_libraries(chessboard_test ${catkin_LIBRARIES})

add_executable(board_recognition src/board_recognition.cpp)
target_link_libraries(board_recognition ${catkin_LIBRARIES} ggp_robot_libs)


