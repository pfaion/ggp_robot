// vision_controller_node.cpp //
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ggp_robot/libs/boards/board.h>
#include <ggp_robot/libs/boardRec/board_recognition.h>
#include <ggp_robot/libs/factories.h>
#include <ggp_robot/libs/vision_controller.h>
#include <ggp_robot/libs/debug.h>


int main(int argc, char** argv)
{
  PRINT("===================================");
  PRINT("starting ROS node vision_controller");
  // set up ROS
  ros::init(argc, argv, "vision_controller");
  ros::NodeHandle nh;

  // variables that can be modified by launch file or command line
  std::string boardClass;
  std::string boardRecognitionClass;
  std::string cameraClass;
  
  // initialize node launch or command line parameters
  ros::NodeHandle private_nh_("~");
  private_nh_.param("boardclass", boardClass, std::string("chessboard1"));
  private_nh_.param("boardrecognitionclass", boardRecognitionClass, std::string("chessboardrec1"));
  private_nh_.param("cameraclass", cameraClass, std::string("xtion"));

  // try to load board
  PlanarBoard* board = Factory<PlanarBoard>::create(boardClass);
  if(!board) {
    PRINT(red, "[ERROR] Specified class '" << boardClass << "'for board not found!");
    ros::shutdown();
    return EXIT_FAILURE;
  }
  PRINT(green, "[main] Successfully created board of type '" << boardClass << "'");

  // try to load board recognition
  BoardRecognition* brec = Factory<BoardRecognition>::create(boardRecognitionClass);
  if(!brec) {
    PRINT(red, "[ERROR] Specified class '" << boardRecognitionClass
      << "'for board recognition not found!");
    ros::shutdown();
    return EXIT_FAILURE;
  }
  PRINT(green, "[main] Successfully created board recognition of type '"
    << boardRecognitionClass << "'");

  // try to load camera 
  Camera* cam = Factory<Camera>::create(cameraClass);
  if(!cam) {
    PRINT(red, "[ERROR] Specified class '" << cameraClass
      << "'for camera not found!");
    ros::shutdown();
    return EXIT_FAILURE;
  }
  PRINT(green, "[main] Successfully created camera of type '"
    << cameraClass << "'");


  // initialize vision controller
  VisionController vc;
  PRINT(blue, "[main] Started vision controller");

  return EXIT_SUCCESS;
}
