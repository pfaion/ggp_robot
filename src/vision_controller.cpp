// vision_controller.cpp //
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ggp_robot/libs/chessboard.h>
#include <ggp_robot/libs/factories.h>



class VisionController {



};


int main(int argc, char** argv)
{
  // set up ROS
  ros::init(argc, argv, "vision_controller");
  ros::NodeHandle nh;

  // variables that can be modified by launch file or command line
  std::string boardClass;
  
  // initialize node launch or command line parameters
  ros::NodeHandle private_nh_("~");
  private_nh_.param("boardclass", boardClass, std::string("chessboard"));

  std::cout << "------------------------------------------" << std::endl;
  std::cout << "Started vision controller with parameters:" << std::endl;
  std::cout << "    boardclass = " << boardClass            << std::endl;
  std::cout << "------------------------------------------" << std::endl;

  VisionController vc;

  PlanarBoard* board = PlanarBoardFactory::create(boardClass);
  if(!board) {
    std::cerr << "[ERROR] Specified class for board not found!" << std::endl
      << "Please include class in 'include/ggp_robot/libs/factory_includes.h'"
      << std::endl;
    ros::shutdown();
    return EXIT_FAILURE;
  }


  ChessBoard* cboard;
  cboard = dynamic_cast<ChessBoard*>(board);
  std::cout << cboard << std::endl;


  return EXIT_SUCCESS;
}
