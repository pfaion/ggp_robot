// vision_controller.cpp //
#include <iostream>
#include <string>

#include <ros/ros.h>

#include <ggp_robot/libs/chessboard.h>
#include <ggp_robot/libs/factories.h>



class VisionController {

  public:
    VisionController(std::string boardClass) {
      PlanarBoard* board = PlanarBoardFactory::create(boardClass);
    }


};


int main(int argc, char** argv)
{
  ChessBoard bbb();
  PlanarBoard* board = PlanarBoardFactory::create("chessboard");
  std::cout << board << std::endl;
  std::cout << PlanarBoardFactory::static_creators().size() << std::endl;
  // set up ROS
  ros::init(argc, argv, "vision_controller");
  ros::NodeHandle nh;
//
//  // variables that can be modified by launch file or command line
//  std::string boardClass;
//  
//  // initialize node launch or command line parameters
//  nh.param("boardclass", boardClass, std::string("chessboard"));
//
//  std::cout << "------------------------------------------" << std::endl;
//  std::cout << "Started vision controller with parameters:" << std::endl;
//  std::cout << "    boardclass = " << boardClass            << std::endl;
//  std::cout << "------------------------------------------" << std::endl;

  VisionController vc(std::string("chessboard"));


  ros::spin();
  return EXIT_SUCCESS;
}
