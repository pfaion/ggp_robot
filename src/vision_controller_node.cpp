// vision_controller_node.cpp //
#include <iostream>
#include <string>
#include <stdexcept>
#include <unistd.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <ggp_robot/libs/boards/board.h>
#include <ggp_robot/libs/boardRec/board_recognition.h>
#include <ggp_robot/libs/tools/factories.h>
#include <ggp_robot/libs/tools/debug.h>
#include <ggp_robot/libs/tools/keylistener.h>

class VisionController {

  public:
    VisionController();
    void spin();

  private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    boost::shared_ptr<PlanarBoard> board;
    boost::shared_ptr<Camera> cam;
    boost::shared_ptr<BoardRecognition> brec;

};   

// Vision Controller Initialization
VisionController::VisionController()
: nh(),
  private_nh("~")
{
  PRINT("[VC] Initializing Vision Controller...");

  // variables that can be modified by launch file or command line
  std::string boardClass;
  std::string boardRecognitionClass;
  std::string cameraClass;
  
  // initialize node parameters from launch file or command line
  ros::NodeHandle private_nh_("~");
  private_nh_.param("boardclass", boardClass, std::string("chessboard1"));
  private_nh_.param("boardrecognitionclass", boardRecognitionClass, std::string("chessboardrec1"));
  private_nh_.param("cameraclass", cameraClass, std::string("xtion"));

  // try to load board
  board = Factory<PlanarBoard>::create(boardClass);
  if(!board) {
    PRINT(red, "[VC][ERROR] Specified class '" << boardClass << "'for board not found!");
    ros::shutdown();
    throw std::invalid_argument("");
  }
  PRINT(green, "[VC] Successfully created board of type '" << boardClass << "'");

  // try to load board recognition
  brec = Factory<BoardRecognition>::create(boardRecognitionClass);
  if(!brec) {
    PRINT(red, "[VC][ERROR] Specified class '" << boardRecognitionClass
      << "'for board recognition not found!");
    ros::shutdown();
    throw std::invalid_argument("");
  }
  PRINT(green, "[VC] Successfully created board recognition of type '"
    << boardRecognitionClass << "'");

  // try to load camera 
  cam = Factory<Camera>::create(cameraClass);
  if(!cam) {
    PRINT(red, "[VC][ERROR] Specified class '" << cameraClass
      << "'for camera not found!");
    ros::shutdown();
    throw std::invalid_argument("");
  }
  PRINT(green, "[VC] Successfully created camera of type '"
    << cameraClass << "'");
  cam->setImageTopic("/camera/rgb/image_raw");

  // success
  PRINT(green, "[VC] Initialization complete.");

}

void VisionController::spin() {
  PRINT("[VC] Starting Vision Controller...");

  // set up key listener for restarting or quitting the vision controller
  KeyListener x;
  x.start();

  // loop as long as the vision controller is running
  while(ros::ok()) {
    PRINT("[VC] Recognizing Board...");
    PRINT("[VC] Set up board recognition.");
    
    brec->setBoard(board);
    brec->setCamera(cam);
    brec->start();

    sleep(1);

    // detect state until reset or quit
    while(ros::ok()) {
      PRINT("[VC] Recognizing State...");

      sleep(1);

      // check for pressed keys
      char key = x.getChar();
      if(!key) continue;
      if(key == 'r') {
        PRINT(blue, "[VC][USER] Restart!");
        break;
      }
      if(key == 'q') {
        PRINT(blue, "[VC][USER] Quit!"); 
        ros::shutdown();
      }
    }
  }
  PRINT("[VC] Shutting down Vision Controller...");
}


  


int main(int argc, char** argv)
{
  PRINT("===================================");
  PRINT("[main] Starting.");

  // set up ROS
  ros::init(argc, argv, "vision_controller");
  
  // initialize vision controller
  VisionController vc;
  vc.spin();

  PRINT("[main] Finished.");

  return EXIT_SUCCESS;
}
