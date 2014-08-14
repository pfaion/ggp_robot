// vision_controller_node.cpp //
#include <iostream>
#include <string>
#include <stdexcept>
#include <unistd.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <ggp_robot/libs/boards/board.h>
#include <ggp_robot/libs/boardRec/board_recognition.h>
#include <ggp_robot/libs/stateRec/state_recognition.h>
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
    boost::shared_ptr<StateRecognition> srec;

};   

// Vision Controller Initialization
VisionController::VisionController()
: nh(),
  private_nh("~")
{
  PRINT("[VC] Initializing Vision Controller...");

  // variables that can be modified by launch file or command line
  std::string boardClass;
  std::string cameraClass;
  std::string boardRecognitionClass;
  std::string stateRecognitionClass;
  
  // initialize node parameters from launch file or command line
  ros::NodeHandle private_nh_("~");
  private_nh_.param("boardclass", boardClass, std::string("chessboard1"));
  private_nh_.param("cameraclass", cameraClass, std::string("xtion"));
  private_nh_.param("boardrecognitionclass", boardRecognitionClass, std::string("chessboardrec1"));
  private_nh_.param("staterecognitionclass", stateRecognitionClass, std::string("chessstaterec1"));

  // try to load board
  board = Factory<PlanarBoard>::create(boardClass);
  if(!board) {
    PRINT(red, "[VC][ERROR] Specified class '" << boardClass << "'for board not found!");
    ros::shutdown();
    throw std::invalid_argument("");
  }
  PRINT(green, "[VC] Successfully created board of type '" << boardClass << "'");

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
  cam->setCloudTopic("/camera/depth/points");


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


  // try to load state recognition
  srec = Factory<StateRecognition>::create(stateRecognitionClass);
  if(!srec) {
    PRINT(red, "[VC][ERROR] Specified class '" << stateRecognitionClass
      << "'for state recognition not found!");
    ros::shutdown();
    throw std::invalid_argument("");
  }
  PRINT(green, "[VC] Successfully created state recognition of type '"
    << stateRecognitionClass << "'");

  // success
  PRINT(green, "[VC] Initialization complete.");

}

void VisionController::spin() {
  PRINT("[VC] Starting Vision Controller...");

  // loop as long as the vision controller is running
  while(ros::ok()) {
    PRINT("[VC] Recognizing Board...");
    PRINT("[VC] Set up board recognition.");
    
    brec->setBoard(board);
    brec->setCamera(cam);
    brec->start();



    PRINT("[VC] Recognizing State...");
    cam->listenToImageStream(false);
    cam->listenToCloudStream(true);
    srec->setBoard(board);
    srec->setCamera(cam);

    bool end = srec->start();
    if(end) break;
  }
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
