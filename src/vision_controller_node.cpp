// vision_controller_node.cpp //
#include <ggp_robot/vision_controller_node.h>

#include <iostream>
#include <string>
#include <stdexcept>
#include <unistd.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ggp_robot/libs/boards/board.h>
#include <ggp_robot/libs/boardRec/board_recognition.h>
#include <ggp_robot/libs/stateRec/state_recognition.h>
#include <ggp_robot/libs/tools/factories.h>
#include <ggp_robot/libs/tools/debug.h>
#include <ggp_robot/libs/tools/keylistener.h>
#include <ggp_robot/Calibrate.h>
#include <ggp_robot/GetState.h>


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
  private_nh_.param("boardrecognitionclass", boardRecognitionClass, std::string("chessboardrec2"));
  private_nh_.param("staterecognitionclass", stateRecognitionClass,
      std::string("chessstaterec3"));

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
  cam->setCloudTopic("/camera/depth_registered/points");


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

  brec->setBoard(board);
  brec->setCamera(cam);
  srec->setBoard(board);
  srec->setCamera(cam);

  ros::ServiceServer calib_service = nh.advertiseService("ggp_robot/calibrate", &VisionController::calibrate, this);
  ros::ServiceClient calib_client = nh.serviceClient<ggp_robot::Calibrate>("ggp_robot/calibrate");
  ros::ServiceServer state_service = nh.advertiseService("ggp_robot/getState", &VisionController::getState, this);
  ros::ServiceClient state_client = nh.serviceClient<ggp_robot::GetState>("ggp_robot/getState");

  // listen for service calls asynchroneously
  ros::AsyncSpinner spinner(0);
  spinner.start();

  KeyListener keyboard;
  keyboard.start();

  while(ros::ok()) {

    // continue visualization... can segfault with multithreading!!!!(wtf!?)
    if(mtx.try_lock()) {
      srec->viewer.spinOnce();
      cv::waitKey(1);
      mtx.unlock();
    }

    char c = keyboard.getChar();
    if(c == 'q') {
      PRINT(blue, "[VC][USER] Quit!");
      break;
    } else if(c == 'c') {
      PRINT(blue, "[VC][USER] Calibrate!");
      ggp_robot::Calibrate srv;
      if(!calib_client.call(srv)) {
        PRINT(red, "[VC] Error calling calibration service!");
      }
    } else if(c == 's') {
      PRINT(blue, "[VC][USER] Get state!");
      ggp_robot::GetState srv;
      if(!state_client.call(srv)) {
        PRINT(red, "[VC] Error calling state service!");
      }
    }

  }
  PRINT("[VC] Shutting down.");
  spinner.stop();
}

bool VisionController::calibrate(ggp_robot::Calibrate::Request &req,
                            ggp_robot::Calibrate::Response &res) {
  boost::lock_guard<boost::mutex> guard(mtx);
  PRINT("[VC] Calibrate service called.");
  brec->start();
  PRINT("[VC] Calibrate service finished.");
  return true;
}


bool VisionController::getState(ggp_robot::GetState::Request &req,
                            ggp_robot::GetState::Response &res) {
  boost::lock_guard<boost::mutex> guard(mtx);
  PRINT("[VC] State service called.");
  srec->start(req,res);
  PRINT("[VC] State service finished.");
  return true;
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
