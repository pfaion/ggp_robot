#ifndef _GGP_ROBOT_VISION_CONTROLLER_NODE_H_
#define _GGP_ROBOT_VISION_CONTROLLER_NODE_H_

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <ggp_robot/libs/boards/board.h>
#include <ggp_robot/libs/cameras/camera.h>
#include <ggp_robot/libs/boardRec/board_recognition.h>
#include <ggp_robot/libs/stateRec/state_recognition.h>
#include <ggp_robot/Calibrate.h>
#include <ggp_robot/GetState.h>

class VisionController {

  public:
    VisionController();
    void spin();
    bool calibrate(ggp_robot::Calibrate::Request &req,
                  ggp_robot::Calibrate::Response &res);
    bool getState(ggp_robot::GetState::Request &req,
                  ggp_robot::GetState::Response &res);

  private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    boost::shared_ptr<PlanarBoard> board;
    boost::shared_ptr<Camera> cam;
    boost::shared_ptr<BoardRecognition> brec;
    boost::shared_ptr<StateRecognition> srec;
    boost::mutex mtx;

};


#endif
