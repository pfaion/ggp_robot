#ifndef __GGP_ROBOT_CHESSSTATEREC1_H
#define __GGP_ROBOT_CHESSSTATEREC1_H

#include <boost/shared_ptr.hpp>
#include <sensor_msgs/image_encodings.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ggp_robot/libs/stateRec/state_recognition.h>
#include <ggp_robot/libs/boards/chessboard1.h>
#include <ggp_robot/libs/cameras/camera.h>

class ChessStateRec1 : public StateRecognition {
  public:
    ChessStateRec1();
    virtual void setBoard(boost::shared_ptr<PlanarBoard>& bp);
    virtual void setCamera(boost::shared_ptr<Camera>& cp);
    virtual void start(); 

  private:
    // this recognizer is only supposed to work for board type chessboard1
    boost::shared_ptr<ChessBoard1>board;
    boost::shared_ptr<Camera> cam;

    pcl::visualization::CloudViewer viewer;

};

#endif
