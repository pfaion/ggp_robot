#ifndef __GGP_ROBOT_CORRGROUP_H
#define __GGP_ROBOT_CORRGROUP_H

#include <boost/shared_ptr.hpp>
#include <sensor_msgs/image_encodings.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ggp_robot/libs/stateRec/state_recognition.h>
#include <ggp_robot/libs/boards/chessboard1.h>
#include <ggp_robot/libs/cameras/camera.h>

class CorrGroup : public StateRecognition {
  public:
    CorrGroup();
    virtual void setBoard(boost::shared_ptr<PlanarBoard>& bp);
    virtual void setCamera(boost::shared_ptr<Camera>& cp);
    virtual bool start(); 

  private:
    // this recognizer is only supposed to work for board type chessboard1
    boost::shared_ptr<ChessBoard1>board;
    boost::shared_ptr<Camera> cam;

    pcl::visualization::PCLVisualizer viewer;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model;

};

#endif
