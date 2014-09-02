#ifndef __GGP_ROBOT_CHESSSTATEREC3_H
#define __GGP_ROBOT_CHESSSTATEREC3_H

#include <boost/shared_ptr.hpp>
#include <sensor_msgs/image_encodings.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ggp_robot/libs/stateRec/state_recognition.h>
#include <ggp_robot/libs/boards/chessboard1.h>
#include <ggp_robot/libs/cameras/camera.h>

class ChessStateRec3 : public StateRecognition {
  public:
    ChessStateRec3();
    virtual void setBoard(boost::shared_ptr<PlanarBoard>& bp);
    virtual void setCamera(boost::shared_ptr<Camera>& cp);
    virtual bool start(); 

    typedef pcl::PointXYZRGB PointType;

  private:
    // this recognizer is only supposed to work for board type chessboard1
    boost::shared_ptr<ChessBoard1>board;
    boost::shared_ptr<Camera> cam;

    pcl::visualization::PCLVisualizer viewer;

    bool pointInPolygon(Eigen::Vector3f pt, std::vector<cv::Point3f> poly);
    int ptTest(float xa, float ya, float xb, float yb, float xc, float yc);

    struct PlaneTools {
      float nx, ny, nz, d;
      PlaneTools(){};
      PlaneTools(float nx, float ny, float nz, float d);
      Eigen::Vector3f project(Eigen::Vector3f pt);
      float distance(Eigen::Vector3f pt);
    };
    struct Cluster {
      Eigen::Vector3f avgpt;
      Eigen::Vector3f base;
      Eigen::Vector3f newBase;
      pcl::PointIndices indices;
      PlaneTools plane;
      pcl::PointCloud<PointType>::Ptr refcloud;
      void analyse();
    };

};

#endif
