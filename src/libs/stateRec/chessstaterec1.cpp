// chessstaterec1.cpp //
#include <ggp_robot/libs/stateRec/chessstaterec1.h>

#include <stdexcept>
#include <sstream>
#include <string>
#include <unistd.h>
#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <visualization_msgs/Marker.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <ggp_robot/libs/boards/board.h>
#include <ggp_robot/libs/boards/chessboard1.h>
#include <ggp_robot/libs/tools/debug.h>



ChessStateRec1::ChessStateRec1()
: viewer("test")
{
  PRINT("[SREC] Recognizer initializing...");
}

void ChessStateRec1::setBoard(boost::shared_ptr<PlanarBoard>& bp) {
  PRINT("[SREC] Setting board.");
  board = boost::dynamic_pointer_cast<ChessBoard1>(bp);
  if(!board) {
    PRINT(red, "[SREC] Wrong type of chessboard for this recognizer!");
    throw std::domain_error("");
  }
}

void ChessStateRec1::setCamera(boost::shared_ptr<Camera>& cp) {
  PRINT("[SREC] Setting camera.");
  cam = cp;
}

void ChessStateRec1::start() {
  PRINT("[SREC] Starting...");
  
  pcl::PCLPointCloud2::ConstPtr cloud = cam->getPclCloud();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromPCLPointCloud2(*cloud, *pc);
  
  
  Eigen::Matrix3f tmp = board->getHullMatrix("a1");
  PRINT(tmp);
  cv::Point3f cvNP = board->getRotatedTransformedRegion("a1")[0];
  Eigen::Vector3f eigNP(cvNP.x, cvNP.y, cvNP.z);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
  int count = 0;
  int allcount = 0;
  for(size_t i=0; i < pc->points.size(); ++i) {
    allcount++;
    Eigen::Vector3f pt(pc->points[i].x, pc->points[i].y, pc->points[i].z); 
    Eigen::Vector3f v = tmp.fullPivLu().solve(pt - eigNP);
    if(0 <= v[0] && v[0] <= 1 && 0 <= v[1] && v[1] <= 1 && 0 <= v[2] && v[2] <= 1) {
      pcl::PointXYZRGB copy(pc->points[i]);
      filtered->push_back(copy);
      count++;
    }
  }
  PRINT(count << "/" << allcount);

  viewer.showCloud(filtered);

  PRINT("[SREC] Stopping...");
}
