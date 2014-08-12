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

void testFun(pcl::visualization::PCLVisualizer& v) {
  v.setBackgroundColor(0.5, 0.5, 0.5);
}

ChessStateRec1::ChessStateRec1()
: viewer("test")
{
  PRINT("[SREC] Recognizer initializing...");
  viewer.runOnVisualizationThreadOnce(testFun);
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
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>());



  pcl::CropBox<pcl::PointXYZRGB> crop;
  std::vector<cv::Point3f> reg = board->getRotatedRegion("a1");
  cv::Point3f min = reg[0];
  cv::Point3f max = reg[0];
  typedef std::vector<cv::Point3f>::iterator type;
  for(type it=reg.begin(); it != reg.end(); ++it) {
    min.x = std::min(min.x, (*it).x);
    min.y = std::min(min.y, (*it).y);
    min.z = std::min(min.z, (*it).z);
    max.x = std::max(max.x, (*it).x);
    max.y = std::max(max.y, (*it).y);
    max.z = std::max(max.z, (*it).z);
  }
  crop.setMin(Eigen::Vector4f(min.x, min.y, min.z, 0));
  crop.setMax(Eigen::Vector4f(max.x, max.y, max.z, 0));
  // here we need the inverse...
  // apparently the filter transforms the cloud by the inverse transform and
  // then applies the cutoff filters
  crop.setTransform(board->transform.inverse());
  crop.setInputCloud(pc);
  crop.filter(*filtered);
  PRINT(filtered->points.size());


  viewer.showCloud(filtered);

  PRINT("[SREC] Stopping...");
}
