// chessstaterec1.cpp //
#include <ggp_robot/libs/stateRec/chessstaterec1.h>

#include <stdexcept>
#include <sstream>
#include <string>
#include <algorithm>
#include <vector>
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
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <ggp_robot/libs/boards/board.h>
#include <ggp_robot/libs/boards/chessboard1.h>
#include <ggp_robot/libs/tools/debug.h>
#include <ggp_robot/libs/tools/keylistener.h>

void testFun(pcl::visualization::PCLVisualizer& v) {
  v.setBackgroundColor(0.5, 0.5, 0.5);
}

ChessStateRec1::ChessStateRec1()
  : viewer("test")
{
  PRINT("[SREC] Recognizer initializing...");

  // viewer settings have to be set with a function...
  struct local{ static void settings(pcl::visualization::PCLVisualizer& v) {
    v.setBackgroundColor(0.5, 0.5, 0.5);
  }};
  viewer.runOnVisualizationThreadOnce(local::settings);
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

bool ChessStateRec1::start() {
  PRINT("[SREC] Starting...");

  KeyListener keyboard;
  keyboard.start();

  PRINT("[SREC] Init buffers.");
  std::map<std::string,int> states;
  // init buffers
  int buffer_size = 10;
  std::map<std::string,boost::circular_buffer<int> > state_buffers;
  typedef std::map<std::string,std::vector<cv::Point3f> >::iterator type;
  for(type it=board->regions.begin(); it != board->regions.end(); ++it) {
    state_buffers[it->first] = boost::circular_buffer<int>(buffer_size);
  }

  int showCloud = 0;
  while(ros::ok()) {
    PRINT("[SREC] Fetching cloud.");
    pcl::PCLPointCloud2::ConstPtr cloud = cam->getPclCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(*cloud, *pc);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());


    bool change = false;
    float maxEntropy = 0;

    std::map<std::string,int> newStates;

    int cloudId = 0;
    typedef std::map<std::string,std::vector<cv::Point3f> >::iterator type;
    for(type it=board->regions.begin(); it != board->regions.end(); ++it) {

      std::string name = it->first;
      if(name != "marker") cloudId++;

      pcl::CropBox<pcl::PointXYZ> crop;
      std::vector<cv::Point3f> reg = board->getRotatedRegion(name);
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
      // enlarge boundingbox
      float enlarge = 0;
      min.x -= (max.x-min.x)*enlarge;
      min.y -= (max.y-min.y)*enlarge;
      min.z += (max.z-min.z)*0.3;
      max.x += (max.x-min.x)*enlarge;
      max.y += (max.y-min.y)*enlarge;
      max.z += (max.z-min.z)*enlarge;
      crop.setMin(Eigen::Vector4f(min.x, min.y, min.z, 0));
      crop.setMax(Eigen::Vector4f(max.x, max.y, max.z, 0));
      // here we need the inverse...
      // apparently the filter transforms the cloud by the inverse transform and
      // then applies the cutoff filters
      crop.setTransform(board->transform.inverse());
      crop.setInputCloud(pc);
      crop.filter(*filtered);

      if(showCloud == cloudId){
        PRINT(cyan, "Showing Cloud: " << name);
        viewer.showCloud(filtered);
      }

      int numPoints = filtered->points.size();
      int state = numPoints > 100;
      state_buffers[name].push_back(state);
      if(state_buffers[name].full()) {
        // calculate statistical mode (central tendency for nominal scales)
        // as well as the entropy for the buffer
        // the entropy is a reasonable measure of variance for nominal data
        std::map<int,int> histogram;
        int size = 0;
        {
          typedef boost::circular_buffer<int>::iterator type;
          for(type it=state_buffers[name].begin(); it != state_buffers[name].end(); ++it) {
            histogram[*it]++;
            size++;
          }
        }
        int mode = histogram.begin()->first;
        float entropy = 0;
        {
          typedef std::map<int,int>::iterator type;
          for(type it=histogram.begin(); it != histogram.end(); ++it) {
            if(it->second > histogram[mode]) mode = it->first;
            entropy -= it->second/(double)size * std::log(it->second/(double)size);
          }
        }
        maxEntropy = std::max(maxEntropy, entropy);
        if(mode != states[name]) {
          change = true;
          newStates[name] = mode;
        } else {
          newStates[name] = states[name];
        }

      }
    }
    // TODO: good threshold for entropy????
    if(change) PRINT(red, "[SREC] Possible change. Entropy in sample-buffer: " << maxEntropy);
    if(maxEntropy < 0.1 && change) {
      PRINT(red, "[SREC] Change detected!");
      states = newStates;
      board->print(states);
    }

    char c = keyboard.getChar();
    if(!c) continue;
    if(c == 'r') {
      PRINT(blue, "[SREC][USER] Restart.");
      return false;
    }
    if(c == 'q') {
      PRINT(blue, "[SREC][USER] Quit.");
      return true;
    }
    if(c == 'p') {
      showCloud++;
      if(showCloud > 20) showCloud = 0;
    }
    if(c == 'm') {
      showCloud--;
      if(showCloud < 0) showCloud = 20;
    }
  }
  PRINT("[SREC] Stopping...");
}
