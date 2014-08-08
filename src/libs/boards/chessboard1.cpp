// chessboard1.cpp //
#include <ggp_robot/libs/boards/chessboard1.h>
#include <vector>
#include <string>
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/assign/std/vector.hpp>
#include <ggp_robot/libs/boards/boardpoint.h>


ChessBoard1::ChessBoard1()
  : FIELD_SIZE(0.0945)
{

  // initialize region descriptions for regular fields
  std::vector<std::string> desc;
  { 
    using namespace boost::assign;
    desc += "a1", "b1", "c1", "d1", "x1"; 
    desc += "a2", "b2", "c2", "d2", "x2"; 
    desc += "a3", "b3", "c3", "d3", "x3"; 
    desc += "a4", "b4", "c4", "d4", "x4"; 
  }
  for(int x = 0; x < 5; x++) {
    for(int y = 0; y < 4; y++) {
      std::string d = desc[x + 4*y];
      this->regions[d] = std::vector<cv::Point3f>();
      this->regions[d].push_back(p(x,y));
      this->regions[d].push_back(p(x+1,y));
      this->regions[d].push_back(p(x+1,y+1));
      this->regions[d].push_back(p(x,y+1));
    }
  }

  // initialize marker region description
  this->regions["marker"].push_back(p(4,0));
  this->regions["marker"].push_back(p(5,0));
  this->regions["marker"].push_back(p(5,1));
  this->regions["marker"].push_back(p(4,1));


  // initialize corners for detection
  this->corners.push_back(p(3,1));
  this->corners.push_back(p(2,1));
  this->corners.push_back(p(1,1));
  this->corners.push_back(p(3,2));
  this->corners.push_back(p(2,2));
  this->corners.push_back(p(1,2));
  this->corners.push_back(p(3,3));
  this->corners.push_back(p(2,3));
  this->corners.push_back(p(1,3));

  // initialize rotation center
  this->center = p(2,2);
}

BoardPoint ChessBoard1::p(float x, float y, float z) {
  return BoardPoint(x * FIELD_SIZE, y * FIELD_SIZE, z * FIELD_SIZE);
}

ChessBoard1::RegionLayout ChessBoard1::getRotatedLayout(float angle) {
  RegionLayout rotLayout;
  typedef RegionLayout::iterator type;
  for(type it = regions.begin(); it != regions.end(); ++it) {
    rotLayout[it->first] = getRotatedRegion(it->first, angle);
  }
  return rotLayout;
}

std::vector<cv::Point3f> ChessBoard1::getRotatedRegion(std::string name, float angle) {
  std::vector<cv::Point3f> v = regions[name];
  std::vector<cv::Point3f> rotRegion;
  typedef std::vector<cv::Point3f>::iterator type;
  for(type it = v.begin(); it != v.end(); ++it) {
    cv::Point3f p = (*it);
    float rotX = cos(angle) * (p.x - center.x) - sin(angle) * (p.y - center.y) + center.x;
    float rotY = sin(angle) * (p.x - center.x) + cos(angle) * (p.y - center.y) + center.y;
    rotRegion.push_back(cv::Point3f(rotX, rotY, p.z));
  }
  return rotRegion;
}

float ChessBoard1::markerPerformanceIndicator(cv::Mat roi, cv::Mat mask) {
  cv::Mat avgMatBGR = cv::Mat(1,1,roi.type());
  avgMatBGR = cv::mean(roi, mask); 

  cv::Mat avgMatHSV;
  avgMatBGR.copyTo(avgMatHSV);
  cv::cvtColor(avgMatBGR, avgMatHSV, CV_BGR2HSV);

  // quick and dirty 1x1-matrix to scalar
  cv::Scalar avgHSV = cv::mean(avgMatHSV);
  
  // returns 1 for hue-value close to red and 0 for far away
  float reddishness = 1 - (std::min(avgHSV[0], 180 - avgHSV[0])/90);

  // Performance indicator is high for reddish colors with high saturation and
  // high value. This seems to be extremely stable.
  return reddishness * (avgHSV[1]/255) * (avgHSV[2]/255);
} 


