// chessboard1.cpp //
#include <ggp_robot/libs/boards/chessboard1.h>
#include <vector>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/assign/std/vector.hpp>
#include <ggp_robot/libs/boards/boardpoint.h>
#include <ggp_robot/libs/tools/debug.h>


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
  for(int y = 0; y < 4; y++) {
    for(int x = 0; x < 5; x++) {
      std::string d = desc[x + 5*y];
      this->regions[d] = std::vector<cv::Point3f>();
      this->regions[d].push_back(p(x,y));
      this->regions[d].push_back(p(x+1,y));
      this->regions[d].push_back(p(x+1,y+1));
      this->regions[d].push_back(p(x,y+1));
      this->regions[d].push_back(p(x,y,2));
    }
  }

  // initialize marker region description
  this->regions["marker"] = std::vector<cv::Point3f>();
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

  // boundingBox
  this->boundingBox = std::vector<cv::Point3f>();
  this->boundingBox.push_back(p(0,0,-0.5));
  this->boundingBox.push_back(p(5,0,-0.5));
  this->boundingBox.push_back(p(5,4,-0.5));
  this->boundingBox.push_back(p(0,4,-0.5));
  this->boundingBox.push_back(p(0,0,3));
  this->boundingBox.push_back(p(5,0,3));
  this->boundingBox.push_back(p(5,4,3));
  this->boundingBox.push_back(p(0,4,3));

  // initialize rotation center
  this->center = p(2,2);

  // set initial rotation to 0
  this->angle = 0.0;

  // set initial transformation to identity
  this->transform.setIdentity();
}

BoardPoint ChessBoard1::p(float x, float y, float z) {
  return BoardPoint(x * FIELD_SIZE, y * FIELD_SIZE, z * FIELD_SIZE);
}

std::vector<cv::Point3f> ChessBoard1::getRotatedBoundingBox() {
  return rotatePoints(boundingBox);
}

std::vector<cv::Point3f> ChessBoard1::rotatePoints(std::vector<cv::Point3f> v) {
  return rotatePoints(v, this->angle);
}

std::vector<cv::Point3f> ChessBoard1::rotatePoints(std::vector<cv::Point3f> v, float angle) {
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

ChessBoard1::RegionLayout ChessBoard1::getRotatedLayout(float angle) {
  RegionLayout rotLayout;
  typedef RegionLayout::iterator type;
  for(type it = regions.begin(); it != regions.end(); ++it) {
    rotLayout[it->first] = getRotatedRegion(it->first, angle);
  }
  return rotLayout;
}

ChessBoard1::RegionLayout ChessBoard1::getRotatedLayout() {
  return getRotatedLayout(this->angle);
}

std::vector<cv::Point3f> ChessBoard1::getRotatedRegion(std::string name, float angle) {
  std::vector<cv::Point3f> v = regions[name];
  return rotatePoints(v, angle);
}

std::vector<cv::Point3f> ChessBoard1::getRotatedRegion(std::string name) {
  return getRotatedRegion(name, this->angle);
}

ChessBoard1::RegionLayout ChessBoard1::getRotatedTransformedLayout() {
  RegionLayout newLayout;
  typedef RegionLayout::iterator type;
  for(type it = regions.begin(); it != regions.end(); ++it) {
    newLayout[it->first] = getRotatedTransformedRegion(it->first);
  }
}

std::vector<cv::Point3f> ChessBoard1::getRotatedTransformedRegion(std::string name) {
  std::vector<cv::Point3f> v = getRotatedRegion(name);
  std::vector<cv::Point3f> newRegion;
  typedef std::vector<cv::Point3f>::iterator type;
  for(type it = v.begin(); it != v.end(); ++it) {
    Eigen::Vector3f p((*it).x, (*it).y, (*it).z);
    Eigen::Vector3f rotP = transform * p;
    newRegion.push_back(cv::Point3f(rotP[0], rotP[1], rotP[2]));
  }
  return newRegion;
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


void ChessBoard1::print(std::map<std::string,int> s) {
  struct {
    std::string operator() (int state) {
      switch(state) {
        case 0: return " ";
        case 1: return "X";
      }
    }
  } r;
  std::cout
    << "   a b c d x " << std::endl
    << "  +---------+" << std::endl
    << "4 |" << r(s["a4"]) << "|" << r(s["b4"]) << "|" << r(s["c4"]) << "|" << r(s["d4"]) << "|" << r(s["x4"]) << "|" << std::endl
    << "3 |" << r(s["a3"]) << "|" << r(s["b3"]) << "|" << r(s["c3"]) << "|" << r(s["d3"]) << "|" << r(s["x3"]) << "|" << std::endl
    << "2 |" << r(s["a2"]) << "|" << r(s["b2"]) << "|" << r(s["c2"]) << "|" << r(s["d2"]) << "|" << r(s["x2"]) << "|" << std::endl
    << "1 |" << r(s["a1"]) << "|" << r(s["b1"]) << "|" << r(s["c1"]) << "|" << r(s["d1"]) << "|" << r(s["x1"]) << "|" << std::endl
    << "  +---------+" << std::endl
    ;
}

