// xtion.cpp //
#include <ggp_robot/libs/cameras/xtion.h>
#include <opencv2/core/core.hpp>

cv::Matx33d Xtion::getCameraMatrix(){
  cv::Matx33d m = cv::Matx33d::zeros();
  m(0,0) = 570.3422241210938;
  m(0,2) = 319.5;
  m(1,1) = 570.3422241210938;
  m(1,2) = 239.5;
  m(2,2) = 1.0;
  return m;
}

