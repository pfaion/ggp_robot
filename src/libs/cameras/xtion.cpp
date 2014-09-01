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

  cv::Matx33d m2 = cv::Matx33d::zeros();
  m2(0,0) = 540.690689;
  m2(0,2) = 316.274259;
  m2(1,1) = 539.383723;
  m2(1,2) = 237.013709;
  m2(2,2) = 1.0;

  cv::Matx33d m3 = cv::Matx33d::zeros();
  m3(0,0) = 578.802309;
  m3(0,2) = 308.979252;
  m3(1,1) = 576.757895;
  m3(1,2) = 245.985417;
  m3(2,2) = 1.0;
  return m;
}

std::vector<double> Xtion::getDistortionCoefficients() {
  std::vector<double> v;
  v.push_back(0.0);
  v.push_back(0.0);
  v.push_back(0.0);
  v.push_back(0.0);
  v.push_back(0.0);

  std::vector<double> v2;
  v2.push_back(0.044658);
  v2.push_back(-0.11184);
  v2.push_back(-0.000838);
  v2.push_back(0.001826);
  v2.push_back(0.0);

  std::vector<double> v3;
  v3.push_back(-0.023604);
  v3.push_back(0.034499);
  v3.push_back(-0.001167);
  v3.push_back(-0.004645);
  v3.push_back(0.0);
  return v;
}

