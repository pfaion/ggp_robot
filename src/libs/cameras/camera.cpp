// camera.cpp //
#include <ggp_robot/libs/cameras/camera.h>
#include <vector>

std::vector<double> Camera::getDistortionCoefficients() {
  // a common value is just (0, 0, 0, 0, 0)
  std::vector<double> distCoeffs(5, 0.0);
  return distCoeffs;
}


Camera::~Camera() {}
