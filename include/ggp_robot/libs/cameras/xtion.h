#ifndef __GGP_ROBOT_XTION_H
#define __GGP_ROBOT_XTION_H

#include <opencv2/core/core.hpp>
#include <ggp_robot/libs/cameras/camera.h>

class Xtion : public Camera {
  public:
    virtual cv::Matx33d getCameraMatrix();  
    virtual std::vector<double> getDistortionCoefficients();

};

#endif
