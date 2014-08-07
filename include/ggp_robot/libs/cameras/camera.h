#ifndef __GGP_ROBOT_CAMERA_H
#define __GGP_ROBOT_CAMERA_H

#include <vector>
#include <opencv2/core/core.hpp>

class Camera {
  public:
    // pure virtual, it makes no sense to define standart values
    virtual cv::Matx33d getCameraMatrix() = 0;

    virtual std::vector<double> getDistortionCoefficients();

    virtual ~Camera();
    

};

#endif
