#ifndef __GGP_ROBOT_BOARDPOINT_H
#define __GGP_ROBOT_BOARDPOINT_H

// includes
#include <iostream>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>

//==============================================================================

struct BoardPoint{
  public:
    float x;
    float y;
    float z;

    BoardPoint(float x, float y, float z=0.0);

    friend std::ostream& operator<<(std::ostream& out, const BoardPoint& p);

    operator cv::Point3f();
    operator cv::Point2f();
    operator Eigen::Vector3d();
};




#endif

