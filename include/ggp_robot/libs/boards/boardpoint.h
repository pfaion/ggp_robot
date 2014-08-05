#ifndef __GGP_ROBOT_BOARDPOINT_H
#define __GGP_ROBOT_BOARDPOINT_H

// includes
#include <iostream>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>

//==============================================================================

struct BoardPoint{
  public:
    double x;
    double y;
    double z;

    BoardPoint(double x, double y, double z=0.0);

    friend std::ostream& operator<<(std::ostream& out, const BoardPoint& p);

    operator cv::Point3d();
    operator cv::Point2d();
    operator Eigen::Vector3d();
};




#endif

