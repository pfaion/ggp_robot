#ifndef __GGP_ROBOT_BOARD_OLD_H
#define __GGP_ROBOT_BOARD_OLD_H

#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>

namespace GgpRobot {

  class BoardPointOld {

    public:

      const double x;
      const double y;
      const double z;

      BoardPointOld(double x, double y, double z=0.0, double fieldSize=1.0);

      operator cv::Point3f();
      operator Eigen::Vector3d();

      friend std::ostream& operator<<(std::ostream&, const BoardPointOld&);

  };

  class Board {

    public:

      static const double FIELD_SIZE;

      std::vector<cv::Point3f> getBoardCorners();
      BoardPointOld p(double x, double y, double z=0.0);

  };


}


#endif

