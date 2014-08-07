#ifndef __GGP_ROBOT_BOARD_H
#define __GGP_ROBOT_BOARD_H

#include <map>
#include <vector>
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>


  struct BoardPoint{
    public:
      double x;
      double y;
      double z;

      BoardPoint(double x, double y, double z) : x(x), y(y), z(z) {}

      friend std::ostream& operator<<(std::ostream& out, const BoardPoint& p) {
        return out << "BoardPoint(" << p.x << "|" << p.y << "|" << p.z << ")";
      }

      operator cv::Point3d() { return cv::Point3d(x,y,z); }
      operator cv::Point2d() { return cv::Point2d(x,y); }
      operator Eigen::Vector3d() { return Eigen::Vector3d(x,y,z); }
  };

  typedef std::map<std::string,std::vector<BoardPoint> > RegionLayout;

  class BoardRecognition {
    public:
      virtual ~BoardRecognition(){}

  };

  class StateRecognition {
    public:
      virtual ~StateRecognition(){}

  };

  class PlanarBoard {
    
    public:
      RegionLayout regions;

      PlanarBoard() : regions(){}
      virtual ~PlanarBoard(){}
      
      BoardPoint p(double x, double y, double z=0.0) {
        return BoardPoint(x,y,z);
      }

 
  };






#endif

