#include <ggp_robot/libs/board_old.h>

namespace GgpRobot {
  
  BoardPointOld::BoardPointOld(double x, double y, double z, double fieldSize):
    x(x * fieldSize),
    y(y * fieldSize),
    z(z * fieldSize)
  {}

  BoardPointOld::operator cv::Point3f() {
    return cv::Point3f(x,y,z);
  }

  BoardPointOld::operator Eigen::Vector3d() {
    return Eigen::Vector3d(x,y,z);
  }

  std::ostream& operator<<(std::ostream& out, const BoardPointOld& p) {
    return out << "BoardPointOld(" << p.x << "|" << p.y << "|" << p.z << ")";
  }





  const double Board::FIELD_SIZE = 0.0945;

  std::vector<cv::Point3f> Board::getBoardCorners(){
    std::vector<cv::Point3f> objectPoints;
    objectPoints.push_back(p(3, 1));
    objectPoints.push_back(p(2, 1));
    objectPoints.push_back(p(1, 1));
    objectPoints.push_back(p(3, 2));
    objectPoints.push_back(p(2, 2));
    objectPoints.push_back(p(1, 2));
    objectPoints.push_back(p(3, 3));
    objectPoints.push_back(p(2, 3));
    objectPoints.push_back(p(1, 3));
    return objectPoints;
  }

  BoardPointOld Board::p(double x, double y, double z){
   return BoardPointOld(x, y, z, FIELD_SIZE); 
  }

}
