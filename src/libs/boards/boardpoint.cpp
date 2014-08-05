// boardpoint.cpp //
#include <ggp_robot/libs/boards/boardpoint.h>

BoardPoint::BoardPoint(double x, double y, double z)
  : x(x), y(y), z(z)
{}

// friend
std::ostream& operator<<(std::ostream& out, const BoardPoint& p) {
  return out << "BoardPoint(" << p.x << "|" << p.y << "|" << p.z << ")";
}

BoardPoint::operator cv::Point3d() { return cv::Point3d(x,y,z); }
BoardPoint::operator cv::Point2d() { return cv::Point2d(x,y); }
BoardPoint::operator Eigen::Vector3d() { return Eigen::Vector3d(x,y,z); }

