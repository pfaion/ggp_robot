// boardpoint.cpp //
#include <ggp_robot/libs/boards/boardpoint.h>

BoardPoint::BoardPoint(float x, float y, float z)
  : x(x), y(y), z(z)
{}

// friend
std::ostream& operator<<(std::ostream& out, const BoardPoint& p) {
  return out << "BoardPoint(" << p.x << "|" << p.y << "|" << p.z << ")";
}

BoardPoint::operator cv::Point3f() { return cv::Point3f(x,y,z); }
BoardPoint::operator cv::Point2f() { return cv::Point2f(x,y); }
BoardPoint::operator Eigen::Vector3f() { return Eigen::Vector3f(x,y,z); }

