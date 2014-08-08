#ifndef __GGP_ROBOT_CHESSBOARD1_H
#define __GGP_ROBOT_CHESSBOARD1_H

// forward declare
class BoardPoint;

// includes
#include <vector>
#include <ggp_robot/libs/boards/board.h>


class ChessBoard1 : public PlanarBoard {

  public:
    typedef std::map<std::string,std::vector<cv::Point3f> > RegionLayout;
    const float FIELD_SIZE;

    std::vector<cv::Point3f> corners;

    cv::Point3f center;

    ChessBoard1();

    RegionLayout getRotatedLayout(float angle);
    std::vector<cv::Point3f> getRotatedRegion(std::string name, float angle);
    virtual BoardPoint p(float x, float y, float z=0.0);

    float markerPerformanceIndicator(cv::Mat roi, cv::Mat mask);

};


#endif
