#ifndef __GGP_ROBOT_CHESSBOARD1_H
#define __GGP_ROBOT_CHESSBOARD1_H

// forward declare
class BoardPoint;

// includes
#include <vector>
#include <Eigen/Dense>
#include <ggp_robot/libs/boards/board.h>


class ChessBoard1 : public PlanarBoard {

  public:
    typedef std::map<std::string,std::vector<cv::Point3f> > RegionLayout;
    const float FIELD_SIZE;

    std::vector<cv::Point3f> corners;
    std::vector<cv::Point3f> boundingBox;
    std::vector<cv::Point3f> getRotatedBoundingBox();

    std::vector<cv::Point3f> markerRegion;

    cv::Point3f center;

    ChessBoard1();
    virtual BoardPoint p(float x, float y, float z=0.0);
    BoardPoint transp(float x, float y, float z=0.0);

    float angle;
    std::vector<cv::Point3f> rotatePoints(std::vector<cv::Point3f> v);
    std::vector<cv::Point3f> rotatePoints(std::vector<cv::Point3f> v, float angle);
    RegionLayout getRotatedLayout();
    RegionLayout getRotatedLayout(float angle);
    std::vector<cv::Point3f> getRotatedRegion(std::string name);
    std::vector<cv::Point3f> getRotatedRegion(std::string name, float angle);

    // having Eigen members in a class apparently has some issues, so the
    // following macro is needed... 
    // documentation here:
    // http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
    Eigen::Transform<float,3,Eigen::Affine> transform;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::vector<cv::Point3f> rotateTransformPoints(std::vector<cv::Point3f> v);
    RegionLayout getRotatedTransformedLayout();
    std::vector<cv::Point3f> getRotatedTransformedRegion(std::string name);

    float markerPerformanceIndicator(cv::Mat roi, cv::Mat mask);


    double piece_radius;
    double piece_height;



    void print(std::map<std::string,int> s);
      
};


#endif
