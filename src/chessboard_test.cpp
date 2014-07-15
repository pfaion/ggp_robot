#include <ros/ros.h>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/types_c.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <visualization_msgs/Marker.h>



static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher marker_pub_;

  public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, 
        &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
     marker_pub_ = nh_.advertise<visualization_msgs::Marker> ("ggp/marker/test", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


    // Detect chessboard pattern
    CvSize patternSize = cvSize(3,3);
    cv::Mat corners;
    bool success;
    success = cv::findChessboardCorners(cv_ptr->image, patternSize, corners);

    // Draw corners on image
    cv::Mat img_corners = cv_ptr->image.clone();
    cv::drawChessboardCorners(img_corners, patternSize, corners, success);
    if(!success) {
      std::cerr << "No corners detected!" <<std::endl;
      return;
    }


    std::vector<cv::Point3f> objectPoints;
    objectPoints.push_back(cv::Point3f(283.5, 94.5, 0));
    objectPoints.push_back(cv::Point3f(189, 94.5, 0));
    objectPoints.push_back(cv::Point3f(94.5, 94.5, 0));
    objectPoints.push_back(cv::Point3f(283.5, 189, 0));
    objectPoints.push_back(cv::Point3f(189, 189, 0));
    objectPoints.push_back(cv::Point3f(94.5, 189, 0));
    objectPoints.push_back(cv::Point3f(283.5, 283.5, 0));
    objectPoints.push_back(cv::Point3f(189, 283.5, 0));
    objectPoints.push_back(cv::Point3f(94.5, 283.5, 0));

    cv::Matx33d cameraMatrix = cv::Matx33d::zeros();
    cameraMatrix(0,0) = 570.3422241210938;
    cameraMatrix(0,2) = 319.5;
    cameraMatrix(1,1) = 570.3422241210938;
    cameraMatrix(1,2) = 239.5;
    cameraMatrix(2,2) = 1.0;

    std::vector<float> distCoeffs(5,0.0);


    cv::Mat rvec;
    cv::Mat tvec;

    bool solveSuccess;
    solveSuccess = solvePnP(objectPoints, corners, cameraMatrix, distCoeffs, rvec, tvec);
    if(!solveSuccess) {
      std::cerr << "Problems solving the PnP problem!" << std::endl;
      return;
    }

    std::vector<cv::Point3f> points;
    points.push_back(cv::Point3f(94.5, 94.5, 189));
    points.push_back(cv::Point3f(94.5, 283.5, 189));
    points.push_back(cv::Point3f(283.5, 94.5, 189));
    points.push_back(cv::Point3f(283.5, 283.5, 189));

    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(points, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);


    for(std::vector<cv::Point2f>::iterator it = projectedPoints.begin(); it != projectedPoints.end(); ++it) {
      cv::circle(img_corners, *it, 3.0, cv::Scalar(0, 255, 0), 1, 8);
    }


    cv::Mat cvRot;
    cv::Rodrigues(rvec, cvRot);

    Eigen::Matrix<double,3,3> rotMat;
    cv2eigen(cvRot, rotMat);


    Eigen::Matrix<double,3,1> transMat;
    cv2eigen(tvec, transMat);


    Eigen::Translation<double,3> translation(transMat);
    Eigen::AngleAxis<double> rotation(rotMat);

    Eigen::Transform<double,3,Eigen::Affine> transform = translation * rotation;
    Eigen::Transform<double,3,Eigen::Affine> invTransform = transform.inverse();



    Eigen::Vector3d boardCorner(0,0,0);
    Eigen::Vector3d p;
    p = transform * boardCorner;



    visualization_msgs::Marker mark;
    mark.header.frame_id = "/camera_rgb_optical_frame";
    mark.header.stamp = ros::Time::now();
    mark.ns = "test_marker";
    mark.id = 0;
    mark.type = visualization_msgs::Marker::CUBE;
    mark.pose.position.x = p(0)/1000;
    mark.pose.position.y = p(1)/1000;
    mark.pose.position.z = p(2)/1000;
    mark.pose.orientation.x = 0.0;
    mark.pose.orientation.y = 0.0;
    mark.pose.orientation.z = 0.0;
    mark.pose.orientation.w = 1.0;
    mark.scale.x = 0.01;
    mark.scale.y = 0.01;
    mark.scale.z = 0.01;
    mark.color.r = 1.0f;
    mark.color.g = 0.0f;
    mark.color.b = 0.0f;
    mark.color.a = 0.5f;
    mark.lifetime = ros::Duration();

   

    marker_pub_.publish(mark);


    cv::imshow(OPENCV_WINDOW, img_corners);
    cv::waitKey(3);



    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());



  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
