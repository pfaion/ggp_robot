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
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>



static const std::string OPENCV_WINDOW = "Image window";

bool reddish(int i, int j) { 
  int idistance = std::min(i, 180-i);
  int jdistance = std::min(j, 180-j);
  return idistance > jdistance;
}

class ImageConverter
{
  // initialize stuff
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher marker_pub_;
  int maxMarkId;

  public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker> ("ggp/marker/test", 1);

    cv::namedWindow(OPENCV_WINDOW);
    maxMarkId = 0;
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }


  // Here the magic happens
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    // convert sensor_msgs to opencv-Mat-like object
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

    // Corresponding points of the chessboard corners in their own coordinate space
    std::vector<cv::Point3f> objectPoints;
    objectPoints.push_back(cv::Point3f(0.2835, 0.0945, 0));
    objectPoints.push_back(cv::Point3f(0.189, 0.0945, 0));
    objectPoints.push_back(cv::Point3f(0.0945, 0.0945, 0));
    objectPoints.push_back(cv::Point3f(0.2835, 0.189, 0));
    objectPoints.push_back(cv::Point3f(0.189, 0.189, 0));
    objectPoints.push_back(cv::Point3f(0.0945, 0.189, 0));
    objectPoints.push_back(cv::Point3f(0.2835, 0.2835, 0));
    objectPoints.push_back(cv::Point3f(0.189, 0.2835, 0));
    objectPoints.push_back(cv::Point3f(0.0945, 0.2835, 0));

    // camera intrinsics
    cv::Matx33d cameraMatrix = cv::Matx33d::zeros();
    cameraMatrix(0,0) = 570.3422241210938;
    cameraMatrix(0,2) = 319.5;
    cameraMatrix(1,1) = 570.3422241210938;
    cameraMatrix(1,2) = 239.5;
    cameraMatrix(2,2) = 1.0;

    // camera distortion coefficients
    std::vector<float> distCoeffs(5,0.0);

    // solve PnP problem -> get rotation- and translation-vector
    cv::Mat rvec;
    cv::Mat tvec;
    bool solveSuccess;
    solveSuccess = solvePnP(objectPoints, corners, cameraMatrix, distCoeffs, rvec, tvec);
    if(!solveSuccess) {
      std::cerr << "Problems solving the PnP problem!" << std::endl;
      return;
    }



    // --------------------------------------
    // Test 1: form a cube out of the corners
    //  -> project 3D data in board-space onto image-plane
    // --------------------------------------
    std::vector<cv::Point3f> points;
    points.push_back(cv::Point3f(0.0945, 0.0945, 0.189));
    points.push_back(cv::Point3f(0.0945, 0.2835, 0.189));
    points.push_back(cv::Point3f(0.2835, 0.0945, 0.189));
    points.push_back(cv::Point3f(0.2835, 0.2835, 0.189));
    // project cube-points onto image-plane
    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(points, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);
    // draw circles for the new points
    for(std::vector<cv::Point2f>::iterator it = projectedPoints.begin(); it != projectedPoints.end(); ++it) {
      cv::circle(img_corners, *it, 3.0, cv::Scalar(0, 255, 0), 1, 8);
    }
    // print the image for visualisation



    // ----------------------------------------------
    // Test 2: construct affine transform and inverse
    //  -> project 3D data from board-space into camera-space and back
    // ----------------------------------------------
    //
    // create rotation-matrix from rotation-vector
    cv::Mat cvRot;
    cv::Rodrigues(rvec, cvRot);
    // convert rotation-matrix to Eigen
    Eigen::Matrix<double,3,3> rotMat;
    cv2eigen(cvRot, rotMat);
    // convert translation-vector to Eigen
    Eigen::Matrix<double,3,1> transMat;
    cv2eigen(tvec, transMat);
    // create Transform objects
    Eigen::Translation<double,3> translation(transMat);
    //std::cout << "translate:" << std::endl << translation.x() << "|" << translation.y() << "|" << translation.z() << std::endl;
    Eigen::AngleAxis<double> rotation(rotMat);
    //std::cout << "rotate:" << std::endl << rotation.matrix() << std::endl;
    // combine to affine transform objects
    // apply scaling (needed for whatever reason)
    Eigen::Transform<double,3,Eigen::Affine> transform = translation * rotation;
    Eigen::Transform<double,3,Eigen::Affine> invTransform = transform.inverse();
    //std::cout << "transform:" << std::endl << transform.matrix() << std::endl;

    // broadcast transform
    static tf::TransformBroadcaster br;
    tf::Transform tf_transform;
    tf_transform.setOrigin(tf::Vector3(transform.matrix()(0,3), transform.matrix()(1,3), transform.matrix()(2,3)));
    tf_transform.setBasis(tf::Matrix3x3(transform.matrix()(0,0), transform.matrix()(0,1), transform.matrix()(0,2), transform.matrix()(1,0), transform.matrix()(1,1), transform.matrix()(1,2), transform.matrix()(2,0), transform.matrix()(2,1), transform.matrix()(2,2)));
    br.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "/camera_rgb_optical_frame", "/board_frame"));
    
    // Test it by transforming a point and showing a Marker in rviz
    Eigen::Vector3d boardCorner(0,0,0);
    Eigen::Vector3d p;
    p = transform * boardCorner;
    //p = Eigen::Scaling(0.001) * p;
    std::string opticalFrame = "/camera_rgb_optical_frame";
    std::string boardFrame = "/board_frame";
    std::string test_ns = "testmarker";
    //pubMarker(test_ns, p, opticalFrame, false,  1.0f, 0.0f, 0.0f);
    //pubMarker(test_ns, boardCorner, opticalFrame, false, 0.0f, 1.0f, 0.0f);
    
    
    
    double n = 0.0;
    double s = 0.0945;
    std::vector<std::vector<cv::Point> > possibleMarkers;
    double xoffset[] = {0.0, 4*s, 3*s, -s};
    double yoffset[] = {-s, 0.0, 4*s, 3*s};
    for(int i=0; i<4; i++){
      std::vector<cv::Point3f> field;
      double x = xoffset[i];
      double y = yoffset[i];
      field.push_back(cv::Point3f(x+n, y+n, n));
      field.push_back(cv::Point3f(x+n, y+s, n));
      field.push_back(cv::Point3f(x+s, y+s, n));
      field.push_back(cv::Point3f(x+s, y+n, n));
      std::vector<cv::Point2f> field_proj;
      cv::projectPoints(field, rvec, tvec, cameraMatrix, distCoeffs, field_proj);
      std::vector<cv::Point> roi;
      for(int j=0; j<4; j++){
        roi.push_back(cvPointFrom32f(field_proj[j]));
      }
      possibleMarkers.push_back(roi);
    }
    
    for(std::vector<std::vector<cv::Point> >::iterator fieldit = possibleMarkers.begin();
        fieldit != possibleMarkers.end();
        ++fieldit) {
        std::vector<cv::Point> field = *fieldit;
      for(std::vector<cv::Point>::iterator it = field.begin();
          it != field.end();
          ++it) {
        cv::circle(img_corners, *it, 3.0, cv::Scalar(0, 255, 0), 1, 8);
      }
    }

   
    std::cout << " ---------------------------- " << std::endl;
    std::vector<int> avgHues;
    for(int i=0; i<4; i++){
      std::vector<cv::Point> roi = possibleMarkers[i];

      std::vector<cv::Point> ROI_Poly;
      cv::approxPolyDP(roi, ROI_Poly, 1.0, true);

      cv::Mat mask = cvCreateMat(cv_ptr->image.size().height, cv_ptr->image.size().width, CV_8UC1);
      for(int x=0; x<mask.cols; x++)
        for(int y=0; y<mask.rows; y++)
          mask.at<uchar>(cv::Point(x,y)) = 0;

      // Fill polygon white
      cv::fillConvexPoly(mask, &ROI_Poly[0], ROI_Poly.size(), 255, 8, 0);     

      cv::Mat masked;
      cv_ptr->image.copyTo(masked, mask);

      std::vector<std::string> wdnames;
      wdnames.push_back("masked0");
      wdnames.push_back("masked1");
      wdnames.push_back("masked2");
      wdnames.push_back("masked3");

      cv::Scalar avg = cv::mean(masked, mask);
      cv::Mat avgBGR;
      avgBGR.create(1,1,cv_ptr->image.type());
      avgBGR = avg;

      cv::Mat avgHSV;
      avgBGR.copyTo(avgHSV);
      cv::cvtColor(avgBGR, avgHSV, CV_BGR2HSV);

      cv::Scalar avH = cv::mean(avgHSV);
      std::cout << "avg HSV" << i << ": " << avH[0] << "|" << avH[1] << "|" << avH[2] << std::endl;
      avgHues.push_back(avH[0]);

      cv::Mat avgBGRBack;
      avgHSV.copyTo(avgBGRBack);
      cv::cvtColor(avgHSV, avgBGRBack, CV_HSV2BGR);

      cv::Mat avgImg;
      cv_ptr->image.copyTo(avgImg);
      avgImg = cv::mean(avgBGRBack);
      cv::imshow(wdnames[i], avgImg);
    }
    
    //  TODO: build better regocnition functio (max "H",S,V all or display error, if only two met)
    int maxId = std::max_element(avgHues.begin(), avgHues.end(), reddish) - avgHues.begin();
    std::cout << maxId << std::endl;

    drawBoardMarker();    


    // show image
    cv::imshow(OPENCV_WINDOW, img_corners);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());



  }


  void debug(const std::string &str){
    std::cout << str << std::endl;
  }

  void drawBoardMarker(){

    double m = 0.04725;
    double s = 0.0945;

    for(int i=0; i<=0; i++){
      for(int j=0; j<=2; j++){
        pubBoardMarker(m+i*s, m+j*s, (i%2==0) != (j%2==0), i+4*j);
      }
    }
  }

  void pubBoardMarker(double x, double y, bool white, int id) {
    visualization_msgs::Marker mark;
    mark.header.frame_id = "/board_frame";
    mark.header.stamp = ros::Time::now();
    mark.ns = "board_fields";
    mark.id = id;
    mark.type = visualization_msgs::Marker::CUBE;

    mark.pose.position.x = x;
    mark.pose.position.y = y;
    mark.pose.position.z = 0.0;

    mark.pose.orientation.x = 0.0;
    mark.pose.orientation.y = 0.0;
    mark.pose.orientation.z = 0.0;
    mark.pose.orientation.w = 1.0;

    mark.scale.x = 0.0945;
    mark.scale.y = 0.0945;
    mark.scale.z = 0.001;

    if(white) {
      mark.color.r = 0.5f;
      mark.color.g = 1.0f;
      mark.color.b = 0.5f;
    } else {
      mark.color.r = 0.5f;
      mark.color.g = 0.0f;
      mark.color.b = 0.0f;
    }

    mark.color.a = 0.5f;
    mark.lifetime = ros::Duration();
    // publish marker
    marker_pub_.publish(mark);   
  }


  void pubMarker(const std::string& namesp, Eigen::Vector3d& p, const std::string& frameId, bool big, float r, float g, float b) {
    visualization_msgs::Marker mark;
    mark.header.frame_id = frameId;
    mark.header.stamp = ros::Time::now();
    mark.ns = namesp;
    mark.id = maxMarkId;
    maxMarkId++;
    mark.type = visualization_msgs::Marker::CUBE;

    if(true) {
      mark.pose.position.x = p(0);
      mark.pose.position.y = p(1);
      mark.pose.position.z = p(2);
    } else {
      mark.pose.position.x = 0;
      mark.pose.position.y = 0;
      mark.pose.position.z = 0;
    }

    mark.pose.orientation.x = 0.0;
    mark.pose.orientation.y = 0.0;
    mark.pose.orientation.z = 0.0;
    mark.pose.orientation.w = 1.0;

    if(big){
      mark.scale.x = 50;
      mark.scale.y = 50;
      mark.scale.z = 50;
    } else {
      mark.scale.x = 0.01;
      mark.scale.y = 0.01;
      mark.scale.z = 0.01;
    }
    mark.color.r = r;
    mark.color.g = g;
    mark.color.b = b;
    mark.color.a = 0.5f;
    mark.lifetime = ros::Duration();
    // publish marker
    marker_pub_.publish(mark);   
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
