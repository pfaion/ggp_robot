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
#include <ggp_robot/libs/board_old.h>
#include <ggp_robot/libs/chessboard.h>
#include <ggp_robot/libs/factories.h>


using namespace GgpRobot;

bool reddish(int i, int j) { 
  int idistance = std::min(i, 180-i);
  int jdistance = std::min(j, 180-j);
  return idistance > jdistance;
}


class BPtmp: public cv::Point3f{
  public:
    static double FIELD_SIZE;
    BPtmp(float x, float y): cv::Point3f(x*FIELD_SIZE, y*FIELD_SIZE, 0.0f) {};
    BPtmp(float x, float y, float z): cv::Point3f(x*FIELD_SIZE, y*FIELD_SIZE, z*FIELD_SIZE) {};
};
double BPtmp::FIELD_SIZE = 0.0945;

class BoardVector: public Eigen::Vector3d{
  public:
    static double FIELD_SIZE;
    BoardVector(double x, double y): Eigen::Vector3d(x*FIELD_SIZE, y*FIELD_SIZE, 0.0) {};
    BoardVector(double x, double y, double z): Eigen::Vector3d(x*FIELD_SIZE, y*FIELD_SIZE, z*FIELD_SIZE) {};
    BoardVector(Eigen::Vector3d vec): Eigen::Vector3d(vec) {};
    Eigen::Vector3d transform(Eigen::Transform<double,3,Eigen::Affine> t) {
      Eigen::Vector3d myclone((*this)[0], (*this)[1], (*this)[2]);
      Eigen::Vector3d transformed(t * myclone);
      return transformed;
    };
};
double BoardVector::FIELD_SIZE = 0.0945;

class DoStuffClass
{
  // initialize stuff
  private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber imageSub_;
  ros::Publisher markerPub_;
  Board board;

  public:
  // standart constructor
  DoStuffClass():
    it_(nh_),
    board()
  {
    std::cout << "CONSTRUCTOR CALLED!" << std::endl;
    // subscribe to input video and advertise marker-topic
    imageSub_ = it_.subscribe("/camera/rgb/image_raw", 1, &DoStuffClass::imageCb, this);
    markerPub_ = nh_.advertise<visualization_msgs::Marker> ("ggp/marker", 1);
  }

  // TODO: destructor?
  ~DoStuffClass(){}

  // callback function for received images... here the magic happens
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    // convert sensor_msgs to opencv-Mat-like object
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Detect chessboard pattern
    cv::Size patternSize = cv::Size(3,3);
    cv::Mat corners;
    bool success;
    success = cv::findChessboardCorners(cv_ptr->image, patternSize, corners);
    if(!success) {
      std::cerr << "No corners detected!" <<std::endl;
      return;
    }

    // Draw corners on image
    cv::Mat img_corners = cv_ptr->image.clone();
    cv::drawChessboardCorners(img_corners, patternSize, corners, success);

    // Corresponding points of the chessboard corners in their own coordinate space
    std::vector<cv::Point3f> objectPoints = board.getBoardCorners();

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


    // ---------- TODO ----------- REPLACE ALL THAT CRAP
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
    double scale = 1.0/1000.0;
    Eigen::Transform<double,3,Eigen::Affine> scaling;
    scaling.setIdentity();
    scaling *= Eigen::Scaling(scale);
    Eigen::Transform<double,3,Eigen::Affine> transform = translation * rotation;
    //Eigen::Transform<double,3,Eigen::Affine> transform = Eigen::Scaling(scale,scale,scale) * Eigen::Translation<double,3>(0.0,0.0,0.0) *  rotation;
    //Eigen::Transform<double,3,Eigen::Affine> transform = Eigen::Translation<double,3>(0.0,0.0,0.0) * Eigen::AngleAxis<double>(Eigen::Matrix<double,3,3>::Identity(3,3));
    //Eigen::Transform<double,3,Eigen::Affine> transform = scaling;
    Eigen::Transform<double,3,Eigen::Affine> invTransform = transform.inverse();
    //std::cout << "transform:" << std::endl << transform.matrix() << std::endl;

    std::cout << transform.matrix() << std::endl;

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
    //double s = 0.0945;
    double s = 1;
    std::vector<std::vector<cv::Point> > possibleMarkers;
    std::vector<std::vector<cv::Point3f> > tmpMarkerBPtmps;
    double xoffset[] = {0.0, 4*s, 3*s, -s};
    double yoffset[] = {-s, 0.0, 4*s, 3*s};
    for(int i=0; i<4; i++){
      std::vector<cv::Point3f> field;
      double x = xoffset[i];
      double y = yoffset[i];
      field.push_back(BPtmp(x+n, y+n, n));
      field.push_back(BPtmp(x+n, y+s, n));
      field.push_back(BPtmp(x+s, y+s, n));
      field.push_back(BPtmp(x+s, y+n, n));
      tmpMarkerBPtmps.push_back(field);
      std::vector<cv::Point2f> field_proj;
      cv::projectPoints(field, rvec, tvec, cameraMatrix, distCoeffs, field_proj);
      std::vector<cv::Point> roi;
      for(int j=0; j<4; j++){
        roi.push_back(cvPointFrom32f(field_proj[j]));
      }
      possibleMarkers.push_back(roi);
    }

    //    for(std::vector<std::vector<cv::Point> >::iterator fieldit = possibleMarkers.begin();
    //        fieldit != possibleMarkers.end();
    //        ++fieldit) {
    //      std::vector<cv::Point> field = *fieldit;
    //      for(std::vector<cv::Point>::iterator it = field.begin();
    //          it != field.end();
    //          ++it) {
    //        std::cout << *it << std::endl;
    //        cv::circle(img_corners, *it, 3.0, cv::Scalar(0, 255, 0), 1, 8);
    //      }
    //    }


    std::vector<cv::Point> roi = possibleMarkers[0];

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
    cv::imshow("masked1", masked);






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

    //drawBoardMarker();    

    //for(int i=0; i<4; i++) {
    for(int j=0; j<4; j++) {
      cv::circle(img_corners, possibleMarkers[maxId][j], 3.0, cv::Scalar(0, 255, 0), 1, 8);
      pubBoardMarker(tmpMarkerBPtmps[maxId][j], true, 10*j);
    }
    //}

    int xb = 0;
    int yb = 0;
    int x;
    int y;
    if(maxId == 0) {
      x = yb;
      y = 3 - xb;
    } else if(maxId == 1) {
      x = xb;
      y = yb;
    } else if(maxId == 2) {
      x = 3 - yb;
      y = xb;
    } else if(maxId == 3) {
      x = 3 - xb;
      y = 3 - yb;
    }
    double mid = 0.5;
    //for(int x=0; x<=2; x++){
    //for(int y=0; y<=0; y++){
    BoardVector pnt(x + mid, y + mid);
    BoardVector pnt_trans = pnt.transform(transform);
    bool white((xb%2==0) != (yb%2==0));
    int fieldId = 1; 
    pubBoardMarker(pnt, white, fieldId);
    //}
    //}



    // show image
    cv::imshow("Camera Image", img_corners);
    cv::waitKey(3);




  }


  void debug(const std::string &str){
    std::cout << str << std::endl;
  }

  void drawBoardMarker(){

    double offset = 0.5;
    for(int i=0; i<=2; i++){
      for(int j=0; j<=0; j++){
        BPtmp p(offset+i, offset+j);
        pubBoardMarker(p, (i%2==0) != (j%2==0), i+4*j);
      }
    }
  }

  void pubBoardMarker(BPtmp p, bool white, int id) {
    visualization_msgs::Marker mark;
    mark.header.frame_id = "/board_frame";
    mark.header.stamp = ros::Time::now();
    mark.ns = "board_fields";
    mark.id = id;
    mark.type = visualization_msgs::Marker::CUBE;

    mark.pose.position.x = p.x;
    mark.pose.position.y = p.y;
    mark.pose.position.z = p.z;

    mark.pose.orientation.x = 0.0;
    mark.pose.orientation.y = 0.0;
    mark.pose.orientation.z = 0.0;
    mark.pose.orientation.w = 1.0;

    double s = BPtmp::FIELD_SIZE;
    mark.scale.x = s;
    mark.scale.y = s;
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
    markerPub_.publish(mark);   
  }
  void pubBoardMarker(cv::Point3f p, bool white, int id) {
    visualization_msgs::Marker mark;
    mark.header.frame_id = "/board_frame";
    mark.header.stamp = ros::Time::now();
    mark.ns = "board_fields";
    mark.id = id;
    mark.type = visualization_msgs::Marker::CUBE;

    mark.pose.position.x = p.x;
    mark.pose.position.y = p.y;
    mark.pose.position.z = p.z;

    mark.pose.orientation.x = 0.0;
    mark.pose.orientation.y = 0.0;
    mark.pose.orientation.z = 0.0;
    mark.pose.orientation.w = 1.0;

    double s = BPtmp::FIELD_SIZE;
    mark.scale.x = s/10;
    mark.scale.y = s/10;
    mark.scale.z = s/10;

    if(white) {
      mark.color.r = 0.5f;
      mark.color.g = 0.5f;
      mark.color.b = 1.0f;
    } else {
      mark.color.r = 0.5f;
      mark.color.g = 0.0f;
      mark.color.b = 0.0f;
    }

    mark.color.a = 1.0f;
    mark.lifetime = ros::Duration();
    // publish marker
    markerPub_.publish(mark);   
  }

  void pubBoardMarker(BoardVector p, bool white, int id) {
    visualization_msgs::Marker mark;
    mark.header.frame_id = "/board_frame";
    mark.header.stamp = ros::Time::now();
    mark.ns = "board_fields";
    mark.id = id;
    mark.type = visualization_msgs::Marker::CUBE;

    mark.pose.position.x = p[0];
    mark.pose.position.y = p[1];
    mark.pose.position.z = p[2];

    mark.pose.orientation.x = 0.0;
    mark.pose.orientation.y = 0.0;
    mark.pose.orientation.z = 0.0;
    mark.pose.orientation.w = 1.0;

    double s = BPtmp::FIELD_SIZE;
    mark.scale.x = s;
    mark.scale.y = s;
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
    markerPub_.publish(mark);   

    // just for testing purpose
    //ros::shutdown();
  }
};


int main(int argc, char** argv)
{
  
  std::cout << "GO GO!" << std::endl;
  PlanarBoard* xyz = PlanarBoardFactory::create("chessboard");
  std::cout << xyz << std::endl;
  std::cout << (*xyz).regions["marker"][0] << std::endl;
  std::cout << "END" << std::endl;


  ros::init(argc, argv, "image_converter");

  DoStuffClass tmpstuff;

  ros::spin();
  return EXIT_SUCCESS;
}
