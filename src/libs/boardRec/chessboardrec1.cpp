// chessboardrec1.cpp //
#include <ggp_robot/libs/boardRec/chessboardrec1.h>

#include <stdexcept>
#include <sstream>
#include <string>
#include <unistd.h>
#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <visualization_msgs/Marker.h>
#include <ggp_robot/libs/boards/board.h>
#include <ggp_robot/libs/boards/chessboard1.h>
#include <ggp_robot/libs/tools/debug.h>



ChessBoardRec1::ChessBoardRec1() {
  PRINT("[BREC] Recognizer initializing...");
}

void ChessBoardRec1::setBoard(boost::shared_ptr<PlanarBoard>& bp) {
  PRINT("[BREC] Setting board.");
  board = boost::dynamic_pointer_cast<ChessBoard1>(bp);
  if(!board) {
    PRINT(red, "[BREC] Wrong type of chessboard for this recognizer!");
    throw std::domain_error("");
  }
}

void ChessBoardRec1::setCamera(boost::shared_ptr<Camera>& cp) {
  PRINT("[BREC] Setting camera.");
  cam = cp;
}

void ChessBoardRec1::start() {
  PRINT("[BREC] Starting...");

  
  // initialize buffer for stability analysis 
  int buffer_size = 50;
  boost::circular_buffer<Eigen::Vector3f> buffer(buffer_size);
  bool done = false;

  float angle = 0;
  Eigen::Transform<float,3,Eigen::Affine> transform;
  transform.setIdentity();

  // we need only the camera's image stream
  cam->listenToImageStream(true);
  cam->listenToCloudStream(false);

  while(ros::ok() && !done) {
    PRINT("[BREC] Grab an image...");
    cv_bridge::CvImageConstPtr cimg = cam->getCvImage();

    // Detect chessboard pattern
    cv::Size patternSize = cv::Size(3,3);
    cv::Mat corners;
    bool success;
    success = cv::findChessboardCorners(cimg->image, patternSize,
        corners);
    if(!success) {
      PRINT(red, "[BREC] No corners detected! Make sure board is not occluded!");
      PRINT(red, "       Trying again...");
      continue;
    }
    
    PRINT(green, "[BREC] Corners successfully detected!");

    // Draw corners on image
    cv::Mat img_corners = cimg->image.clone();
    cv::drawChessboardCorners(img_corners, patternSize, corners,
        success);

    cv::imshow("Detected corners", img_corners);
    cv::waitKey(1);

    // Corresponding points of the chessboard corners in their own coordinate space
    std::vector<cv::Point3f> objectPoints = board->corners;

    // camera intrinsics
    cv::Matx33d cameraMatrix = cam->getCameraMatrix();

    // camera distortion coefficients
    std::vector<double> distCoeffs = cam->getDistortionCoefficients();

    // solve PnP problem -> get rotation- and translation-vector
    cv::Mat rvec;
    cv::Mat tvec;
    bool solveSuccess;
    solveSuccess = solvePnP(objectPoints, corners, cameraMatrix, distCoeffs, rvec, tvec);
    if(!solveSuccess) {
      PRINT(red, "[BREC] Problems solving the PnP-Problen!");
      continue;
    }


    PRINT(green, "[BREC] Transformation calculated.");
    PRINT("[BREC] Processing rotational ambiguities.");



    std::vector<float> markerKPIs;
    // try four different rotations...
    for(int i=0; i < 4; i++) {
      angle = M_PI/2.0 * i;

      // get rotated marker ROI
      std::vector<cv::Point3f> marker = board->getRotatedRegion("marker", angle);

      // project marker onto image-plane for pixel extraction
      std::vector<cv::Point2f> marker_proj;
      cv::projectPoints(marker, rvec, tvec, cameraMatrix, distCoeffs, marker_proj);
      std::vector<cv::Point> marker_roi;
      {
        // convertion to integer points (pixel-coordinates)
        typedef std::vector<cv::Point2f>::iterator type;
        for(type it = marker_proj.begin(); it != marker_proj.end(); ++it) {
          marker_roi.push_back(cvPointFrom32f(*it));
        }
      }
  
      // make polygon from marker points
      std::vector<cv::Point> poly;
      cv::approxPolyDP(marker_roi, poly, 1.0, true);

      // create empty image mask
      cv::Mat mask = cv::Mat::zeros(cimg->image.size().height, cimg->image.size().width, CV_8UC1);

      // Fill polygon area white on mask -> set this area to true
      cv::fillConvexPoly(mask, &poly[0], poly.size(), 255, 8, 0);

      float kpi = board->markerPerformanceIndicator(cimg->image, mask);
      markerKPIs.push_back(kpi);
    }

    int maxIdx = std::max_element(markerKPIs.begin(), markerKPIs.end()) - markerKPIs.begin();
    PRINT(green, "[BREC] Marker recognized. Internal rotation with " <<
        (maxIdx*90) << " degrees necessary.");

    angle = M_PI/2.0 * maxIdx;
    board->angle = angle;
    std::vector<cv::Point3f> marker = board->getRotatedRegion("marker");
    
    // create Eigen::Transform object for transforming in 3D space
    // opencv sadly cannot do pure affine 3D transforms without projecting to the image plane
    cv::Mat cvRot;
    cv::Rodrigues(rvec, cvRot);
    Eigen::Matrix<float,3,3> rotMat;
    cv2eigen(cvRot, rotMat);
    Eigen::Matrix<float,3,1> transMat;
    cv2eigen(tvec, transMat);
    Eigen::Translation<float,3> translation(transMat);
    Eigen::AngleAxis<float> rotation(rotMat);
    transform = translation * rotation;

    // for the stability analysis, take the first point of the marker region
    Eigen::Vector3f point_board(marker[0].x, marker[0].y, marker[0].z);
    Eigen::Vector3f point_cam = (transform * point_board);

    
    // if the buffer is full, a stability analysis is done...
    if(!buffer.full()) {
      PRINT("[BREC] Collecting samples... (" << buffer.size() << "/" << buffer.capacity() << ")");
    } else {
      PRINT("[BREC] " << buffer.size() << " samples, start stability analysis...");
      typedef boost::circular_buffer<Eigen::Vector3f>::iterator type;

      // note: there is a build-in library for statistical analysis of
      // boost-containers, but I encountered a compile-error that I was not able
      // to get rid of, so I did it by hand

      // calculate the mean of the samples
      Eigen::Vector3f mean(0,0,0);
      int n = 0;
      for(type it=buffer.begin(); it != buffer.end(); ++it) {
        mean += (*it); 
        n++;
      }
      mean = mean/n;

      // calculate the standart deviation of the samples
      Eigen::Vector3f var(0,0,0);
      for(type it=buffer.begin(); it != buffer.end(); ++it) {
        var += (((*it) - mean).array() * ((*it) - mean).array()).matrix();
      }
      var = var/n;
      Eigen::Vector3f std = var.array().sqrt();

      // the stability criterion is a low variability in the samples and then a
      // new sample that is close to the mean
      float stdNorm = std.norm();
      Eigen::Vector3f diff = (mean - point_cam).array().abs().matrix();
      float diffNorm = diff.norm();

      PRINT(red, "[BREC] Variability amongst samples: " << stdNorm);
      PRINT(red, "[BREC] Distance to average-sample: " << diffNorm);
      // TODO: Make stability criteria dependent on buffer-size... apperently
      // more samples lead to greater variability (mean might be independent...
      // test this)
      // also: magic number. avoid.
      if(stdNorm < 0.001 && diffNorm < 0.001) {
        PRINT(magenta, "[BREC] Stable solution found! Stopping here!");
        done = true;
      }
    }

    buffer.push_back(point_cam);

    // TODO tmp display
    PRINT("[BREC] Showing marker on image...");
    cv::Mat img_marker = cimg->image.clone();
    // project marker onto image-plane for pixel extraction
    std::vector<cv::Point2f> marker_proj;
    cv::projectPoints(marker, rvec, tvec, cameraMatrix, distCoeffs, marker_proj);
    std::vector<cv::Point> marker_roi;
    {
      // convertion to integer points (pixel-coordinates)
      typedef std::vector<cv::Point2f>::iterator type;
      for(type it = marker_proj.begin(); it != marker_proj.end(); ++it) {
        marker_roi.push_back(cvPointFrom32f(*it));
      }
    }
    {
      typedef std::vector<cv::Point>::iterator type;
      for(type it = marker_roi.begin(); it != marker_roi.end(); ++it) {
        cv::circle(img_marker, *it, 3.0, cv::Scalar(255, 0, 0), 1, 8);
      }
    }
    cv::imshow("Detected marker", img_marker);
    cv::waitKey(1);

  }
  PRINT("[BREC] Initialization done. Setting board parameters...");
  board->angle = angle;
  board->transform = transform;
  PRINT("[BREC] Stopping...");
}
