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
#include <ggp_robot/libs/boards/board.h>
#include <ggp_robot/libs/boards/chessboard1.h>
#include <ggp_robot/libs/tools/debug.h>



ChessBoardRec1::ChessBoardRec1() {
  PRINT("[BREC] Recognizer initializing...");
}

void ChessBoardRec1::setBoard(boost::shared_ptr<PlanarBoard> bp) {
  PRINT("[BREC] Setting board.");
  board = boost::dynamic_pointer_cast<ChessBoard1>(bp);
  if(!board) {
    PRINT(red, "[BREC] Wrong type of chessboard for this recognizer!");
    throw std::domain_error("");
  }
}

void ChessBoardRec1::setCamera(boost::shared_ptr<Camera> cp) {
  PRINT("[BREC] Setting camera.");
  cam = cp;
}

void ChessBoardRec1::start() {
  PRINT("[BREC] Starting...");

  
  // initialize buffer for stability analysis 
  boost::circular_buffer<Eigen::Vector3f> buffer(100);

  bool done = false;

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
      float angle = M_PI/2.0 * i;

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

    std::vector<cv::Point3f> marker = board->getRotatedRegion("marker", (M_PI/2.0*maxIdx));
    
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
    Eigen::Transform<float,3,Eigen::Affine> transform = translation * rotation;

    Eigen::Vector3f point_board(marker[0].x, marker[0].y, marker[0].z);
    Eigen::Vector3f point_cam = 1000 * (transform * point_board);

    if(buffer.full()) {
      PRINT("[BREC] Buffer full, start analysing stability...");
      typedef boost::circular_buffer<Eigen::Vector3f>::iterator type;
      Eigen::Vector3f sum(0,0,0);
      int n = 0;
      for(type it=buffer.begin(); it != buffer.end(); ++it) {
        sum += (*it); 
        n++;
      }
      Eigen::Vector3f mean = sum/n;
      Eigen::Vector3f diff = (mean - point_cam).array().abs().matrix();
      float norm = diff.norm();
      PRINT(red, norm);
      if(norm < 0.05) {
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

  PRINT("[BREC] Stopping...");
}

