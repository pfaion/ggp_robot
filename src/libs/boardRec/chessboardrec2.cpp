// chessboardrec2.cpp //
#include <ggp_robot/libs/boardRec/chessboardrec2.h>

#include <stdexcept>
#include <sstream>
#include <string>
#include <typeinfo>
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

std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

ChessBoardRec2::ChessBoardRec2() {
  PRINT("[BREC] Recognizer initializing...");
}

void ChessBoardRec2::setBoard(boost::shared_ptr<PlanarBoard>& bp) {
  PRINT("[BREC] Setting board.");
  board = boost::dynamic_pointer_cast<ChessBoard1>(bp);
  if(!board) {
    PRINT(red, "[BREC] Wrong type of chessboard for this recognizer!");
    throw std::domain_error("");
  }
}

void ChessBoardRec2::setCamera(boost::shared_ptr<Camera>& cp) {
  PRINT("[BREC] Setting camera.");
  cam = cp;
}

void ChessBoardRec2::start() {
  PRINT("[BREC] Starting...");

  
  // initialize buffer for stability analysis 
  int buffer_size = 10;
  boost::circular_buffer<Eigen::Vector3f> buffer(buffer_size);
  bool done = false;

  float angle = 0;
  Eigen::Transform<float,3,Eigen::Affine> transform;
  transform.setIdentity();

  // we need only the camera's image stream
  cam->listenToImageStream(true);
  cam->listenToCloudStream(true);

  while(ros::ok() && !done) {
    PRINT("[BREC] Grab an image...");
    cv_bridge::CvImageConstPtr cimg = cam->getCvImage();
    PRINT("[BREC] Grab a cloud...");
    pcl::PCLPointCloud2::ConstPtr cloud = cam->getPclCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(*cloud, *pc);

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


    PRINT("[BREC] Getting world coordinates of corners...");
    // get the world-coordinates of the corner-points from the point-cloud
    // note: corner-points are float, so we need a bilinear interpolation
    std::vector<cv::Point3f> worldCorners;
    std::vector<Eigen::Vector3f> worldCornersEig;
    // transform every corner
    for(int i=0; i<corners.rows; ++i) {
      // keep in mind:
      // column (index 0) <=> y-coordinate (index 1)
      // row (index 1) <=> x-coordinate (index 0)
      //
      //         x/col
      //      +----------->
      //   y/ |
      //  row |
      //      |
      //      v
      float col = corners.at<cv::Vec2f>(i)[1];
      float row = corners.at<cv::Vec2f>(i)[0];

      // floor cast
      // TODO: handling image border! (wont be necessary in 99.9999999% of the cases)
      int x0 = (int)row;
      int x1 = x0 + 1;
      int y0 = (int)col;
      int y1 = y0 + 1;

      // differences (for weights)
      float dx0 = row - x0;
      float dx1 = x1 - row;
      float dy0 = col - y0;
      float dy1 = y1 - col;

      // interpolate x, y and z
      float out0 = dx0 * dy0 * pc->at(x0,y0).x + dx1 * dy0 * pc->at(x1,y0).x + dx0 * dy1 * pc->at(x0,y1).x + dx1 * dy1 * pc->at(x1,y1).x;
      float out1 = dx0 * dy0 * pc->at(x0,y0).y + dx1 * dy0 * pc->at(x1,y0).y + dx0 * dy1 * pc->at(x0,y1).y + dx1 * dy1 * pc->at(x1,y1).y;
      float out2 = dx0 * dy0 * pc->at(x0,y0).z + dx1 * dy0 * pc->at(x1,y0).z + dx0 * dy1 * pc->at(x0,y1).z + dx1 * dy1 * pc->at(x1,y1).z;
      
      // store transformed corner
      Eigen::Vector3f veig(out0, out1, out2);
      worldCornersEig.push_back(veig);

    }

    Eigen::Vector3f b02 = worldCornersEig[2] - worldCornersEig[0];
    Eigen::Vector3f b06 = worldCornersEig[6] - worldCornersEig[0];
    Eigen::Vector3f b28 = worldCornersEig[8] - worldCornersEig[2];
    Eigen::Vector3f b68 = worldCornersEig[8] - worldCornersEig[6];
    
    Eigen::Vector3f cross0 = b06.cross(b02);
    cross0 *= 0.5*(b06.norm()+b02.norm())/cross0.norm();
    cross0 = worldCornersEig[0] + cross0;
    worldCornersEig.push_back(cross0);
    Eigen::Vector3f cross2 = b28.cross(b02);
    cross2 *= 0.5*(b28.norm()+b02.norm())/cross2.norm();
    cross2 = worldCornersEig[2] + cross2;
    worldCornersEig.push_back(cross2);
    Eigen::Vector3f cross6 = b06.cross(b68);
    cross6 *= 0.5*(b06.norm()+b68.norm())/cross6.norm();
    cross6 = worldCornersEig[6] + cross6;
    worldCornersEig.push_back(cross6);
    Eigen::Vector3f cross8 = b28.cross(b68);
    cross8 *= 0.5*(b28.norm()+b68.norm())/cross8.norm();
    cross8 = worldCornersEig[8] + cross8;
    worldCornersEig.push_back(cross8);

    for(int i=0; i<13; ++i) {
      float x = worldCornersEig[i][0];
      float y = worldCornersEig[i][1];
      float z = worldCornersEig[i][2];
      worldCorners.push_back(cv::Point3f(x,y,z));
    }

    std::vector<Eigen::Vector3f> boardCorners;
    boardCorners.push_back(board->p(3,1,0));
    boardCorners.push_back(board->p(2,1,0));
    boardCorners.push_back(board->p(1,1,0));
    boardCorners.push_back(board->p(3,2,0));
    boardCorners.push_back(board->p(2,2,0));
    boardCorners.push_back(board->p(1,2,0));
    boardCorners.push_back(board->p(3,3,0));
    boardCorners.push_back(board->p(2,3,0));
    boardCorners.push_back(board->p(1,3,0));
    boardCorners.push_back(board->p(3,1,2));
    boardCorners.push_back(board->p(1,1,2));
    boardCorners.push_back(board->p(3,3,2));
    boardCorners.push_back(board->p(1,3,2));

    Eigen::MatrixXf worldData(worldCornersEig.size(),4);
    for(int i=0; i< worldCornersEig.size(); ++i) {
      worldData.row(i) << worldCornersEig[i][0], worldCornersEig[i][1], worldCornersEig[i][2], 1;
    }
    PRINT(red, worldData);

    Eigen::MatrixXf boardData(boardCorners.size(),3);
    for(int i=0; i< boardCorners.size(); ++i) {
      boardData.row(i) << boardCorners[i][0], boardCorners[i][1], boardCorners[i][2];
    }
    PRINT(cyan, boardData);

    Eigen::MatrixXf leastSquaresSolution;
    leastSquaresSolution = worldData.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(boardData);
    Eigen::Matrix4f transformMatrix;
    transformMatrix.topRows(3) = leastSquaresSolution.transpose();
    transformMatrix.row(3) << 0,0,0,1;
    PRINT(yellow, transformMatrix);
    Eigen::Transform<float,3,Eigen::Affine> newTrans(transformMatrix);
    newTrans = newTrans.inverse();



    // Corresponding points of the chessboard corners in their own coordinate space
    std::vector<cv::Point3f> objectPoints = board->corners;

    // camera intrinsics
    cv::Matx33d cameraMatrix = cam->getCameraMatrix();

    // camera distortion coefficients
    std::vector<double> distCoeffs = cam->getDistortionCoefficients();

    // solve PnP problem -> get rotation- and translation-vector
    /*cv::Mat rvec;
    cv::Mat tvec;
    bool solveSuccess;
    solveSuccess = solvePnP(objectPoints, corners, cameraMatrix, distCoeffs, rvec, tvec);
    if(!solveSuccess) {
      PRINT(red, "[BREC] Problems solving the PnP-Problen!");
      continue;
    }*/

    cv::Mat tvec = (cv::Mat_<float>(1,3) << newTrans.matrix()(0,3),
        newTrans.matrix()(1,3), newTrans.matrix()(2,3)); 

    Eigen::Matrix3f rotmateig = newTrans.matrix().topLeftCorner(3,3).matrix();
    Eigen::AngleAxisf aaf(rotmateig);
    Eigen::Vector3f aavec = aaf.angle() * aaf.axis();

    cv::Mat rvec = (cv::Mat_<float>(1,3) << aavec[0], aavec[1], aavec[2]);


    // TODO tmp display
    cv::Mat img_worldCorners = cimg->image.clone();
    std::vector<cv::Point2f> corners_proj;
    cv::Mat tmprvec(1,3,CV_64FC1,cv::Scalar(0));
    cv::Mat tmptvec(1,3,CV_64FC1,cv::Scalar(0));
    cv::projectPoints(worldCorners, tmprvec, tmptvec, cameraMatrix, distCoeffs,
        corners_proj);
    std::vector<cv::Point> corners_roi;
    {
      // convertion to integer points (pixel-coordinates)
      typedef std::vector<cv::Point2f>::iterator type;
      for(type it = corners_proj.begin(); it != corners_proj.end(); ++it) {
        corners_roi.push_back(cvPointFrom32f(*it));
      }
    }
    {
      typedef std::vector<cv::Point>::iterator type;
      for(type it = corners_roi.begin(); it != corners_roi.end(); ++it) {
        cv::circle(img_worldCorners, *it, 3.0, cv::Scalar(255, 0, 0), 1, 8);
      }
    }
    cv::imshow("Corners from world coordinates", img_worldCorners);
    cv::waitKey(1);



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

    PRINT(magenta, transform.inverse().matrix());

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
      if(stdNorm < 0.05 && diffNorm < 0.005) {
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
