// chessstaterec2.cpp //
#include <ggp_robot/libs/stateRec/chessstaterec2.h>

#include <stdexcept>
#include <sstream>
#include <string>
#include <algorithm>
#include <vector>
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
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <ggp_robot/libs/boards/board.h>
#include <ggp_robot/libs/boards/chessboard1.h>
#include <ggp_robot/libs/tools/debug.h>
#include <ggp_robot/libs/tools/keylistener.h>


ChessStateRec2::ChessStateRec2()
  : viewer("test")
{
  PRINT("[SREC] Recognizer initializing...");

  viewer.setBackgroundColor(0.5, 0.5, 0.5);
}

void ChessStateRec2::setBoard(boost::shared_ptr<PlanarBoard>& bp) {
  PRINT("[SREC] Setting board.");
  board = boost::dynamic_pointer_cast<ChessBoard1>(bp);
  if(!board) {
    PRINT(red, "[SREC] Wrong type of chessboard for this recognizer!");
    throw std::domain_error("");
  }
}

void ChessStateRec2::setCamera(boost::shared_ptr<Camera>& cp) {
  PRINT("[SREC] Setting camera.");
  cam = cp;
}

bool ChessStateRec2::start() {
  PRINT("[SREC] Starting...");
  cam->listenToCloudStream(false);
  cam->setCloudTopic("/camera/depth/points");
  cam->listenToCloudStream(true);
  cam->listenToImageStream(false);

  KeyListener keyboard;
  keyboard.start();

  while(ros::ok()) {
    PRINT("[SREC] Fetching cloud.");
    pcl::PCLPointCloud2::ConstPtr cloud = cam->getPclCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(*cloud, *pc);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new
        pcl::PointCloud<pcl::PointXYZ>());

    pcl::CropBox<pcl::PointXYZ> crop;
    std::vector<cv::Point3f> reg = board->getRotatedBoundingBox();
    cv::Point3f min = reg[0];
    cv::Point3f max = reg[0];
    typedef std::vector<cv::Point3f>::iterator type;
    for(type it=reg.begin(); it != reg.end(); ++it) {
      min.x = std::min(min.x, (*it).x);
      min.y = std::min(min.y, (*it).y);
      min.z = std::min(min.z, (*it).z);
      max.x = std::max(max.x, (*it).x);
      max.y = std::max(max.y, (*it).y);
      max.z = std::max(max.z, (*it).z);
    }   
    crop.setMin(Eigen::Vector4f(min.x, min.y, min.z, 0));
    crop.setMax(Eigen::Vector4f(max.x, max.y, max.z, 0));
    // here we need the inverse...
    // apparently the filter transforms the cloud by the inverse transform and
    // then applies the cutoff filters
    crop.setTransform(board->transform.inverse());
    crop.setInputCloud(pc);
    crop.filter(*filtered);




    // PCL Cylinder Segmentation Tutorial:

    // Estimate point normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setSearchMethod (tree);
    ne.setInputCloud (filtered);
    ne.setKSearch (50);
    ne.compute (*normals);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (filtered);
    seg.setInputNormals (normals);
    // Obtain the plane inliers and coefficients
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    seg.segment (*inliers_plane, *coefficients_plane);

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    // Remove the planar inliers, extract the rest
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered2(new pcl::PointCloud<pcl::PointXYZ>());
    extract.setNegative (true);
    extract.filter (*filtered2);
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*normals2);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0.022, 0.027);
    seg.setInputCloud (filtered2);
    seg.setInputNormals (normals2);
    // Obtain the cylinder inliers and coefficients
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    seg.segment (*inliers_cylinder, *coefficients_cylinder);

    PRINT(magenta, "[SREC] radius: " << coefficients_cylinder->values[6]);

    // Extract cylinder inliers
    extract.setInputCloud (filtered2);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new
        pcl::PointCloud<pcl::PointXYZ> ());
    extract.filter (*cloud_cylinder);


    viewer.removeAllPointClouds();
    viewer.removeAllShapes();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      lightRed(cloud_cylinder, 255,130,130);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_cylinder, lightRed,
        "cylinderpts");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      red(cloud_cylinder, 255,0,0);
    //viewer.addCylinder(*coefficients_cylinder, "cylinder");

    extract.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered3(new pcl::PointCloud<pcl::PointXYZ>());
    extract.filter(*filtered3);
    viewer.addPointCloud<pcl::PointXYZ>(filtered3, "rest");


    // test: compute intersection between ransac plane and cylinder
    float nx = coefficients_plane->values[0];
    float ny = coefficients_plane->values[1];
    float nz = coefficients_plane->values[2];
    float p = coefficients_plane->values[3];
    float ox = coefficients_cylinder->values[0];
    float oy = coefficients_cylinder->values[1];
    float oz = coefficients_cylinder->values[2];
    float vx = coefficients_cylinder->values[3];
    float vy = coefficients_cylinder->values[4];
    float vz = coefficients_cylinder->values[5];
    Eigen::Matrix4f M;
    M << nx, ny, nz, 0,   1, 0, 0, vx,   0, 1, 0, vy,   0, 0, 1, vz;
    Eigen::Vector4f b;
    b << -p, ox, oy, oz;
    Eigen::Vector4f x = M.partialPivLu().solve(b);
    pcl::PointCloud<pcl::PointXYZ>::Ptr intersect(new
        pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ pt;
    pt.x = x(0);
    pt.y = x(1);
    pt.z = x(2);
    intersect->push_back(pt);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      green(intersect, 0,255,0);
    viewer.addPointCloud<pcl::PointXYZ>(intersect, green, "intersect");
    viewer.setPointCloudRenderingProperties
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "intersect");





    viewer.spinOnce(100);
    char c = keyboard.getChar();
    if(c == 'q') return true;
    if(c == 'r') return false;
    if(c == 's') viewer.spin();

  }
}
