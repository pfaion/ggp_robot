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
  cam->setCloudTopic("/camera/depth_registered/points");
  cam->listenToCloudStream(true);
  cam->listenToImageStream(true);

  KeyListener keyboard;
  keyboard.start();

  typedef pcl::PointXYZRGB PointType;

  while(ros::ok()) {

    viewer.removeAllPointClouds();
    viewer.removeAllShapes();

    PRINT("[SREC] Fetching cloud.");
    pcl::PCLPointCloud2::ConstPtr cloud = cam->getPclCloud();
    pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>());
    pcl::fromPCLPointCloud2(*cloud, *pc);

    pcl::PointCloud<PointType>::Ptr filtered(new
        pcl::PointCloud<PointType>());

    pcl::CropBox<PointType> crop;
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
    pcl::NormalEstimation<PointType, pcl::Normal> ne;
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setSearchMethod (tree);
    ne.setInputCloud (filtered);
    ne.setKSearch (50);
    ne.compute (*normals);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentationFromNormals<PointType, pcl::Normal> seg;
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
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud (filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);
    pcl::visualization::PointCloudColorHandlerRGBField<PointType>
      rgb(filtered);
    viewer.addPointCloud<PointType>(filtered, rgb,
        "planepts");

    // Remove the planar inliers, extract the rest
    pcl::PointCloud<PointType>::Ptr filtered2(new pcl::PointCloud<PointType>());
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
    seg.setRadiusLimits (0, 0.1);
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
    pcl::PointCloud<PointType>::Ptr cloud_cylinder (new
        pcl::PointCloud<PointType> ());
    extract.filter (*cloud_cylinder);





    pcl::visualization::PointCloudColorHandlerCustom<PointType>
      lightRed(cloud_cylinder, 255,130,130);
    viewer.addPointCloud<PointType>(cloud_cylinder, lightRed,
        "cylinderpts");
    pcl::visualization::PointCloudColorHandlerCustom<PointType>
      red(cloud_cylinder, 255,0,0);
    //viewer.addCylinder(*coefficients_cylinder, "cylinder");

    extract.setNegative(true);
    pcl::PointCloud<PointType>::Ptr filtered3(new pcl::PointCloud<PointType>());
    extract.filter(*filtered3);
    viewer.addPointCloud<PointType>(filtered3, "rest");


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
    pcl::PointCloud<PointType>::Ptr intersect(new
        pcl::PointCloud<PointType>);
    PointType pt;
    pt.x = x(0);
    pt.y = x(1);
    pt.z = x(2);
    intersect->push_back(pt);
    pcl::visualization::PointCloudColorHandlerCustom<PointType>
      green(intersect, 0,255,0);
    viewer.addPointCloud<PointType>(intersect, green, "intersect");
    viewer.setPointCloudRenderingProperties
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "intersect");


    if(cloud_cylinder->size() != 0) {
      Eigen::Vector3f vec(0,0,0);
      // re-adjust basepoint from known radius
      for(pcl::PointCloud<PointType>::iterator it = cloud_cylinder->begin();
          it != cloud_cylinder->end();
          ++it) {
        // distance between base point pt and the projection of cloud-point (it)
        // onto the cylinder axis (scaled relative to normal-vector-length)
        // notice: the normal vector of the plane is the axis-vector of the
        // cylinder
        float factor = ((it->x - pt.x)*nx + (it->y - pt.y)*ny + (it->z -
              pt.z)*nz);
        // sum the vectors from the cloud-point to their projection-point on the
        // cylinder axis 
        vec(0) += (pt.x + factor*nx) - it->x;
        vec(1) += (pt.y + factor*ny) - it->y;
        vec(2) += (pt.z + factor*nz) - it->z;
      }
      // unit length
      vec = 1/std::sqrt(vec(0)*vec(0) + vec(1)*vec(1) + vec(2)*vec(2)) * vec;

      // re-scaling with known radius
      float radius = 0.025;
      float radius_scaling = radius - coefficients_cylinder->values[6];
      Eigen::Vector3f newBase_raw = Eigen::Vector3f(pt.x, pt.y, pt.z) + radius_scaling * vec;

      // now project new base onto plane
      float distanceToPlane = nx*newBase_raw(0) + ny*newBase_raw(1) +
        nz*newBase_raw(2) + p;
      Eigen::Vector3f newBase = newBase_raw - distanceToPlane * Eigen::Vector3f(nx,ny,nz);

      PRINT(Eigen::Vector3f(nx, ny, nz));
      // show me your basepoint!
      pcl::PointCloud<PointType>::Ptr base(new
          pcl::PointCloud<PointType>);
      PointType pt2;
      pt2.x = newBase(0);
      pt2.y = newBase(1);
      pt2.z = newBase(2);
      base->push_back(pt2);
      pcl::visualization::PointCloudColorHandlerCustom<PointType>
        basecol(base, 200,0,0);
      viewer.addPointCloud<PointType>(base, basecol, "base");
    viewer.setPointCloudRenderingProperties
        (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "base");

      pcl::PointCloud<PointType>::Ptr line(new
          pcl::PointCloud<PointType>);
      pcl::PointCloud<PointType>::Ptr line2(new
          pcl::PointCloud<PointType>);
      for(float i=0; i<150; i++) {
        Eigen::Vector3f tmp = newBase + i/1000 * Eigen::Vector3f(nx, ny, nz);
        PointType tmpp;
        tmpp.x = tmp(0);
        tmpp.y = tmp(1);
        tmpp.z = tmp(2);
        line->push_back(tmpp);
        Eigen::Vector3f tmp2 = newBase + i/1000 * Eigen::Vector3f(vx, vy, vz);
        PointType tmpp2;
        tmpp2.x = tmp2(0);
        tmpp2.y = tmp2(1);
        tmpp2.z = tmp2(2);
        line2->push_back(tmpp2);
        Eigen::Vector3f tmp3 = newBase - i/1000 * Eigen::Vector3f(vx, vy, vz);
        PointType tmpp3;
        tmpp3.x = tmp3(0);
        tmpp3.y = tmp3(1);
        tmpp3.z = tmp3(2);
        line2->push_back(tmpp3);
      }
      pcl::visualization::PointCloudColorHandlerCustom<PointType>
        tmpcol(line, 200,0,200);
      viewer.addPointCloud<PointType>(line, tmpcol, "tmp");
    viewer.setPointCloudRenderingProperties
        (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tmp");
      pcl::visualization::PointCloudColorHandlerCustom<PointType>
        tmpcol2(line2, 0,200,200);
      viewer.addPointCloud<PointType>(line2, tmpcol2, "tmp2");
    viewer.setPointCloudRenderingProperties
        (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tmp2");

    }




    viewer.spinOnce(100);
    char c = keyboard.getChar();
    if(c == 'q') return true;
    if(c == 'r') return false;
    if(c == 's') {
      while(ros::ok()) {
        viewer.spinOnce(1);
        if(keyboard.getChar() == 's') break;
      }
    }


  }
}
