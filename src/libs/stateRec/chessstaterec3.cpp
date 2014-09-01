// chessstaterec3.cpp //
#include <ggp_robot/libs/stateRec/chessstaterec3.h>

#include <stdexcept>
#include <sstream>
#include <string>
#include <algorithm>
#include <vector>
#include <stdlib.h>
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
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <ggp_robot/libs/boards/board.h>
#include <ggp_robot/libs/boards/chessboard1.h>
#include <ggp_robot/libs/tools/debug.h>
#include <ggp_robot/libs/tools/keylistener.h>


ChessStateRec3::ChessStateRec3()
  : viewer("test")
{
  PRINT("[SREC] Recognizer initializing...");

  viewer.setBackgroundColor(0.5, 0.5, 0.5);
}

void ChessStateRec3::setBoard(boost::shared_ptr<PlanarBoard>& bp) {
  PRINT("[SREC] Setting board.");
  board = boost::dynamic_pointer_cast<ChessBoard1>(bp);
  if(!board) {
    PRINT(red, "[SREC] Wrong type of chessboard for this recognizer!");
    throw std::domain_error("");
  }
}

void ChessStateRec3::setCamera(boost::shared_ptr<Camera>& cp) {
  PRINT("[SREC] Setting camera.");
  cam = cp;
}

ChessStateRec3::PlaneTools::PlaneTools(float nx, float ny, float nz, float d)
  :nx(nx), ny(ny), nz(nz), d(d)
{}

Eigen::Vector3f ChessStateRec3::PlaneTools::project(Eigen::Vector3f pt) {
  return pt - (nx*pt(0) + ny*pt(1) + nz*pt(2) + d) * Eigen::Vector3f(nx,ny,nz);
}

bool ChessStateRec3::start() {
  PRINT("[SREC] Starting...");

  // set up camera streams
  cam->listenToCloudStream(false);
  cam->setCloudTopic("/camera/depth_registered/points");
  cam->listenToCloudStream(true);
  cam->listenToImageStream(false);

  // set up keyboard listener
  KeyListener keyboard;
  keyboard.start();

  typedef pcl::PointXYZRGB PointType;

  // loop over the incoming pointclouds until keyboard interruption
  while(ros::ok()) {

    // cleanup the viewer
    viewer.removeAllPointClouds();

    // get a cloud from the camera
    PRINT("[SREC] Fetching cloud.");
    pcl::PCLPointCloud2::ConstPtr cloud_blob = cam->getPclCloud();

    // convert from blob to uncompressed
    // -> enables to extract points
    pcl::PointCloud<PointType>::Ptr input_cloud(new pcl::PointCloud<PointType>());
    pcl::fromPCLPointCloud2(*cloud_blob, *input_cloud);
    PRINT("[SREC] Fetched cloud with " << input_cloud->points.size() << " points.");




    // ========================================================================
    // STEP 1:
    // crop out the bounding box around the board
    // ========================================================================
    pcl::PointCloud<PointType>::Ptr board_bbox_cloud(new pcl::PointCloud<PointType>());

    pcl::CropBox<PointType> crop;
    std::vector<cv::Point3f> reg = board->getRotatedBoundingBox();
    // need min and max points of boundingbox...
    // iterate over all components of the bounding-box-defining points
    // initialize with first point
    cv::Point3f board_bbox_min = reg[0];
    cv::Point3f board_bbox_max = reg[0];
    typedef std::vector<cv::Point3f>::iterator type;
    for(type it=reg.begin(); it != reg.end(); ++it) {
      board_bbox_min.x = std::min(board_bbox_min.x, (*it).x);
      board_bbox_min.y = std::min(board_bbox_min.y, (*it).y);
      board_bbox_min.z = std::min(board_bbox_min.z, (*it).z);
      board_bbox_max.x = std::max(board_bbox_max.x, (*it).x);
      board_bbox_max.y = std::max(board_bbox_max.y, (*it).y);
      board_bbox_max.z = std::max(board_bbox_max.z, (*it).z);
    }   
    crop.setMin(Eigen::Vector4f(board_bbox_min.x, board_bbox_min.y, board_bbox_min.z, 0));
    crop.setMax(Eigen::Vector4f(board_bbox_max.x, board_bbox_max.y, board_bbox_max.z, 0));
    // here we need the inverse...
    // apparently the filter transforms the cloud by the inverse transform and
    // then applies the cutoff filters
    crop.setTransform(board->transform.inverse());
    crop.setInputCloud(input_cloud);
    crop.filter(*board_bbox_cloud);
    PRINT("[SREC] Cropped out board. " << board_bbox_cloud->points.size() << "points left.");

    pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(board_bbox_cloud);
    viewer.addPointCloud<PointType>(board_bbox_cloud, rgb, "planepts");





    // ========================================================================
    // STEP 2:
    // remove the largest plane
    // ========================================================================

    // Estimate point normals
    pcl::NormalEstimation<PointType, pcl::Normal> ne;
    pcl::search::KdTree<PointType>::Ptr plane_search_tree(new pcl::search::KdTree<PointType>());
    pcl::PointCloud<pcl::Normal>::Ptr board_bbox_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setSearchMethod (plane_search_tree);
    ne.setInputCloud (board_bbox_cloud);
    ne.setKSearch (50);
    ne.compute (*board_bbox_normals);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentationFromNormals<PointType, pcl::Normal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.04);
    seg.setInputCloud (board_bbox_cloud);
    seg.setInputNormals (board_bbox_normals);
    // Obtain the plane inliers and coefficients
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    seg.segment (*inliers_plane, *coefficients_plane);

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointType> extractor;
    extractor.setInputCloud (board_bbox_cloud);
    extractor.setIndices (inliers_plane);

    // Remove the planar inliers, extract the rest
    pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());
    extractor.setNegative (true);
    extractor.filter (*filtered);
    pcl::ExtractIndices<pcl::Normal> extractor_normals;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    extractor_normals.setNegative (true);
    extractor_normals.setInputCloud (board_bbox_normals);
    extractor_normals.setIndices (inliers_plane);
    extractor_normals.filter (*normals);
    PRINT("[SREC] Removed " << inliers_plane->indices.size() << "points from plane. " << filtered->points.size() << " points left to look for pieces." );

    // init plane-tools for later use
    PlaneTools plane(
        coefficients_plane->values[0],
        coefficients_plane->values[1],
        coefficients_plane->values[2],
        coefficients_plane->values[3]
        );




    // ========================================================================
    // STEP 3:
    // clustering of the remaining cloud
    // ========================================================================

    pcl::search::KdTree<PointType>::Ptr cluster_tree(new pcl::search::KdTree<PointType>);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    cluster_tree->setInputCloud(filtered);
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(cluster_tree);
    ec.setInputCloud(filtered);
    ec.extract(cluster_indices);

    if(cluster_indices.size() == 0) {
      PRINT(red, "[SREC] No clusters found.");
    } else {
      PRINT(green, "[SREC] Found " << cluster_indices.size() << " clusters.");
    }




    // ========================================================================
    // STEP 4:
    // process clusters
    // ========================================================================

    // loop over all clusters and keep an id
    int cl_id = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {

      // string from id for naming pointclouds in viewer
      std::stringstream ss;
      ss << cl_id;
      std::string cl_idstr = ss.str();


      // =========================
      // a) get cluster base point
      // =========================

      // extract clusterpoints to own pointcloud
      // simultaneously find the cluster center
      pcl::PointCloud<PointType>::Ptr cloud_cluster (new pcl::PointCloud<PointType>);
      Eigen::Vector3f avgpt;
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {
        // extract
        cloud_cluster->points.push_back (filtered->points[*pit]);
        // sum points for averaging
        PointType tmppt = filtered->points[*pit];
        avgpt += Eigen::Vector3f(tmppt.x, tmppt.y, tmppt.z);
      }
      // avg: devide bu numper of points
      avgpt *= 1.0/cloud_cluster->points.size();
      // set header for extracted cloud
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      // basepoint: project center onto plane
      Eigen::Vector3f base = plane.project(avgpt);


      // =======================================
      // b) optimize base point for known radius
      // =======================================

      float radius = 0.0175;
      
      // project the view direction on the plane
      // view direction = base - nullvector ( = base)
      // projected view direction = base - projected nullvector
      Eigen::Vector3f null_proj = plane.project(Eigen::Vector3f(0,0,0));
      Eigen::Vector3f view_dir_proj = base - null_proj;
      // normalize
      view_dir_proj *= 1.0/view_dir_proj.norm();

      // calculate weighted average of the vectors of all cluster-inliers to the base
      // (everything projected onto the plane)
      Eigen::Vector3f avgvec;
      // keep track of the weights for normalization
      float weightsum = 0;
      for(pcl::PointCloud<PointType>::iterator cit = cloud_cluster->begin();
          cit != cloud_cluster->end();
          cit++) {
        // get cluster-point and project onto plane
        Eigen::Vector3f pt(cit->x, cit->y, cit->z);
        Eigen::Vector3f projpt = plane.project(pt);
        // get the direction to the base
        Eigen::Vector3f direction = base - projpt;
        // weight by the cosine of the angle between the direction-vector for this point
        // and the projected view direction
        // cos(phi) = (a dot b )/(a.norm() * b.norm())
        float weight = view_dir_proj.dot(direction)/direction.norm();
        // ignore the negative part (i.e. angles of less than PI or more than -PI)
        weight = std::max(0.0f, weight);
        avgvec += weight * direction;
        // keep track of the weights for normalization
        weightsum += weight;
      }
      avgvec = avgvec / weightsum;

      // calculate new base such that it might have the known radius
      // as distance to the front of the pointcloud
      Eigen::Vector3f newBase = base + (radius - avgvec.norm())* view_dir_proj;




      // ======================================================================
      // VISULIZATION
      // ======================================================================

      pcl::PointCloud<PointType>::Ptr base_cloud (new pcl::PointCloud<PointType>);
      PointType tmppt;
      tmppt.x = base(0);
      tmppt.y = base(1);
      tmppt.z = base(2);
      base_cloud->push_back(tmppt);
      pcl::visualization::PointCloudColorHandlerCustom<PointType> basecol(base_cloud, 255,0,0);
      viewer.addPointCloud<PointType>(base_cloud, basecol, "base"+cl_idstr); 
      viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "base"+cl_idstr);

      pcl::PointCloud<PointType>::Ptr newBase_cloud (new pcl::PointCloud<PointType>);
      tmppt.x = newBase(0);
      tmppt.y = newBase(1);
      tmppt.z = newBase(2);
      newBase_cloud->push_back(tmppt);
      pcl::visualization::PointCloudColorHandlerCustom<PointType> newBasecol(newBase_cloud, 0,255,0);
      viewer.addPointCloud<PointType>(newBase_cloud, newBasecol, "newBase"+cl_idstr); 
      viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "newBase"+cl_idstr);

      int r = rand() % 256;
      int g = rand() % 256;
      int b = rand() % 256;
      pcl::visualization::PointCloudColorHandlerCustom<PointType> tmpcol(cloud_cluster, r,g,b);
      viewer.addPointCloud<PointType>(cloud_cluster, tmpcol, "tmp"+cl_idstr); 
      cl_id++;
    }


    // show grid
    pcl::PointCloud<PointType>::Ptr grid(new
        pcl::PointCloud<PointType>);
    for(int x=0; x<=50; x++) {
      for(int y=0; y<=40; y++) {
        for(int z=0; z<5; z++) {
          if(x%10==0 || y%10==0) {
            Eigen::Vector3f lpt = board->transp(x/10.0,y/10.0,z/10.0);
            PointType tmp;
            tmp.x = lpt(0);
            tmp.y = lpt(1);
            tmp.z = lpt(2);
            grid->push_back(tmp);
          }
        }
      }
    }
    pcl::visualization::PointCloudColorHandlerCustom<PointType> gridcol(grid, 0,200,200);
    viewer.addPointCloud<PointType>(grid, gridcol, "grid");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "grid");

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
