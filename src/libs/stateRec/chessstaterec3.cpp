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

float ChessStateRec3::PlaneTools::distance(Eigen::Vector3f pt) {
  return (nx*pt(0) + ny*pt(1) + nz*pt(2) + d);
}

Eigen::Vector3f ChessStateRec3::PlaneTools::project(Eigen::Vector3f pt) {
  return pt - distance(pt) * Eigen::Vector3f(nx,ny,nz);
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

  // loop over the incoming pointclouds until keyboard interruption
  while(ros::ok()) {

    // check for stability
    int window_size = 4;
    boost::circular_buffer<Eigen::Vector3f> normalVector_buffer(window_size);
    boost::circular_buffer<std::map<std::string, int> > states_buffer(window_size);
    boost::circular_buffer<std::map<std::string, Eigen::Vector3f> > grabPoints_buffer(window_size);

    bool stable = false;
    while(ros::ok() && !stable) {



      // init cluster-field-assignment with empty lists
      std::map<std::string, std::vector<Cluster> > field_cluster_map;
      std::map<std::string, std::vector<cv::Point3f> > board_layout = board->getRotatedLayout();
      for(std::map<std::string, std::vector<cv::Point3f> >::iterator it = board_layout.begin();
          it != board_layout.end();
          ++it){
        field_cluster_map[it->first] = std::vector<Cluster>();
      }

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

      // plane coefficients:
      float plane_nx = coefficients_plane->values[0];
      float plane_ny = coefficients_plane->values[1];
      float plane_nz = coefficients_plane->values[2];
      float plane_d = coefficients_plane->values[3];

      // init plane-tools for later use
      PlaneTools plane(plane_nx, plane_ny, plane_nz, plane_d);

      // normal vector
      Eigen::Vector3f normalVector(plane_nx, plane_ny, plane_nz);




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

      float height = 0.11;

      // loop over all clusters
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end(); ++it) {


        // create cluster and calculate all relevant points
        Cluster c;
        c.indices = (*it);
        c.refcloud = filtered;
        c.plane = plane;
        c.analyse();

        // omit cluster, if not high enough
        // 1/4 as factor ensures that all clusters with a hight of more than half a
        // can will be recognised
        if(plane.distance(c.avgpt) < 1.0/4.0 * height) continue;

        // match cluster to a region and store it
        for(std::map<std::string, std::vector<cv::Point3f> >::iterator it_reg = board_layout.begin();
            it_reg != board_layout.end();
            ++it_reg){
          if(pointInPolygon(board->transform.inverse() * c.newBase, it_reg->second)) {
            field_cluster_map[it_reg->first].push_back(c);
          }
        }
      }

      // ========================================================================
      // STEP 5:
      // merge clusters within regions
      // get state and grab point
      // ========================================================================

      std::map<std::string, int> states;
      std::map<std::string, Eigen::Vector3f> grabPoints; 

      // loop over regions
      for(std::map<std::string, std::vector<Cluster> >::iterator it_reg = field_cluster_map.begin();
          it_reg != field_cluster_map.end();
          ++it_reg){

        std::string name = it_reg->first;
        std::vector<Cluster> clusters = it_reg->second;
        Cluster c;

        // if no clusters found (field is empty)
        if(clusters.size() == 0) {

          // state=0 means no piece
          states[name] = 0;

          // calculate center of region
          std::vector<cv::Point3f> region = board->getRotatedTransformedRegion(name);
          cv::Point3f center;
          for(std::vector<cv::Point3f>::iterator it_regpt = region.begin(); it_regpt != region.end(); ++it_regpt) {
            center += (*it_regpt);
          }
          center *= 1.0/region.size();
          Eigen::Vector3f center_eig(center.x, center.y, center.z);
          Eigen::Vector3f center_plane = plane.project(center_eig);
          // grabpoint is at the top of a centered piece
          grabPoints[name] = center_plane + height * normalVector;

        } else {

          // copy header informaion from first cluster 
          pcl::PointIndices ptIndices = clusters[0].indices;
          // loop over clusters and merge indices-vectors
          std::vector<int> ind;
          for(std::vector<Cluster>::iterator it_cl = clusters.begin();
              it_cl != clusters.end();
              ++it_cl) {
            std::vector<int> mergeInd;
            std::vector<int> newInd = it_cl->indices.indices;
            mergeInd.reserve(ind.size() + newInd.size());
            mergeInd.insert(mergeInd.end(), ind.begin(), ind.end());
            mergeInd.insert(mergeInd.end(), newInd.begin(), newInd.end());
            ind = mergeInd;
          }
          // substitute the indices-data
          ptIndices.indices = ind;

          // create merged cluster and calculate new data
          c.indices = ptIndices;
          c.plane = plane;
          c.refcloud = filtered;
          c.analyse();

          // state=1 means piece found
          states[name] = 1;

          // grabpoint is at the top of the centered piece
          grabPoints[name] = c.newBase + height * normalVector;
        }





        // =============
        // VISUALIZATION
        // =============
        std::string cl_idstr = it_reg->first;
        PointType tmppt;
        // newbase
        pcl::PointCloud<PointType>::Ptr newBase_cloud (new pcl::PointCloud<PointType>);
        tmppt.x = grabPoints[name](0);
        tmppt.y = grabPoints[name](1);
        tmppt.z = grabPoints[name](2);
        newBase_cloud->push_back(tmppt);
        pcl::visualization::PointCloudColorHandlerCustom<PointType> newBasecol(newBase_cloud, 0,255,0);
        viewer.addPointCloud<PointType>(newBase_cloud, newBasecol, "newBase"+cl_idstr); 
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "newBase"+cl_idstr);

        if(states[name] == 1) {
          // base
          pcl::PointCloud<PointType>::Ptr base_cloud (new pcl::PointCloud<PointType>);
          tmppt.x = c.avgpt(0);
          tmppt.y = c.avgpt(1);
          tmppt.z = c.avgpt(2);
          base_cloud->push_back(tmppt);
          pcl::visualization::PointCloudColorHandlerCustom<PointType> basecol(base_cloud, 255,0,0);
          viewer.addPointCloud<PointType>(base_cloud, basecol, "base"+cl_idstr); 
          viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "base"+cl_idstr);
          // cluster
          pcl::PointCloud<PointType>::Ptr cloud_cluster(new pcl::PointCloud<PointType>());
          extractor.setInputCloud(filtered);
          extractor.setNegative(false);
          pcl::PointIndices::Ptr ind_ptr(new pcl::PointIndices(c.indices));
          extractor.setIndices(ind_ptr);
          extractor.filter(*cloud_cluster);
          int r = rand() % 256;
          int g = rand() % 256;
          int b = rand() % 256;
          pcl::visualization::PointCloudColorHandlerCustom<PointType> tmpcol(cloud_cluster, r,g,b);
          viewer.addPointCloud<PointType>(cloud_cluster, tmpcol, "tmp"+cl_idstr);
        }
      }



      // show grid
      pcl::PointCloud<PointType>::Ptr grid(new pcl::PointCloud<PointType>);
      pcl::PointCloud<PointType>::Ptr field_cloud (new pcl::PointCloud<PointType>);
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

      // push results in buffer
      states_buffer.push_back(states);
      normalVector_buffer.push_back(normalVector);
      grabPoints_buffer.push_back(grabPoints);

      // if buffer full -> stable?
      if(states_buffer.full()) {
        PRINT("[SREC] Buffer full."); 
        stable = true;
        typedef boost::circular_buffer<std::map<std::string, int> >::iterator buftype;
        typedef std::map<std::string, std::vector<cv::Point3f> >::iterator regtype;
        for(regtype it_reg = board->regions.begin(); it_reg != board->regions.end(); ++it_reg) {
          std::string name = it_reg->first;
          int checkstate = states_buffer[0][name];
          for(buftype it_buf = states_buffer.begin(); it_buf != states_buffer.end(); ++it_buf) {
            if(checkstate != (*it_buf)[name]) stable = false;
          }
        }
        if(stable) {
          PRINT(green, "[SREC] Stable."); 
          board->print(states);
        } else {
          PRINT(red, "[SREC] Not stable! Continuing..."); 
        }

      } else {
        PRINT("[SREC] Collecting samples..."); 
      }

    }

    viewer.spinOnce(100);
    char c;
    while(ros::ok()) {
      viewer.spinOnce(1);
      c = keyboard.getChar();
      if(c == 'c' || c == 'q' || c == 'r') break;
    }
    if(c == 'q') return true;
    if(c == 'r') return false;


  }
}

bool ChessStateRec3::pointInPolygon(Eigen::Vector3f pt, std::vector<cv::Point3f> poly) {
  int t = -1;
  typedef std::vector<cv::Point3f>::iterator type;
  for(type it = poly.begin(); it != poly.end(); it++) {
    type it2 = it + 1;
    if(it2 == poly.end()) it2 = poly.begin();
    t *= ptTest(pt(0), pt(1), it->x, it->y, it2->x, it2->y);  
  }
  return t == 1;
}

int ChessStateRec3::ptTest(float xa, float ya, float xb, float yb, float xc, float yc) {
  if(ya == yb && yb == yc) {
    if( (xb <= xa && xa <= xc) || (xc <= xa && xa <= xb) )
      return 0;
    else
      return 1;
  }
  if(yb > yc) {
    float tmpx = xb;
    float tmpy = yb;
    xb = xc;
    yb = yc;
    xc = tmpx;
    yc = tmpy;
  }
  if(ya == yb && xa == xb)
    return 0;
  if(ya <= yb || ya > yc)
    return 1;
  float delta = (xb - xa)*(yc - ya) - (yb - ya)*(xc - xa);
  if(delta > 0)
    return -1;
  if(delta < 0)
    return 1;
  return 0;
}


void ChessStateRec3::Cluster::analyse() {

  std::vector<int> ind = indices.indices;

  // =========================
  // a) get cluster base point
  // =========================

  // find the cluster center
  for (std::vector<int>::const_iterator pit = ind.begin (); pit != ind.end (); pit++) {
    // sum points for averaging
    PointType tmppt = refcloud->points[*pit];
    avgpt += Eigen::Vector3f(tmppt.x, tmppt.y, tmppt.z);
  }
  // avg: devide bu numper of points
  avgpt *= 1.0/ind.size();
  // basepoint: project center onto plane
  base = plane.project(avgpt);


  // =======================================
  // b) optimize base point for known radius
  // =======================================
  //
  // Since the camera detects only one side of the cylindrical pieces,
  // the calculated center-point will be shifted to the camera. With the
  // knowledge of the real radius of the pieces, it is possible to project
  // the base point away from the camera, such that it resembles the real
  // base more accurately.

  float radius = 0.0175;

  // project the view direction on the plane
  // view direction = base - nullvector ( = base)
  // projected view direction = base - projected nullvector
  Eigen::Vector3f null_proj = plane.project(Eigen::Vector3f(0,0,0));
  Eigen::Vector3f view_dir_proj = base - null_proj;
  // normalize
  view_dir_proj *= 1.0/view_dir_proj.norm();

  // calculate weighted average of the distance between the cluster-inliers to the base
  // weighted by the cosine of the angle between the directions
  // (everything projected onto the plane)
  float avgdist;
  // keep track of the weights for normalization
  float weightsum = 0;
  for (std::vector<int>::const_iterator pit = ind.begin (); pit != ind.end (); pit++) {
    PointType cpt = refcloud->points[*pit];
    // get cluster-point and project onto plane
    Eigen::Vector3f pt(cpt.x, cpt.y, cpt.z);
    Eigen::Vector3f projpt = plane.project(pt);
    // get the direction to the base
    Eigen::Vector3f direction = base - projpt;
    // weight by the cosine of the angle between the direction-vector for this point
    // and the projected view direction
    // cos(phi) = (a dot b )/(a.norm() * b.norm())
    float weight = view_dir_proj.dot(direction)/direction.norm();
    // ignore the negative part (i.e. angles of less than PI or more than -PI)
    weight = std::max(0.0f, weight);
    avgdist += weight * view_dir_proj.dot(direction);
    // keep track of the weights for normalization
    weightsum += weight;
  }
  avgdist = avgdist / weightsum;

  // calculate new base such that it might have the known radius
  // as distance to the front of the pointrefcloud
  newBase = base + (radius - avgdist)* view_dir_proj;
}
