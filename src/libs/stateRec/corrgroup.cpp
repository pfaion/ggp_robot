// corrgroup.cpp //
#include <ggp_robot/libs/stateRec/corrgroup.h>

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

// TODO: clean up correspondence grouping includes
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>


CorrGroup::CorrGroup()
  : viewer("test"),
  model(new pcl::PointCloud<pcl::PointXYZRGB>)
{
  PRINT("[SREC] Recognizer initializing...");

  viewer.setBackgroundColor(1,1,1);

  // generate model point cloud
  float xmax = 40;
  float ymax = 40;
  float zmax = 85;
  float scale = 0.001;
  for(float x=0; x<=xmax; x+=5) {
    for(float y=0; y<=ymax; y+=5) {
      for(float z=0; z<=zmax; z+=5) {
        pcl::PointXYZRGB pt_tmp;
        pt_tmp.r = 200;
        pt_tmp.g = 0;
        pt_tmp.b = 0;
        // one coordinate has to be at a border
        if(x==0 || y == 0 || z==0 || x==xmax || y==ymax || z==zmax) {
          pcl::PointXYZRGB pt(pt_tmp);
          pt.x = scale * x; 
          pt.y = scale * y; 
          pt.z = scale * z;
          model->push_back(pt);
        }
      }
    }
  }
}

void CorrGroup::setBoard(boost::shared_ptr<PlanarBoard>& bp) {
  PRINT("[SREC] Setting board.");
  board = boost::dynamic_pointer_cast<ChessBoard1>(bp);
  if(!board) {
    PRINT(red, "[SREC] Wrong type of chessboard for this recognizer!");
    throw std::domain_error("");
  }
}

void CorrGroup::setCamera(boost::shared_ptr<Camera>& cp) {
  PRINT("[SREC] Setting camera.");
  cam = cp;
}

bool CorrGroup::start() {
  PRINT("[SREC] Starting...");
  cam->listenToCloudStream(false);
  cam->setCloudTopic("/camera/depth_registered/points");
  cam->listenToCloudStream(true);
  cam->listenToImageStream(false);

  KeyListener keyboard;
  keyboard.start();

  while(ros::ok()) {



    // PCL CORRESPONDENCE GROUPING TUTORIAL
    typedef pcl::PointXYZRGB PointType;
    typedef pcl::Normal NormalType;
    typedef pcl::ReferenceFrame RFType;
    typedef pcl::SHOT352 DescriptorType;
    float model_ss_ (0.01f);
    float scene_ss_ (0.03f);
    float rf_rad_ (0.015f);
    float descr_rad_ (0.02f);
    float cg_size_ (0.01f);
    float cg_thresh_ (5.0f);
    pcl::PointCloud<PointType>::Ptr model_keypoints (new
        pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene_keypoints (new
        pcl::PointCloud<PointType> ());
    pcl::PointCloud<NormalType>::Ptr model_normals (new
        pcl::PointCloud<NormalType> ());
    pcl::PointCloud<NormalType>::Ptr scene_normals (new
        pcl::PointCloud<NormalType> ());
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new
        pcl::PointCloud<DescriptorType> ());
    pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new
        pcl::PointCloud<DescriptorType> ());


    PRINT("[SREC] Fetching cloud.");
    pcl::PCLPointCloud2::ConstPtr cloud = cam->getPclCloud();
    pcl::PointCloud<PointType>::Ptr pc (new pcl::PointCloud<PointType> ());
    pcl::fromPCLPointCloud2(*cloud, *pc);

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
    crop.filter(*scene);



    //  Set up resolution invariance
    struct {
      double
        computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
        {
          double res = 0.0;
          int n_points = 0;
          int nres;
          std::vector<int> indices (2);
          std::vector<float> sqr_distances (2);
          pcl::search::KdTree<PointType> tree;
          tree.setInputCloud (cloud);

          for (size_t i = 0; i < cloud->size (); ++i)
          {
            if (! pcl_isfinite ((*cloud)[i].x))
            {
              continue;
            }
            //Considering the second neighbor since the first is the point itself.
            nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
            if (nres == 2)
            {
              res += sqrt (sqr_distances[1]);
              ++n_points;
            }
          }
          if (n_points != 0)
          {
            res /= n_points;
          }
          return res;
        }
    } local;

    if(false) {
      float resolution = static_cast<float> (local.computeCloudResolution (model));
      if (resolution != 0.0f)
      {
        model_ss_   *= resolution;
        scene_ss_   *= resolution;
        rf_rad_     *= resolution;
        descr_rad_  *= resolution;
        cg_size_    *= resolution;
      }

      std::cout << "Model resolution:       " << resolution << std::endl;
      std::cout << "Model sampling size:    " << model_ss_ << std::endl;
      std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
      std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
      std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
      std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
    }

    //  Compute Normals
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    norm_est.setKSearch (10);
    norm_est.setInputCloud (model);
    norm_est.compute (*model_normals);

    norm_est.setInputCloud (scene);
    norm_est.compute (*scene_normals);

    //  Downsample Clouds to Extract keypoints
    pcl::PointCloud<int> sampled_indices;

    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud (model);
    uniform_sampling.setRadiusSearch (model_ss_);
    uniform_sampling.compute (sampled_indices);
    pcl::copyPointCloud (*model, sampled_indices.points, *model_keypoints);
    std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

    uniform_sampling.setInputCloud (scene);
    uniform_sampling.setRadiusSearch (scene_ss_);
    uniform_sampling.compute (sampled_indices);
    pcl::copyPointCloud (*scene, sampled_indices.points, *scene_keypoints);
    std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;

    //  Compute Descriptor for keypoints
    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
    descr_est.setRadiusSearch (descr_rad_);

    descr_est.setInputCloud (model_keypoints);
    descr_est.setInputNormals (model_normals);
    descr_est.setSearchSurface (model);
    descr_est.compute (*model_descriptors);

    descr_est.setInputCloud (scene_keypoints);
    descr_est.setInputNormals (scene_normals);
    descr_est.setSearchSurface (scene);
    descr_est.compute (*scene_descriptors);



    //  Find Model-Scene Correspondences with KdTree
    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

    pcl::KdTreeFLANN<DescriptorType> match_search;
    match_search.setInputCloud (model_descriptors);

    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
    for (size_t i = 0; i < scene_descriptors->size (); ++i)
    {
      std::vector<int> neigh_indices (1);
      std::vector<float> neigh_sqr_dists (1);
      if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
      {
        continue;
      }
      int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
      if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
      {
        pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
        model_scene_corrs->push_back (corr);
      }
    }
    std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;







    char c = keyboard.getChar();
    if(c == 'q') return true;
    if(c == 'r') return false;

  }
}
