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

    // set up all relevant pointclouds
    pcl::PointCloud<PointType>::Ptr input_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr board_bbox_cloud(new pcl::PointCloud<PointType>());



    // cleanup the viewer
    viewer.removeAllPointClouds();

    // get a cloud from the camera and convert to uncompressed
    // -> enables to extract points
    PRINT("[SREC] Fetching cloud.");
    pcl::PCLPointCloud2::ConstPtr cloud_blob = cam->getPclCloud();
    pcl::fromPCLPointCloud2(*cloud_blob, *input_cloud);


    pcl::CropBox<PointType> crop(true);
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
    crop.setInputCloud(input_cloud);
    crop.filter(*board_bbox_cloud);




    // PCL Cylinder Segmentation Tutorial:

    // Estimate point normals
    pcl::NormalEstimation<PointType, pcl::Normal> ne;
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setSearchMethod (tree);
    ne.setInputCloud (board_bbox_cloud);
    ne.setKSearch (50);
    ne.compute (*normals);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentationFromNormals<PointType, pcl::Normal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.04);
    seg.setInputCloud (board_bbox_cloud);
    seg.setInputNormals (normals);
    // Obtain the plane inliers and coefficients
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    seg.segment (*inliers_plane, *coefficients_plane);

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud (board_bbox_cloud);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);
    pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(board_bbox_cloud);
    viewer.addPointCloud<PointType>(board_bbox_cloud, rgb, "planepts");
    //viewer.addPointCloud<PointType>(board_bbox_cloud, "planepts");

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




    // ====================================================
    // start cylinder segmentation

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.1);


    int cylid = 0;
    int iteration = 0;
    while(cylid<4 && iteration < 20 && ros::ok()) {
      iteration++;

      std::stringstream ss;
      ss << cylid;
      std::string cylidstr = ss.str();

      PRINT(cyan, "[SREC] looking for cylinders in cloud size: " << filtered2->size());

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

      if(cloud_cylinder->size() != 0) {
        cylid++;

        pcl::visualization::PointCloudColorHandlerCustom<PointType>
          lightRed(cloud_cylinder, 255,130,130);
        viewer.addPointCloud<PointType>(cloud_cylinder, lightRed,
            "cylinderpts"+cylidstr);
        pcl::visualization::PointCloudColorHandlerCustom<PointType>
          red(cloud_cylinder, 255,0,0);

        //extract.setNegative(true);
        //pcl::PointCloud<PointType>::Ptr filtered3(new pcl::PointCloud<PointType>());
        //extract.filter(*filtered3);
        //viewer.addPointCloud<PointType>(filtered3, "rest");


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
        viewer.addPointCloud<PointType>(intersect, green, "intersect"+cylidstr);
        viewer.setPointCloudRenderingProperties
          (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "intersect"+cylidstr);


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
        float radius = 0.0175;
        float height = 0.11;
        float radius_scaling = radius - coefficients_cylinder->values[6];
        Eigen::Vector3f newBase_raw = Eigen::Vector3f(pt.x, pt.y, pt.z) + radius_scaling * vec;

        // now project new base onto plane
        float distanceToPlane = nx*newBase_raw(0) + ny*newBase_raw(1) +
          nz*newBase_raw(2) + p;
        Eigen::Vector3f newBase = newBase_raw - distanceToPlane * Eigen::Vector3f(nx,ny,nz);



        // extract region around cylinder

        // get points for boundingbox in boardcoords
        Eigen::Vector3f newBase_board = board->transform.inverse() * newBase;
        // all within 1cm is considered noise
        float noise_offset = 0.01;
        Eigen::Vector3f bbox_pt1 = newBase_board
          + (radius + noise_offset)*Eigen::Vector3f(1,0,0)
          + (radius + noise_offset)*Eigen::Vector3f(0,1,0)
          + (height + noise_offset)*Eigen::Vector3f(0,0,1);
        Eigen::Vector3f bbox_pt2 = newBase_board
          - (radius + noise_offset)*Eigen::Vector3f(1,0,0)
          - (radius + noise_offset)*Eigen::Vector3f(0,1,0)
          - (noise_offset)*Eigen::Vector3f(0,0,1);

        Eigen::Vector4f bbox_min;
        bbox_min(0) = std::min(bbox_pt1(0), bbox_pt2(0));
        bbox_min(1) = std::min(bbox_pt1(1), bbox_pt2(1));
        bbox_min(2) = std::min(bbox_pt1(2), bbox_pt2(2));
        Eigen::Vector4f bbox_max;
        bbox_max(0) = std::max(bbox_pt1(0), bbox_pt2(0));
        bbox_max(1) = std::max(bbox_pt1(1), bbox_pt2(1));
        bbox_max(2) = std::max(bbox_pt1(2), bbox_pt2(2));


        crop.setMin(bbox_min);
        crop.setMax(bbox_max);
        crop.setTransform(board->transform.inverse());
        crop.setInputCloud(filtered2);
        crop.setNegative(true);
        crop.filter(*filtered2);

        pcl::PointIndices::Ptr inliers_bbox(new pcl::PointIndices);
        crop.getRemovedIndices(*inliers_bbox);

        extract_normals.setNegative (true);
        extract_normals.setInputCloud (normals2);
        extract_normals.setIndices (inliers_bbox);
        extract_normals.filter (*normals2);








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
        viewer.addPointCloud<PointType>(base, basecol, "base"+cylidstr);
        viewer.setPointCloudRenderingProperties
          (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "base"+cylidstr);

        pcl::PointCloud<PointType>::Ptr line(new
            pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr line2(new
            pcl::PointCloud<PointType>);



        // PRINT CYLINDER

        // find a vector perpendicular to the planes normal-vector (means it lies
        // in the plane)
        // -> set a nonzero coeffient to 0, change the other two and multiply one
        // of the other two by -1
        Eigen::Vector3f plane_vec;
        {
          int i=0;
          Eigen::Vector3f n(nx, ny, nz);
          while(n(i)==0 && i<3) i++;
          plane_vec(i) = 0;
          plane_vec((i+1)%3) = -n((i+2)%3);
          plane_vec((i+2)%3) = n((i+1)%3);
          plane_vec *= radius / std::sqrt(plane_vec(0)*plane_vec(0) +
              plane_vec(1)*plane_vec(1) + plane_vec(2)*plane_vec(2));
        }
        for(int i=0; i<150; i+=2) {
          Eigen::Vector3f starting_pt = newBase + plane_vec;
          float x = starting_pt(0);
          float y = starting_pt(1);
          float z = starting_pt(2);
          float a = newBase(0);
          float b = newBase(1);
          float c = newBase(2);
          float u = nx;
          float v = ny;
          float w = nz;
          for(int s=0; s<20; s++){
            // theta = angle of rotation
            float t = s/20.0 * 2 * 3.1415;
            PointType surf_pt;
            // roate point around axis
            surf_pt.x = i/1000.0*nx +
              (a*(v*v+w*w)-u*(b*v+c*w-u*x-v*y-w*z))*(1-cos(t))+x*cos(t)+(-c*v+b*w-w*y+v*z)*sin(t);
            surf_pt.y = i/1000.0*ny +
              (b*(u*u+w*w)-v*(a*u+c*w-u*x-v*y-w*z))*(1-cos(t))+y*cos(t)+(c*u-a*w+w*x-u*z)*sin(t);
            surf_pt.z = i/1000.0*nz +
              (c*(u*u+v*v)-w*(a*u+b*v-u*x-v*y-w*z))*(1-cos(t))+z*cos(t)+(-b*u+a*v-v*x+u*y)*sin(t);

            line->push_back(surf_pt);
          }
        }


        pcl::visualization::PointCloudColorHandlerCustom<PointType>
          tmpcol(line, 200,0,200);
        viewer.addPointCloud<PointType>(line, tmpcol, "tmp"+cylidstr);
        viewer.setPointCloudRenderingProperties
          (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tmp"+cylidstr);
        pcl::visualization::PointCloudColorHandlerCustom<PointType>
          tmpcol2(line2, 0,200,200);
        viewer.addPointCloud<PointType>(line2, tmpcol2, "tmp2"+cylidstr);
        viewer.setPointCloudRenderingProperties
          (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tmp2"+cylidstr);

      }

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
    pcl::visualization::PointCloudColorHandlerCustom<PointType>
      gridcol(grid, 0,200,200);
    viewer.addPointCloud<PointType>(grid, gridcol, "grid");
    viewer.setPointCloudRenderingProperties
      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "grid");

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
