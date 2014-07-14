#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <visualization_msgs/Marker.h>


ros::Publisher pub_cloud;
ros::Publisher pub_cloud_proj;
ros::Publisher pub_cloud_subsample;
ros::Publisher pub_point_minmax;




  void 
cloud_cb (const sensor_msgs::PointCloud2& input)
{
  // conversions
  pcl::PCLPointCloud2::Ptr pc2(new pcl::PCLPointCloud2());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl_conversions::toPCL(input, *pc2);
  pcl::fromPCLPointCloud2(*pc2, *pc);

  std::cout << pc->isOrganized() << std::endl;

  // passThrough filtering
  // cutoff in z-direction (front/back)
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (pc);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (1.0, 2.0);
  pass.filter (*pc);
  // cutoff in x-direction (left/right)
  pass.setInputCloud (pc);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-0.2, 0.5);
  pass.filter (*pc);






  // plane-segmentation
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // configure segmentation object
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  // segment
  seg.setInputCloud (pc);
  seg.segment (*inliers, *coefficients);
  // check if segment was found
  if (inliers->indices.size () == 0)
  {
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    return;
  }

  // extract pointcloud of segment
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_segment(new pcl::PointCloud<pcl::PointXYZRGB>());
  extract.setInputCloud (pc);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*pc_segment);



  
  pcl::toPCLPointCloud2(*pc_segment, *pc2);
  sensor_msgs::PointCloud2 pcmsg;
  pcl_conversions::fromPCL(*pc2,pcmsg);
  pub_cloud.publish(pcmsg);



  // print points
  if(false) {
  for (size_t i = 0; i < pc_segment->points.size (); ++i) {
    std::cerr << "    " << (int)pc_segment->points[i].r << " " 
      << (int)pc_segment->points[i].g << " " 
      << (int)pc_segment->points[i].b << std::endl;
  }
  }

  // print indices
  if(false) {
    for(std::vector<int>::iterator it = inliers->indices.begin(); it != inliers->indices.end(); ++it) {
      std::cout << *it << std::endl;
    }
  }

  

  // project inliers onto model plane
  pcl::ProjectInliers<pcl::PointXYZRGB> projection;
  projection.setModelType(pcl::SACMODEL_PLANE);
  projection.setInputCloud(pc_segment);
  projection.setModelCoefficients(coefficients);
  projection.filter(*pc_segment);

  std::cout << pc->isOrganized() << std::endl;

  // min/max stuff
  pcl::PointXYZRGB proj_min;
  pcl::PointXYZRGB proj_max;
  pcl::getMinMax3D(*pc_segment, proj_min, proj_max);

  if(true) {
    std::cout << "min: (" << (float) proj_min.x << "|" << (float) proj_min.y << "|" << (float) proj_min.z << ")" << std::endl;
    std::cout << "max: (" << (float) proj_max.x << "|" << (float) proj_max.y << "|" << (float) proj_max.z << ")" << std::endl;
  }

  visualization_msgs::Marker min_marker;
  min_marker.header.frame_id = "/camera_rgb_optical_frame";
  min_marker.header.stamp = ros::Time::now();
  min_marker.ns = "minmax_marker";
  min_marker.id = 0;
  min_marker.type = visualization_msgs::Marker::CUBE;
  min_marker.pose.position.x = proj_min.x;
  min_marker.pose.position.y = proj_min.y;
  min_marker.pose.position.z = proj_min.z;
  min_marker.pose.orientation.x = 0.0;
  min_marker.pose.orientation.y = 0.0;
  min_marker.pose.orientation.z = 0.0;
  min_marker.pose.orientation.w = 1.0;
  min_marker.scale.x = 0.05;
  min_marker.scale.y = 0.05;
  min_marker.scale.z = 0.05;
  min_marker.color.r = 1.0f;
  min_marker.color.g = 0.0f;
  min_marker.color.b = 0.0f;
  min_marker.color.a = 0.5f;
  min_marker.lifetime = ros::Duration();
  pub_point_minmax.publish(min_marker);

  visualization_msgs::Marker max_marker;
  max_marker = min_marker;
  max_marker.id = 1;
  max_marker.pose.position.x = proj_max.x;
  max_marker.pose.position.y = proj_max.y;
  max_marker.pose.position.z = proj_max.z;
  pub_point_minmax.publish(max_marker);

  // convert to PCL PointCloud2
  pcl::toPCLPointCloud2(*pc_segment, *pc2);


  // subsampling with VoxelGrid
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(pc2);
  sor.setLeafSize(0.01, 0.01, 0.01);
  pcl::PCLPointCloud2::Ptr pc2_subsample(new pcl::PCLPointCloud2());
  sor.filter(*pc2_subsample);


  // convert clouds to sensor_msg
  sensor_msgs::PointCloud2 pcmsg_proj;
  pcl_conversions::fromPCL(*pc2,pcmsg_proj);
  sensor_msgs::PointCloud2 pcmsg_subsample;
  pcl_conversions::fromPCL(*pc2_subsample,pcmsg_subsample);


  // publish clouds
  pub_cloud_proj.publish(pcmsg_proj);
  pub_cloud_subsample.publish(pcmsg_subsample);
}


  int
main (int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "ggp_process_cloud");
  ros::NodeHandle nh;


  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("ggp/points", 1);
  pub_cloud_proj = nh.advertise<sensor_msgs::PointCloud2> ("ggp/points/projected", 1);
  pub_cloud_subsample = nh.advertise<sensor_msgs::PointCloud2> ("ggp/points/subsample", 1);
  pub_point_minmax = nh.advertise<visualization_msgs::Marker> ("ggp/marker/minmax", 1);



  // Spin
  ros::spin ();
}
