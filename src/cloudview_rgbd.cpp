#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

class MySubscriber {

  pcl::visualization::CloudViewer viewer;

  public:
    MySubscriber();
    void cbfun(const sensor_msgs::PointCloud2& input);

};

MySubscriber::MySubscriber()
: viewer("title") {

}


void MySubscriber::cbfun(const sensor_msgs::PointCloud2& input) {
  
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(input, pcl_pc);

  pcl::PointCloud<pcl::PointXYZRGB> * cloud = new pcl::PointCloud<pcl::PointXYZRGB>();
  pcl::fromPCLPointCloud2(pcl_pc, *cloud);
 
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr con_pc_ptr(cloud);

  viewer.showCloud(con_pc_ptr);  

}

int
main (int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "ggp_pclview_rgbd");
  ros::NodeHandle nh;

  MySubscriber subs;
  
// Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("mycloud", 1, &MySubscriber::cbfun, &subs);

  // Spin
  ros::spin ();
}
