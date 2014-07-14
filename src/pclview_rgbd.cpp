#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>






// ---------------- VISUALIZER --------------------
class MyVisualizer {
  bool update;
  boost::mutex updateModelMutex;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  pcl::visualization::PCLVisualizer::Ptr viewer;


  public:
  MyVisualizer();

  void updateCloud(const sensor_msgs::PointCloud2& input) {

    boost::mutex::scoped_lock updateLock(updateModelMutex);
    update = true;

    pcl::PCLPointCloud2::Ptr pc2(new pcl::PCLPointCloud2());

    pcl_conversions::toPCL(input, *pc2);
    pcl::fromPCLPointCloud2(*pc2, *cloud);

    updateLock.unlock();


  }
  void visualize()  
  {  

    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::mutex::scoped_lock updateLock(updateModelMutex);
      if(update)
      {
        if(!viewer->updatePointCloud(cloud))
          viewer->addPointCloud(cloud);
        update = false;
      }
      updateLock.unlock();

    }   
  }  
};

MyVisualizer::MyVisualizer()
    : update(false)
      , cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
      , viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"))
  {
    boost::thread* thr = new boost::thread(boost::bind(&MyVisualizer::visualize, this));
  }



int main (int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "ggp_pclview_rgbd");
  ros::NodeHandle nh;

  MyVisualizer vis;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("mycloud", 1, &MyVisualizer::updateCloud, &vis);

  ros::spin();

}
