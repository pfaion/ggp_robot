// camera.cpp //
#include <ggp_robot/libs/cameras/camera.h>
#include <vector>
#include <string>
#include <unistd.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <ggp_robot/libs/tools/debug.h>


// a common value is just (0, 0, 0, 0, 0)
std::vector<double> Camera::getDistortionCoefficients() {
  std::vector<double> distCoeffs(5, 0.0);
  return distCoeffs;
}

Camera::Camera()
: nh(),
  imgTrans(nh),
  storedImage(),
  storedCloud()
{
  //nh.setCallbackQueue(&cbq);
  PRINT("cam init");
}

Camera::~Camera() {}

void Camera::setImageTopic(std::string imageTopic) {
  imgSub = imgTrans.subscribe(imageTopic, 1, &Camera::imageCb, this);
}

void Camera::setCloudTopic(std::string cloudTopic) {
  cloudSub = nh.subscribe(cloudTopic, 1, &Camera::cloudCb, this);
}

void Camera::imageCb(const sensor_msgs::ImageConstPtr& msg) {
  PRINT("[CAM] Image received. Converting.");
  // convert sensor_msgs to opencv-Mat-like object
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  imgMtx.lock();
  storedImage = cv_ptr;
  imgMtx.unlock();
}

cv_bridge::CvImageConstPtr Camera::getCvImage(){
  cv_bridge::CvImageConstPtr copy;
  do {
    ros::spinOnce();
    imgMtx.lock();
    copy = storedImage;
    imgMtx.unlock();
  } while(!copy);
  PRINT("[CAM] Returning image.");
  return copy;
} 

void Camera::cloudCb(const sensor_msgs::PointCloud2& cloud) {
  pcl::PCLPointCloud2::Ptr pc2(new pcl::PCLPointCloud2());
  PRINT("[CAM] Cloud received. Converting.");
  pcl_conversions::toPCL(cloud, *pc2);
  pclMtx.lock();
  storedCloud = pc2;
  pclMtx.unlock();
}

pcl::PCLPointCloud2::ConstPtr Camera::getPclCloud(){
  pcl::PCLPointCloud2::ConstPtr copy;
  do {
    ros::spinOnce();
    pclMtx.lock();
    copy = storedCloud;
    pclMtx.unlock();
  } while(!copy);
  PRINT("[CAM] Returning cloud.");
  return copy;
} 
