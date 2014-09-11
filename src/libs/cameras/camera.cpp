// camera.cpp //
#include <ggp_robot/libs/cameras/camera.h>
#include <vector>
#include <string>
#include <sys/time.h>
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
  storedCloud(),
  imgActive(false),
  cloudActive(false)
{
  PRINT("[CAM] init");
  imgLastStamp = msecStamp();
  cloudLastStamp = msecStamp();
}

long long int Camera::msecStamp() {
  struct timeval tp;
  gettimeofday(&tp, NULL);
  long long int stamp = tp.tv_sec;
  stamp *= 1000000;
  stamp += tp.tv_usec;
  return stamp;
}

Camera::~Camera() {}

void Camera::setImageTopic(std::string top) {
  imgTopic = top;
}

void Camera::listenToImageStream(bool state) {
  if(state == imgActive) return;
  if(state) {
    imgSub = imgTrans.subscribe(imgTopic, 1, &Camera::imageCb, this);
    PRINT("[CAM] Looking for image stream.");
    while(ros::ok() && !imgActive) {
      ros::spinOnce();
      imgMtx.lock();
      imgActive = storedImage;
      imgMtx.unlock();
    }
    PRINT(green, "[CAM] Found image stream!");
  } else {
    imgSub.shutdown();
    imgActive = false;
  }
}

void Camera::setCloudTopic(std::string top) {
  cloudTopic = top;
}

void Camera::listenToCloudStream(bool state) {
  if(state == cloudActive) return;
  if(state) {
    cloudSub = nh.subscribe(cloudTopic, 1, &Camera::cloudCb, this);
    PRINT("[CAM] Looking for cloud stream.");
    storedCloud.reset();
    while(ros::ok() && !cloudActive) {
      ros::spinOnce();
      cloudMtx.lock();
      cloudActive = storedCloud;
      cloudMtx.unlock();
    }
    PRINT(green, "[CAM] Found cloud stream!");
  } else {
    cloudSub.shutdown();
    cloudActive = false;
  }
}

void Camera::imageCb(const sensor_msgs::ImageConstPtr& msg) {
  //PRINT("[CAM] Image received.");
  imgMtx.lock();
  storedImage = msg;
  imgLastStamp = msecStamp();
  imgMtx.unlock();
}

cv_bridge::CvImageConstPtr Camera::getCvImage(bool waitForNew){
  long long int stamp = msecStamp();
  // convert sensor_msgs to opencv-Mat-like object
  cv_bridge::CvImageConstPtr cv;
  do {
    ros::spinOnce();
    imgMtx.lock();
    if(!waitForNew || imgLastStamp > stamp) {
      cv = cv_bridge::toCvShare(storedImage, sensor_msgs::image_encodings::BGR8);
    }
    imgMtx.unlock();
  } while(ros::ok() && !cv);
  PRINT("[CAM] Converting and returning image.");
  return cv;
} 

void Camera::cloudCb(const sensor_msgs::PointCloud2::Ptr cloud) {
  //PRINT("[CAM] Cloud received.");
  cloudMtx.lock();
  storedCloud = cloud;
  cloudLastStamp = msecStamp();
  cloudMtx.unlock();
}

pcl::PCLPointCloud2::ConstPtr Camera::getPclCloud(bool waitForNew){
  long long int stamp = msecStamp();
  // convert cloud to pcl-blob
  pcl::PCLPointCloud2::Ptr pc2(new pcl::PCLPointCloud2());
  do {
    ros::spinOnce();
    cloudMtx.lock();
    if(!waitForNew || (cloudLastStamp > stamp)) {
      pcl_conversions::toPCL(*storedCloud, *pc2);
      cloudMtx.unlock();
      break;
    }
    cloudMtx.unlock();
  } while(ros::ok());
  PRINT("[CAM] Converting and returning cloud.");
  return pc2;
} 
