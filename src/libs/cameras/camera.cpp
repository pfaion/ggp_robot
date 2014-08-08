// camera.cpp //
#include <ggp_robot/libs/cameras/camera.h>
#include <vector>
#include <string>
#include <unistd.h>
#include <ggp_robot/libs/tools/debug.h>


// a common value is just (0, 0, 0, 0, 0)
std::vector<double> Camera::getDistortionCoefficients() {
  std::vector<double> distCoeffs(5, 0.0);
  return distCoeffs;
}

Camera::Camera()
: nh(),
  imgTrans(nh),
  storedImage()
{
  //nh.setCallbackQueue(&cbq);
  PRINT("cam init");
}

Camera::~Camera() {}

void Camera::setImageTopic(std::string imageTopic) {
  imgSub = imgTrans.subscribe(imageTopic, 1, &Camera::imageCb, this);
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
  mtx.lock();
  storedImage = cv_ptr;
  mtx.unlock();
}

cv_bridge::CvImageConstPtr Camera::getCvImage(){
  cv_bridge::CvImageConstPtr copy;
  do {
    ros::spinOnce();
    mtx.lock();
    copy = storedImage;
    mtx.unlock();
  } while(!copy);
  PRINT("[CAM] Returning image.");
  return copy;
} 
