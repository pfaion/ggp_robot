#ifndef __GGP_ROBOT_CAMERA_H
#define __GGP_ROBOT_CAMERA_H

#include <vector>
#include <string>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>

class Camera {
  public:
    // pure virtual, it makes no sense to define standart values
    virtual cv::Matx33d getCameraMatrix() = 0;

    virtual std::vector<double> getDistortionCoefficients();

    virtual ~Camera();
    Camera();

    cv_bridge::CvImageConstPtr getCvImage();
    void setImageTopic(std::string imgTopic);

  private:
    ros::NodeHandle nh;
    image_transport::ImageTransport imgTrans;
    image_transport::Subscriber imgSub;
    ros::CallbackQueue cbq;
    boost::mutex mtx;
    cv_bridge::CvImageConstPtr storedImage;
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

};

#endif
