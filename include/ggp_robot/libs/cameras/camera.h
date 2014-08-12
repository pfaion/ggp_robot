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
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>

class Camera {
  public:
    // pure virtual, it makes no sense to define standart values
    virtual cv::Matx33d getCameraMatrix() = 0;

    virtual std::vector<double> getDistortionCoefficients();

    virtual ~Camera();
    Camera();

    cv_bridge::CvImageConstPtr getCvImage(bool newOne=false);
    void setImageTopic(std::string imgTopic);

    pcl::PCLPointCloud2::ConstPtr getPclCloud();
    void setCloudTopic(std::string cloutTopic);

  private:
    ros::NodeHandle nh;
    image_transport::ImageTransport imgTrans;
    image_transport::Subscriber imgSub;
    ros::Subscriber cloudSub;

    boost::mutex imgMtx;
    bool newImg;
    cv_bridge::CvImageConstPtr storedImage;
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    boost::mutex pclMtx;
    pcl::PCLPointCloud2::ConstPtr storedCloud;
    void cloudCb(const sensor_msgs::PointCloud2& cloud);
};

#endif
