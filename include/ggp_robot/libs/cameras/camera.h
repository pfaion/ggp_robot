#ifndef __GGP_ROBOT_CAMERA_H
#define __GGP_ROBOT_CAMERA_H

#include <vector>
#include <string>
#include <time.h>
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
    // should return 0 coefficients, unless implemented in child classes
    virtual std::vector<double> getDistortionCoefficients();

    virtual ~Camera();
    Camera();
    void setImageTopic(std::string top);
    void setCloudTopic(std::string top);

    // activation/deactivation of streams
    // this is necessary, because some cameras (e.g. asus xtion) switch into a
    // synchronized mode, if there is a cloud-subscriber subscibed, which causes
    // the image topic to be published way less often -> performance issues!
    // recommended usage: set cloudstream to false, if you are only listening
    // for images!
    void listenToImageStream(bool state);
    void listenToCloudStream(bool state);

    cv_bridge::CvImageConstPtr getCvImage(bool waitForNew=true);
    pcl::PCLPointCloud2::ConstPtr getPclCloud(bool waitForNew=true);

  private:
    // subscription stuff
    ros::NodeHandle nh;
    ros::Subscriber cloudSub;
    image_transport::ImageTransport imgTrans;
    image_transport::Subscriber imgSub;

    // image stuff
    boost::mutex imgMtx;
    bool imgActive;
    long long int imgLastStamp;
    std::string imgTopic;
    sensor_msgs::ImageConstPtr storedImage;
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    // cloud stuff
    boost::mutex cloudMtx;
    bool cloudActive;
    long long int cloudLastStamp;
    std::string cloudTopic;
    sensor_msgs::PointCloud2::Ptr storedCloud;
    void cloudCb(const sensor_msgs::PointCloud2::Ptr cloud);

    // time is money
    long long int msecStamp();
};

#endif
