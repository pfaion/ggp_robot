#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>


static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, 
        &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);

    std::cout << "converter initialized" << std::endl;
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    std::cout << "callback called" << std::endl;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat img = cv::imread("/home/pfaion/catkin_ws/src/ggp_robot/src/board_low.jpg", CV_LOAD_IMAGE_COLOR);

    // Detect keypoints and get descriptor
    int numKeyPoints = 100;
    cv::ORB orb(numKeyPoints);
    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;
    cv::Mat descriptor1;
    cv::Mat descriptor2;
    orb(cv_ptr->image, cv::Mat(), keypoints1, descriptor1);
    orb(img, cv::Mat(), keypoints2, descriptor2);

    std::cout << "callback start matching" << std::endl;

    // Match descriptors
    cv::BFMatcher matcher(cv::NORM_L2);
    std::vector<cv::DMatch> matches;
    matcher.match(descriptor1, descriptor2, matches);

    std::cout << "callback end matching" << std::endl;

    // Draw matches
    cv::Mat img_matches;
    cv::drawMatches(cv_ptr->image, keypoints1, img, keypoints2, matches, img_matches);
    cv::imshow(OPENCV_WINDOW, img_matches);
    cv::waitKey(3);


    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());



  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
