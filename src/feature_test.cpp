#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>


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

    cv::Mat img = cv::imread("/home/pfaion/catkin_ws/src/ggp_robot/src/board_low2.jpg", CV_LOAD_IMAGE_COLOR);

    // Detect keypoints and get descriptor
    //cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create("ORB");
    cv::ORB detector(100);
    std::cout << "keypoin detection initialised" << std::endl;
    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;
    std::cout << "keypoin detection initialised" << std::endl;
    detector.detect(cv_ptr->image, keypoints1);
    detector.detect(img, keypoints2);
    std::cout << "keypoints detected" << std::endl;
    cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create("ORB");
    cv::Mat descriptor1;
    cv::Mat descriptor2;
    extractor->compute(cv_ptr->image, keypoints1, descriptor1);
    extractor->compute(cv_ptr->image, keypoints2, descriptor2);



    std::cout << "callback start matching" << std::endl;

    //    cv::FlannBasedMatcher matcher;
    //    std::vector<cv::DMatch> matches;
    //    matcher.match(descriptor1, descriptor2, matches);

    cv::BFMatcher matcher(cv::NORM_L2);
    std::vector<cv::DMatch> matches;
    matcher.match(descriptor1, descriptor2, matches);

    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;
    for(int i=0; i < matches.size(); i++) {
      obj.push_back(keypoints1[matches[i].queryIdx].pt);
      scene.push_back(keypoints2[matches[i].queryIdx].pt);
    }

    cv::Mat H = cv::findHomography(obj, scene, CV_RANSAC);




    //    // Match descriptors
    //    cv::BFMatcher matcher(cv::NORM_L2);
    //    std::vector<cv::DMatch> matches;
    //    matcher.match(descriptor1, descriptor2, matches);
    //
    //    std::cout << "callback end matching" << std::endl;






    // Draw matches
    cv::Mat img_matches;
    cv::drawMatches(img, keypoints2, cv_ptr->image, keypoints1, matches, img_matches);



    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<cv::Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img.cols, 0 );
    obj_corners[2] = cvPoint( img.cols, img.rows ); obj_corners[3] = cvPoint( 0, img.rows );
    std::vector<cv::Point2f> scene_corners(4);

    perspectiveTransform( obj_corners, scene_corners, H);

    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line( img_matches, scene_corners[0] + cv::Point2f( img.cols, 0), scene_corners[1] + cv::Point2f( img.cols, 0), cv::Scalar(0, 255, 0), 4 );
    line( img_matches, scene_corners[1] + cv::Point2f( img.cols, 0), scene_corners[2] + cv::Point2f( img.cols, 0), cv::Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[2] + cv::Point2f( img.cols, 0), scene_corners[3] + cv::Point2f( img.cols, 0), cv::Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[3] + cv::Point2f( img.cols, 0), scene_corners[0] + cv::Point2f( img.cols, 0), cv::Scalar( 0, 255, 0), 4 );









    cv::imshow(OPENCV_WINDOW, img_matches);
    cv::waitKey(100);


    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());



  }
};

int main(int argc, char** argv)
{
  //cv::initModule_nonfree();
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
