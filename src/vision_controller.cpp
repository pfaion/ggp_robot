#include <ros/ros.h>
#include <ggp_robot/libs/chessboard.h>


using namespace GgpRobot;

class VisionController {
  public:

  VisionController() {


}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_controller");

  ros::NodeHandle n;

  std::string boardClass;
  n.getParam("boardClass", boardClass);

  VisionController vc;

  ros::spin();
  return EXIT_SUCCESS;
}
