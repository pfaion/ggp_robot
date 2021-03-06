#ifndef __GGP_ROBOT_BOARD_RECOGNITION_H
#define __GGP_ROBOT_BOARD_RECOGNITION_H

#include <boost/shared_ptr.hpp>
#include <ggp_robot/libs/boards/board.h>
#include <ggp_robot/libs/cameras/camera.h>

class BoardRecognition {
  public:
    virtual ~BoardRecognition();

    virtual void setBoard(boost::shared_ptr<PlanarBoard>& board) = 0;
    virtual void setCamera(boost::shared_ptr<Camera>& cam) = 0;

    virtual void start() = 0;

};




#endif

