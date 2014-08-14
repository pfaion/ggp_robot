#ifndef __GGP_ROBOT_STATE_RECOGNITION_H
#define __GGP_ROBOT_STATE_RECOGNITION_H

#include <boost/shared_ptr.hpp>
#include <ggp_robot/libs/boards/board.h>
#include <ggp_robot/libs/cameras/camera.h>

class StateRecognition {
  public:
    virtual ~StateRecognition();

    virtual void setBoard(boost::shared_ptr<PlanarBoard>& board) = 0;
    virtual void setCamera(boost::shared_ptr<Camera>& cam) = 0;

    virtual bool start() = 0;

};




#endif

