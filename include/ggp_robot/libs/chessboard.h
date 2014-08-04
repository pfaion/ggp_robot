#ifndef __GGP_ROBOT_CHESSBOARD_H
#define __GGP_ROBOT_CHESSBOARD_H

#include <iostream>
#include <boost/assign/std/vector.hpp>
#include <ggp_robot/libs/board.h>

namespace GgpRobot {

  class CbBRec1 : public BoardRecognition {

  };

  class CbSRec1 : public StateRecognition {

  };

  class ChessBoard : public PlanarBoard {

    public:
      const double FIELD_SIZE;
      std::vector<BoardPoint> corners;

      ChessBoard();

      BoardPoint p(double x, double y, double z=0.0);




  };



}

#endif
