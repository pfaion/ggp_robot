#ifndef __GGP_ROBOT_CHESSBOARD1_H
#define __GGP_ROBOT_CHESSBOARD1_H

// forward declare
class BoardPoint;

// includes
#include <vector>
#include <ggp_robot/libs/boards/board.h>


class ChessBoard1 : public PlanarBoard {

  public:

    const double FIELD_SIZE;

    std::vector<BoardPoint> corners;

    ChessBoard1();

    BoardPoint p(double x, double y, double z=0.0);

};


#endif
