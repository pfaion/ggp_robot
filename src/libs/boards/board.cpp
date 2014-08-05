// board.cpp //
#include <ggp_robot/libs/boards/board.h>
#include <ggp_robot/libs/boards/boardpoint.h>

PlanarBoard::PlanarBoard() {}
PlanarBoard::~PlanarBoard() {}

// No coordiante transform for base class.
BoardPoint PlanarBoard::p(double x, double y, double z) {
  return BoardPoint(x, y, z);
}

