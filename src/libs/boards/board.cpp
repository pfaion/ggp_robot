// board.cpp //
#include <ggp_robot/libs/boards/board.h>
#include <ggp_robot/libs/boards/boardpoint.h>

PlanarBoard::PlanarBoard() {}
PlanarBoard::~PlanarBoard() {}

// No coordiante transform for base class.
BoardPoint PlanarBoard::p(float x, float y, float z) {
  return BoardPoint(x, y, z);
}

