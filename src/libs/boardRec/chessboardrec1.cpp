// chessboardrec1.cpp //
#include <ggp_robot/libs/boardRec/chessboardrec1.h>

#include <stdexcept>
#include <boost/shared_ptr.hpp>
#include <ggp_robot/libs/boards/board.h>
#include <ggp_robot/libs/boards/chessboard1.h>
#include <ggp_robot/libs/tools/debug.h>



ChessBoardRec1::ChessBoardRec1() {
  PRINT("[BREC] Recognizer initializing...");
}

void ChessBoardRec1::setBoard(boost::shared_ptr<PlanarBoard> bp) {
  PRINT("[BREC] Setting board.");
  board = boost::dynamic_pointer_cast<ChessBoard1>(bp);
  if(!board) {
    PRINT(red, "[BREC] Wrong type of chessboard for this recognizer!");
    throw std::domain_error("");
  }
}

void ChessBoardRec1::setCamera(boost::shared_ptr<Camera> cp) {
  PRINT("[BREC] Setting camera.");
  cam = cp;
}

void ChessBoardRec1::start() {
  PRINT("[BREC] Starting...");
}

