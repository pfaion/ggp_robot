/** ---------------------------------------------------------------------------
  FACTORY REGISTRATION
  List all header files and register the necessary classes.

  Syntax:
    #include <header.h>
    REGISTER_TYPE(ClassName1, "identifier1");
    REGISTER_TYPE(ClassName2, "identifier2");
---------------------------------------------------------------------------- */

#include <ggp_robot/libs/chessboard.h>
REGISTER_TYPE(ChessBoard, "chessboard");
