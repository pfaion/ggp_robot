#ifndef __GGP_ROBOT_FACTORIES_H
#define __GGP_ROBOT_FACTORIES_H

#include <ggp_robot/libs/tools/factoryclass.h>

/** ---------------------------------------------------------------------------
  FACTORY REGISTRATION
  Include derived classes and register them.

  Syntax:
    #include <derivedHeader.h>
    REGISTER_TYPE(BaseClassName, DerivedClassName, "identifier");

  -----  
  Example-usage: registering a derived class DerivedA from base class Base with

      REGISTER_TYPE(Base, DerivedA, "myA");

  allows to call

      Base* b = Factory<Base>::create("myA");

  which returns a Base-Pointer to an instance of the derived class DerivedA;
---------------------------------------------------------------------------- */

#include <ggp_robot/libs/boards/chessboard1.h>
REGISTER_TYPE(PlanarBoard, ChessBoard1, "chessboard1");

#include <ggp_robot/libs/boardRec/chessboardrec1.h>
REGISTER_TYPE(BoardRecognition, ChessBoardRec1, "chessboardrec1");

#include <ggp_robot/libs/cameras/xtion.h>
REGISTER_TYPE(Camera, Xtion, "xtion");


#endif
