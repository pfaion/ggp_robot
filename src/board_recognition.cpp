// board_recognition.cpp //
#include <iostream>
#include <string>
#include <ggp_robot/libs/board.h>
#include <ggp_robot/libs/factories.h>


struct DoStuffClass
{

 
};


int main(int argc, char** argv) { 
  PlanarBoard* xyz = PlanarBoardFactory::create("chessboard");
  std::cout << xyz << std::endl;

  DoStuffClass tmpstuff;

  return 1;
}
