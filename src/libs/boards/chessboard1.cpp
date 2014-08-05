// chessboard1.cpp //
#include <ggp_robot/libs/boards/chessboard1.h>
#include <vector>
#include <string>
#include <boost/assign/std/vector.hpp>
#include <ggp_robot/libs/boards/boardpoint.h>


ChessBoard1::ChessBoard1()
  : FIELD_SIZE(0.0945)
{

  // initialize region descriptions for regular fields
  std::vector<std::string> desc;
  { 
    using namespace boost::assign;
    desc += "a1", "b1", "c1", "d1", "x1"; 
    desc += "a2", "b2", "c2", "d2", "x2"; 
    desc += "a3", "b3", "c3", "d3", "x3"; 
    desc += "a4", "b4", "c4", "d4", "x4"; 
  }
  for(int x = 0; x < 5; x++) {
    for(int y = 0; y < 4; y++) {
      std::string d = desc[x + 4*y];
      this->regions[d] = std::vector<BoardPoint>();
      this->regions[d].push_back(p(x,y));
      this->regions[d].push_back(p(x+1,y));
      this->regions[d].push_back(p(x+1,y+1));
      this->regions[d].push_back(p(x,y+1));
    }
  }

  // initialize marker region description
  this->regions["marker"].push_back(p(4,0));
  this->regions["marker"].push_back(p(5,0));
  this->regions["marker"].push_back(p(5,1));
  this->regions["marker"].push_back(p(4,1));


  // initialize corners for detection
  this->corners.push_back(p(3,1));
  this->corners.push_back(p(2,1));
  this->corners.push_back(p(1,1));
  this->corners.push_back(p(3,2));
  this->corners.push_back(p(2,2));
  this->corners.push_back(p(1,2));
  this->corners.push_back(p(3,3));
  this->corners.push_back(p(2,3));
  this->corners.push_back(p(1,3));
}

BoardPoint ChessBoard1::p(double x, double y, double z) {
  return BoardPoint(x * FIELD_SIZE, y * FIELD_SIZE, z * FIELD_SIZE);
}


