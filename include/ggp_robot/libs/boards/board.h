#ifndef __GGP_ROBOT_BOARD_H
#define __GGP_ROBOT_BOARD_H

// forward declare
class BoardPoint;

// includes
#include <map>
#include <vector>
#include <string>

//==============================================================================

class PlanarBoard {

  public:
    typedef std::map<std::string,std::vector<BoardPoint> > RegionLayout;
 
    RegionLayout regions;

    PlanarBoard();
    virtual ~PlanarBoard();
    
    // The idea of this function is to provide a purely scaled change of
    // coordinates, to avoid e.g. giving coordinates in meters all the time,
    // instead of intrinsic board coordinates.
    // To implement a specific transformation, just override this function in a
    // derived class.
    BoardPoint p(double x, double y, double z=0.0);

};


#endif

