#ifndef __GGP_ROBOT_BOARD_H
#define __GGP_ROBOT_BOARD_H

// includes
#include <map>
#include <vector>
#include <string>
#include <ggp_robot/libs/boards/boardpoint.h>

//==============================================================================

class PlanarBoard {

  public:
    typedef std::map<std::string,std::vector<cv::Point3f> > RegionLayout;
 
    RegionLayout regions;

    PlanarBoard();
    virtual ~PlanarBoard();
    
    // The idea of this function is to provide a purely scaled change of
    // coordinates, to avoid e.g. giving coordinates in meters all the time,
    // instead of intrinsic board coordinates.
    // To implement a specific transformation, just override this function in a
    // derived class.
    virtual BoardPoint p(float x, float y, float z=0.0);

};


#endif

