/**
 * @ingroup icub_primatevision_utils
 *
 * \defgroup icub_primatevision_utils_coord Coord
 *
 * 
 * A simple coordinate structure, here due to common reuse.
 *
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at \in src/primateVision/utils/coord/coord.h
 */

#ifndef __COORD_H__
#define __COORD_H__


#include <stdlib.h>

using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {     
      
      struct Coord
      {
	int x, y;
	
	Coord() {}
	Coord(int a, int b) { x = a; y = b; }
	
	Coord operator- ()        { return Coord(-x, -y); }
	Coord operator+ (Coord a) { return Coord(x + a.x, y + a.y); }
	Coord operator- (Coord a) { return Coord(x - a.x, y - a.y); }
	bool  operator< (Coord a) { return (x <  a.x) && (y <  a.y); }
	bool  operator<=(Coord a) { return (x <= a.x) && (y <= a.y); }
	bool  operator> (Coord a) { return (x >  a.x) && (y >  a.y); }
	bool  operator>=(Coord a) { return (x >= a.x) && (y >= a.y); }
	bool  operator==(Coord a) { return (x == a.x) && (y == a.y); }
	bool  operator!=(Coord a) { return (x != a.x) || (y != a.y); }
      };

    }
  }
}
#endif
