
#include <iostream>
using namespace std;

// GCC ONLY
/*#include <ext/hash_map>
#include <ext/hash_fun.h>
using namespace __gnu_cxx;
typedef yarp::os::NetInt32 lint;
typedef yarp::os::NetInt32 longint;
*/// END GCC ONLY

#include <math.h>

class OrientedPatch
{
public:
  double x, y, dx, dy;
  double area;
  int lo_x, hi_x, lo_y, hi_y;
  //int x_lo, x_hi, y_lo, y_hi;

  double dist(double x1, double y1, double x2, double y2) {
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
  }

  int operator == (const OrientedPatch& op)
    { return 0; }

  double GetLen()
    {
      return dist(lo_x,lo_y,hi_x,hi_y);
    }

  void Write(ostream& os)
    {
      char buf[256];
      sprintf(buf,"%g %g %g %g %g %d %d %d %d ", x, y, dx, dy, area,
	      lo_x, lo_y, hi_x, hi_y);
      os << buf;
    }

  void Read(istream& is)
    {
      is >> x >> y >> dx >> dy >> area >> lo_x >> lo_y >> hi_x >> hi_y;
    }
};

typedef hash_map<lint,lint,hash<lint>,equal_to<lint> > hash_ii;

typedef hash_map<lint,OrientedPatch,hash<lint>,equal_to<lint> > hash_ip;


#include "Extender.h"
#include "MyLabel.h"
