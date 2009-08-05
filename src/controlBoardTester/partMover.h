#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

class partMover
{
 public:
  void setPolyDriver(PolyDriver*);
  partMover();
  ~partMover();

 private:
  int numberOfJoints;
  PolyDriver *dd;
  IPositionControl *pos;
  IVelocityControl *vel;
  IEncoders *enc;
  IPidControl *pid;
  IAmplifierControl *amp;
  IControlLimits *lim;

};
