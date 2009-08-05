#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>

#include "FpsStats.h"


using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

class statCollector: public RateThread
{
 public:
  void setDeviceDriver(PolyDriver *);
  void setPort(BufferedPort<Vector> *);
  void threadRelease();
  statCollector(int r);
  ~statCollector();
  void run();
 private:
  FpsStats *vStat;                //statistics for outputs
  FpsStats *pStat;                //statistics for positions
  FpsStats iStat;                //statistics for input port

  int numberOfJoints;
  double *out;
  double *q;

  PolyDriver *dd;
  IPositionControl *pos;
  IVelocityControl *vel;
  IEncoders *enc;
  IPidControl *pid;
  IAmplifierControl *amp;
  IControlLimits *lim;
};
