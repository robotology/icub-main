#include <ace/OS.h>
#include <ace/Log_Msg.h>
#include <math.h>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <yarp/String.h> 

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>

//cqp labraries for soling LQ problems
#include "cqpSolve.h"

//getting last command
#include "collectorPort.h"

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp;



const char cableNames[8] = {'a', 'b', 'g', 'd', 'r', 'p', 'P', 'c'};
const int NUMBER_OF_CONSTRAINTS = 8;
const int CABLE_WARNING = 20;

class cableLengthGuard: public RateThread
{
public:
  cableLengthGuard(PolyDriver *, collectorPort *, Matrix, Vector, int );
  ~cableLengthGuard();
  bool threadInit();
  void setThetaMap(int *);
  void threadRelease();
  void run();
  void fixedTimeMove(const double*, const double*, double);
    
private:
  PolyDriver *Arm_dd;
	
  IPositionControl *pos;
  IVelocityControl *vel;
  IEncoders *enc;
  IPidControl *pid;
  IAmplifierControl *amp;
  IControlLimits *lim;

  int numberOfJoints;
  double *theta;
  int    *thetaMap;
  double **cableLength;

  cqpProblem  *lq;
  gsl_cqp_data* cqp_data;
  gsl_cqpminimizer* cqp_solution;
  const gsl_cqpminimizer_type * T;

  collectorPort *lastCommandPos;

  Matrix C;
  Vector d;
};


