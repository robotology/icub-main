#include <ace/OS.h>
#include <ace/Log_Msg.h>
#include <math.h>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <yarp/String.h> 

//#include <yarp/dev/ControlBoardInterfaces.h>
//#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>

#include "genericControlBoardDumper.h"

//using namespace yarp::dev;
//using namespace yarp::os;
//using namespace yarp;



class controlBoardDumper: public RateThread
{
public:
  void setDevice(PolyDriver *, int, ConstString, ConstString);
  controlBoardDumper();
  ~controlBoardDumper();
  bool threadInit();
  void setThetaMap(int *, int);
  void threadRelease();
  void run();
  void setGetter(GetData *);
    
private:
  PolyDriver *Arm_dd;
  GetData *getter;
  Stamp stmp;

  Port *port;

  IPositionControl *pos;
  IVelocityControl *vel;
  IEncoders *enc;
  IPidControl *pid;
  IAmplifierControl *amp;
  IControlLimits *lim;

  int numberOfJoints;
  double *data;

  int numberOfJointsRead;
  double *dataRead;
  int    *dataMap;  

};
