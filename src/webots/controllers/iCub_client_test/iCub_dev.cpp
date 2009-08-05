

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/all.h>
#include <yarp/String.h>
#include <ace/OS.h>
#include <ace/Log_Msg.h>
#include <ace/Sched_Params.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;


int main(int argc,char **argv) 
{
  

  Network::init();
  
  Property options;
  
  options.put("robot","icub");
  options.put("device","remote_controlboard");
  options.put("local","/icub/left_arm/control");
  options.put("remote","/icub/left_arm");

  PolyDriver ddLArm(options);

  
  if(!ddLArm.isValid())
    {
      ACE_OS::printf("Device not available. Here are the known devices:\n");
      ACE_OS::printf("%s", Drivers::factory().toString().c_str());
      Network::fini();
      return 1;
    }

  IPositionControl *LArmpos;
 
  
  ddLArm.view(LArmpos);


 
  if(LArmpos==0)
    {
      ACE_OS::printf("Error getting IPositionControl interface.\n");
      Network::fini();
      return 1;
    }

  int jnts;
  LArmpos->getAxes(&jnts);
  ACE_OS::printf("left arm number of axes: %d\n",jnts);

  

  double angle[7];
  //int axe;

  if(argc>7)
    for(int i=0;i<7;i++)
      angle[i] = atof(argv[i+1]);
  else
    for(int i=0;i<7;i++)
    angle[i] = 0.0;
    
    
  // ACE_OS::printf("moving axe %d to position %f",axe,angle);

  LArmpos->setPositionMode();

  LArmpos->positionMove((const double*)angle);
  
  //we wait a little
  for(int i = 0;i<100000;i++)
    {
      ACE_OS::printf(".");
    }
  ACE_OS::printf("\n");

  bool fin = false;
  
  while(!fin)
    LArmpos->checkMotionDone(&fin);
     
  ACE_OS::printf("motion completed\n");

  return 1;
}
 
