#include "iCubWebotsMotionControl.h"
#include "webots_common.h"


#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/all.h>
#include <ace/OS.h>
#include <ace/Log_Msg.h>
#include <ace/Sched_Params.h>
#include <ace/String_Base.h>

#include <webots/robot.h>

#ifdef main
#undef main
#endif

#include "Cameras.h"



PolyDriver *ddLArm,*ddRArm,*ddLLeg,*ddRLeg,*ddTorso,*ddHead;
WebotsCommon *webots_instance;


int run(int ms)
{
  webots_instance->sem_control_post();
  
  webots_instance->sem_webots_wait();
  
  return 16;
}


void reset()
{
  ///get a pointer to the singleton class that allows synchronization
  webots_instance = WebotsCommon::getInstance();
  
  Property options;
  
  ////start left arm device driver
  options.fromConfigFile("configFiles/left_arm.ini");
  ddLArm = new PolyDriver(options);
  

  ///start right arm device driver
  options.fromConfigFile("configFiles/right_arm.ini");
  ddRArm = new PolyDriver(options);


  ////start left leg device driver
  options.fromConfigFile("configFiles/left_leg.ini");
  ddLLeg = new PolyDriver(options);


  /////start right leg device driver
  options.fromConfigFile("configFiles/right_leg.ini");
  ddRLeg = new PolyDriver(options);


  /////start torso device driver
  options.fromConfigFile("configFiles/torso.ini");
  ddTorso = new PolyDriver(options);


  /////start head device driver
  options.fromConfigFile("configFiles/head.ini");
  ddHead = new PolyDriver(options);

  if(!ddLArm->isValid() || !ddRArm->isValid() || !ddLLeg->isValid() || !ddRLeg->isValid() || !ddTorso->isValid())
    {
      ACE_OS::printf("Device not available. Here are the known devices:\n");
      ACE_OS::printf("%s", Drivers::factory().toString().c_str());
      Network::fini();
      exit(1);
    }


}


void exit_robot()
{
  ACE_OS::printf("robot is quitting\n");
  
  ddLArm->close();
  ddRArm->close();
  ddLLeg->close();
  ddRLeg->close();
  ddTorso->close();
  ddHead->close();

}



int main() 
{
	
  wb_robot_init();	
	
  Drivers::factory().add(new DriverCreatorOf<iCubWebotsMotionControl>("webotsmotioncontrol", 
							"controlboard",
							"iCubWebotsMotionControl"));
  

  Network::init();

  reset();

  //Enables the cameras
  Cameras cameras;
   
  int count = 0;
  while (wb_robot_step(64) != -1) 
    {
		//Send images to Yarp.
        cameras.SendImages();
        count++;
  		run(64);
  	}   	
 
  exit_robot();
  return 0;
}
 
