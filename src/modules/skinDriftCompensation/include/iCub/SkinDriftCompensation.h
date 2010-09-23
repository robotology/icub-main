#ifndef __ICUB_SKINDRIFTCOMPENSATION_H__
#define __ICUB_SKINDRIFTCOMPENSATION_H__

#include <iostream>
#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>

#include "iCub/MyThread.h"
 
using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;


class SkinDriftCompensation:public RFModule
{
   /* module parameters */
   string moduleName;
   string robotName;

   // names of the ports
   string compensatedTactileDataPortName;
   string handlerPortName;

   /* class variables */
   BufferedPort<Bottle> compensatedTactileDataPort;	
   Port handlerPort;									// a port to handle messages

   bool calibrationAllowed;								// if false the thread is not allowed to run the calibration
   bool forceCalibration;								// if true a calibration is executed as soon as possible, 
														// after that the variable is set to false

   /* pointer to a new thread to be created and started in configure() and stopped in close() */
   MyThread *myThread;


public:
   
   bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
   bool interruptModule();                       // interrupt, e.g., the ports 
   bool close();                                 // close and shut down the module
   bool respond(const Bottle& command, Bottle& reply);
   double getPeriod(); 
   bool updateModule();
};


#endif // __ICUB_SKINDRIFTCOMPENSATION_H__
//empty line to make gcc happy

