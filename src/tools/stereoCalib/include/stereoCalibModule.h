#include <iostream>
#include <string>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include "stereoCalibThread.h"

#define PATH_LEN 255

using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
  

class stereoCalibModule:public RFModule
{
    string moduleName;
    string inputLeftPortName;
    string inputRightPortName;
    string outputPortNameRight;
    string outputPortNameLeft;  
    string handlerPortName;
    string outputCalibPath;
    string dir;

    int thresholdValue;

    char dirName[PATH_LEN];

    BufferedPort<ImageOf<PixelBgr> > imageOut;
    Port handlerPort;


    stereoCalibThread *calibThread;


public:

    bool configure(yarp::os::ResourceFinder &rf);
    bool interruptModule();                       
    bool close();                                 
    bool respond(const Bottle& command, Bottle& reply);
    double getPeriod(); 
    bool updateModule();    
};

