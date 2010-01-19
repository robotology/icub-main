
#include <iostream>
#include <iomanip>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/all.h>

#include "camera.h"


using namespace std;


using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;




using namespace yarp::os;
using namespace yarp::sig;

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;








// The thread launched by the application which 
// control smooth pursuit eye movement 

class PredSmoothP : public RateThread
{
protected:
    
    PolyDriver       *drvHead;
    IEncoders        *encHead;
	
   
    //BufferedPort<Bottle> *port_pixel;
	BufferedPort<Bottle> *port_velocityJoint;
	BufferedPort<Bottle> *port_stateTarget;
	Vector  ve ;
    unsigned int period;
    unsigned int joint;
	string localName;
	double Ts;
	int nJointsHead;
   

   

public:
	PredSmoothP( PolyDriver *_drvHead, const string &_localName,unsigned int _period,unsigned int _joint);

   
    virtual bool threadInit();
    virtual void afterStart(bool s);
    virtual void run();
    virtual void threadRelease();
};





