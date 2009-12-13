
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <iCub/affActionPrimitives.h>

#include "drivers.h"

#include <iostream>
#include <iomanip>
#include <string>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;


class testModule: public RFModule
{
protected:
	affActionPrimitives *action;
	BufferedPort<Bottle> inPort;

    Vector graspOrien;
    Vector graspDisp;

public:
    testModule()
	{
		action=NULL;

        graspOrien.resize(4);
        graspDisp.resize(3);

        graspOrien[0]=0.133;
        graspOrien[1]=0.428;
        graspOrien[2]=-0.894;
        graspOrien[3]=2.757;

        graspDisp[0]=0.0;
        graspDisp[1]=0.0;
        graspDisp[2]=0.08;
	}

    virtual bool configure(ResourceFinder &rf)
    {
		Property option("(robot icubSim) (local test) (part left_arm) (traj_time 3.0)\
						(torso_pitch on) (torso_pitch_max 30.0)\
                        (torso_roll off)\
                        (torso_yaw on)\
                        (fingers_close_poss (10.0 40.0 90.0 10.0 70.0 70.0 70.0 70.0 110.0))");
        option.put("hand_calibration_file",rf.findFile("from"));

		action=new affActionPrimitives(option);

		if (!action->isValid())
		{
			delete action;
			return false;
		}

		inPort.open("/testMod/in");

        return true;
    }

    virtual bool close()
    {
		if (action!=NULL)
			delete action;
		
		inPort.close();
        
		return true;
    }

    virtual double getPeriod()
	{
		return 1.0;
	}

    virtual bool updateModule()
	{		
        // get a target object position from a YARP port
		Bottle *b=inPort.read();	// blocking call

        if (b!=NULL)
		{
			Vector xd(3), dRel(3);
			dRel=0.0;
			
			xd[0]=b->get(0).asDouble();
			xd[1]=b->get(1).asDouble();
			xd[2]=b->get(2).asDouble();
			dRel[2]=0.1;

			xd[0]=xd[0]>-0.1?-0.1:xd[0];	// safe thresholding

			action->grasp(xd,graspOrien,graspDisp,true);    // grasp it (wait until it's done)
			action->reach(xd+dRel,graspOrien,true);         // lift the object
			action->openHand(true);                         // release the object
		}		

		return true;
	}

	bool interruptModule()
	{
		action->syncCheckInterrupt();
		inPort.interrupt();

		return true;
	}
};


int main(int argc, char *argv[])
{
    Network yarp;	

    if (!yarp.checkNetwork())
        return -1;

	DriverCollection dev;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("affActionPrimitives/conf");
    rf.setDefaultConfigFile("object_sensing.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    testModule mod;

    return mod.runModule(rf);
}



