
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <iCub/affActionPrimitives.h>

#ifdef USE_ICUB_MOD
    #include "drivers.h"
#endif

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
	affActionPrimitives *mot;
	BufferedPort<Bottle> inPort;

public:
    testModule()
	{
		mot=NULL;
	}

    virtual bool configure(ResourceFinder &rf)
    {
		Property option("(robot icubSim) (local test) (part left_arm) (trajTime 3.0)");
        option.put("calibFile",rf.findFile("from"));

		mot=new affActionPrimitives;

		if (!mot->open(option))
			return false;

		Vector od(4);
		od[0]=-0.029;
	    od[1]=-0.763;
		od[2]=0.646;
		od[3]=3.110;
		mot->setTapOrien(od);

		od[0]=0.133;
	    od[1]=0.428;
		od[2]=-0.894;
		od[3]=2.757;
		mot->setGraspOrien(od);

		Vector h(3);
		h[0]=0.0;
		h[1]=0.0;
		h[2]=0.08;
		mot->setGraspDeltaHeight(h);

		Vector d(3);
		d[0]=0.0;
		d[1]=0.1;
		d[2]=0.0;
		mot->setTapDisplacement(d);

		inPort.open("/testMod/in");

        return true;
    }

    virtual bool close()
    {
		if (mot!=NULL)
			delete mot;
		
		inPort.close();
        
		return true;
    }

    virtual double getPeriod()
	{
		return 1.0;
	}

    virtual bool updateModule()
	{		
		Bottle *b=inPort.read();	// blocking

        if (b!=NULL)
		{
			Vector xd(3), dRel(3);
			dRel=0.0;
			
			xd[0]=b->get(0).asDouble();
			xd[1]=b->get(1).asDouble();
			xd[2]=b->get(2).asDouble();
			dRel[2]=0.1;

			xd[0]=xd[0]>-0.1?-0.1:xd[0];	// safe thres

			mot->grasp(xd,true);
			mot->reach(xd+dRel,mot->getGraspOrien(),true);
			mot->openHand(true);
		}		

		return true;
	}

	bool interruptModule()
	{
		mot->syncCheckInterrupt();
		inPort.interrupt();

		return true;
	}
};


int main(int argc, char *argv[])
{
    Network yarp;	

    if (!yarp.checkNetwork())
        return -1;

#ifdef USE_ICUB_MOD
	DriverCollection dev;
#endif

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("affActionPrimitives/conf");
    rf.setDefaultConfigFile("object_sensing.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    testModule mod;

    return mod.runModule(rf);
}
