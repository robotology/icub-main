
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
#include <deque>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;


class testModule: public RFModule
{
protected:
	affActionPrimitivesLayer1 *action;
	BufferedPort<Bottle> inPort;

    Vector graspOrien, home_x, home_o;

public:
    testModule()
	{
		action=NULL;

        graspOrien.resize(4);
        home_x.resize(3);
        home_o.resize(4);

        graspOrien[0]=-0.171542;
        graspOrien[1]= 0.124396;
        graspOrien[2]=-0.977292;
        graspOrien[3]= 3.058211;
        
        home_x[0]=-0.29;
        home_x[1]=-0.21; 
        home_x[2]= 0.11;
        home_o[0]=-0.029976;
        home_o[1]= 0.763076;
        home_o[2]=-0.645613;
        home_o[3]= 2.884471;

	}

    virtual bool configure(ResourceFinder &rf)
    {
		Property option("(robot icub) (local testMod) (part left_arm) (traj_time 2.0)\
						(torso_pitch on) (torso_pitch_max 20.0) (torso_roll off) (torso_yaw on)");
        option.put("hand_calibration_file",rf.findFile("calibFile"));
        option.put("hand_sequences_file",rf.findFile("seqFile"));

		action=new affActionPrimitivesLayer1(option);

		if (!action->isValid())
		{
			delete action;
			return false;
		}
        
        deque<string> q=action->getHandSeqList();
        cout<<"List of available hand sequence keys:"<<endl;
        for (size_t i=0; i<q.size(); i++)
            cout<<q[i]<<endl;

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
            bool f;
			Vector xd(3), dOffs(3), graspDisp(3);
			
			xd[0]=b->get(0).asDouble();
			xd[1]=b->get(1).asDouble();
			xd[2]=b->get(2).asDouble();

            dOffs[0]=-0.02;
            dOffs[1]=-0.04;
            dOffs[2]=-0.02;

            xd=xd+dOffs;

			xd[0]=xd[0]>-0.1?-0.1:xd[0];	// safe thresholding

            graspDisp[0]=0.0;
            graspDisp[1]=0.0;
            graspDisp[2]=0.05;

            // grasp it (wait until it's done)
			action->grasp(xd,graspOrien,graspDisp);
            action->checkActionsDone(f,true);

            Vector dRel(3);
            dRel[0]=0.0;
            dRel[1]=0.0;
            dRel[2]=0.10;

            // lift the object (wait until it's done)
			action->pushAction(xd+dRel,graspOrien);
            action->checkActionsDone(f,true);

            // release the object (wait until it's done)
            action->pushAction("open_hand");
            action->checkActionsDone(f,true);

            // go home :)
            action->pushAction(home_x,home_o);
		}		

		return true;
	}

	bool interruptModule()
	{
		action->syncCheckInterrupt(true);
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
    rf.setDefault("calibFile","object_sensing.ini");
    rf.setDefault("seqFile","hand_sequences.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    testModule mod;

    return mod.runModule(rf);
}



