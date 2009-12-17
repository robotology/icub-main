
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

#define USE_LEFT    0
#define USE_RIGHT   1

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;


class testModule: public RFModule
{
protected:
    string partUsed;

	affActionPrimitivesLayer1 *actionL;
    affActionPrimitivesLayer1 *actionR;
    affActionPrimitivesLayer1 *action;
	BufferedPort<Bottle> inPort;

    Vector graspOrienL, graspOrienR;
    Vector graspDispL, graspDispR;
    Vector dOffsL, dOffsR;
    Vector dRelL, dRelR;
    Vector home_xL, home_oL, home_xR, home_oR;

    Vector *graspOrien;
    Vector *graspDisp;
    Vector *dOffs;
    Vector *dRel;
    Vector *home_x, *home_o;

public:
    testModule()
	{		
        // initialization of arm-dependent quantities
        graspOrienL.resize(4);    graspOrienR.resize(4);
        graspDispL.resize(4);     graspDispR.resize(3);
        dOffsL.resize(3);         dOffsR.resize(3);
        dRelL.resize(3);          dRelR.resize(3);
        home_xL.resize(3);        home_xR.resize(3);
        home_oL.resize(4);        home_oR.resize(4);

        graspOrienL[0]=-0.171542; graspOrienR[0]=-0.0191;
        graspOrienL[1]= 0.124396; graspOrienR[1]=-0.983248;
        graspOrienL[2]=-0.977292; graspOrienR[2]=0.181269;
        graspOrienL[3]= 3.058211; graspOrienR[3]=3.093746;

        graspDispL[0]=0.0;        graspDispR[0]=0.0; 
        graspDispL[1]=0.0;        graspDispR[1]=0.0; 
        graspDispL[2]=0.05;       graspDispR[2]=0.08;

        dOffsL[0]=-0.03;          dOffsR[0]=-0.03;
        dOffsL[1]=-0.07;          dOffsR[1]= 0.04;
        dOffsL[2]=-0.02;          dOffsR[2]= 0.01;

        dRelL[0]=0.0;             dRelR[0]=0.0;  
        dRelL[1]=0.0;             dRelR[1]=0.0;  
        dRelL[2]=0.15;            dRelR[2]=0.10; 
        
        home_xL[0]=-0.29;         home_xR[0]=-0.29;
        home_xL[1]=-0.21;         home_xR[1]= 0.24;
        home_xL[2]= 0.11;         home_xR[2]= 0.07;
        home_oL[0]=-0.029976;     home_oR[0]=-0.193426;
        home_oL[1]= 0.763076;     home_oR[1]=-0.63989;
        home_oL[2]=-0.645613;     home_oR[2]= 0.743725;
        home_oL[3]= 2.884471;     home_oR[3]= 2.995693;

        action=actionL=actionR=NULL;
        graspOrien=NULL;
        graspDisp=NULL;
        dOffs=NULL;
        dRel=NULL;
        home_x=NULL;
        home_o=NULL;
	}

    virtual bool configure(ResourceFinder &rf)
    {
        partUsed=rf.check("part",Value("both_arms")).asString().c_str();
        if (partUsed!="both_arms" && partUsed!="left_arm" && partUsed!="right_arm")
        {
            cout<<"Invalid part requested !"<<endl;
            return false;
        }

		Property option("(robot icub) (local testMod) (traj_time 2.0) (reach_tol 0.007)\
						(torso_pitch on) (torso_pitch_max 20.0)\
                        (torso_roll off) (torso_yaw on)");
        option.put("hand_calibration_file",rf.findFile("calibFile"));
        option.put("hand_sequences_file",rf.findFile("seqFile"));

        Property optionL(option); optionL.put("part","left_arm");
        Property optionR(option); optionR.put("part","right_arm");

        if (partUsed=="both_arms" || partUsed=="left_arm")
        {    
            cout<<"***** Instantiating primitives for left_arm"<<endl;
            actionL=new affActionPrimitivesLayer1(optionL);

            if (!actionL->isValid())
            {
                delete actionL;
                return false;
            }
            else
                useArm(USE_LEFT);
        }

        if (partUsed=="both_arms" || partUsed=="right_arm")
        {    
            cout<<"***** Instantiating primitives for right_arm"<<endl;
            actionR=new affActionPrimitivesLayer1(optionR);

            if (!actionR->isValid())
            {
                delete actionR;

                // remind to check to delete the left as well (if any)
                if (actionL)
                    delete actionL;

                return false;
            }
            else
                useArm(USE_RIGHT);
        }

        deque<string> q=action->getHandSeqList();
        cout<<"***** List of available hand sequence keys:"<<endl;
        for (size_t i=0; i<q.size(); i++)
            cout<<q[i]<<endl;

		inPort.open("/testMod/in");

        return true;
    }

    virtual bool close()
    {
		if (actionL!=NULL)
			delete actionL;
		
        if (actionR!=NULL)
            delete actionR;

        if (!inPort.isClosed())
            inPort.close();

		return true;
    }

    virtual double getPeriod()
	{
		return 0.1;
	}

    void useArm(const int arm)
    {
        if (arm==USE_LEFT)
        {
            action=actionL;

            graspOrien=&graspOrienL;
            graspDisp=&graspDispL;
            dOffs=&dOffsL;
            dRel=&dRelL;
            home_x=&home_xL;
            home_o=&home_oL;
        }
        else if (arm==USE_RIGHT)
        {
            action=actionR;

            graspOrien=&graspOrienR;
            graspDisp=&graspDispR;
            dOffs=&dOffsR;
            dRel=&dRelR;
            home_x=&home_xR;
            home_o=&home_oR;
        }
    }

    virtual bool updateModule()
	{		
        // get a target object position from a YARP port
		Bottle *b=inPort.read();	// blocking call

        if (b!=NULL)
		{
			Vector xd(3);
            bool f;
			
			xd[0]=b->get(0).asDouble();
			xd[1]=b->get(1).asDouble();
			xd[2]=b->get(2).asDouble();

            // switch only if it's allowed
            if (partUsed=="both_arms")
            {
                if (xd[1]>0.0)
                    useArm(USE_RIGHT);
                else
                    useArm(USE_LEFT);
            }

            // apply systematic offset
            // due to uncalibrated kinematic
            xd=xd+*dOffs;

            // safe thresholding
			xd[0]=xd[0]>-0.1?-0.1:xd[0];

            // grasp it (wait until it's done)
			action->grasp(xd,*graspOrien,*graspDisp);
            action->checkActionsDone(f,true);

            // lift the object (wait until it's done)
			action->pushAction(xd+*dRel,*graspOrien);
            action->checkActionsDone(f,true);

            // release the object (wait until it's done)
            action->pushAction("open_hand");
            action->checkActionsDone(f,true);

            // go home :)
            action->pushAction(*home_x,*home_o);
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



