/** 
\defgroup affActionPrimitivesExample affActionPrimitivesExample
 
@ingroup icub_module  
 
Example of grasping module based upon \ref affActionPrimitives 
library. 

Copyright (C) 2009 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0. 

\section intro_sec Description 
An example module that makes use of \ref affActionPrimitives 
library in order to execute a sequence of simple actions: 
reaches for an object, tries to grasp it, then lifts it and 
finally releases it. 
 
1) A bottle containing the 3-d position of the object to grasp 
is received (let be x1).
 
2) To this 3-d point a systematic offset is added in order to
compensate for the uncalibrated kinematics of the arm (let be
x2=x1+systematic_offset). 
 
3) The robot reaches for a position located on top of the object 
with the specified orientation (let be reach(x2+grasp_disp,o)). 
 
4) The robot grasps the object (let be reach(x2,o)). 
 
5) The hand is closed. 
 
6) The robot lifts the object to a specified location (let be 
reach(x2+lift_displacement,o)). 
 
7) The robot releases the grasped object. 
 
8) The robot steers the arm to home position. 
 
\note A video on iCub grasping objects can be seen <a 
    href="http://eris.liralab.it/misc/icubvideos/icub_grasps_sponges.wmv">here</a>.
  
\section lib_sec Libraries 
- YARP libraries. 
- \ref affActionPrimitives library.  

\section parameters_sec Parameters
--name \e name
- specify the module name, which is \e affActionPrimitivesMod by
  default.
 
--part \e type 
- specify which arm has to be used: type can be \e left_arm, \e 
  right_arm, \e both_arms (default=both_arms).
 
\section portsa_sec Ports Accessed
Assumes that \ref icub_iCubInterface (with ICartesianControl 
interface implemented) is running. 
 
\section portsc_sec Ports Created 
Aside from the internal ports created by \ref 
affActionPrimitives library, we also have: 
 
- \e /<modName>/in receives a bottle containing the 3-d position 
  of the object to grasp.
 
- \e /<modName>/rpc remote procedure call. 
    Recognized remote commands:
    -'quit' quit the module
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files 
--hand_sequences_file \e file 
- specify the path to the file containing the hand motion 
  sequences relative to the current context ( \ref
  affActionPrimitives ).
 
--from \e file 
- specify the configuration file (use \e --context option to 
  select the current context).
 
The configuration file passed through the option \e --from
should look like as follows:
 
\code 
[general]
// options used to open a affActionPrimitives object 
robot                           icub
thread_period                   50
default_exec_time               3.0
reach_tol                       0.007
verbosity                       on 
torso_pitch                     on
torso_roll                      off
torso_yaw                       on
torso_pitch_max                 30.0 
tracking_mode                   off 
verbosity                       on 
 
// arm-dependent options 
[left_arm]
grasp_orientation               (-0.171542 0.124396 -0.977292 3.058211)
grasp_displacement              (0.0 0.0 0.05)
systematic_error_displacement   (-0.03 -0.07 -0.02)
lifting_displacement            (0.0 0.0 0.2)
home_position                   (-0.29 -0.21 0.11)
home_orientation                (-0.029976 0.763076 -0.645613 2.884471)

[right_arm]
grasp_orientation               (-0.0191 -0.983248 0.181269 3.093746)
grasp_displacement              (0.0 0.0 0.05)
systematic_error_displacement   (-0.03 -0.07 -0.02)
lifting_displacement            (0.0 0.0 0.2)
home_position                   (-0.29 -0.21 0.11)
home_orientation                (-0.193426 -0.63989 0.743725 2.995693)
\endcode 

\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <iCub/actions/affActionPrimitives.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <deque>

#define USE_LEFT    0
#define USE_RIGHT   1

#define AFFACTIONPRIMITIVESLAYER    affActionPrimitivesLayer1

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace actions;


class exampleModule: public RFModule
{
protected:
    string partUsed;

	AFFACTIONPRIMITIVESLAYER *actionL;
    AFFACTIONPRIMITIVESLAYER *actionR;
    AFFACTIONPRIMITIVESLAYER *action;
	BufferedPort<Bottle>      inPort;
    Port                      rpcPort;

    Vector graspOrienL, graspOrienR;
    Vector graspDispL,  graspDispR;
    Vector dOffsL,      dOffsR;
    Vector dLiftL,      dLiftR;
    Vector home_xL,     home_xR;

    Vector *graspOrien;
    Vector *graspDisp;
    Vector *dOffs;
    Vector *dLift;
    Vector *home_x;

    bool openPorts;
    bool firstRun;

public:
    exampleModule()
	{		        
        graspOrienL.resize(4);    graspOrienR.resize(4);
        graspDispL.resize(4);     graspDispR.resize(3);
        dOffsL.resize(3);         dOffsR.resize(3);
        dLiftL.resize(3);         dLiftR.resize(3);
        home_xL.resize(3);        home_xR.resize(3);

        // default values for arm-dependent quantities
        graspOrienL[0]=-0.171542; graspOrienR[0]=-0.0191;
        graspOrienL[1]= 0.124396; graspOrienR[1]=-0.983248;
        graspOrienL[2]=-0.977292; graspOrienR[2]= 0.181269;
        graspOrienL[3]= 3.058211; graspOrienR[3]= 3.093746;

        graspDispL[0]= 0.0;       graspDispR[0]= 0.0;
        graspDispL[1]= 0.0;       graspDispR[1]= 0.0;
        graspDispL[2]= 0.05;      graspDispR[2]= 0.05;

        dOffsL[0]=-0.03;          dOffsR[0]=-0.03;
        dOffsL[1]=-0.07;          dOffsR[1]=-0.07;
        dOffsL[2]=-0.02;          dOffsR[2]=-0.02;

        dLiftL[0]= 0.0;           dLiftR[0]= 0.0;  
        dLiftL[1]= 0.0;           dLiftR[1]= 0.0;  
        dLiftL[2]= 0.15;          dLiftR[2]= 0.15; 
        
        home_xL[0]=-0.29;         home_xR[0]=-0.29;
        home_xL[1]=-0.21;         home_xR[1]= 0.24;
        home_xL[2]= 0.11;         home_xR[2]= 0.07;

        action=actionL=actionR=NULL;
        graspOrien=NULL;
        graspDisp=NULL;
        dOffs=NULL;
        dLift=NULL;
        home_x=NULL;

        openPorts=false;
        firstRun=true;
	}

    void getArmDependentOptions(Bottle &b, Vector &_gOrien, Vector &_gDisp,
                                Vector &_dOffs, Vector &_dLift, Vector &_home_x)
    {
        if (Bottle *pB=b.find("grasp_orientation").asList())
        {
            int sz=pB->size();
            int len=_gOrien.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _gOrien[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("grasp_displacement").asList())
        {
            int sz=pB->size();
            int len=_gDisp.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _gDisp[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("systematic_error_displacement").asList())
        {
            int sz=pB->size();
            int len=_dOffs.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _dOffs[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("lifting_displacement").asList())
        {
            int sz=pB->size();
            int len=_dLift.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _dLift[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("home_position").asList())
        {
            int sz=pB->size();
            int len=_home_x.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _home_x[i]=pB->get(i).asDouble();
        }
    }

    virtual bool configure(ResourceFinder &rf)
    {
        string name=rf.find("name").asString().c_str();
        setName(name.c_str());

        partUsed=rf.check("part",Value("both_arms")).asString().c_str();
        if (partUsed!="both_arms" && partUsed!="left_arm" && partUsed!="right_arm")
        {
            cout<<"Invalid part requested !"<<endl;
            return false;
        }        

        Property config; config.fromConfigFile(rf.findFile("from").c_str());
        Bottle &bGeneral=config.findGroup("general");
        if (bGeneral.isNull())
        {
            cout<<"Error: group general is missing!"<<endl;
            return false;
        }

        // parsing general config options
        Property option;
        for (int i=1; i<bGeneral.size(); i++)
        {
            Bottle *pB=bGeneral.get(i).asList();
            if (pB->size()==2)
                option.put(pB->get(0).asString().c_str(),pB->get(1));
            else
            {
                cout<<"Error: invalid option!"<<endl;
                return false;
            }
        }

        option.put("local",name.c_str());
        option.put("hand_sequences_file",rf.findFile("hand_sequences_file"));

        Property optionL(option); optionL.put("part","left_arm");
        Property optionR(option); optionR.put("part","right_arm");

        // parsing left_arm config options
        Bottle &bLeft=config.findGroup("left_arm");
        if (bLeft.isNull())
        {
            cout<<"Error: group left_arm is missing!"<<endl;
            return false;
        }
        else 
            getArmDependentOptions(bLeft,graspOrienL,graspDispL,
                                   dOffsL,dLiftL,home_xL);

        // parsing right_arm config options
        Bottle &bRight=config.findGroup("right_arm");
        if (bRight.isNull())
        {
            cout<<"Error: group right_arm is missing!"<<endl;
            return false;
        }
        else
            getArmDependentOptions(bRight,graspOrienR,graspDispR,
                                   dOffsR,dLiftR,home_xR);

        if (partUsed=="both_arms" || partUsed=="left_arm")
        {    
            cout<<"***** Instantiating primitives for left_arm"<<endl;
            actionL=new AFFACTIONPRIMITIVESLAYER(optionL);

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
            actionR=new AFFACTIONPRIMITIVESLAYER(optionR);

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

        string fwslash="/";
        inPort.open((fwslash+name+"/in").c_str());
        rpcPort.open((fwslash+name+"/rpc").c_str());
        attach(rpcPort);

        openPorts=true;

        return true;
    }

    virtual bool close()
    {
		if (actionL!=NULL)
			delete actionL;
		
        if (actionR!=NULL)
            delete actionR;

        if (openPorts)
        {
            inPort.close();
            rpcPort.close();
        }

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
            dLift=&dLiftL;
            home_x=&home_xL;
        }
        else if (arm==USE_RIGHT)
        {
            action=actionR;

            graspOrien=&graspOrienR;
            graspDisp=&graspDispR;
            dOffs=&dOffsR;
            dLift=&dLiftR;
            home_x=&home_xR;
        }
    }

    void init()
    {
        bool f;

        if (partUsed=="both_arms" || partUsed=="right_arm")
        {
            useArm(USE_RIGHT);
            action->pushAction(*home_x,"open_hand");
            action->checkActionsDone(f,true);
            action->enableArmWaving(*home_x);
        }

        if (partUsed=="both_arms" || partUsed=="left_arm")
        {
            useArm(USE_LEFT);
            action->pushAction(*home_x,"open_hand");
            action->checkActionsDone(f,true);
            action->enableArmWaving(*home_x);
        }
    }

    // we don't need a thread since the actions library already
    // incapsulates one inside dealing with all the tight time constraints
    virtual bool updateModule()
	{
        // do it only once
        if (firstRun)
        {
            init();
            firstRun=false;
        }

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

            // grasp (wait until it's done)
			action->grasp(xd,*graspOrien,*graspDisp);
            action->checkActionsDone(f,true);
            action->areFingersInPosition(f);

            // if fingers are not in position,
            // it's likely that we've just grasped
            // something, so lift it up!
            if (!f)
            {
                cout<<"Wow, got something!"<<endl;

                // lift the object (wait until it's done)
    			action->pushAction(xd+*dLift,*graspOrien);
                action->checkActionsDone(f,true);
            }
            else
                cout<<"Sorry :( ... nothing to grasp"<<endl;

            // release the object or just open the
            // hand (wait until it's done)
            action->pushAction("open_hand");
            action->checkActionsDone(f,true);

            // go home :) (wait until it's done, since
            // we may use two arms that share the torso)
            action->pushAction(*home_x);
            action->checkActionsDone(f,true);

            // let the hand wave a bit around home position
            // the waving will be disabled before commencing
            // a new action
            action->enableArmWaving(*home_x);
		}		

		return true;
	}

	bool interruptModule()
	{
        // since a call to checkActionsDone() blocks
        // the execution until it's done, we need to 
        // take control and exit from the waiting state
		action->syncCheckInterrupt(true);        

        inPort.interrupt();
        rpcPort.interrupt();

		return true;
	}
};


int main(int argc, char *argv[])
{
    Network yarp;	

    if (!yarp.checkNetwork())
        return -1;

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("affActionPrimitivesExample/conf");
    rf.setDefaultConfigFile("config.ini");
    rf.setDefault("hand_sequences_file","hand_sequences.ini");
    rf.setDefault("name","affActionPrimitivesMod");
    rf.configure("ICUB_ROOT",argc,argv);

    exampleModule mod;

    return mod.runModule(rf);
}



