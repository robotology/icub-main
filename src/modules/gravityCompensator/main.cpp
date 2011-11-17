/**
@ingroup icub_module

\defgroup gravityCompensator gravityCompensator
 
Estimates the gravitational contribute to motors based on estimation of the robot dynamics
 
Copyright (C) 2008 RobotCub Consortium
 
Author: Matteo Fumagalli
 
Date: first release 24/07/2010 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description

This module estimates the gravitational term acting on joints due to links weight.
The estimation is perfomed relying on rigid body dynamics using CAD 
parameters. 
For further information about the use of this module and of the iCub force control interface, please refer to the force control page:
http://eris.liralab.it/wiki/Force_Control

\section lib_sec Libraries 
- YARP libraries. 
- iDyn library.  

\section parameters_sec Parameters

--rate \e r 
- The parameter \e r identifies the rate the thread will work. If not
  specified \e 20ms is assumed. The minimum suggested rate is \e 10ms.

--no_legs
- This option disables the gravity compensation for the legs joints.

\section portsa_sec Ports Accessed
The port the service is listening to.

\section portsc_sec Ports Created
None.
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section conf_file_sec Configuration Files
None
 
\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example
By launching the following command: 
 
\code 
gravityCompensator 
\endcode 
 
the module add offset values which are assigned to the IImpedanceControl interface and ITorqueControl interface
  
\author Matteo Fumagalli

This file can be edited at \in src/gravityCompensator/main.cpp.
*/ 

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include <iostream>
#include <iomanip>
#include <string.h>

#include "gravityThread.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace std;



class gravityModuleCompensator: public RFModule
{
private:
	int rate;
	gravityCompensatorThread *g_comp;

	Property OptionsLeftArm;
    Property OptionsRightArm;
    Property OptionsHead;
    Property OptionsLeftLeg;
    Property OptionsRightLeg;
    Property OptionsTorso;

    Port rpcPort;

    PolyDriver *dd_left_arm;
    PolyDriver *dd_right_arm;
    PolyDriver *dd_head;
    PolyDriver *dd_left_leg;
    PolyDriver *dd_right_leg;
    PolyDriver *dd_torso;

	bool legs_enabled;
	bool right_arm_enabled;
	bool left_arm_enabled;
	string m_side;
	string m_part;

public:
    gravityModuleCompensator()
    {
		legs_enabled  = true;
		left_arm_enabled  = true;
		right_arm_enabled  = true;
		dd_left_arm   = 0;
		dd_right_arm  = 0;
		dd_head       = 0;
		dd_left_leg   = 0;
		dd_right_leg  = 0;
		dd_torso      = 0;
		m_side = "right";
		m_part = "arm";
    }

   virtual bool createDriver(PolyDriver *&_dd, Property options)
    {
        int trials=0;
        double start_time = yarp::os::Time::now();

        do
        {
            double current_time = yarp::os::Time::now();

            //remove previously existing drivers
            if (_dd)
            {
                delete _dd;
                _dd=0;
            }

            //creates the new device driver
            _dd = new PolyDriver(options);
            bool connected =_dd->isValid();

            //check if the driver is connected
            if (connected) break;
        
            //check if the timeout (60s) is expired
            if (current_time-start_time > 60.0)
            {
                fprintf(stderr,"It is not possible to instantiate the device driver. I tried %d times!\n", trials);
                return false;
            }

            yarp::os::Time::delay(5);
            trials++;
            fprintf(stderr,"\nUnable to connect the device driver, trying again...\n");
        }
        while (true);

		IEncoders         *encs     = 0;
		IControlMode      *ctrlMode = 0;
		IImpedanceControl *imp      = 0;
		ITorqueControl    *tqs      = 0;

        bool ok = true;
        ok = ok & _dd->view(encs);
        ok = ok & _dd->view(ctrlMode);
        ok = ok & _dd->view(imp);
        ok = ok & _dd->view(tqs);
        if (!ok)
        {
            fprintf(stderr,"ERROR: one or more devices has not been viewed\nreturning...");
            return false;
        }

        return true;
    }

    bool configure(ResourceFinder &rf)
    {		
		string fwdSlash = "/";

        string name;
        name = "gravityCompensator";	
        
        int rate;
        if (rf.check("rate"))
            rate = rf.find("rate").asInt();
        else rate = 20;

		//-----------------GET THE ROBOT NAME-------------------//
		string robot_name;
		if (rf.check("robot"))
			 robot_name = rf.find("robot").asString();
		else robot_name = "icub";

		//------------SPECIAL PARAM TP DEFINE THE HEAD TYPE-----//
		string icub_type;
		if (rf.check("head_v2"))
		{
			fprintf(stderr,"'head_v2' option found. Using icubV2 head kinematics.\n");
			icub_type = "V2";
		}

		//------------------CHECK IF LEGS ARE ENABLED-----------//
		if (rf.check("no_legs"))
		{
			legs_enabled= false;
			fprintf(stderr,"'no_legs' option found. Legs will be disabled.\n");
		}
		//------------------CHECK IF ARMS ARE ENABLED-----------//
		if (rf.check("no_left_arm"))
		{
			left_arm_enabled= false;
			fprintf(stderr,"'no_left_arm' option found. Left arm will be disabled.\n");
		}
		//------------------CHECK IF ARMS ARE ENABLED-----------//
		if (rf.check("no_right_arm"))
		{
			right_arm_enabled= false;
			fprintf(stderr,"'no_right_arm' option found. Right arm will be disabled.\n");
		}
        //---------------------DEVICES--------------------------//
        
		OptionsHead.put("device","remote_controlboard");
		OptionsHead.put("local","/gravityCompensator/head/client");
		OptionsHead.put("remote",string("/"+robot_name+"/head").c_str());

		if (!createDriver(dd_head, OptionsHead))
		{
			fprintf(stderr,"ERROR: unable to create head device driver...quitting\n");
			return false;
		}
		else
			fprintf(stderr,"device driver created\n");
        
		if (left_arm_enabled)
		{
			OptionsLeftArm.put("device","remote_controlboard");
			OptionsLeftArm.put("local","/gravityCompensator/left_arm/client");
			OptionsLeftArm.put("remote",string("/"+robot_name+"/left_arm").c_str());
			if (!createDriver(dd_left_arm,OptionsLeftArm))
			{
				fprintf(stderr,"ERROR: unable to create left arm device driver...quitting\n");
				return false;
			}
		}

		if (right_arm_enabled)
		{
			OptionsRightArm.put("device","remote_controlboard");
			OptionsRightArm.put("local","/gravityCompensator/right_arm/client");
			OptionsRightArm.put("remote",string("/"+robot_name+"/right_arm").c_str());
			if (!createDriver(dd_right_arm,OptionsRightArm))
			{
				fprintf(stderr,"ERROR: unable to create right arm device driver...quitting\n");
				return false;
			}
		}

		if (legs_enabled)
		{
			OptionsLeftLeg.put("device","remote_controlboard");
			OptionsLeftLeg.put("local","/gravityCompensator/left_leg/client");
			OptionsLeftLeg.put("remote",string("/"+robot_name+"/left_leg").c_str());
			if (!createDriver(dd_left_leg,OptionsLeftLeg))
			{
				fprintf(stderr,"ERROR: unable to create left leg device driver...quitting\n");
				return false;
			}

			OptionsRightLeg.put("device","remote_controlboard");
			OptionsRightLeg.put("local","/gravityCompensator/right_leg/client");
			OptionsRightLeg.put("remote",string("/"+robot_name+"/right_leg").c_str());
			if (!createDriver(dd_right_leg,OptionsRightLeg))
			{
				fprintf(stderr,"ERROR: unable to create right leg device driver...quitting\n");
				return false;
			}
		}
		
		OptionsTorso.put("device","remote_controlboard");
		OptionsTorso.put("local","/gravityCompensator/torso/client");
		OptionsTorso.put("remote",string("/"+robot_name+"/torso").c_str());

		if (!createDriver(dd_torso,OptionsTorso))
		{
			fprintf(stderr,"ERROR: unable to create head device driver...quitting\n");
			return false;
		}
		else
			fprintf(stderr,"device driver created\n");

        rpcPort.open(("/"+name+"/rpc").c_str());
        attach(rpcPort);        


        //--------------------------THREAD--------------------------

        g_comp = new gravityCompensatorThread(rate, dd_left_arm, dd_right_arm, dd_head, dd_left_leg, dd_right_leg, dd_torso, icub_type);
        fprintf(stderr,"ft thread istantiated...\n");
        g_comp->start();
        fprintf(stderr,"thread started\n");
        return true;
    }

	bool respond(const Bottle& command, Bottle& reply) 
	{
		Bottle position_bot;
		string helpMessage =  string(getName().c_str()) + 
							" commands are: \n" +  
							"help       to display this message\n" + 
							"on         to set the gravity compensation term \n" + 
							"off        to set the zero torque reference \n";

		  reply.clear(); 
		if (command.get(0).asString()=="help")
		{
			cout << helpMessage;
			reply.addString(helpMessage.c_str());
		}
		else if (command.get(0).asString()=="on" ||
			     command.get(0).asString()=="ON" )
		{
			if (g_comp) 
			{
				g_comp->gravity_mode = GRAVITY_COMPENSATION_ON;
				reply.addString("assigned gravity compensation feed-forward term");
			}
	    }
		else if (command.get(0).asString()=="off" ||
			     command.get(0).asString()=="OFF" )
	    {
			if (g_comp)
			{
				g_comp->gravity_mode = GRAVITY_COMPENSATION_OFF;
				reply.addString("gravity compensation off");
			}
	    }
	    else
		{
			reply.addString("unknown command. type help.");
		}
		
		return true;
	}


    bool close()
    {
		//stop thread 
		if(g_comp)
        {
            g_comp->stop();
            delete g_comp; g_comp = 0;
        }
		
		//closing interfaces
		if (dd_left_arm)	{delete dd_left_arm;  dd_left_arm=0;  }
		if (dd_right_arm)	{delete dd_right_arm; dd_right_arm=0; }
		if (dd_left_leg)	{delete dd_left_leg;  dd_left_leg=0;  }
		if (dd_right_leg)	{delete dd_right_leg; dd_right_leg=0; }
		if (dd_head)	    {delete dd_head;      dd_head=0;      }
		if (dd_torso)	    {delete dd_torso;     dd_torso=0;     }

		//closing ports
        rpcPort.interrupt();
		rpcPort.close();

        return true;
    }

    double getPeriod()  { return 1.0;  }
    bool updateModule()
	{
	    static unsigned long int alive_counter = 0;
        static double curr_time = Time::now();
        if (Time::now() - curr_time > 60)
        {
            printf ("gravityCompensator is alive! running for %ld mins.\n",++alive_counter);
            curr_time = Time::now();
        }

		if (g_comp==0) return false;

		thread_status_enum thread_status = g_comp->getThreadStatus();

		if (thread_status==STATUS_OK)
			return true;
		else if (thread_status==STATUS_DISCONNECTED)
		{
			printf ("gravityCompensator module lost connection with iCubInterface, now closing...\n");
			return false;
		}
		else
		{
			fprintf(stderr,"gravityCompensator module was closed successfully! \n");    
			return true;
		}
	}
};


int main(int argc, char * argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--context context: where to find the called resource (referred to $ICUB_ROOT/app: default wrechObserver/conf)" << endl;
        cout << "\t--from       from: the name of the file.ini to be used for calibration"                                        << endl;
        cout << "\t--rate       rate: the period used by the module. default 100ms (not less than 15ms)"						  << endl;
        cout << "\t--no_legs    this option disables the gravity compensation for the legs joints"								  << endl;
		cout << "\t--head_v2    use the model of the headV2" << endl;  
		cout << "\t--no_left_arm    disables the left arm" << endl;
		cout << "\t--no_right_arm   disabled the right arm" << endl;
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    gravityModuleCompensator gcomp;

    return gcomp.runModule(rf);
}


