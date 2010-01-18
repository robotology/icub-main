/** 
\defgroup PredictiveSP 
 
@ingroup icub_module  
 
A predictive smoot pursuit eye movement based on online learning of target trajectories

Copyright (C) 2009 RobotCub Consortium
 


CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description

This module provides a controller for the smooth pursuit eye movement
capable of prediction of the target trajectories. The predictor is based 
on the RLS algorithm. After an initial learning phase the eye velocity 
is aligned to object velocity  



/*--ctrlName \e name 
- The parameter \e name identifies the controller's name; all 
  the open ports will be tagged with the prefix
  /<ctrlName>/<part>/. If not specified \e PredictiveSP is
  assumed.
 
--robot \e name 
- The parameter \e name selects the robot name to connect to; if
  not specified \e icub is assumed.
 


--fps \e fps
	The parameter \e fps is the minimum frame rate of the three input ports.
	If not speified \e 30.0 is assumed.

--joint \e joint
	The parameter \e joint is the joint number that will be controlled.
	If not speified \e 4 is assumed.
	

	
The module creates the usual ports required for the 
communication with the robot (through interfaces) and the 
following ports: 

- \e /<ctrlName>/stateTarget:i receives the current target 
  position and velocity expressed in image planes. It accepts 2 double 
  in this order: [pos vel].

- \e /<ctrlName>/velJoint:i receives the current joint velocity 
	expressed in image planes. It accepts 2 double 
	in this order: [pos vel].

- \e /<ctrlName>/rpc remote procedure call. 
    Recognized remote commands:
    -'quit' quit the module
    -'susp' suspend the controller
    -'run' resume the controller


\section tested_os_sec Tested OS
Windows
*/  
#include <ace/config.h>
#include <stdio.h>

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>


#include <yarp/sig/all.h>
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iostream>
#include <iomanip>
#include <string>


#include "PredSmoothP.h"
using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;




class CtrlModule: public Module
{
protected:

	PolyDriver     *drvHead;
	Camera *camera;
    BufferedPort<Bottle> rpcPort;

public:
    CtrlModule() { }

    virtual bool open(Searchable &s)
    {
        string ctrlName;
        string robotName;
		string partName;
		string configFile;
		double fps;		
        
		   Time::turboBoost();

        Property options(s.toString());

        if (options.check("ctrlName"))
            ctrlName=options.find("ctrlName").asString();
        else
            ctrlName="PredictiveSP";


       if (!options.check("device"))
            options.put("device","remote_controlboard");

        if (options.check("robot"))
            robotName=options.find("robot").asString();
        else
            robotName="icub";

        

		 if (options.check("fps"))
            fps=options.find("fps").asDouble();
        else
            fps=30.0;

    
		Property optHead(options);
        
        string remoteHeadName="/"+robotName;
        string localHeadName="/"+ctrlName;
        optHead.put("remote",remoteHeadName.c_str());
        optHead.put("local",localHeadName.c_str());

       

        drvHead =new PolyDriver(optHead);

        if (!drvHead->isValid())
            {
                cout << "Device drivers not available!" << endl;
                return false;
            }
		
         

        string rpcPortName=localHeadName+"/rpc";
        rpcPort.open(rpcPortName.c_str());
        attach(rpcPort,true);

        return true;
    }

    virtual bool respond(const Bottle &command, Bottle &reply)
    {
        cout << "Receiving command from rpc port" << endl;

        if (command.size())
        {
            switch (command.get(0).asVocab())
            {
                case VOCAB4('s','u','s','p'):
                {
                    
                    reply.addVocab(Vocab::encode("ack"));
                    return true;
                }

                case VOCAB3('r','u','n'):
                {
                    
                    reply.addVocab(Vocab::encode("ack"));
                    return true;
                }

                default:
                    return Module::respond(command,reply);
            }
        }
        else
        {
            reply.add("command size==0");
            return false;
        }
    }

    virtual bool close()
    {
       

        rpcPort.close();

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
	virtual bool   updateModule() { 
		return true; }
};



int main(int argc, char *argv[])
{
    Property options;
    options.fromCommand(argc, argv);
    if (options.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--ctrlName  name: controller name (default PredictiveSP)"                     << endl;
        cout << "\t--robot     name: robot name to connect to (default: icub)"                << endl;
        cout << "\t--fps       fps: minimum frame rate of input port, (default: 30)"            << endl;
		cout << "\t--joint     joint: robot head port name, (default: head)"                      << endl;
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
		return -1;

    CtrlModule mod;

    return mod.runModule(argc,argv);
	
}



