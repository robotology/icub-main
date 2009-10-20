/** 
\defgroup iKinGazeCtrl iKinGazeCtrl 
 
@ingroup icub_module  
 
Gaze controller based on iKin.

Copyright (C) 2008 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description

This module provides a controller for the iCub gaze capable of 
steering the neck and the eyes independently performing 
saccades, pursuit, vergence and VOC (vestibulo-oculocollic 
reflex). VOR (vestibulo-ocular reflex relying on inertial data) 
is not provided at time being but can be easily implemented. 
 
The controller can be seen as cartesian gaze controller since it
receives as input a 3D position in the task space or, in turn, 
can convert the relative position of the target in the two image
planes in relative displacement in 3D task space with respect to 
the actual fixation point. 
 
\note A video on iCub gazing at a target can be seen at
      http://eris.liralab.it/misc/icubvideos/gazing_IIT_ISR.wmv

\section lib_sec Libraries 
- YARP libraries. 
- \ref iKin "iKin" library (it requires IPOPT lib: see 
  http://eris.liralab.it/wiki/Installing_IPOPT ).

\section parameters_sec Parameters
--ctrlName \e name 
- The parameter \e name identifies the controller's name; all 
  the open ports will be tagged with the prefix
  /<ctrlName>/<part>/. If not specified \e iKinGazeCtrl is
  assumed.
 
--robot \e name 
- The parameter \e name selects the robot name to connect to; if
  not specified \e icub is assumed.
 
--part \e name 
- The parameter \e type selects the robot's head port to connect
  to; if not specified \e head is assumed.

--torso \e name 
- The parameter \e name selects the robot's torso port to 
  connect to; if not specified \e torso is assumed.
 
--inertial \e name 
- The parameter \e name selects the robot's inertial port to 
  connect to; if not specified \e inertial is assumed.
 
--Tneck \e time
- specify the neck movements time in seconds; by default \e time
  is 0.70 seconds. (Tneck cannot be lower than Teyes).
 
--Teyes \e time
- specify the eyes movements time in seconds; by default \e time
  is 0.20 seconds.
 
--cx \e x 
- The parameter \e x represents the central x image plane 
  coordinate in pixel; by default \e x = 160.
 
--cy \e y 
- The parameter \e y represents the central y image plane 
  coordinate in pixel; by default \e y = 120.
 
--config \e file 
- The parameter \e file specifies the file name used for 
  aligning eyes kinematic to image planes (see below).
 
--context \e dir
- Resource finder searching dir for config file.
 
--noRobot
- avoid connecting to robot: simulation only.
 
\section portsa_sec Ports Accessed
 
The ports the module is connected to: e.g. 
/icub/head/command:i and so on. 

\section portsc_sec Ports Created 
 
There are two different ways of commanding a new target fixation 
point: 
 
- by sending the absolute 3D position to gaze at in the task 
  space through /<ctrlName>/xd:i port.
- by localizing the target in the two image planes and thus 
  sending its relative coordinates to /<ctrlName>/pixel:i port.
 
The module creates the usual ports required for the 
communication with the robot (through interfaces) and the 
following ports: 
 
- \e /<ctrlName>/<part>/xd:i receives the target fixation point. 
  It accepts 3 double (also as a Bottle object) for xyz
  coordinates.
 
- \e /<ctrlName>/<part>/pixel:i receives the current target 
  position expressed in image planes. It accepts 4 double (also
  as a Bottle object) in this order: [leftx lefty rightx
  righty].
 
- \e /<ctrlName>/<part>/x:o returns the actual fixation point 
  (Vector of 3 double).
 
- \e /<ctrlName>/<part>/qd:o returns the target joints 
  configuration which achieves the target fixation point (Vector
  of 9 double: 3 for the non-controlled torso, 3 for neck and 3
  for eyes). The order for torso angles is the one defined by
  kinematic chain (reversed order). Units in deg.
 
- \e /<ctrlName>/<part>/q:o returns the actual joints 
  configuration during movement (Vector of 9 double). The order
  for torso angles is the one defined by kinematic chain
  (reversed order). Units in deg.

- \e /<ctrlName>/<part>/v:o returns the computed joints 
  velocities which steers the head to gaze at the target
  fixation point (Vector of 6 double). Units in deg/s.
 
- \e /<ctrlName>/<part>/rpc remote procedure call. 
    Recognized remote commands:
    -'quit' quit the module
    -'susp' suspend the controller
    -'run' resume the controller
 
\section coor_sys_sec Coordinate System 
Positions (meters) refer to the root reference frame attached to
the waist as in 
http://eris.liralab.it/wiki/ICubForwardKinematics . 

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files
The configuration file passed through the option \e --config 
should contain the fields required to reconstruct the virtual 
links (given in terms of lenght,offset,twist,joint parameters) 
which are appended to the eye kinematic in order to achieve the 
alignment with the optical axes. 
 
Example: 
 
\code 
[LEFT]
length 4.15777e-007 -4.44085e-007   // [m]
offset 1.11013e-006 -1.84301e-007   // [m]
twist  0.163382 -0.12788            // [rad]
joint  -0.181402 0.20654            // [rad]

[RIGHT]
length 0.000100176 -0.000105106     // [m]
offset 0.000103259 -9.96225e-005    // [m]
twist  0.296114 -0.194269           // [rad]
joint  -0.34062 0.294211            // [rad]
\endcode 
 
\section tested_os_sec Tested OS
Windows, Linux

Note that \ref iKinGazeView "iKinGazeView" module can show
interactively the iCub head configuration launching MATLAB in 
background. Just connect the ports with the viewer and play. 

\author Ugo Pattacini
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iostream>
#include <iomanip>
#include <string>

#include "localizer.h"
#include "solver.h"
#include "controller.h"

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;


// Usual YARP stuff...
class CtrlModule: public RFModule
{
protected:
    Localizer     *loc;
    EyePinvRefGen *eyesRefGen;
    Solver        *slv;
    Controller    *ctrl;
    PolyDriver    *drvTorso, *drvHead;
    exchangeData   commData;
    Port           rpcPort;

public:
    CtrlModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        string ctrlName;
        string robotName;
        string partName;
        string torsoName;
        string inertialName;
        string configFile;
        double neckTime;
        double eyesTime;
        int    cx, cy;
        bool   Robotable;

        Time::turboBoost();

        if (rf.check("ctrlName"))
            ctrlName=rf.find("ctrlName").asString();
        else
            ctrlName="iKinGazeCtrl";

        if (rf.check("robot"))
            robotName=rf.find("robot").asString();
        else
            robotName="icub";

        if (rf.check("part"))
            partName=rf.find("part").asString();
        else
            partName="head";

        if (rf.check("torso"))
            torsoName=rf.find("torso").asString();
        else
            torsoName="torso";

        if (rf.check("inertial"))
            inertialName=rf.find("inertial").asString();
        else
            inertialName="inertial";

        if (rf.check("Tneck"))
            neckTime=rf.find("Tneck").asDouble();
        else
            neckTime=0.70;

        if (rf.check("Teyes"))
            eyesTime=rf.find("Teyes").asDouble();
        else
            eyesTime=0.20;

        if (rf.check("cx"))
            cx=rf.find("cx").asInt();
        else
            cx=160;

        if (rf.check("cy"))
            cy=rf.find("cy").asInt();
        else
            cy=120;

        if (rf.check("noRobot"))
            Robotable=false;
        else
            Robotable=true;

        if (rf.check("config"))
        {    
            configFile=rf.findFile(rf.find("config").asString());

            if (configFile=="")
                return false;
        }
        else
            configFile.clear();

        Property optHead("(device remote_controlboard)");
        Property optTorso("(device remote_controlboard)");

        string remoteHeadName="/"+robotName+"/"+partName;
        string localHeadName="/"+ctrlName+"/"+partName;
        optHead.put("remote",remoteHeadName.c_str());
        optHead.put("local",localHeadName.c_str());

        string remoteTorsoName="/"+robotName+"/"+torsoName;
        string localTorsoName=localHeadName+"/"+torsoName;
        optTorso.put("remote",remoteTorsoName.c_str());
        optTorso.put("local",localTorsoName.c_str());

        if (Robotable)
        {
            drvTorso=new PolyDriver(optTorso);
            drvHead =new PolyDriver(optHead);

            if (!drvTorso->isValid() || !drvHead->isValid())
            {
                cout << "Device drivers not available!" << endl;

                delete drvTorso;
                delete drvHead;

                return false;
            }
        }
        else
            drvTorso=drvHead=NULL;

        // starts threads
        loc=new Localizer(&commData,localHeadName,cx,cy,20);
        loc->start();

        eyesRefGen=new EyePinvRefGen(drvTorso,drvHead,&commData,robotName,
                                     localHeadName,inertialName,configFile,20);        
        eyesRefGen->start();

        slv=new Solver(drvTorso,drvHead,&commData,eyesRefGen,loc,
                       localHeadName,configFile,20);
        slv->start();

        ctrl=new Controller(drvTorso,drvHead,&commData,robotName,
                            localHeadName,neckTime,eyesTime,10,5);
        ctrl->start();

        string rpcPortName=localHeadName+"/rpc";
        rpcPort.open(rpcPortName.c_str());
        attach(rpcPort);
        attachTerminal();

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
                    ctrl->suspend();
                    reply.addVocab(Vocab::encode("ack"));
                    return true;
                }

                case VOCAB3('r','u','n'):
                {
                    slv->setStart();
                    ctrl->resume();
                    reply.addVocab(Vocab::encode("ack"));
                    return true;
                }

                default:
                    return RFModule::respond(command,reply);
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
        loc->stop();
        eyesRefGen->stop();
        slv->stop();
        ctrl->stop();

        delete loc;
        delete eyesRefGen;
        delete slv;
        delete ctrl;

        if (drvTorso)
            delete drvTorso;

        if (drvHead)
            delete drvHead;

        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};



int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("default/conf");
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--ctrlName  name: controller name (default iKinGazeCtrl)"                     << endl;
        cout << "\t--robot     name: robot name to connect to (default: icub)"                   << endl;
        cout << "\t--part      name: robot head port name, (default: head)"                      << endl;
        cout << "\t--torso     name: robot torso port name (default: torso)"                     << endl;
        cout << "\t--inertial  name: robot inertial port name (default: inertial)"               << endl;
        cout << "\t--Tneck     time: specify the neck movements time in seconds (default: 0.70)" << endl;
        cout << "\t--Teyes     time: specify the eyes movements time in seconds (default: 0.20)" << endl;
        cout << "\t--cx           x: central x coordinate of image plane (default: 160)"         << endl;
        cout << "\t--cy           y: central y coordinate of image plane (default: 120)"         << endl;
        cout << "\t--config    file: file name for aligning eyes kinematic to image planes"      << endl;
        cout << "\t--context    dir: resource finder searching dir for config file"              << endl;
        cout << "\t--noRobot       : avoid connecting to robot (simulation purpose)"             << endl;

        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    CtrlModule mod;

    return mod.runModule(rf);
}



