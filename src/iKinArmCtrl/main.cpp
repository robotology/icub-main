/** 
\defgroup iKinArmCtrl iKinArmCtrl 
 
@ingroup icub_module  
 
Multi-Referential framework together with IPOPT applied to iCub 
arm to solve the velocities trajectories planning task 
(reaching). 

Copyright (C) 2008 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description

This module provides a controller for the iCub arm which 
implements the multi-referential approach based on the paper 
from Micha Hersch available <a 
href="http://infoscience.epfl.ch/record/114045">here</a>. Unlike
the Micha Hersch baseline, the VITE's controllers are 
substituted with time-base minimum jerk generators. Essentially,
the generated trajectories of end-effector are quasi-straight 
similar to the ones measured in human reaching movements. The 
controller is capable of handling the complete pose (xyz 
cartesian position + orientation in axis/angle mode). 
 
The module can be selected to control a usual 7-DOF arm or the 
extended 10-DOF arm which includes the torso structure.
 
The approach untertaken herein is to discouple the kinematic 
Solver part (running the IPOPT algorithm in this implementation) 
from the Controller part. The underlying reason is that the 
Solver may require some computational effort from time to time 
depending on the current pose to be attained and the chosen 
algorithm, it should not interrupt the Controller and, finally, 
it can be called at different rate. 
 
\note A video on iCub performing grasp-priming can be seen <a 
      href="http://eris.liralab.it/misc/icubvideos/reaching_IIT_ISR.wmv">here</a>.
 
\note A video on iCub courting my sister :) can be seen <a 
      href="http://eris.liralab.it/misc/icubvideos/icub_courting.wmv">here</a>.
 
\section lib_sec Libraries 
- YARP libraries. 
- \ref iKin "iKin" library (it requires IPOPT lib: see the <a 
  href="http://eris.liralab.it/wiki/Installing_IPOPT">wiki</a>).

\section parameters_sec Parameters
--ctrlName \e name 
- The parameter \e name identifies the controller's name; all 
  the open ports will be tagged with the prefix
  /<ctrlName>/<part>/. If not specified \e iKinArmCtrl is
  assumed.
 
--robot \e name 
- The parameter \e name selects the robot name to connect to; if
  not specified \e icub is assumed.
 
--part \e type 
- The parameter \e type selects the robot's arm to work with. It
  can be \e right_arm or \e left_arm; if not specified
  \e right_arm is assumed.

--torso \e name 
- The parameter \e name selects the robot's torso port to 
  connect to; if not specified \e torso is assumed.
 
--T \e time
- specify the task execution time in seconds; by default \e time
  is 2.0 second.
 
--DOF8
- enable the control of torso yaw joint. 
 
--DOF9
- enable the control of torso yaw/pitch joints. 
 
--DOF10
- enable the control of torso yaw/roll/pitch joints. 
 
--onlyXYZ  
- disable orientation control. 
 
--noRobot
- avoid connecting to robot: simulation only.
 
\section portsa_sec Ports Accessed
 
The ports the module is connected to: e.g. 
/icub/right_arm/command:i and so on. 

\section portsc_sec Ports Created 
 
The module creates the usual ports required for the 
communication with the robot (through interfaces) and the 
following ports: 
 
- \e /<ctrlName>/<part>/xd:i receives the target end-effector 
  pose. It accepts 7 double (also as a Bottle object): 3 for xyz
  and 4 for orientation in axis/angle mode.
 
- \e /<ctrlName>/<part>/x:o returns the actual end-effector 
  position in task space during movement (Vector of 7 double:
  xyz + axis_xyz + theta).
 
- \e /<ctrlName>/<part>/qd:o returns the target joints 
  configuration which achieves the target pose xd (Vector of 10
  double). The order for torso angles is the one defined by
  kinematic chain (reversed order). Units in deg.
 
- \e /<ctrlName>/<part>/q:o returns the actual joints 
  configuration during movement (Vector of 10 double). The order
  for torso angles is the one defined by kinematic chain
  (reversed order). Units in deg.

- \e /<ctrlName>/<part>/v:o returns the computed joints 
  velocities which steers the arm toward the target pose (Vector
  of 10 double). Units in deg/s.
 
- \e /<ctrlName>/<part>/rpc remote procedure call. 
    Recognized remote commands:
    -'quit' quit the module
    -'susp' suspend the controller
    -'run' resume the controller
 
\section coor_sys_sec Coordinate System 
Positions (meters) and orientation (radians) refer to the root 
reference frame attached to the waist as in the <a 
href="http://eris.liralab.it/wiki/ICubForwardKinematics">wiki</a>. 
 
To specify a target orientation you shall provide three 
components for the rotation axis (whose norm is 1) and a fourth 
component for the rotation angle in radians as required by the 
axis/angle notation (click <a 
href="http://en.wikipedia.org/wiki/Axis_angle">here</a>). The 
reference axes still belong to the root frame; as result, 
commanding for instance a target orientation of [0 1 0 
30*pi/180] will steer the axes attached to the end-effector in a 
way to attain a final orientation equal to the rotation of the 
root axes of 30° around y (y in the root frame). 

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files
None. 
 
\section tested_os_sec Tested OS
Windows, Linux

Note that \ref iKinArmView "iKinArmView" module can show
interactively the iCub arm configuration launching MATLAB in 
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

#include "utils.h"
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
    Solver       *slv;
    Controller   *ctrl;
    PolyDriver   *drvTorso, *drvArm;
    exchangeData  commData;
    Port          rpcPort;

public:
    CtrlModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        string ctrlName;
        string robotName;
        string partName;
        string torsoName;
        double execTime;
        unsigned int ctrlPose;
        unsigned int ctrlTorso;
        bool Robotable;

        Time::turboBoost();

        if (rf.check("ctrlName"))
            ctrlName=rf.find("ctrlName").asString();
        else
            ctrlName="iKinArmCtrl";

        if (rf.check("robot"))
            robotName=rf.find("robot").asString();
        else
            robotName="icub";

        if (rf.check("part"))
            partName=rf.find("part").asString();
        else
            partName="right_arm";

        if (rf.check("torso"))
            torsoName=rf.find("torso").asString();
        else
            torsoName="torso";

        if (rf.check("T"))
            execTime=rf.find("T").asDouble();
        else
            execTime=2.0;

        if (rf.check("DOF10"))
            ctrlTorso=3;
        else if (rf.check("DOF9"))
            ctrlTorso=2;
        else if (rf.check("DOF8"))
            ctrlTorso=1;
        else
            ctrlTorso=0;

        if (rf.check("onlyXYZ"))
            ctrlPose=IKINCTRL_POSE_XYZ;
        else
            ctrlPose=IKINCTRL_POSE_FULL;

        if (rf.check("noRobot"))
            Robotable=false;
        else
            Robotable=true;

        Property optArm("(device remote_controlboard)");
        Property optTorso("(device remote_controlboard)");

        string remoteArmName="/"+robotName+"/"+partName;
        string localArmName="/"+ctrlName+"/"+partName;
        optArm.put("remote",remoteArmName.c_str());
        optArm.put("local",localArmName.c_str());

        string remoteTorsoName="/"+robotName+"/"+torsoName;
        string localTorsoName=localArmName+"/"+torsoName;
        optTorso.put("remote",remoteTorsoName.c_str());
        optTorso.put("local",localTorsoName.c_str());

        if (Robotable)
        {
            drvTorso=new PolyDriver(optTorso);
            drvArm  =new PolyDriver(optArm);

            if (!drvTorso->isValid() || !drvArm->isValid())
            {
                cout << "Device drivers not available!" << endl;

                delete drvTorso;
                delete drvArm;

                return false;
            }
        }
        else
            drvTorso=drvArm=NULL;

        // Note that Solver and Controller operate on
        // different arm objects (instantiated internally
        // and separately) in order to avoid any interaction.
        slv=new Solver(drvTorso,drvArm,&commData,localArmName,partName,
                       ctrlTorso,ctrlPose,30);
        slv->start();

        ctrl=new Controller(drvTorso,drvArm,&commData,localArmName,partName,
                            execTime,ctrlTorso,ctrlPose,10);
        ctrl->start();

        string rpcPortName=localArmName+"/rpc";
        rpcPort.open(rpcPortName.c_str());
        attach(rpcPort);

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
        slv->stop();
        ctrl->stop();

        delete slv;
        delete ctrl;

        if (drvTorso)
            delete drvTorso;

        if (drvArm)
            delete drvArm;

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
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--ctrlName name: controller name (default iKinArmCtrl)"                      << endl;
        cout << "\t--robot    name: robot name to connect to (default: icub)"                   << endl;
        cout << "\t--part     type: robot arm type, left_arm or right_arm (default: right_arm)" << endl;
        cout << "\t--torso    name: robot torso port name (default: torso)"                     << endl;
        cout << "\t--T        time: specify the task execution time in seconds (default: 2.0)"  << endl;
        cout << "\t--DOF10        : control the torso yaw/roll/pitch as well"                   << endl;
        cout << "\t--DOF9         : control the torso yaw/pitch as well"                        << endl;
        cout << "\t--DOF8         : control the torso yaw as well"                              << endl;
        cout << "\t--onlyXYZ      : disable orientation control"                                << endl;
        cout << "\t--noRobot      : avoid connecting to robot (simulation purpose)"             << endl;

        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    CtrlModule mod;

    return mod.runModule(rf);
}



