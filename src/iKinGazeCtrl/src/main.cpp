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
receives as input a 3D position in the task space. Nonetheless, 
further command modalities are available, listed in order of 
implementation: 1) the relative position of the target in the 
two image planes can be converted in relative displacement in 3D
task space with respect to the actual fixation point; 2) in case
only a monocular vision is exploited, the coordinates (u,v) of
just one pixel in the image plane along with a guessed distance 
z wrt the eye's reference frame can be given to the module; 3) 
the head-centered azimuth and elevation angles along with the 
vergence angle can be passed to the module both in absolute and 
relative mode (i.e. wrt to the current head position).

Moreover, this module also implements the server part of the <a 
href="http://eris.liralab.it/yarpdoc/d2/df5/classyarp_1_1dev_1_1IGazeControl.html">Gaze 
Control Interface</a>.
 
<b>Reminder</b> \n 
If you experience a slow speed motion, please check the shift 
factors settings within your low-level configuration file of the 
head part: they should be properly tuned. Usually a value of 8 
is enough. 
 
Rule: a lower shift factor allows to yield an higher joint speed
and at the same time it reduces the minimum speed that can be 
achieved. 
 
Example: look in the file <i>icub_head_torso_safe.ini</i> of 
your robot setup; you should find something similar to: 
\code 
[VELOCITY] 
Shifts 8 8 8 8 8 8 8 8 8 8 
\endcode 

\note A video on iCub gazing at a target can be seen <a 
      href="http://eris.liralab.it/misc/icubvideos/gazing_IIT_ISR.wmv">here</a>.

\section lib_sec Libraries 
- YARP libraries. 
- \ref iKin "iKin" library (it requires IPOPT lib: see the <a 
  href="http://eris.liralab.it/wiki/Installing_IPOPT">wiki</a>).

\section parameters_sec Parameters
--ctrlName \e name 
- The parameter \e name identifies the controller's name; all 
  the open ports will be tagged with the prefix
  /<ctrlName>/<part>/. If not specified \e iKinGazeCtrl is
  assumed.
 
--robot \e name 
- The parameter \e name selects the robot name to connect to; if
  not specified \e icub is assumed.
 
--part \e type 
- The parameter \e type selects the robot's head port to connect
  to; if not specified \e head is assumed.

--torso \e name 
- The parameter \e name selects the robot's torso port to 
  connect to; if not specified \e torso is assumed.
 
--inertial \e name 
- The parameter \e name selects the robot's inertial port to 
  connect to; if not specified \e inertial is assumed.
 
--Tneck \e time
- specify the neck trajectory execution time in point-to-point 
  movements [expressed in seconds]; by default \e time is 0.70
  seconds. (Tneck cannot be lower than Teyes).
 
--Teyes \e time
- specify the eyes trajectory execution time in point-to-point 
  movements [expressed in seconds]; by default \e time is 0.20
  seconds.
 
--config \e file 
- The parameter \e file specifies the file name used to read 
  kinematics and cameras parameters (see below).
 
--context \e dir
- Resource finder searching dir for config file.
 
--simulation
- simulate the presence of the robot. 
 
-- ping_robot_tmo \e tmo
- The parameter \e tmo is the timeout (in seconds) to allow to
  start-up the robot before connecting to it.
 
-- eye_tilt_min \e min
- The parameter \e min specifies the minimum eye tilt angle 
  [deg] in order to prevent the eye from being covered by the
  eyelid (when they're wide open) while moving.
 
-- eye_tilt_max \e max
- The parameter \e max specifies the maximum eye tilt angle 
  [deg] in order to prevent the eye from being covered by the
  eyelid (when they're wide open) while moving.
 
\section portsa_sec Ports Accessed
 
The ports the module is connected to: e.g. 
/icub/head/command:i and so on. 

\section portsc_sec Ports Created 
 
There are different ways of commanding a new target fixation
point: 
 
- by sending the absolute 3D position to gaze at in the task 
  space through /<ctrlName>/xd:i port.
- by localizing the target in the two image planes and thus 
  sending its relative coordinates to the /<ctrlName>/stereo:i
  port. There's no need here to know the intrinsic cameras
  parameters but it's required to feed continuosly the port with
  new feedback while converging to the target.
- by localizing the target in just one image plane and then 
  sending its relative coordinates together with a guessed
  distance z from the eye's frame to the /<ctrlName>/mono:i
  port. In this mode the intrinsic cameras parameters are
  required.
- by sending the head-centered azimuth/elevation couple in 
  degrees wrt either to the current head position or to the
  absolute head position (computed with the robot looking
  straight ahead). Vergence is also to be given either in
  relative mode or absolute mode.
 
The module creates the usual ports required for the 
communication with the robot (through interfaces) and the 
following ports: 
 
- \e /<ctrlName>/<part>/xd:i receives the target fixation point. 
  It accepts 3 double (also as a Bottle object) for xyz
  coordinates.
 
- \e /<ctrlName>/<part>/stereo:i receives the current target 
  position expressed in image planes. It accepts 4 double (also
  as a Bottle object) in this order: [ul vl ur vr].
 
- \e /<ctrlName>/<part>/mono:i receives the current target 
  position expressed in one image plane. The input data format
  is the Bottle [type u v z], where \e type can be left or
  right, <i> (u,v) </i> is the pixel coordinates and \e z is the
  guessed distance relative to the eye's reference frame.
 
- \e /<ctrlName>/<part>/angles:i receives the current target 
  position expressed as azimuth/elevation/vergence triplet in
  degrees. It accepts 1 string and 3 doubles (also as a Bottle
  object) in this order: [mode azi ele ver], where \e mode can
  be \e rel or \e abs. A positive azimuth will turn the gaze to
  the right, whereas a positive elevation will move the gaze
  upward.
 
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
 
- \e /<ctrlName>/<part>/angles:o returns the current 
  azimuth/elevation couple wrt to the absolute head position,
  together with the current vergence (Vector of 3 double). Units
  in deg.
 
- \e /<ctrlName>/<part>/rpc remote procedure call. 
    Recognized remote commands:
    - [quit]: quit the module.
    - [susp]: suspend the module.
    - [run]: resume the module.
    - [block] [pitch] <val>: block the neck pitch at <val>
     degrees.
    - [block] [yaw] <val>: block the neck yaw at <val> degrees.
    - [clear] [pitch]: restore the neck pitch range.
    - [clear] [yaw]: restore the neck yaw range.
    - [get] [Tneck]: returns the neck movements execution time.
    - [get] [Teyes]: returns the eyes movements execution time.
    - [get] [track]: returns the current controller's tracking
      mode (0/1).
    - [get] [done]: returns 1 iff motion is done, 0 otherwise.
    - [set] [Tneck] <val>: sets a new movements execution time
      for neck movements.
    - [set] [Teyes] <val>: sets a new movements execution time
      for eyes movements.
    - [set] [track] <val>: sets the controller's tracking mode;
      val can be 0/1.
 
@note When the tracking mode is active and the controller has 
      reached the target, it keeps on sending velocities to the
      head in order to compensate for any movements induced by
      the torso. If tracking mode is switched off, the
      controller automatically disconnects once the target is
      attained and reconnects at the next requested target. The
      controller starts by default in non-tracking mode.
 
\section coor_sys_sec Coordinate System 
Positions (meters) refer to the root reference frame attached to
the waist as in the <a 
href="http://eris.liralab.it/wiki/ICubForwardKinematics">wiki</a>. 

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files
A configuration file passed through \e --config contains the
fields required to specify the intrinsic cameras parameters 
along with the virtual links (given in terms of 
lenght,offset,twist parameters) which are appended to the eye 
kinematic in order to achieve the alignment with the optical 
axes compensating for possible unknown offsets. 
 
Example: 
 
\code 
[CAMERA_CALIBRATION_RIGHT]
fx 225.904
fy 227.041
cx 157.858
cy 113.51

[CAMERA_CALIBRATION_LEFT]
fx 219.057
fy 219.028
cx 174.742
cy 102.874
 
[ALIGN_KIN_LEFT]
length 4.15777e-007 -4.44085e-007   // [m]
offset 1.11013e-006 -1.84301e-007   // [m]
twist  0.163382 -0.12788            // [rad] 

[ALIGN_KIN_RIGHT]
length 0.000100176 -0.000105106     // [m]
offset 0.000103259 -9.96225e-005    // [m]
twist  0.296114 -0.194269           // [rad] 
\endcode 
 
\section tested_os_sec Tested OS
Windows, Linux

Note that \ref iKinGazeView "iKinGazeView" module can show
interactively the iCub head configuration launching MATLAB in 
background. Just connect the ports with the viewer and play. 

\author Ugo Pattacini
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/impl/NameClient.h>
#include <yarp/os/impl/Carriers.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iostream>
#include <iomanip>
#include <string>

#include <iCub/localizer.hpp>
#include <iCub/solver.hpp>
#include <iCub/controller.hpp>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::os::impl;
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

    void waitPart(const Property &partOpt, const double ping_robot_tmo)
    {
        string portName=const_cast<Property&>(partOpt).find("remote").asString().c_str();
        portName+="/state:o";
        double t0=Time::now();
    
        while (Time::now()-t0<ping_robot_tmo)
        {   
            cout << "Checking if " << portName << " port is active ... ";
        
            NameClient &nic=NameClient::getNameClient();
            Address address=nic.queryName(portName.c_str());
            bool ret;
        
            if (address.isValid())
            {    
                if (OutputProtocol *out=Carriers::connect(address))
                {
                    out->close();
                    delete out;
        
                    ret=true;
                }
                else
                    ret=false;
            }
            else
                ret=false;
        
            cout << (ret?"ok":"not yet") << endl;
    
            if (ret)
                return;
            else
            {
                double t1=Time::now();
                while (Time::now()-t1<1.0);
            }
        }
    }

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
        double eyeTiltMin;
        double eyeTiltMax;
        bool   Robotable;
        double ping_robot_tmo;

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

        if (rf.check("eyeTiltMin"))
            eyeTiltMin=rf.find("eyeTiltMin").asDouble();
        else
            eyeTiltMin=-25.0;

        if (rf.check("eyeTiltMax"))
            eyeTiltMax=rf.find("eyeTiltMax").asDouble();
        else
            eyeTiltMax=15.0;

        if (rf.check("simulation"))
            Robotable=false;
        else
            Robotable=true;

        if (rf.check("ping_robot_tmo"))
            ping_robot_tmo=rf.find("ping_robot_tmo").asDouble();
        else
            ping_robot_tmo=0.0;

        if (rf.check("config"))
        {    
            configFile=rf.findFile(rf.find("config").asString().c_str());

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

        waitPart(optHead,ping_robot_tmo);
        waitPart(optTorso,ping_robot_tmo);

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

        // create and start threads
        ctrl=new Controller(drvTorso,drvHead,&commData,robotName,
                            localHeadName,neckTime,eyesTime,eyeTiltMin,eyeTiltMax,10);        

        loc=new Localizer(&commData,localHeadName,configFile,10);

        eyesRefGen=new EyePinvRefGen(drvTorso,drvHead,&commData,robotName,
                                     localHeadName,inertialName,configFile,
                                     eyeTiltMin,eyeTiltMax,20);        

        slv=new Solver(drvTorso,drvHead,&commData,eyesRefGen,loc,ctrl,
                       localHeadName,configFile,eyeTiltMin,eyeTiltMax,20);

        // this switch-on order does matter !!
        eyesRefGen->start();
        slv->start();
        ctrl->start();
        loc->start();

        string rpcPortName=localHeadName+"/rpc";
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
                    eyesRefGen->suspend();
                    slv->suspend();
                    return true;
                }

                case VOCAB3('r','u','n'):
                {
                    slv->resume();
                    eyesRefGen->resume();
                    ctrl->resume();
                    return true;
                }

                case VOCAB4('b','l','o','c'):
                {
                    if (command.size()>2)
                    {
                        int joint=command.get(1).asVocab();
                        double val=command.get(2).asDouble();

                        if (joint==VOCAB4('p','i','t','c'))
                            slv->blockNeckPitch(val);
                        else if (joint==VOCAB3('y','a','w'))
                            slv->blockNeckYaw(val);
                        else
                            return false;
                    }
                    else
                        return false;

                    return true;
                }

                case VOCAB4('c','l','e','a'):
                {
                    if (command.size()>1)
                    {
                        int joint=command.get(1).asVocab();
    
                        if (joint==VOCAB4('p','i','t','c'))
                            slv->clearNeckPitch();
                        else if (joint==VOCAB3('y','a','w'))
                            slv->clearNeckYaw();
                        else
                            return false;
                    }
                    else
                        return false;

                    return true;
                }

                case VOCAB3('g','e','t'):
                {
                    if (command.size()>1)
                    {
                        int type=command.get(1).asVocab();

                        if (type==VOCAB4('T','n','e','c'))
                            reply.addDouble(ctrl->getTneck());
                        else if (type==VOCAB4('T','e','y','e'))
                            reply.addDouble(ctrl->getTeyes());
                        else if (type==VOCAB4('d','o','n','e'))
                            reply.addInt((int)ctrl->isMotionDone());
                        else if (type==VOCAB4('t','r','a','c'))
                            reply.addInt((int)ctrl->getTrackingMode());
                        else
                            return false;
                    }
                    else
                        return false;
    
                    return true;
                }

                case VOCAB3('s','e','t'):
                {
                    if (command.size()>2)
                    {
                        int type=command.get(1).asVocab();                        
    
                        if (type==VOCAB4('T','n','e','c'))
                        {
                            double execTime=command.get(2).asDouble();
                            ctrl->setTneck(execTime);
                        }
                        else if (type==VOCAB4('T','e','y','e'))
                        {
                            double execTime=command.get(2).asDouble();
                            ctrl->setTeyes(execTime);
                        }
                        else if (type==VOCAB4('t','r','a','c'))
                        {
                            bool mode=(command.get(2).asInt()>0);
                            ctrl->setTrackingMode(mode);
                        }
                        else
                            return false;
                    }
                    else
                        return false;
    
                    return true;
                }

                default:
                    return RFModule::respond(command,reply);
            }
        }
        else
            return false;
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
        cout << "\t--ctrlName      name: controller name (default iKinGazeCtrl)"                     << endl;
        cout << "\t--robot         name: robot name to connect to (default: icub)"                   << endl;
        cout << "\t--part          name: robot head port name, (default: head)"                      << endl;
        cout << "\t--torso         name: robot torso port name (default: torso)"                     << endl;
        cout << "\t--inertial      name: robot inertial port name (default: inertial)"               << endl;
        cout << "\t--Tneck         time: specify the neck movements time in seconds (default: 0.70)" << endl;
        cout << "\t--Teyes         time: specify the eyes movements time in seconds (default: 0.20)" << endl;
        cout << "\t--config        file: file name for kinematics and cameras parameters"            << endl;
        cout << "\t--context        dir: resource finder searching dir for config file"              << endl;
        cout << "\t--simulation        : simulate the presence of the robot"                         << endl;
        cout << "\t--ping_robot_tmo tmo: connection timeout (s) to start-up the robot"               << endl;
        cout << "\t--eye_tilt_min   min: minimum eye tilt angle [deg]"                               << endl;
        cout << "\t--eye_tilt_max   max: maximum eye tilt angle [deg]"                               << endl;

        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    CtrlModule mod;

    return mod.runModule(rf);
}



