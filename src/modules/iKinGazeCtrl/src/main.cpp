/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/** 
\defgroup iKinGazeCtrl iKinGazeCtrl 
 
@ingroup icub_module
 
Gaze controller based on iKin.

Copyright (C) 2010 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description

This module provides a controller for the iCub gaze capable of 
steering the neck and the eyes independently performing 
saccades, pursuit, vergence and OCR (oculo-collic reflex).
VOR (vestibulo-ocular reflex relying on inertial data) is not 
provided at time being but can be easily implemented. 
 
The controller can be seen as cartesian gaze controller since it
receives as input a 3D position in the task space. Nonetheless, 
further command modalities are available, listed in order of 
implementation: 1) the position of the target within the two 
image planes can be converted in relative displacement in 3D 
task space with respect to the actual fixation point; 2) in case 
only a monocular vision is exploited, the coordinates (u,v) of 
just one pixel in the image plane along with a guessed distance 
z wrt the eye's reference frame can be given to the module; 3) 
the head-centered azimuth and elevation angles along with the 
vergence angle can be passed to the module both in absolute and 
relative mode (i.e. wrt to the current head position).

Moreover, this module also implements the server part of the <a 
href="http://eris.liralab.it/yarpdoc/d2/df5/classyarp_1_1dev_1_1IGazeControl.html">Gaze 
Control Interface</a>. For a tutorial on how to use the 
interface, please go <a 
href="http://eris.liralab.it/iCub/main/dox/html/icub_gaze_interface.html">here</a>. 
 
\note If the torso is not detected alive then the module will 
      try to keep on working with just the head part.
 
\note <b>If you're going to use this controller for your work, 
      please quote it within any resulting publication</b>.
 
<b>Reminder</b> \n 
If you experience a slow speed motion, please check the shift 
factors settings within your low-level configuration file of the 
head part: they should be properly tuned. Usually a value of 8 
is enough. 
 
Rule: a lower shift factor allows to yield an higher joint speed
and at the same time it increases the value of minimum speed
that can be executed. 
 
Example: look in the file <i>icub_head_torso_safe.ini</i> of 
your robot setup; you should find something similar to: 
\code 
[VELOCITY] 
Shifts 8 8 8 8 8 8 ...
\endcode 
 
Read more on <a 
href="http://eris.liralab.it/wiki/ControlBoard_configuration_file#Shifts">Shifts 
Factors</a>. 

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
 
--ping_robot_tmo \e tmo
- The parameter \e tmo is the timeout (in seconds) to allow to
  start-up the robot before connecting to it.
 
--eyeTiltMin \e min
- The parameter \e min specifies the minimum eye tilt angle 
  [deg] in order to prevent the eye from being covered by the
  eyelid (when they're wide open) while moving.
 
--eyeTiltMax \e max
- The parameter \e max specifies the maximum eye tilt angle 
  [deg] in order to prevent the eye from being covered by the
  eyelid (when they're wide open) while moving.
 
--minAbsVel \e min
- The parameter \e min specifies the minimum absolute velocity 
  that can be achieved by the robot [deg/s] due to the
  approximation performed while delivering data over the
  network. By default this value is 0.0 having no result on the
  controller's operations. In case it is different from 0.0, the
  controller will implement a bang-bang approach whenever the
  velocity to be delivered goes under the minimum threshold.
 
\section portsa_sec Ports Accessed
 
The ports the module is connected to: e.g. 
/icub/head/command:i and so on. 

\section portsc_sec Ports Created 
 
There are different ways of commanding a new target fixation
point: 
 
- by sending the absolute 3D position to gaze at in the task 
  space through /<ctrlName>/xd:i port.
- by localizing the target in just one image plane and then 
  sending its coordinates together with a guessed
  distance z from the eye's frame to the /<ctrlName>/mono:i
  port. <b> In this mode the intrinsic cameras parameters
  (fx,fy,cx,cy) are required </b>.
- by localizing the target in the two image planes and thus 
  sending its coordinates to the /<ctrlName>/stereo:i port.
  Notice that t's required to feed continuosly the port with new
  feedback while converging to the target. <b> In this mode the
  intrinsic cameras parameters (cx,cy) are required </b>.
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

- \e /<ctrlName>/<part>/mono:i receives the current target 
  position expressed in one image plane. The input data format
  is the Bottle [type u v z], where \e type can be left or
  right, <i> (u,v) </i> is the pixel coordinates and \e z is the
  guessed distance relative to the eye's reference frame.
 
- \e /<ctrlName>/<part>/stereo:i receives the current target 
  position expressed in image planes. It accepts 4 double (also
  as a Bottle object) in this order: [ul vl ur vr].
 
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
    - [stop]: stop the motion immediately.
    - [bind] [pitch] <min> <max>: bind the neck pitch within a
      range given in degrees.
    - [bind] [yaw] <min> <max>: bind the neck yaw within a range
      given in degrees.
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
 
\note When the tracking mode is active and the controller has 
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
 
\note The virtual links are meaningful only as result of the 
calibration of the extrinsic camera parameters. In case the 
calibration has not been carried out then remove the relative 
groups from within the configuration file. 
 
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
length 0.0 0.0   // [m]
offset 0.0 0.0   // [m]
twist  0.0 0.0   // [rad] 

[ALIGN_KIN_RIGHT]
length 0.0 0.0   // [m]
offset 0.0 0.0   // [m]
twist  0.0 0.0   // [rad] 
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

#include <iCub/localizer.hpp>
#include <iCub/solver.hpp>
#include <iCub/controller.hpp>

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
    bool waitPart(const Property &partOpt, const double ping_robot_tmo)
    {
        string portName=const_cast<Property&>(partOpt).find("remote").asString().c_str();
        portName+="/state:o";
        double t0=Time::now();
    
        while ((Time::now()-t0)<ping_robot_tmo)
        {   
            fprintf(stdout,"Checking if %s port is active ... ",portName.c_str());
            bool ok=Network::exists(portName.c_str(),true);
            fprintf(stdout,"%s\n",ok?"ok":"not yet");
    
            if (ok)
                return true;
            else
            {
                double t1=Time::now();
                while ((Time::now()-t1)<1.0);
            }
        }

        return false;
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
        double minAbsVel;
        bool   Robotable;
        double ping_robot_tmo;

        Time::turboBoost();

        // get params from the command-line
        ctrlName=rf.check("ctrlName",Value("iKinGazeCtrl")).asString().c_str();
        robotName=rf.check("robot",Value("icub")).asString().c_str();
        partName=rf.check("part",Value("head")).asString().c_str();
        torsoName=rf.check("torso",Value("torso")).asString().c_str();
        inertialName=rf.check("inertial",Value("inertial")).asString().c_str();
        neckTime=rf.check("Tneck",Value(0.7)).asDouble();
        eyesTime=rf.check("Teyes",Value(0.2)).asDouble();
        eyeTiltMin=rf.check("eyeTiltMin",Value(-1e9)).asDouble();
        eyeTiltMax=rf.check("eyeTiltMax",Value(1e9)).asDouble();
        minAbsVel=CTRL_DEG2RAD*rf.check("minAbsVel",Value(0.0)).asDouble();
        ping_robot_tmo=rf.check("ping_robot_tmo",Value(0.0)).asDouble();

        // minAbsVel is given in absolute form
        // hence it must be positive
        if (minAbsVel<0.0)
            minAbsVel=-minAbsVel;

        if (rf.check("simulation"))
            Robotable=false;
        else
            Robotable=true;

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

        if (Robotable)
        {
            waitPart(optTorso,ping_robot_tmo);
            drvTorso=new PolyDriver(optTorso);

            if (!drvTorso->isValid())
            {
                fprintf(stdout,"Torso device driver not available!\n");
                fprintf(stdout,"Perhaps only the head is running; trying to continue ...\n");

                delete drvTorso;
                drvTorso=NULL;
            }

            waitPart(optHead,ping_robot_tmo);
            drvHead =new PolyDriver(optHead);

            if (!drvHead->isValid())
            {
                fprintf(stdout,"Head device driver not available!\n");

                delete drvHead;
                return false;
            }
        }
        else
            drvTorso=drvHead=NULL;

        // create and start threads
        ctrl=new Controller(drvTorso,drvHead,&commData,robotName,
                            localHeadName,neckTime,eyesTime,
                            eyeTiltMin,eyeTiltMax,minAbsVel,10);

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
        int ack=Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        if (command.size())
        {
            switch (command.get(0).asVocab())
            {
                case VOCAB4('s','u','s','p'):
                {
                    ctrl->suspend();
                    eyesRefGen->suspend();
                    slv->suspend();
                    reply.addVocab(ack);
                    return true;
                }

                case VOCAB3('r','u','n'):
                {
                    slv->resume();
                    eyesRefGen->resume();
                    ctrl->resume();
                    reply.addVocab(ack);
                    return true;
                }

                case VOCAB4('s','t','o','p'):
                {
                    eyesRefGen->stopControl();
                    reply.addVocab(ack);
                    return true;
                }

                case VOCAB4('b','i','n','d'):
                {
                    if (command.size()>2)
                    {
                        int joint=command.get(1).asVocab();
                        double min=command.get(2).asDouble();
                        double max=command.get(3).asDouble();

                        if (joint==VOCAB4('p','i','t','c'))
                            slv->bindNeckPitch(min,max);
                        else if (joint==VOCAB3('y','a','w'))
                            slv->bindNeckYaw(min,max);
                        else
                        {
                            reply.addVocab(nack);
                            return false;
                        }
                    }
                    else
                    {
                        reply.addVocab(nack);
                        return false;
                    }

                    reply.addVocab(ack);
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
                        {
                            reply.addVocab(nack);
                            return false;
                        }
                    }
                    else
                    {
                        reply.addVocab(nack);
                        return false;
                    }

                    reply.addVocab(ack);
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
                        {
                            reply.addVocab(nack);
                            return false;
                        }
                    }
                    else
                    {
                        reply.addVocab(nack);
                        return false;
                    }
    
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
                        {
                            reply.addVocab(nack);
                            return false;
                        }
                    }
                    else
                    {
                        reply.addVocab(nack);
                        return false;
                    }
    
                    reply.addVocab(ack);
                    return true;
                }

                default:
                    return RFModule::respond(command,reply);
            }
        }
        else
        {
            reply.addVocab(nack);
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
        fprintf(stdout,"Options:\n\n");
        fprintf(stdout,"\t--ctrlName      name: controller name (default iKinGazeCtrl)\n");
        fprintf(stdout,"\t--robot         name: robot name to connect to (default: icub)\n");
        fprintf(stdout,"\t--part          name: robot head port name, (default: head)\n");
        fprintf(stdout,"\t--torso         name: robot torso port name (default: torso)\n");
        fprintf(stdout,"\t--inertial      name: robot inertial port name (default: inertial)\n");
        fprintf(stdout,"\t--Tneck         time: specify the neck movements time in seconds (default: 0.70)\n");
        fprintf(stdout,"\t--Teyes         time: specify the eyes movements time in seconds (default: 0.20)\n");
        fprintf(stdout,"\t--config        file: file name for kinematics and cameras parameters\n");
        fprintf(stdout,"\t--context        dir: resource finder searching dir for config file\n");
        fprintf(stdout,"\t--simulation        : simulate the presence of the robot\n");
        fprintf(stdout,"\t--ping_robot_tmo tmo: connection timeout (s) to start-up the robot\n");
        fprintf(stdout,"\t--eyeTiltMin     min: minimum eye tilt angle [deg]\n");
        fprintf(stdout,"\t--eyeTiltMax     max: maximum eye tilt angle [deg]\n");
        fprintf(stdout,"\t--minAbsVel      min: minimum absolute velocity that can be achieved [deg/s] (default 0.0)\n");

        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    CtrlModule mod;

    return mod.runModule(rf);
}



