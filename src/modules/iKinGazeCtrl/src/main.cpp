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
saccades, pursuit, vergence, OCR (oculo-collic reflex) and VOR 
(vestibulo-ocular reflex relying on inertial data). 
 
The controller can be seen as cartesian gaze controller since it
receives as input a 3D position in the task space. Nonetheless, 
further command modalities are available: 1) the coordinates
(u,v) of just one pixel in the image plane along with a guessed 
component z in the eye's reference frame can be provided, or 
alternatively the vergence angle; 2) the position of the target 
within the two image planes can be converted in the 3D task 
space using the monocular approach coupled with a pid on the 
component z; 3) the head-centered azimuth and elevation angles 
along with the vergence angle can be given to the module both in 
absolute and relative mode. 

Moreover, this module also implements the server part of the <a 
href="http://wiki.icub.org/yarpdoc/classyarp_1_1dev_1_1IGazeControl.html">Gaze
Control Interface</a>. For a tutorial on how to use the 
interface, please go \ref icub_gaze_interface "here". 
 
\note If the torso is not detected alive then the module will 
      try to keep on working with just the head part.
 
\note <b>If you're going to use this controller for your work, 
      please quote it within any resulting publication</b>: <i>
      Pattacini U., Modular Cartesian Controllers for
      Humanoid Robots: Design and Implementation on the iCub,
      Ph.D. Dissertation, RBCS, Istituto Italiano di
      Tecnologia, 2011</i>.
 
<b>Reminder</b> \n 
If you experience a slow speed motion, please check the shift 
factors settings within your low-level configuration file of the 
head part: they should be properly tuned. Usually a value of 8 
is enough. 
 
Rule: a lower shift factor allows to yield an higher joint speed
and at the same time it increases the value of minimum speed
that can be executed. 
 
Example: look in the file <i>icub_head_torso.ini</i> of your
robot setup; you should find something similar to: 
\code 
[VELOCITY] 
Shifts 8 8 8 8 8 8 ...
\endcode 
 
Read more on <a 
href="http://wiki.icub.org/wiki/ControlBoard_configuration_file#Shifts">Shifts
Factors</a>. 

\note A video on iCub gazing at a target can be seen <a 
      href="http://www.youtube.com/watch?v=ZF3LOlg4fuA">here</a>.

\section lib_sec Libraries 
- YARP libraries. 
- \ref iKin "iKin" library (it requires IPOPT lib: see the <a 
  href="http://wiki.icub.org/wiki/Installing_IPOPT">wiki</a>).

\section parameters_sec Parameters
--name \e ctrlName 
- The parameter \e ctrlName identifies the controller's name; 
  all the open ports will be tagged with the prefix
  /<ctrlName>. If not specified, \e iKinGazeCtrl is assumed.
 
--robot \e name 
- The parameter \e name selects the robot name to connect to; if
  not specified, \e icub is assumed.
 
--head \e name 
- The parameter \e name selects the robot's head port to connect
  to; if not specified, \e head is assumed.

--torso \e name 
- The parameter \e name selects the robot's torso port to 
  connect to; if not specified, \e torso is assumed. The special
  string \e disabled can be used to skip opening the torso
  device.
 
--Tneck \e time
- Specify the neck trajectory execution time in point-to-point 
  movements [expressed in seconds]; by default \e time is 0.75
  seconds. (Tneck cannot be set equal or lower than Teyes).
 
--Teyes \e time
- Specify the eyes trajectory execution time in point-to-point 
  movements [expressed in seconds]; by default \e time is 0.25
  seconds.
 
--camerasContext \e dir 
- The parameter \e dir specifies the context used to locate the 
  cameras parameters file (see below).
 
--camerasFile \e file 
- The parameter \e file specifies the file name used to read  
  cameras parameters.
 
--context \e dir
- Resource finder default searching directory for configuration 
  files; if not specified, \e iKinGazeCtrl is assumed.
 
--from \e file
- Resource finder default configuration file; if not specified, 
  \e config.ini is assumed.
 
--saccades \e switch 
- Enable/disable saccadic movements; the parameter \e switch can
  be therefore ["on"|"off"], being "on" by default.
 
--neck_position_control \e switch
- Enable/disable low-level position control of the neck; the 
  parameter \e switch can be therefore ["on"|"off"], being "on"
  by default.
 
--vor \e gain
- Specify the contribution of the vestibulo-ocular reflex (VOR)
  in compute the final counter-rotation of the eyes due to
  neck rotation. To turn off the VOR just set the \e gain equal
  to 0.0. By default \e gain is 1.0, that means "full
  contribution". Values of the gain greater than 1.0 mean
  "contribution amplified".
 
--ocr \e gain
- Specify the contribution of the oculo-collic reflex (OCR) in 
  computing the counter-rotation of the eyes due to neck
  rotation. To turn off the OCR just set the \e gain equal to
  0.0 (as per default).
 
--simulation
- Simulate the presence of the robot. 
 
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
 
--headV2 
- When this options is specified then the kinematic structure of
  the hardware v2 of the head is referred.
 
--tweakFile \e file 
- The parameter \e file specifies the file name (located in 
  module context) used to read/write options that are tweakable
  by the user; if not provided, \e tweak.ini is assumed.
 
--tweakOverwrite \e switch 
- If "on", at startup default values and cameras values 
  retrieved from file will be overwritten by those values
  contained in the tweak file. The \e switch is "on" by default.
 
\section portsa_sec Ports Accessed
 
The ports the module is connected to: e.g. 
/icub/head/command:i and so on. 

\section portsc_sec Ports Created 
 
There are different ways of commanding a new target fixation
point: 
 
- by sending the absolute 3D position to gaze at in the task 
  space through /<ctrlName>/xd:i port.
- by localizing the target in just one image plane and then 
  sending its coordinates together with a guessed component z
  in the eye's reference frame to the /<ctrlName>/mono:i port.
  Alternatively, the z component can be replaced by the vergence
  angle.
  <b>In this mode the cameras intrinsic parameters are
  required</b>.
- by localizing the target in the two image planes and thus 
  sending its coordinates to the /<ctrlName>/stereo:i port. This
  strategy employs the monocular approach along with a pid that
  varies the component z incrementally according to the actual
  error; to achieve that it's required to feed continuosly the
  port with new feedback while converging to the target. <b>In
  this mode the cameras intrinsic parameters are required</b>.
- by sending the head-centered azimuth/elevation couple in 
  degrees wrt either to the current head position or to the
  absolute head position (computed with the robot looking
  straight ahead and all neck and eyes encoders zeroed).
  Vergence is also to be given either in relative mode or
  absolute mode. \n In this mode a final conversion to 3D points
  is always performed wrt to the absolute head position since
  the controller is intrinsically cartesian. Moreover, as the
  center of the head varies over time because of the motion
  induced by the neck, the ultimate target vergence given for
  instance as delta wrt the current one may differ from what
  expected beacuse what is actually achieved is the
  corresponding 3D point.
 
The module creates the usual ports required for the 
communication with the robot (through interfaces) and the 
following ports: 
 
- \e /<ctrlName>/xd:i receives the target fixation point. 
  It accepts 3 double (also as a Bottle object) for xyz
  coordinates.

- \e /<ctrlName>/mono:i receives the current target position 
  expressed in one image plane. The input data format is the
  Bottle [type u v z], where \e type can be "left" or "right",
  <i> (u,v) </i> is the pixel coordinates and \e z is the
  guessed z-component in the eye's reference frame. An
  alternative command modality employs the vergence given in
  degrees in place of the z-component and its format is: [type u
  v "ver" ver].
 
- \e /<ctrlName>/stereo:i receives the current target position 
  expressed in image planes. It accepts 4 double (also as a
  Bottle object) in this order: [ul vl ur vr].
 
- \e /<ctrlName>/angles:i receives the current target position 
  expressed as azimuth/elevation/vergence triplet in degrees. It
  accepts 1 string and 3 doubles (also as a Bottle object) in
  this order: [mode azi ele ver], where \e mode can be \e rel or
  \e abs. A positive azimuth will turn the gaze to the right,
  whereas a positive elevation will move the gaze upward.
 
- \e /<ctrlName>/x:o returns the actual fixation point (Vector 
  of 3 double). Units in meters.
 
- \e /<ctrlName>/q:o returns the actual joints configuration 
  during movement (Vector of 9 double). The order for torso
  angles is the one defined by kinematic chain (reversed order).
  Useful in conjunction with the \ref iKinGazeView "viewer".
  Units in degrees.

- \e /<ctrlName>/angles:o returns the current azimuth/elevation 
  couple wrt to the absolute head position, together with the
  current vergence (Vector of 3 double). Units in degrees.
 
- \e /<ctrlName>/events:o streams out the event associated to 
  the controller's state. \n Available events are:
   - "motion-onset" <time>: sent out at the beginning of the
     motion; comprise the time instant of the source when the
     event took place.
   - "motion-done" <time>: sent out at the end of the motion;
     comprise the time instant of the source when the event took
     place.
   - "motion-ongoing" <time> <checkpoint>: sent out when the
     portion of path given in [0,1] by the checkpoint parameter
     has been attained; comprise the time instant of the source
     when the event took place as well as the checkpoint.
   - "saccade-onset" <time>: sent out at the beginning of the
     saccade; comprise the time instant of the source when the
     event took place.
   - "saccade-done" <time>: sent out at the end of the saccade;
     comprise the time instant of the source when the event took
     place.
   - "closing" <time>: sent out when the controller is being
     shut down; comprise the time instant of the source when the
     event took place.
   - "suspended" <time>: sent out when the controller is
     suspended; comprise the time instant of the source when the
     event took place.
   - "resumed" <time>: sent out when the controller is resumed;
     comprise the time instant of the source when the event took
     place.
   - "comm-timeout" <time>: sent out when the controller gets 
     suspended because of a communication timeout; comprise the
     time instant of the source when the event took place.
 
- \e /<ctrlName>/rpc remote procedure call. \n 
    Recognized remote commands (be careful, <b>commands dealing
    with geometric projections will only work if the cameras
    intrinsic parameters are provided</b>):
    - [stop]: stop the motion immediately.
    - [bind] [pitch] <min> <max>: bind the neck pitch within a
      range given in degrees.
    - [bind] [roll] <min> <max>: bind the neck roll within a
      range given in degrees.
    - [bind] [yaw] <min> <max>: bind the neck yaw within a range
      given in degrees.
    - [bind] [eyes] <ver>: bind the eyes to look always straight
      with a specified vergence given in degrees.
    - [clear] [pitch]: restore the neck pitch range.
    - [clear] [roll]: restore the neck roll range.
    - [clear] [yaw]: restore the neck yaw range.
    - [clear] [eyes]: restore the eyes movements.
    - [get] [Tneck]: returns the neck movements execution time.
    - [get] [Teyes]: returns the eyes movements execution time.
    - [get] [vor]: returns the vor gain.
    - [get] [ocr]: returns the ocr gain.
    - [get] [sacc]: returns the saccades control status [0/1].
    - [get] [sinh]: returns the saccades inhibition period [s].
    - [get] [sact]: returns the saccades activation angle [deg].
    - [get] [track]: returns the current controller's tracking
      mode [0/1].
    - [get] [done]: returns 1 iff motion is done, 0 otherwise.
    - [get] [sdon]: returns 1 iff saccade is done, 0 if still
      underway.
    - [get] [pitch]: returns in degrees the current range of
      neck pitch joint.
    - [get] [roll]: returns in degrees the current range of
      neck roll joint.
    - [get] [yaw]: returns in degrees the current range of
      neck yaw joint.
    - [get] [eyes]: returns in degrees the vergence set when
      eyes are bound to look straight ahead; a negative values
      means no constraint.
    - [get] [ntol]: returns in degrees the current user
      tolerance for gazing with the neck.
    - [get] [des]: returns the desired head joints angles that
      achieve the target [deg].
    - [get] [vel]: returns the head joints velocities commanded
      by the controller [deg/s].
    - [get] [pose] <type>: returns (enclosed in a list) the left
      eye pose if type=="left", the right eye pose if
      type=="right" and the head-centered pose if type=="head".
      The pose is given in axis/angle representation (i.e.
      7-componenets vector). Additionally, a stamp is appended
      as further list accounting for the time (second element of
      the list) relative to the encoders positions used to
      compute the pose.
    - [get] [2D] (<type> <x> <y> <z>): returns the 2D pixel
      point whose cartesian coordinates (x,y,z) are given wrt
      the root reference frame as the result of its projection
      into the image plane <type> ["left"|"right"].
    - [get] [3D] [mono] (<type> < u> <v> <z>): returns the 3D
      point whose projected pixel coordinates (u,v) in the image
      plane <type> ["left"|"right"] along with third component
      <z> in the eye's reference frame are given.
    - [get] [3D] [stereo] (< ul> <vl> <ur> <vr>): returns the 3D
      point whose projected pixels coordinates (ul,vl) and
      (ur,vr) in the image planes are provided as the result of
      the triangulation.
      @note The triangulation is deeply affected by
      uncertainties in the cameras extrinsic parameters and
      cameras alignment.
    - [get] [3D] [proj] (<type> < u> <v> < a> < b> < c> <d>):
      returns the 3D point with projected pixel coordinates
      (u,v) in the image plane <type> ["left"|"right"] that
      results from the intersection with the plane expressed
      with its implicit equation ax+by+cz+d=0 in the root
      reference frame.
    - [get] [3D] [ang] (<type> <azi> <ele> <ver>): transforms
      angular coordinates into cartesian coordinates. The
      options <type> can be ["abs"|"rel"].
    - [get] [ang] (<x> <y> <z>): transforms cartesian
      coordinates into absolute angular coordinates.
    - [get] [pid]: returns (enclosed in a list) a property-like
      bottle containing the pid values used to converge to the
      target with stereo input.
    - [get] [info]: returns (enclosed in a list) a property-like
      bottle containing useful information, such as the
      "head_version" (e.g. 1.0, 2.0, ...), the
      "min_allowed_vergence" (in degrees), a list of the
      available "events", the intrinsic and extrinsic camera
      parameters used.
    - [get] [tweak]: returns (enclosed in a list) a
      property-like bottle containing low-level information on
      the current controller's configuration.
    - [set] [Tneck] <val>: sets a new movements execution time
      for neck movements.
    - [set] [Teyes] <val>: sets a new movements execution time
      for eyes movements.
    - [set] [vor] <val>: sets a new gain for vor.
    - [set] [ocr] <val>: sets a new gain for ocr.
    - [set] [sacc] <val>: enables/disables saccades; val can be
      [0/1].
    - [set] [sinh] <val>: sets the saccades inhibition period
      [s].
    - [set] [sact] <val>: sets the saccades activation angle
      [deg].
    - [set] [ntol] <val>: sets in degrees the new user tolerance
      for gazing with the neck.
    - [set] [track] <val>: sets the controller's tracking mode;
      val can be [0/1].
    - [set] [pid] ((prop0 (<val> <val> ...)) (prop1) (<val>
      <val> ...)): sets the pid values used to converge to the
      target with stereo input. The pid is implemented in
      parallel form (\ref PIDs). Aside from the usual pid
      parameters a further option "dominantEye" is available
      that enables the user to chose the dominant eye employed
      for the monocular approach.
    - [set] [tweak] ((prop0 (<val> <val> ...)) (prop1) (<val>
      <val> ...)): sets parameters for the low-level
      controller's configuration.
    - [store]: store the controller context returning an integer
      identifier. The context comprises the values of internal
      controller variables, such as the tracking mode, the
      trajectory time and so on.
    - [restore] <id>: restore a previously stored controller
      context referred by the identifier \e id.
    - [del] (<id0> <id1> ...): delete all the contexts whose ids
      are contained in the list.
    - [register] [ongoing] <checkpoint>: register the
      "motion-ongoing" event with the given checkpoint in [0,1].
    - [unregister] [ongoing] <checkpoint>: unregister the
      "motion-ongoing" event with the given checkpoint in [0,1].
    - [list] [ongoing]: return the list of registered
      "motion-ongoing" events.
    - [susp]: suspend the module.
    - [run]: resume the module.
    - [status]: returns "running" or "suspended".
    - [quit]: quit the module.
 
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
href="http://wiki.icub.org/wiki/ICubForwardKinematics">wiki</a>.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Camera Configuration File
A configuration file passed through \e --camerasFile contains 
the fields required to specify the cameras intrinsic parameters 
along with a roto-translation matrix appended to the eye 
kinematic (see the iKinChain::setHN method) in order to achieve
the alignment with the optical axes compensating for possible 
unknown offsets. 
 
The final roto-translation matrix is meaningful only as result 
of the calibration of the cameras extrinsic parameters that can 
be obtained for instance through the \ref icub_stereoCalib 
module. 
 
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
HN (0.0 1.0 2.0 ... 15.0)  // list of 4x4 doubles (per rows) 

[ALIGN_KIN_RIGHT]
HN (0.0 1.0 2.0 ... 15.0)  // list of 4x4 doubles (per rows)  
\endcode 
 
\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini
*/ 

#include <fstream>
#include <iomanip>
#include <map>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <iCub/localizer.h>
#include <iCub/solver.h>
#include <iCub/controller.h>

#define GAZECTRL_SERVER_VER     1.1

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;


/************************************************************************/
class CtrlModule: public RFModule
{
protected:
    ResourceFinder *rf;
    Localizer      *loc;
    EyePinvRefGen  *eyesRefGen;
    Solver         *slv;
    Controller     *ctrl;
    PolyDriver     *drvTorso, *drvHead;
    exchangeData    commData;
    RpcServer       rpcPort;
    bool            interrupting;
    bool            doSaveTweakFile;
    Mutex           savingTweakFile;

    struct Context
    {
        // controller part
        double eyesTime;
        double neckTime;
        bool   mode;

        // solver part
        double neckPitchMin;
        double neckPitchMax;
        double neckRollMin;
        double neckRollMax;
        double neckYawMin;
        double neckYawMax;
        double neckAngleUserTolerance;
        double eyesBoundVer;
        Vector counterRotGain;
        bool   saccadesOn;
        double saccadesInhibitionPeriod;
        double saccadesActivationAngle;

        // localizer part
        Bottle pidOptions;
    };

    int contextIdCnt;
    std::map<int,Context> contextMap;

    /************************************************************************/
    PolyDriver *waitPart(const Property &partOpt, const double ping_robot_tmo)
    {    
        Property &options=const_cast<Property&>(partOpt);
        string partName=options.find("part").asString().c_str();
        PolyDriver *pDrv=NULL;

        double t0=Time::now();
        while (Time::now()-t0<ping_robot_tmo)
        {
            if (pDrv!=NULL)
                delete pDrv;

            pDrv=new PolyDriver(options);
            bool ok=pDrv->isValid();

            printf("Checking if %s part is active ... ",partName.c_str());
            if (ok)
            {
                printf("yes\n");
                return pDrv;
            }
            else
            {
                double dt=ping_robot_tmo-(Time::now()-t0);
                printf("not yet: still %.1f [s] to timeout expiry\n",dt>0.0?dt:0.0);

                double t1=Time::now();
                while (Time::now()-t1<1.0)
                    Time::delay(0.1);
            }

            if (interrupting)
                break;
        }

        return pDrv;
    }

    /************************************************************************/
    void storeContext(int *id)
    {
        Context &context=contextMap[contextIdCnt];

        // controller part
        context.eyesTime=ctrl->getTeyes();
        context.neckTime=ctrl->getTneck();
        context.mode=ctrl->getTrackingMode();
        
        // solver part
        slv->getCurNeckPitchRange(context.neckPitchMin,context.neckPitchMax);
        slv->getCurNeckRollRange(context.neckRollMin,context.neckRollMax);
        slv->getCurNeckYawRange(context.neckYawMin,context.neckYawMax);
        context.neckAngleUserTolerance=slv->getNeckAngleUserTolerance();
        context.eyesBoundVer=eyesRefGen->getEyesBoundVer();
        context.counterRotGain=eyesRefGen->getCounterRotGain();
        context.saccadesOn=eyesRefGen->isSaccadesOn();
        context.saccadesInhibitionPeriod=eyesRefGen->getSaccadesInhibitionPeriod();
        context.saccadesActivationAngle=eyesRefGen->getSaccadesActivationAngle();

        // localizer part
        loc->getPidOptions(context.pidOptions);

        if (id!=NULL)
            *id=contextIdCnt++;
    }

    /************************************************************************/
    bool restoreContext(const int id)
    {
        map<int,Context>::iterator itr=contextMap.find(id);
        if (itr!=contextMap.end())
        {
            Context &context=itr->second;

            // controller part
            ctrl->setTeyes(context.eyesTime);   // always remind to set eyes before the neck
            ctrl->setTneck(context.neckTime);   // due to internal saturation
            ctrl->setTrackingMode(context.mode);

            // solver part
            slv->bindNeckPitch(context.neckPitchMin,context.neckPitchMax);
            slv->bindNeckRoll(context.neckRollMin,context.neckRollMax);
            slv->bindNeckYaw(context.neckYawMin,context.neckYawMax);
            slv->setNeckAngleUserTolerance(context.neckAngleUserTolerance);
            eyesRefGen->manageBindEyes(context.eyesBoundVer);
            eyesRefGen->setCounterRotGain(context.counterRotGain);
            eyesRefGen->setSaccades(context.saccadesOn);
            eyesRefGen->setSaccadesInhibitionPeriod(context.saccadesInhibitionPeriod);
            eyesRefGen->setSaccadesActivationAngle(context.saccadesActivationAngle);

            // localizer part
            loc->setPidOptions(context.pidOptions);

            return true;
        }
        else
            return false;
    }

    /************************************************************************/
    bool deleteContexts(Bottle *contextIdList)
    {
        if (contextIdList!=NULL)
        {
            for (int i=0; i<contextIdList->size(); i++)
            {
                int id=contextIdList->get(i).asInt();
                map<int,Context>::iterator itr=contextMap.find(id);
                if (itr!=contextMap.end())
                    contextMap.erase(itr);
            }
    
            return true;
        }
        else
            return false;
    }

    /************************************************************************/
    bool getInfo(Bottle &info)
    {
        info.clear();

        Bottle &serverVer=info.addList();
        serverVer.addString("server_version");
        serverVer.addDouble(GAZECTRL_SERVER_VER);

        Bottle &headVer=info.addList();
        headVer.addString("head_version");
        headVer.addDouble(commData.head_version);

        Bottle &minVer=info.addList();
        minVer.addString("min_allowed_vergence");
        minVer.addDouble(CTRL_RAD2DEG*commData.get_minAllowedVergence());

        Bottle &intrinsicsLeft=info.addList();
        intrinsicsLeft.addString("camera_intrinsics_left");
        Bottle &intrinsicsLeftValues=intrinsicsLeft.addList();
        Matrix PrjL;
        if (loc->getIntrinsicsMatrix("left",PrjL))
            for (int r=0; r<PrjL.rows(); r++)
                for (int c=0; c<PrjL.cols(); c++)
                    intrinsicsLeftValues.addDouble(PrjL(r,c));

        Bottle &intrinsicsRight=info.addList();
        intrinsicsRight.addString("camera_intrinsics_right");
        Bottle &intrinsicsRightValues=intrinsicsRight.addList();
        Matrix PrjR;
        if (loc->getIntrinsicsMatrix("right",PrjR))
            for (int r=0; r<PrjR.rows(); r++)
                for (int c=0; c<PrjR.cols(); c++)
                    intrinsicsRightValues.addDouble(PrjR(r,c));

        Bottle &extrinsicsLeft=info.addList();
        extrinsicsLeft.addString("camera_extrinsics_left");
        Bottle &extrinsicsLeftValues=extrinsicsLeft.addList();
        Matrix HL;
        if (loc->getExtrinsicsMatrix("left",HL))
            for (int r=0; r<HL.rows(); r++)
                for (int c=0; c<HL.cols(); c++)
                    extrinsicsLeftValues.addDouble(HL(r,c));

        Bottle &extrinsicsRight=info.addList();
        extrinsicsRight.addString("camera_extrinsics_right");
        Bottle &extrinsicsRightValues=extrinsicsRight.addList();
        Matrix HR;
        if (loc->getExtrinsicsMatrix("right",HR))
            for (int r=0; r<HR.rows(); r++)
                for (int c=0; c<HR.cols(); c++)
                    extrinsicsRightValues.addDouble(HR(r,c));

        Bottle &events=info.addList();
        events.addString("events");
        Bottle &eventsList=events.addList();
        eventsList.addString("motion-onset");
        eventsList.addString("motion-done");
        eventsList.addString("motion-ongoing");
        eventsList.addString("saccade-onset");
        eventsList.addString("saccade-done");
        eventsList.addString("closing");
        eventsList.addString("suspended");
        eventsList.addString("resumed");
        eventsList.addString("comm-timeout");
        eventsList.addString("*");

        return true;
    }

    /************************************************************************/
    bool tweakSet(const Bottle &options)
    {
        Bottle &opt=const_cast<Bottle&>(options);
        savingTweakFile.lock();

        if (Bottle *pB=opt.find("camera_intrinsics_left").asList())
        {
            Matrix Prj(3,4); Prj=0.0;
            loc->getIntrinsicsMatrix("left",Prj);

            int r=0; int c=0;
            for (int i=0; i<pB->size(); i++)
            {
                Prj(r,c)=pB->get(i).asDouble();
                if (++c>=Prj.cols())
                {
                    c=0;
                    if (++r>=Prj.rows())
                        break;
                }
            }

            loc->setIntrinsicsMatrix("left",Prj);
            doSaveTweakFile=commData.tweakOverwrite;
        }

        if (Bottle *pB=opt.find("camera_intrinsics_right").asList())
        {
            Matrix Prj(3,4); Prj=0.0;
            loc->getIntrinsicsMatrix("right",Prj);

            int r=0; int c=0;
            for (int i=0; i<pB->size(); i++)
            {
                Prj(r,c)=pB->get(i).asDouble();
                if (++c>=Prj.cols())
                {
                    c=0;
                    if (++r>=Prj.rows())
                        break;
                }
            }

            loc->setIntrinsicsMatrix("right",Prj);
            doSaveTweakFile=commData.tweakOverwrite;
        }

        bool doMinAllowedVer=false;
        if (Bottle *pB=opt.find("camera_extrinsics_left").asList())
        {
            Matrix HN=eye(4,4);
            loc->getExtrinsicsMatrix("left",HN);

            int r=0; int c=0;
            for (int i=0; i<pB->size(); i++)
            {
                HN(r,c)=pB->get(i).asDouble();
                if (++c>=HN.cols())
                {
                    c=0;
                    if (++r>=HN.rows())
                        break;
                }
            }

            // enforce the homogeneous property
            HN(3,0)=HN(3,1)=HN(3,2)=0.0;
            HN(3,3)=1.0;

            loc->setExtrinsicsMatrix("left",HN);
            ctrl->setExtrinsicsMatrix("left",HN);
            slv->setExtrinsicsMatrix("left",HN);
            eyesRefGen->setExtrinsicsMatrix("left",HN);
            doSaveTweakFile=commData.tweakOverwrite;
            doMinAllowedVer=true;
        }

        if (Bottle *pB=opt.find("camera_extrinsics_right").asList())
        {
            Matrix HN=eye(4,4);
            loc->getExtrinsicsMatrix("right",HN);

            int r=0; int c=0;
            for (int i=0; i<pB->size(); i++)
            {
                HN(r,c)=pB->get(i).asDouble();
                if (++c>=HN.cols())
                {
                    c=0;
                    if (++r>=HN.rows())
                        break;
                }
            }

            // enforce the homogeneous property
            HN(3,0)=HN(3,1)=HN(3,2)=0.0;
            HN(3,3)=1.0;

            loc->setExtrinsicsMatrix("right",HN);
            ctrl->setExtrinsicsMatrix("right",HN);
            slv->setExtrinsicsMatrix("right",HN);
            eyesRefGen->setExtrinsicsMatrix("right",HN);
            doSaveTweakFile=commData.tweakOverwrite;
            doMinAllowedVer=true;
        }

        if (doMinAllowedVer)
        {
            ctrl->findMinimumAllowedVergence();
            ctrl->minAllowedVergenceChanged();
            eyesRefGen->minAllowedVergenceChanged();
        }

        savingTweakFile.unlock();
        return true;
    }

    /************************************************************************/
    bool tweakGet(Bottle &options)
    {
        options.clear();
        return true;
    }

    /************************************************************************/
    void saveTweakFile()
    {
        Matrix PrjL;
        bool validIntrinsicsL=loc->getIntrinsicsMatrix("left",PrjL);

        Matrix PrjR;
        bool validIntrinsicsR=loc->getIntrinsicsMatrix("right",PrjR);

        Matrix HNL;
        loc->getExtrinsicsMatrix("left",HNL);

        Matrix HNR;
        loc->getExtrinsicsMatrix("right",HNR);

        ofstream fout;
        string tweakFile=rf->getHomeContextPath().c_str();
        tweakFile+="/"+commData.tweakFile;
        fout.open(tweakFile.c_str());
        if (fout.is_open())
        {
            if (validIntrinsicsL)
            {                
                fout<<"[CAMERA_CALIBRATION_LEFT]"<<endl;
                fout<<"fx "<<PrjL(0,0)<<endl;
                fout<<"fy "<<PrjL(1,1)<<endl;
                fout<<"cx "<<PrjL(0,2)<<endl;
                fout<<"cy "<<PrjL(1,2)<<endl;
                fout<<endl;
            }

            if (validIntrinsicsR)
            {                
                fout<<"[CAMERA_CALIBRATION_RIGHT]"<<endl;
                fout<<"fx "<<PrjR(0,0)<<endl;
                fout<<"fy "<<PrjR(1,1)<<endl;
                fout<<"cx "<<PrjR(0,2)<<endl;
                fout<<"cy "<<PrjR(1,2)<<endl;
                fout<<endl;
            }
            
            fout.precision(16);
            fout<<"[ALIGN_KIN_LEFT]"<<endl;
            fout<<"HN (";
            for (int r=0; r<HNL.rows(); r++)
                for (int c=0; c<HNL.cols(); c++)
                    fout<<HNL(r,c)<<((r==HNL.rows()-1)&&(c==HNL.cols()-1)?"":" ");
            fout<<")"<<endl;
            fout<<endl;
            
            fout<<"[ALIGN_KIN_RIGHT]"<<endl;
            fout<<"HN (";
            for (int r=0; r<HNR.rows(); r++)
                for (int c=0; c<HNR.cols(); c++)
                    fout<<HNR(r,c)<<((r==HNR.rows()-1)&&(c==HNR.cols()-1)?"":" ");
            fout<<")"<<endl;
            fout<<endl;

            fout.close();
        }
    }

public:
    /************************************************************************/
    CtrlModule() : interrupting(false), doSaveTweakFile(false) { }

    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string ctrlName;
        string headName;
        string torsoName;
        double neckTime;
        double eyesTime;
        double minAbsVel;
        bool   saccadesOn;
        bool   neckPosCtrlOn;
        bool   Robotable;        
        double ping_robot_tmo;
        Vector counterRotGain(2);        

        // request high resolution scheduling
        Time::turboBoost();

        // save pointer to rf
        this->rf=&rf;

        // get params from the command-line
        ctrlName=rf.check("name",Value("iKinGazeCtrl")).asString().c_str();        
        headName=rf.check("head",Value("head")).asString().c_str();
        torsoName=rf.check("torso",Value("torso")).asString().c_str();
        neckTime=rf.check("Tneck",Value(0.75)).asDouble();
        eyesTime=rf.check("Teyes",Value(0.25)).asDouble();
        minAbsVel=CTRL_DEG2RAD*rf.check("minAbsVel",Value(0.0)).asDouble();
        ping_robot_tmo=rf.check("ping_robot_tmo",Value(0.0)).asDouble();
        saccadesOn=(rf.check("saccades",Value("on")).asString()=="on");
        neckPosCtrlOn=(rf.check("neck_position_control",Value("on")).asString()=="on");
        counterRotGain[0]=rf.check("vor",Value(1.0)).asDouble();
        counterRotGain[1]=rf.check("ocr",Value(0.0)).asDouble();
        Robotable=!rf.check("simulation");

        commData.robotName=rf.check("robot",Value("icub")).asString().c_str();
        commData.eyeTiltMin=rf.check("eyeTiltMin",Value(-1e9)).asDouble();
        commData.eyeTiltMax=rf.check("eyeTiltMax",Value(1e9)).asDouble();
        commData.head_version=rf.check("headV2")?2.0:1.0;
        commData.tweakOverwrite=(rf.check("tweakOverwrite",Value("on")).asString()=="on");

        // minAbsVel is given in absolute form
        // hence it must be positive
        if (minAbsVel<0.0)
            minAbsVel=-minAbsVel;

        if (rf.check("camerasFile"))
        {
            commData.rf_cameras.setQuiet();
            rf.check("camerasContext")?
            commData.rf_cameras.setDefaultContext(rf.find("camerasContext").asString().c_str()):
            commData.rf_cameras.setDefaultContext(rf.getContext().c_str());
            commData.rf_cameras.setDefaultConfigFile(rf.find("camerasFile").asString().c_str());            
            commData.rf_cameras.configure(0,NULL);
        }

        commData.rf_tweak.setQuiet();
        commData.rf_tweak.setDefaultContext(rf.getContext().c_str());
        commData.tweakFile=rf.check("tweakFile",Value("tweak.ini")).asString().c_str();
        commData.rf_tweak.setDefaultConfigFile(commData.tweakFile.c_str());
        commData.rf_tweak.configure(0,NULL);

        printf("Controller configured for head version %g\n",commData.head_version);

        commData.localStemName="/"+ctrlName;
        string remoteHeadName="/"+commData.robotName+"/"+headName;
        string localHeadName=commData.localStemName+"/"+headName;
        string remoteTorsoName="/"+commData.robotName+"/"+torsoName;
        string localTorsoName=commData.localStemName+"/"+torsoName;        

        if (Robotable)
        {
            Property optTorso("(device remote_controlboard)");
            optTorso.put("remote",remoteTorsoName.c_str());
            optTorso.put("local",localTorsoName.c_str());
            optTorso.put("part",torsoName.c_str());

            Property optHead("(device remote_controlboard)");
            optHead.put("remote",remoteHeadName.c_str());
            optHead.put("local",localHeadName.c_str());
            optHead.put("part",headName.c_str());
            // mixed position/velocity control entails
            // to send two packets per control slot
            optHead.put("writeStrict","on");

            if (torsoName!="disabled")
            {
                drvTorso=(ping_robot_tmo>0.0)?
                         waitPart(optTorso,ping_robot_tmo):
                         new PolyDriver(optTorso);

                if (!drvTorso->isValid())
                {
                    printf("Torso device driver not available!\n");
                    printf("Perhaps only the head is running; trying to continue ...\n");

                    delete drvTorso;
                    drvTorso=NULL;
                }
            }
            else
            {
                printf("Torso device disabled!\n");
                drvTorso=NULL;
            }

            drvHead=(ping_robot_tmo>0.0)?
                    waitPart(optHead,ping_robot_tmo):
                    new PolyDriver(optHead);

            if (!drvHead->isValid())
            {
                printf("Head device driver not available!\n");

                delete drvHead;
                delete drvTorso;
                drvTorso=drvHead=NULL;
                return false;
            }
        }
        else
        {
            printf("Controller running in simulation mode\n");
            drvTorso=drvHead=NULL;
        }

        // create and start threads
        // creation order does matter (for the minimum allowed vergence computation) !!
        ctrl=new Controller(drvTorso,drvHead,&commData,neckPosCtrlOn,neckTime,eyesTime,minAbsVel,10);
        loc=new Localizer(&commData,10);
        eyesRefGen=new EyePinvRefGen(drvTorso,drvHead,&commData,ctrl,saccadesOn,counterRotGain,20);
        slv=new Solver(drvTorso,drvHead,&commData,eyesRefGen,loc,ctrl,20);

        // this switch-on order does matter !!
        eyesRefGen->start();
        slv->start();
        ctrl->start();
        loc->start();

        rpcPort.open((commData.localStemName+"/rpc").c_str());
        attach(rpcPort);

        contextIdCnt=0;

        // reserve id==0 for start-up context
        int id0;
        storeContext(&id0);

        return true;
    }

    /************************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        int ack=Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        if (command.size()>0)
        {
            switch (command.get(0).asVocab())
            {
                //-----------------
                case VOCAB3('g','e','t'):
                {
                    if (command.size()>1)
                    {
                        int type=command.get(1).asVocab();

                        if (type==VOCAB4('T','n','e','c'))
                        {
                            reply.addVocab(ack);
                            reply.addDouble(ctrl->getTneck());
                            return true;
                        }
                        else if (type==VOCAB4('T','e','y','e'))
                        {
                            reply.addVocab(ack);
                            reply.addDouble(ctrl->getTeyes());
                            return true;
                        }
                        else if (type==VOCAB3('v','o','r'))
                        {
                            Vector gain=eyesRefGen->getCounterRotGain();
                            reply.addVocab(ack);
                            reply.addDouble(gain[0]);
                            return true;
                        }
                        else if (type==VOCAB3('o','c','r'))
                        {
                            Vector gain=eyesRefGen->getCounterRotGain();
                            reply.addVocab(ack);
                            reply.addDouble(gain[1]);
                            return true;
                        }
                        else if (type==VOCAB4('s','a','c','c'))
                        {
                            reply.addVocab(ack);
                            reply.addInt((int)eyesRefGen->isSaccadesOn());
                            return true;
                        }
                        else if (type==VOCAB4('s','i','n','h'))
                        {
                            reply.addVocab(ack);
                            reply.addDouble(eyesRefGen->getSaccadesInhibitionPeriod());
                            return true;
                        }
                        else if (type==VOCAB4('s','a','c','t'))
                        {
                            reply.addVocab(ack);
                            reply.addDouble(eyesRefGen->getSaccadesActivationAngle());
                            return true;
                        }
                        else if (type==VOCAB4('t','r','a','c'))
                        {
                            reply.addVocab(ack);
                            reply.addInt((int)ctrl->getTrackingMode());
                            return true;
                        }
                        else if (type==VOCAB4('d','o','n','e'))
                        {
                            reply.addVocab(ack);
                            reply.addInt((int)ctrl->isMotionDone());
                            return true;
                        }
                        else if (type==VOCAB4('s','d','o','n'))
                        {
                            reply.addVocab(ack);
                            reply.addInt((int)!commData.get_isSaccadeUnderway());
                            return true;
                        }
                        else if (type==VOCAB4('p','i','t','c'))
                        {
                            double min_deg,max_deg;
                            slv->getCurNeckPitchRange(min_deg,max_deg);

                            reply.addVocab(ack);
                            reply.addDouble(min_deg);
                            reply.addDouble(max_deg);
                            return true;
                        }
                        else if (type==VOCAB4('r','o','l','l'))
                        {
                            double min_deg,max_deg;
                            slv->getCurNeckRollRange(min_deg,max_deg);

                            reply.addVocab(ack);
                            reply.addDouble(min_deg);
                            reply.addDouble(max_deg);
                            return true;
                        }
                        else if (type==VOCAB3('y','a','w'))
                        {
                            double min_deg,max_deg;
                            slv->getCurNeckYawRange(min_deg,max_deg);

                            reply.addVocab(ack);
                            reply.addDouble(min_deg);
                            reply.addDouble(max_deg);
                            return true;
                        }
                        else if (type==VOCAB4('e','y','e','s'))
                        {
                            double ver=eyesRefGen->getEyesBoundVer();

                            reply.addVocab(ack);
                            reply.addDouble(ver);
                            return true;
                        }
                        else if (type==VOCAB4('n','t','o','l'))
                        {
                            double angle=slv->getNeckAngleUserTolerance();

                            reply.addVocab(ack);
                            reply.addDouble(angle);
                            return true;
                        }
                        else if (type==VOCAB3('d','e','s'))
                        {
                            Vector des;
                            if (ctrl->getDesired(des))
                            {
                                reply.addVocab(ack);
                                Bottle &bDes=reply.addList();
                                for (size_t i=0; i<des.length(); i++)
                                    bDes.addDouble(des[i]);

                                return true;
                            }
                        }
                        else if (type==VOCAB3('v','e','l'))
                        {
                            Vector vel;
                            if (ctrl->getVelocity(vel))
                            {
                                reply.addVocab(ack);
                                Bottle &bVel=reply.addList();
                                for (size_t i=0; i<vel.length(); i++)
                                    bVel.addDouble(vel[i]);

                                return true;
                            }
                        }
                        else if ((type==VOCAB4('p','o','s','e')) && (command.size()>2))
                        {
                            string poseSel=command.get(2).asString().c_str();
                            Vector x;
                            Stamp  stamp;

                            if (ctrl->getPose(poseSel,x,stamp))
                            {
                                reply.addVocab(ack);
                                Bottle &bPose=reply.addList();
                                for (size_t i=0; i<x.length(); i++)
                                    bPose.addDouble(x[i]);

                                Bottle &bStamp=reply.addList();
                                bStamp.addInt(stamp.getCount());
                                bStamp.addDouble(stamp.getTime());

                                return true;
                            }
                        }
                        else if ((type==VOCAB2('2','D')) && (command.size()>2))
                        {
                            if (Bottle *bOpt=command.get(2).asList())
                            {
                                if (bOpt->size()>3)
                                {
                                    Vector x(3);
                                    string eye=bOpt->get(0).asString().c_str();
                                    x[0]=bOpt->get(1).asDouble();
                                    x[1]=bOpt->get(2).asDouble();
                                    x[2]=bOpt->get(3).asDouble();

                                    Vector px;
                                    if (loc->projectPoint(eye,x,px))
                                    {
                                        reply.addVocab(ack);
                                        Bottle &bPixel=reply.addList();
                                        for (size_t i=0; i<px.length(); i++)
                                            bPixel.addDouble(px[i]);

                                        return true;
                                    }
                                }
                            }
                        }
                        else if ((type==VOCAB2('3','D')) && (command.size()>3))
                        {
                            int subType=command.get(2).asVocab();
                            if (subType==VOCAB4('m','o','n','o'))
                            {
                                if (Bottle *bOpt=command.get(3).asList())
                                {
                                    if (bOpt->size()>3)
                                    {
                                        string eye=bOpt->get(0).asString().c_str();
                                        double u=bOpt->get(1).asDouble();
                                        double v=bOpt->get(2).asDouble();
                                        double z=bOpt->get(3).asDouble();

                                        Vector x;
                                        if (loc->projectPoint(eye,u,v,z,x))
                                        {
                                            reply.addVocab(ack);
                                            Bottle &bPoint=reply.addList();
                                            for (size_t i=0; i<x.length(); i++)
                                                bPoint.addDouble(x[i]);
                                            
                                            return true;
                                        }
                                    }
                                }
                            }
                            else if (subType==VOCAB4('s','t','e','r'))
                            {
                                if (Bottle *bOpt=command.get(3).asList())
                                {
                                    if (bOpt->size()>3)
                                    {
                                        Vector pxl(2),pxr(2);
                                        pxl[0]=bOpt->get(0).asDouble();
                                        pxl[1]=bOpt->get(1).asDouble();
                                        pxr[0]=bOpt->get(2).asDouble();
                                        pxr[1]=bOpt->get(3).asDouble();

                                        Vector x;
                                        if (loc->triangulatePoint(pxl,pxr,x))
                                        {
                                            reply.addVocab(ack);
                                            Bottle &bPoint=reply.addList();
                                            for (size_t i=0; i<x.length(); i++)
                                                bPoint.addDouble(x[i]);

                                            return true;
                                        }
                                    }
                                }
                            }
                            else if (subType==VOCAB4('p','r','o','j'))
                            {
                                if (Bottle *bOpt=command.get(3).asList())
                                {
                                    if (bOpt->size()>6)
                                    {
                                        Vector plane(4);
                                        string eye=bOpt->get(0).asString().c_str();
                                        double u=bOpt->get(1).asDouble();
                                        double v=bOpt->get(2).asDouble();
                                        plane[0]=bOpt->get(3).asDouble();
                                        plane[1]=bOpt->get(4).asDouble();
                                        plane[2]=bOpt->get(5).asDouble();
                                        plane[3]=bOpt->get(6).asDouble();

                                        Vector x;
                                        if (loc->projectPoint(eye,u,v,plane,x))
                                        {
                                            reply.addVocab(ack);
                                            Bottle &bPoint=reply.addList();
                                            for (size_t i=0; i<x.length(); i++)
                                                bPoint.addDouble(x[i]);

                                            return true;
                                        }
                                    }
                                }
                            }
                            else if (subType==VOCAB3('a','n','g'))
                            {
                                if (Bottle *bOpt=command.get(3).asList())
                                {
                                    if (bOpt->size()>3)
                                    {
                                        Vector ang(3);
                                        string type=bOpt->get(0).asString().c_str();
                                        ang[0]=CTRL_DEG2RAD*bOpt->get(1).asDouble();
                                        ang[1]=CTRL_DEG2RAD*bOpt->get(2).asDouble();
                                        ang[2]=CTRL_DEG2RAD*bOpt->get(3).asDouble();

                                        Vector x=loc->get3DPoint(type,ang);
                                        reply.addVocab(ack);
                                        Bottle &bPoint=reply.addList();
                                        for (size_t i=0; i<x.length(); i++)
                                            bPoint.addDouble(x[i]);

                                        return true;
                                    }
                                }
                            }
                        }
                        else if ((type==VOCAB3('a','n','g')) && (command.size()>2))
                        {
                            if (Bottle *bOpt=command.get(2).asList())
                            {
                                if (bOpt->size()>2)
                                {
                                    Vector x(3);
                                    x[0]=bOpt->get(0).asDouble();
                                    x[1]=bOpt->get(1).asDouble();
                                    x[2]=bOpt->get(2).asDouble();

                                    Vector ang=CTRL_RAD2DEG*loc->getAbsAngles(x);
                                    reply.addVocab(ack);
                                    Bottle &bAng=reply.addList();
                                    for (size_t i=0; i<ang.length(); i++)
                                        bAng.addDouble(ang[i]);

                                    return true;
                                }
                            }
                        }
                        else if (type==VOCAB3('p','i','d'))
                        {
                            Bottle options;
                            loc->getPidOptions(options);

                            reply.addVocab(ack);
                            reply.addList()=options;
                            return true;
                        }
                        else if (type==VOCAB4('i','n','f','o'))
                        {
                            Bottle info;
                            if (getInfo(info))
                            {
                                reply.addVocab(ack);
                                reply.addList()=info;
                                return true;
                            }
                        }
                        else if (type==VOCAB4('t','w','e','a'))
                        {
                            Bottle options;
                            if (tweakGet(options))
                            {
                                reply.addVocab(ack);
                                reply.addList()=options;
                                return true;
                            }
                        }
                    }

                    break;
                }

                //-----------------
                case VOCAB3('s','e','t'):
                {
                    if (command.size()>2)
                    {
                        int type=command.get(1).asVocab();
                        if (type==VOCAB4('T','n','e','c'))
                        {
                            double execTime=command.get(2).asDouble();
                            ctrl->setTneck(execTime);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (type==VOCAB4('T','e','y','e'))
                        {
                            double execTime=command.get(2).asDouble();
                            ctrl->setTeyes(execTime);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (type==VOCAB3('v','o','r'))
                        {
                            Vector gain=eyesRefGen->getCounterRotGain();
                            gain[0]=command.get(2).asDouble();
                            eyesRefGen->setCounterRotGain(gain);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (type==VOCAB3('o','c','r'))
                        {
                            Vector gain=eyesRefGen->getCounterRotGain();
                            gain[1]=command.get(2).asDouble();
                            eyesRefGen->setCounterRotGain(gain);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (type==VOCAB4('s','a','c','c'))
                        {
                            bool mode=(command.get(2).asInt()>0);
                            eyesRefGen->setSaccades(mode);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (type==VOCAB4('s','i','n','h'))
                        {
                            double period=command.get(2).asDouble();
                            eyesRefGen->setSaccadesInhibitionPeriod(period);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (type==VOCAB4('s','a','c','t'))
                        {
                            double angle=command.get(2).asDouble();
                            eyesRefGen->setSaccadesActivationAngle(angle);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (type==VOCAB4('n','t','o','l'))
                        {
                            double angle=command.get(2).asDouble();
                            slv->setNeckAngleUserTolerance(angle);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (type==VOCAB4('t','r','a','c'))
                        {
                            bool mode=(command.get(2).asInt()>0);
                            ctrl->setTrackingMode(mode);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (type==VOCAB3('p','i','d'))
                        {
                            if (Bottle *bOpt=command.get(2).asList())
                            {
                                loc->setPidOptions(*bOpt);
                                reply.addVocab(ack);
                                return true;
                            }
                        }
                        else if (type==VOCAB4('t','w','e','a'))
                        {
                            if (Bottle *bOpt=command.get(2).asList())
                            {
                                if (tweakSet(*bOpt))
                                    reply.addVocab(ack);
                                else
                                    reply.addVocab(nack);

                                return true;
                            }
                        }
                    }

                    break;
                }

                //-----------------
                case VOCAB4('s','t','o','p'):
                {
                    ctrl->stopControl();
                    reply.addVocab(ack);
                    return true;
                }

                //-----------------
                case VOCAB4('s','t','o','r'):
                {
                    int id;
                    storeContext(&id);                    
                    reply.addVocab(ack);
                    reply.addInt(id);
                    return true;
                }

                //-----------------
                case VOCAB4('r','e','s','t'):
                {
                    if (command.size()>1)
                    {
                        int id=command.get(1).asInt();
                        if (restoreContext(id))
                        {
                            reply.addVocab(ack);
                            return true;
                        }
                    }

                    break;
                }

                //-----------------
                case VOCAB3('d','e','l'):
                {
                    if (command.size()>1)
                    {
                        Bottle *ids=command.get(1).asList();
                        if (deleteContexts(ids))
                        {
                            reply.addVocab(ack);
                            return true;
                        }
                    }

                    break;
                }

                //-----------------
                case VOCAB4('b','i','n','d'):
                {
                    if (command.size()>2)
                    {
                        int joint=command.get(1).asVocab();
                        if (joint==VOCAB4('p','i','t','c'))
                        {
                            double min=command.get(2).asDouble();
                            double max=command.get(3).asDouble();
                            slv->bindNeckPitch(min,max);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (joint==VOCAB4('r','o','l','l'))
                        {
                            double min=command.get(2).asDouble();
                            double max=command.get(3).asDouble();
                            slv->bindNeckRoll(min,max);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (joint==VOCAB3('y','a','w'))
                        {
                            double min=command.get(2).asDouble();
                            double max=command.get(3).asDouble();
                            slv->bindNeckYaw(min,max);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (joint==VOCAB4('e','y','e','s'))
                        {
                            double ver=command.get(2).asDouble();
                            eyesRefGen->bindEyes(ver);
                            reply.addVocab(ack);
                            return true;
                        }
                    }

                    break;
                }

                //-----------------
                case VOCAB4('c','l','e','a'):
                {
                    if (command.size()>1)
                    {
                        int joint=command.get(1).asVocab();
                        if (joint==VOCAB4('p','i','t','c'))
                        {
                            slv->clearNeckPitch();
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (joint==VOCAB4('r','o','l','l'))
                        {
                            slv->clearNeckRoll();
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (joint==VOCAB3('y','a','w'))
                        {
                            slv->clearNeckYaw();
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (joint==VOCAB4('e','y','e','s'))
                        {
                            eyesRefGen->clearEyes();
                            reply.addVocab(ack);
                            return true;
                        }
                    }

                    break;
                }

                //-----------------
                case VOCAB4('r','e','g','i'):
                {
                    if (command.size()>1)
                    {
                        int type=command.get(1).asVocab();
                        if (type==VOCAB4('o','n','g','o'))
                        {
                            if (command.size()>2)
                            {
                                double checkPoint=command.get(2).asDouble();
                                if (ctrl->registerMotionOngoingEvent(checkPoint))
                                {
                                    reply.addVocab(ack);
                                    return true;
                                }
                            }
                        }
                    }

                    break;
                }

                //-----------------
                case VOCAB4('u','n','r','e'):
                {
                    if (command.size()>1)
                    {
                        int type=command.get(1).asVocab();
                        if (type==VOCAB4('o','n','g','o'))
                        {
                            if (command.size()>2)
                            {
                                double checkPoint=command.get(2).asDouble();
                                if (ctrl->unregisterMotionOngoingEvent(checkPoint))
                                {
                                    reply.addVocab(ack);
                                    return true;
                                }
                            }
                        }
                    }

                    break;
                }

                //-----------------
                case VOCAB4('l','i','s','t'):
                {
                    if (command.size()>1)
                    {
                        int type=command.get(1).asVocab();
                        if (type==VOCAB4('o','n','g','o'))
                        {
                            reply.addVocab(ack);
                            reply.addList()=ctrl->listMotionOngoingEvents();
                            return true;
                        }
                    }

                    break;
                }

                //-----------------
                case VOCAB4('s','u','s','p'):
                {
                    ctrl->suspend();
                    eyesRefGen->suspend();
                    slv->suspend();
                    reply.addVocab(ack);
                    return true;
                }

                //-----------------
                case VOCAB3('r','u','n'):
                {
                    slv->resume();
                    eyesRefGen->resume();
                    ctrl->resume();
                    reply.addVocab(ack);
                    return true;
                }

                //-----------------
                case VOCAB4('s','t','a','t'):
                {
                    reply.addVocab(ack);
                    if (ctrl->isSuspended())
                        reply.addString("suspended");
                    else
                        reply.addString("running");
                    return true; 
                }

                //-----------------
                default:
                    return RFModule::respond(command,reply);
            }
        }

        reply.addVocab(nack);
        return true;
    }

    /************************************************************************/
    bool interruptModule()
    {
        interrupting=true;
        return true;
    }

    /************************************************************************/
    bool close()
    {
        loc->stop();
        eyesRefGen->stop();
        slv->stop();
        ctrl->stop();

        delete loc;
        delete eyesRefGen;
        delete slv;
        delete ctrl;
        delete drvTorso;
        delete drvHead;

        rpcPort.interrupt();
        rpcPort.close();

        contextMap.clear();

        return true;
    }

    /************************************************************************/
    double getPeriod()
    {
        return 1.0;
    }

    /************************************************************************/
    bool updateModule()
    {
        if (doSaveTweakFile)
        {
            savingTweakFile.lock();
            saveTweakFile();
            doSaveTweakFile=false;
            savingTweakFile.unlock();
        }

        return true;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext("iKinGazeCtrl");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("YARP server not available!\n");
        return -1;
    }

    CtrlModule mod;
    return mod.runModule(rf);
}



