/*
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini, Alessandro Roncone
 * email:  ugo.pattacini@iit.it, alessandro.roncone@iit.it
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

Authors: Ugo Pattacini, Alessandro Roncone

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
      please quote it within any resulting publication</b>:
      Roncone A., Pattacini U., Metta G. & Natale L.,
      "A Cartesian 6-DoF Gaze Controller for Humanoid Robots",
      <i>Proceedings of Robotics: Science and Systems</i>,
      Ann Arbor, MI, June 18-22, 2016.

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

\note A video on iCub gazing can be seen <a
      href="https://youtu.be/I4ZKfAvs1y0">here</a>.

\section lib_sec Libraries
- YARP libraries.
- \ref iKin "iKin" library (it requires IPOPT lib: see the <a
  href="http://wiki.icub.org/wiki/Installing_IPOPT">wiki</a>).

\section parameters_sec Parameters
--context \e dir
- Resource finder default searching directory for configuration
  files; if not specified, \e iKinGazeCtrl is assumed. Beware of
  the <a href="http://www.yarp.it/yarp_data_dirs.html">context
  search policy</a>.

--from \e file
- Resource finder default configuration file; if not specified,
  \e config.ini is assumed.

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
 *string \e off can be used to skip opening the torso device.  

--trajectory_time::neck \e time
- Specify the neck trajectory execution time in point-to-point
  movements [expressed in seconds]; by default \e time is 0.75
  seconds. (Tneck cannot be set equal or lower than Teyes).

--trajectory_time::eyes \e time
- Specify the eyes trajectory execution time in point-to-point
  movements [expressed in seconds]; by default \e time is 0.25
  seconds.

--cameras::context \e dir
- The parameter \e dir specifies the context used to locate the
  cameras parameters file (see below).

--cameras::file \e file
- The parameter \e file specifies the file name used to read
  cameras parameters.

--saccades \e switch
- Enable/disable saccadic movements; the parameter \e switch can
  be therefore ["on"|"off"], being "on" by default.

--neck_position_control \e switch
- Enable/disable low-level position control of the neck; the
  parameter \e switch can be therefore ["on"|"off"], being "on"
  by default.

--imu::mode \e switch
- Enable/disable stabilization using IMU data; the parameter
  \e switch can be therefore ["on"|"off"], being "on"
  by default.

--imu::source_port_name \e name
- Allow specifying a different source port for the IMU data
  (see IMU filtering tools such as e.g. \ref imuFilter).

--imu::stabilization_gain \e gain
- Specify the integral gain (in [1/s]) used for gaze
  stabilization; the \e gain is 11.0 [1/s] by default.

--imu::gyro_noise_threshold \e thres
- Specify a different threshold \e thres given in [deg/s] to
  filter out the residual bias in gyro readouts.

--imu::vor \e gain
- Specify the contribution of the vestibulo-ocular reflex (VOR)
  in computing the final counter-rotation of the eyes due to
  neck rotation. To turn off the VOR just set the \e gain equal
  to 0.0. By default \e gain is 1.0, that means <i>"full
  contribution"</i>. If <i>imu::mode</i> is "off", then the gain
  is 0.0 by default. Values of the gain greater than 1.0 mean
  <i>"contribution amplified"</i>.
--imu::timeout
- Specify the read timeout (in seconds) for the imu data. The
  default is 0.04 seconds.

--ocr \e gain
- Specify the contribution of the oculo-collic reflex (OCR) in
  computing the counter-rotation of the eyes due to neck
  rotation. To turn off the OCR just set the \e gain equal to
  0.0. Default values are 0.0 if <i>imu::mode</i> is "off", 1.0
  otherwise.

--ping_robot_tmo \e tmo
- The parameter \e tmo is the timeout (in seconds) that allows
  starting up the robot before connecting to it; by default we
  have a timeout of 40.0 [s].

--eye_tilt::min \e min
- The parameter \e min specifies the minimum eye tilt angle
  [deg] in order to prevent the eye from being covered by the
  eyelid (when they're wide open) while moving; default value is
  -12 [deg].

--eye_tilt::max \e max
- The parameter \e max specifies the maximum eye tilt angle
  [deg] in order to prevent the eye from being covered by the
  eyelid (when they're wide open) while moving; default value is
  15 [deg].

--min_abs_vel \e vel
- The parameter \e vel specifies the minimum absolute velocity
  that can be achieved by the robot [deg/s] due to the
  approximation performed while delivering data over the
  network. By default this value is 0.0 having no result on the
  controller's operations. In case it is different from 0.0, the
  controller will implement a bang-bang approach whenever the
  velocity to be delivered goes under the minimum threshold.

--head_version \e ver
- This option specifies the kinematic structure of the head; the value
  \e ver is a double in the set {1.0, 2.0, 2.5, 2.6, 3.0}, being 1.0
  the default version.

--verbose
- Enable some output print-out.

--tweak::file \e file
- The parameter \e file specifies the file name (located in
  module context) used to read/write options that are tweakable
  by the user; if not provided, \e tweak.ini is assumed.

--tweak::overwrite \e switch
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
  angles is the one defined by kinematic chain.
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
   - "stabilization-on" <time>: notify that gaze stabilization
     has been turned on; comprise the time instant of the source
     when the event took place.
   - "stabilization-off" <time>: notify that gaze stabilization
     has been turned off; comprise the time instant of the
     source when the event took place.
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
      ahead (the tilt can vary) with a specified vergence given
      in degrees.
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
    - [get] [stab]: returns 1 iff gaze stabilization is active,
      0 otherwise.
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
      option <type> can be ["abs"|"rel"].
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
    - [set] [stab] <val>: turns on/off the gaze stabilization
      (if enabled at start-up); val can be [0/1].
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
    - [look] [3D] (<x> <y> <z>): yields gazing at target specified
      as 3D point. Distances are in meters.
    - [look] [mono] (<type> < u> <v> <z>): yields gazing at target
      specified in terms of pixel coordinates (u,v) in the image
      plane <type> ["left"|"right"] along with third component
      <z> in the eye's reference frame. Distances are in meters.
    - [look] [mono] (<type> < u> <v> "ver" <ver>): yields gazing at target
      specified in terms of pixel coordinates (u,v) in the image
      plane <type> ["left"|"right"] along with the vergence <ver>.
      Angles are in degrees.
    - [look] [stereo] (< ul> <vl> <ur> <vr>): yields gazing at target
      specified in terms of projected pixels coordinates (ul,vl) and
      (ur,vr) in the image planes.
    - [look] [ang] (<type> <azi> <ele> <ver>): yields gazing at target
      specified in angular coordinates. The option <type> can be ["abs"|"rel"].
      Angles are in degrees.
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
A configuration file passed through \e --cameras::file contains
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

\author Ugo Pattacini, Alessandro Roncone
*/

#include <mutex>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <map>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>

#include <iCub/localizer.h>
#include <iCub/solver.h>
#include <iCub/controller.h>

#define GAZECTRL_SERVER_VER     1.2

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;


/************************************************************************/
class GazeModule: public RFModule
{
protected:
    ResourceFinder *rf;
    Localizer      *loc;
    EyePinvRefGen  *eyesRefGen;
    Solver         *slv;
    Controller     *ctrl;
    PolyDriver     *drvTorso, *drvHead;
    PolyDriver      mas_client;
    ExchangeData    commData;
    bool            interrupting;
    bool            doSaveTweakFile;
    mutex           mutexContext;
    mutex           mutexTweak;

    IThreeAxisGyroscopes* iGyro;
    IThreeAxisLinearAccelerometers* iAccel;

    RpcServer rpcPort;

    struct Context
    {
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

        // controller part
        double eyesTime;
        double neckTime;
        bool   trackingOn;
        bool   gazeStabilizationOn;

        // localizer part
        Bottle pidOptions;
    };

    int contextIdCnt;
    std::map<int,Context> contextMap;

    /************************************************************************/
    PolyDriver *waitPart(const Property &partOpt, const double ping_robot_tmo)
    {
        string partName=partOpt.find("part").asString();
        PolyDriver *pDrv=nullptr;

        double t0=Time::now();
        while (Time::now()-t0<ping_robot_tmo)
        {
            delete pDrv;
            pDrv=new PolyDriver(const_cast<Property&>(partOpt));
            bool ok=pDrv->isValid();

            if (ok)
            {
                yInfo("Checking if %s part is active ... yes",partName.c_str());
                return pDrv;
            }
            else
            {
                double dt=ping_robot_tmo-(Time::now()-t0);
                yInfo("Checking if %s part is active ... not yet: still %.1f [s] to timeout expiry",
                      partName.c_str(),dt>0.0?dt:0.0);

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
        lock_guard<mutex> lg(mutexContext);
        Context &context=contextMap[contextIdCnt];

        // solver part
        slv->getCurNeckPitchRange(context.neckPitchMin,context.neckPitchMax);
        slv->getCurNeckRollRange(context.neckRollMin,context.neckRollMax);
        slv->getCurNeckYawRange(context.neckYawMin,context.neckYawMax);
        context.neckAngleUserTolerance=slv->getNeckAngleUserTolerance();
        context.eyesBoundVer=commData.eyesBoundVer;
        context.counterRotGain=eyesRefGen->getCounterRotGain();
        context.saccadesOn=commData.saccadesOn;
        context.saccadesInhibitionPeriod=commData.saccadesInhibitionPeriod;
        context.saccadesActivationAngle=commData.saccadesActivationAngle;

        // controller part
        context.eyesTime=ctrl->getTeyes();
        context.neckTime=ctrl->getTneck();
        context.trackingOn=ctrl->getTrackingMode();
        context.gazeStabilizationOn=ctrl->getGazeStabilization();

        // localizer part
        loc->getPidOptions(context.pidOptions);

        if (id!=nullptr)
            *id=contextIdCnt++;
    }

    /************************************************************************/
    bool restoreContext(const int id)
    {
        lock_guard<mutex> lg(mutexContext);
        auto itr=contextMap.find(id);
        if (itr!=contextMap.end())
        {
            Context &context=itr->second;

            // solver part
            slv->bindNeckPitch(context.neckPitchMin,context.neckPitchMax);
            slv->bindNeckRoll(context.neckRollMin,context.neckRollMax);
            slv->bindNeckYaw(context.neckYawMin,context.neckYawMax);
            slv->setNeckAngleUserTolerance(context.neckAngleUserTolerance);
            eyesRefGen->manageBindEyes(context.eyesBoundVer);
            eyesRefGen->setCounterRotGain(context.counterRotGain);
            commData.saccadesOn=context.saccadesOn;
            commData.saccadesInhibitionPeriod=context.saccadesInhibitionPeriod;
            commData.saccadesActivationAngle=context.saccadesActivationAngle;

            // controller part
            ctrl->setTeyes(context.eyesTime);
            ctrl->setTneck(context.neckTime);
            ctrl->setTrackingMode(context.trackingOn);
            ctrl->setGazeStabilization(context.gazeStabilizationOn);

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
        if (contextIdList!=nullptr)
        {
            lock_guard<mutex> lg(mutexContext);
            for (int i=0; i<contextIdList->size(); i++)
            {
                int id=contextIdList->get(i).asInt();
                auto itr=contextMap.find(id);
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
        minVer.addDouble(CTRL_RAD2DEG*commData.minAllowedVergence);

        Bottle &events=info.addList();
        events.addString("events");
        Bottle &eventsList=events.addList();
        eventsList.addString("motion-onset");
        eventsList.addString("motion-done");
        eventsList.addString("motion-ongoing");
        eventsList.addString("saccade-onset");
        eventsList.addString("saccade-done");
        eventsList.addString("stabilization-on");
        eventsList.addString("stabilization-off");
        eventsList.addString("closing");
        eventsList.addString("suspended");
        eventsList.addString("resumed");
        eventsList.addString("comm-timeout");
        eventsList.addString("*");

        tweakGet(info);

        return true;
    }

    /************************************************************************/
    bool tweakSet(const Bottle &options)
    {
        lock_guard<mutex> lg(mutexTweak);
        if (Bottle *pB=options.find("camera_intrinsics_left").asList())
        {
            Matrix Prj=zeros(3,4); int w,h;
            loc->getIntrinsicsMatrix("left",Prj,w,h);

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

            loc->setIntrinsicsMatrix("left",Prj,w,h);
            doSaveTweakFile=commData.tweakOverwrite;
        }

        if (Bottle *pB=options.find("camera_intrinsics_right").asList())
        {
            Matrix Prj=zeros(3,4); int w,h;
            loc->getIntrinsicsMatrix("right",Prj,w,h);

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

            loc->setIntrinsicsMatrix("right",Prj,w,h);
            doSaveTweakFile=commData.tweakOverwrite;
        }

        if (options.check("camera_width_left"))
        {
            Matrix Prj=zeros(3,4); int w,h;
            loc->getIntrinsicsMatrix("left",Prj,w,h);
            w=options.find("camera_width_left").asInt();
            loc->setIntrinsicsMatrix("left",Prj,w,h);
        }

        if (options.check("camera_height_left"))
        {
            Matrix Prj=zeros(3,4); int w,h;
            loc->getIntrinsicsMatrix("left",Prj,w,h);
            h=options.find("camera_height_left").asInt();
            loc->setIntrinsicsMatrix("left",Prj,w,h);
        }

        if (options.check("camera_width_right"))
        {
            Matrix Prj=zeros(3,4); int w,h;
            loc->getIntrinsicsMatrix("right",Prj,w,h);
            w=options.find("camera_width_right").asInt();
            loc->setIntrinsicsMatrix("right",Prj,w,h);
        }

        if (options.check("camera_height_right"))
        {
            Matrix Prj=zeros(3,4); int w,h;
            loc->getIntrinsicsMatrix("right",Prj,w,h);
            h=options.find("camera_height_right").asInt();
            loc->setIntrinsicsMatrix("right",Prj,w,h);
        }

        bool doMinAllowedVer=false;
        if (Bottle *pB=options.find("camera_extrinsics_left").asList())
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

        if (Bottle *pB=options.find("camera_extrinsics_right").asList())
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

        return true;
    }

    /************************************************************************/
    bool tweakGet(Bottle &options)
    {
        lock_guard<mutex> lg(mutexTweak);
        Bottle &intrinsicsLeft=options.addList();
        intrinsicsLeft.addString("camera_intrinsics_left");
        Bottle &intrinsicsLeftValues=intrinsicsLeft.addList();
        Matrix PrjL; int wL,hL;
        if (loc->getIntrinsicsMatrix("left",PrjL,wL,hL))
            for (int r=0; r<PrjL.rows(); r++)
                for (int c=0; c<PrjL.cols(); c++)
                    intrinsicsLeftValues.addDouble(PrjL(r,c));

        Bottle &widthLeft=options.addList();
        widthLeft.addString("camera_width_left");
        widthLeft.addInt(wL);
        Bottle &heightLeft=options.addList();
        heightLeft.addString("camera_height_left");
        heightLeft.addInt(hL);

        Bottle &intrinsicsRight=options.addList();
        intrinsicsRight.addString("camera_intrinsics_right");
        Bottle &intrinsicsRightValues=intrinsicsRight.addList();
        Matrix PrjR; int wR,hR;
        if (loc->getIntrinsicsMatrix("right",PrjR,wR,hR))
            for (int r=0; r<PrjR.rows(); r++)
                for (int c=0; c<PrjR.cols(); c++)
                    intrinsicsRightValues.addDouble(PrjR(r,c));

        Bottle &widthRight=options.addList();
        widthRight.addString("camera_width_right");
        widthRight.addInt(wR);
        Bottle &heightRight=options.addList();
        heightRight.addString("camera_height_right");
        heightRight.addInt(hR);

        Bottle &extrinsicsLeft=options.addList();
        extrinsicsLeft.addString("camera_extrinsics_left");
        Bottle &extrinsicsLeftValues=extrinsicsLeft.addList();
        Matrix HL;
        if (loc->getExtrinsicsMatrix("left",HL))
            for (int r=0; r<HL.rows(); r++)
                for (int c=0; c<HL.cols(); c++)
                    extrinsicsLeftValues.addDouble(HL(r,c));

        Bottle &extrinsicsRight=options.addList();
        extrinsicsRight.addString("camera_extrinsics_right");
        Bottle &extrinsicsRightValues=extrinsicsRight.addList();
        Matrix HR;
        if (loc->getExtrinsicsMatrix("right",HR))
            for (int r=0; r<HR.rows(); r++)
                for (int c=0; c<HR.cols(); c++)
                    extrinsicsRightValues.addDouble(HR(r,c));

        return true;
    }

    /************************************************************************/
    void saveTweakFile()
    {
        Matrix PrjL; int wL,hL;
        bool validIntrinsicsL=loc->getIntrinsicsMatrix("left",PrjL,wL,hL);

        Matrix PrjR; int wR,hR;
        bool validIntrinsicsR=loc->getIntrinsicsMatrix("right",PrjR,wR,hR);

        Matrix HNL;
        loc->getExtrinsicsMatrix("left",HNL);

        Matrix HNR;
        loc->getExtrinsicsMatrix("right",HNR);

        ofstream fout;
        string tweakFile=rf->getHomeContextPath();
        tweakFile+="/"+commData.tweakFile;
        fout.open(tweakFile.c_str());
        if (fout.is_open())
        {
            if (validIntrinsicsL)
            {
                fout<<"[CAMERA_CALIBRATION_LEFT]"<<endl;
                fout<<"w  "<<wL<<endl;
                fout<<"h  "<<hL<<endl;
                fout<<"fx "<<PrjL(0,0)<<endl;
                fout<<"fy "<<PrjL(1,1)<<endl;
                fout<<"cx "<<PrjL(0,2)<<endl;
                fout<<"cy "<<PrjL(1,2)<<endl;
                fout<<endl;
            }

            if (validIntrinsicsR)
            {
                fout<<"[CAMERA_CALIBRATION_RIGHT]"<<endl;
                fout<<"w  "<<wR<<endl;
                fout<<"h  "<<hR<<endl;
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

    /************************************************************************/
    double constrainHeadVersion(const double ver_in)
    {
        // std::map<k,v> is ordered based on std::less<k>
        map<double,double> d;
        d[fabs(1.0-ver_in)]=1.0;
        d[fabs(2.0-ver_in)]=2.0;
        d[fabs(2.5-ver_in)]=2.5;
        d[fabs(2.6-ver_in)]=2.6;
        d[fabs(3.0-ver_in)]=3.0;

        double ver_out=d.begin()->second;
        if (ver_out!=ver_in)
        {
            yWarning("Unknown \"head_version\" %g requested => used \"head_version\" %g instead",
                     ver_in,ver_out);
        }

        return ver_out;
    }

public:
    /************************************************************************/
    GazeModule() : rf{nullptr},
                   contextIdCnt{0},
                   loc{nullptr},
                   eyesRefGen{nullptr},
                   slv{nullptr},
                   ctrl{nullptr},
                   drvTorso{nullptr},
                   drvHead{nullptr},
                   interrupting{false},
                   doSaveTweakFile{false},
                   iGyro{nullptr},
                   iAccel{nullptr}
    {
    }

    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string ctrlName;
        string headName;
        string torsoName;
        double neckTime;
        double eyesTime;
        double min_abs_vel;
        double ping_robot_tmo;
        Vector counterRotGain(2);

        // save pointer to rf
        this->rf=&rf;

        // retrieve groups
        Bottle &imuGroup=rf.findGroup("imu");
        Bottle &eyeTiltGroup=rf.findGroup("eye_tilt");
        Bottle &trajTimeGroup=rf.findGroup("trajectory_time");
        Bottle &camerasGroup=rf.findGroup("cameras");
        Bottle &tweakGroup=rf.findGroup("tweak");

        // get params from the command-line
        ctrlName=rf.check("name",Value("iKinGazeCtrl")).asString();
        headName=rf.check("head",Value("head")).asString();
        torsoName=rf.check("torso",Value("torso")).asString();
        neckTime=trajTimeGroup.check("neck",Value(0.75)).asDouble();
        eyesTime=trajTimeGroup.check("eyes",Value(0.25)).asDouble();
        min_abs_vel=CTRL_DEG2RAD*fabs(rf.check("min_abs_vel",Value(0.0)).asDouble());
        ping_robot_tmo=rf.check("ping_robot_tmo",Value(40.0)).asDouble();

        commData.robotName=rf.check("robot",Value("icub")).asString();
        commData.eyeTiltLim[0]=eyeTiltGroup.check("min",Value(-20.0)).asDouble();
        commData.eyeTiltLim[1]=eyeTiltGroup.check("max",Value(15.0)).asDouble();
        commData.head_version=constrainHeadVersion(rf.check("head_version",Value(1.0)).asDouble());
        commData.verbose=rf.check("verbose");
        commData.saccadesOn=(rf.check("saccades",Value("on")).asString()=="on");
        commData.neckPosCtrlOn=(rf.check("neck_position_control",Value("on")).asString()=="on");
        commData.stabilizationOn=(imuGroup.check("mode",Value("on")).asString()=="on");
        commData.stabilizationGain=imuGroup.check("stabilization_gain",Value(11.0)).asDouble();
        commData.gyro_noise_threshold=CTRL_DEG2RAD*imuGroup.check("gyro_noise_threshold",Value(5.0)).asDouble();
        commData.debugInfoEnabled=rf.check("debugInfo",Value("off")).asString()=="on";

        if (commData.stabilizationOn)
        {
            counterRotGain[0]=imuGroup.check("vor",Value(1.0)).asDouble();
            counterRotGain[1]=rf.check("ocr",Value(0.0)).asDouble();
        }
        else
        {
            counterRotGain[0]=imuGroup.check("vor",Value(0.0)).asDouble();
            counterRotGain[1]=rf.check("ocr",Value(1.0)).asDouble();
        }

        if (camerasGroup.check("file"))
        {
            camerasGroup.check("context")?
            commData.rf_cameras.setDefaultContext(camerasGroup.find("context").asString()):
            commData.rf_cameras.setDefaultContext(rf.getContext());
            commData.rf_cameras.setDefaultConfigFile(camerasGroup.find("file").asString().c_str());
            commData.rf_cameras.configure(0,nullptr);
        }

        commData.rf_tweak.setDefaultContext(rf.getContext());
        commData.tweakFile=tweakGroup.check("file",Value("tweak.ini")).asString();
        commData.tweakOverwrite=(tweakGroup.check("overwrite",Value("on")).asString()=="on");
        commData.rf_tweak.setDefaultConfigFile(commData.tweakFile.c_str());
        commData.rf_tweak.configure(0,nullptr);

        yInfo("Controller configured for head version %g",commData.head_version);

        commData.localStemName="/"+ctrlName;
        string remoteHeadName="/"+commData.robotName+"/"+headName;
        string localHeadName=commData.localStemName+"/"+headName;
        string remoteTorsoName="/"+commData.robotName+"/"+torsoName;
        string localTorsoName=commData.localStemName+"/"+torsoName;
        string remoteInertialName="/"+commData.robotName+"/head/inertials";
        string localInertialName=commData.localStemName+"/head/inertials";

        float imuTimeout {0.04};
        // check if we have to retrieve IMU data from a different port
        if (imuGroup.check("source_port_name"))
            remoteInertialName=imuGroup.find("source_port_name").asString();
        if (imuGroup.check("timeout"))
            imuTimeout=imuGroup.find("timeout").asFloat32();

        Property optTorso("(device remote_controlboard)");
        optTorso.put("remote",remoteTorsoName);
        optTorso.put("local",localTorsoName);
        optTorso.put("part",torsoName);

        Property optHead("(device remote_controlboard)");
        optHead.put("remote",remoteHeadName);
        optHead.put("local",localHeadName);
        optHead.put("part",headName);
        // mixed position/velocity control entails
        // to send two packets per control slot
        optHead.put("writeStrict","on");

        if (torsoName!="off")
        {
            drvTorso=(ping_robot_tmo>0.0)?
                     waitPart(optTorso,ping_robot_tmo):
                     new PolyDriver(optTorso);

            if (!drvTorso->isValid())
            {
                yWarning("Torso device driver not available!");
                yWarning("Perhaps only the head is running; trying to continue ...");

                delete drvTorso;
                drvTorso=nullptr;
            }
        }
        else
        {
            yWarning("Torso device is off!");
            drvTorso=nullptr;
        }

        drvHead=(ping_robot_tmo>0.0)?
                waitPart(optHead,ping_robot_tmo):
                new PolyDriver(optHead);

        if (!drvHead->isValid())
        {
            yError("Head device driver not available!");
            dispose();
            return false;
        }
        if (commData.stabilizationOn)
        {
            Property mas_conf{{"device", Value("multipleanalogsensorsclient")},
                              {"remote", Value(remoteInertialName)},
                              {"local",  Value(localInertialName)},
                              {"timeout",Value(imuTimeout)}};

            if (!(mas_client.open(mas_conf)))
            {
                yError("Unable to open the MAS client");
                dispose();
                return false;
            }

            if (!(mas_client.view(iGyro)) ||
                !(mas_client.view(iAccel))) {

                yError("View failed of the MAS interfaces");
                dispose();
                return false;
            }

            commData.iGyro  = iGyro;
            commData.iAccel = iAccel;
        }
        else {
            yWarning("IMU data will be not received/used");
        }

        if (commData.debugInfoEnabled)
            yDebug("Commands to robot will be also streamed out on debug port");

        // create and start threads
        // creation order does matter (for the minimum allowed vergence computation) !!
        ctrl=new Controller(drvTorso,drvHead,&commData,neckTime,eyesTime,min_abs_vel,10);
        loc=new Localizer(&commData,10);
        eyesRefGen=new EyePinvRefGen(drvTorso,drvHead,&commData,ctrl,counterRotGain,20);
        slv=new Solver(drvTorso,drvHead,&commData,eyesRefGen,loc,ctrl,20);

        commData.port_xd=new xdPort(slv);
        commData.port_xd->open(commData.localStemName+"/xd:i");

        // this switch-on order does matter !!
        eyesRefGen->start();
        slv->start();
        ctrl->start();
        loc->start();

        rpcPort.open(commData.localStemName+"/rpc");
        attach(rpcPort);

        contextIdCnt=0;

        // reserve id==0 for start-up context
        int id0;
        storeContext(&id0);

        return true;
    }

    /************************************************************************/
    bool respond(const Bottle &command, Bottle &reply) override
    {
        int ack=Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        if (command.size()>0)
        {
            switch (command.get(0).asVocab())
            {
                //-----------------
                case createVocab('g','e','t'):
                {
                    if (command.size()>1)
                    {
                        int type=command.get(1).asVocab();

                        if (type==createVocab('T','n','e','c'))
                        {
                            reply.addVocab(ack);
                            reply.addDouble(ctrl->getTneck());
                            return true;
                        }
                        else if (type==createVocab('T','e','y','e'))
                        {
                            reply.addVocab(ack);
                            reply.addDouble(ctrl->getTeyes());
                            return true;
                        }
                        else if (type==createVocab('v','o','r'))
                        {
                            Vector gain=eyesRefGen->getCounterRotGain();
                            reply.addVocab(ack);
                            reply.addDouble(gain[0]);
                            return true;
                        }
                        else if (type==createVocab('o','c','r'))
                        {
                            Vector gain=eyesRefGen->getCounterRotGain();
                            reply.addVocab(ack);
                            reply.addDouble(gain[1]);
                            return true;
                        }
                        else if (type==createVocab('s','a','c','c'))
                        {
                            reply.addVocab(ack);
                            reply.addInt((int)commData.saccadesOn);
                            return true;
                        }
                        else if (type==createVocab('s','i','n','h'))
                        {
                            reply.addVocab(ack);
                            reply.addDouble(commData.saccadesInhibitionPeriod);
                            return true;
                        }
                        else if (type==createVocab('s','a','c','t'))
                        {
                            reply.addVocab(ack);
                            reply.addDouble(commData.saccadesActivationAngle);
                            return true;
                        }
                        else if (type==createVocab('t','r','a','c'))
                        {
                            reply.addVocab(ack);
                            reply.addInt((int)ctrl->getTrackingMode());
                            return true;
                        }
                        else if (type==createVocab('s','t','a','b'))
                        {
                            reply.addVocab(ack);
                            reply.addInt((int)ctrl->getGazeStabilization());
                            return true;
                        }
                        else if (type==createVocab('d','o','n','e'))
                        {
                            reply.addVocab(ack);
                            reply.addInt((int)ctrl->isMotionDone());
                            return true;
                        }
                        else if (type==createVocab('s','d','o','n'))
                        {
                            reply.addVocab(ack);
                            reply.addInt((int)!commData.saccadeUnderway);
                            return true;
                        }
                        else if (type==createVocab('p','i','t','c'))
                        {
                            double min_deg,max_deg;
                            slv->getCurNeckPitchRange(min_deg,max_deg);

                            reply.addVocab(ack);
                            reply.addDouble(min_deg);
                            reply.addDouble(max_deg);
                            return true;
                        }
                        else if (type==createVocab('r','o','l','l'))
                        {
                            double min_deg,max_deg;
                            slv->getCurNeckRollRange(min_deg,max_deg);

                            reply.addVocab(ack);
                            reply.addDouble(min_deg);
                            reply.addDouble(max_deg);
                            return true;
                        }
                        else if (type==createVocab('y','a','w'))
                        {
                            double min_deg,max_deg;
                            slv->getCurNeckYawRange(min_deg,max_deg);

                            reply.addVocab(ack);
                            reply.addDouble(min_deg);
                            reply.addDouble(max_deg);
                            return true;
                        }
                        else if (type==createVocab('e','y','e','s'))
                        {
                            reply.addVocab(ack);
                            reply.addDouble(commData.eyesBoundVer);
                            return true;
                        }
                        else if (type==createVocab('n','t','o','l'))
                        {
                            double angle=slv->getNeckAngleUserTolerance();

                            reply.addVocab(ack);
                            reply.addDouble(angle);
                            return true;
                        }
                        else if (type==createVocab('d','e','s'))
                        {
                            Vector des;
                            if (ctrl->getDesired(des))
                            {
                                reply.addVocab(ack);
                                reply.addList().read(des);
                                return true;
                            }
                        }
                        else if (type==createVocab('v','e','l'))
                        {
                            Vector vel;
                            if (ctrl->getVelocity(vel))
                            {
                                reply.addVocab(ack);
                                reply.addList().read(vel);
                                return true;
                            }
                        }
                        else if ((type==createVocab('p','o','s','e')) && (command.size()>2))
                        {
                            string poseSel=command.get(2).asString();
                            Vector x;
                            Stamp  stamp;

                            if (ctrl->getPose(poseSel,x,stamp))
                            {
                                reply.addVocab(ack);
                                reply.addList().read(x);

                                Bottle &bStamp=reply.addList();
                                bStamp.addInt(stamp.getCount());
                                bStamp.addDouble(stamp.getTime());

                                return true;
                            }
                        }
                        else if ((type==createVocab('2','D')) && (command.size()>2))
                        {
                            if (Bottle *bOpt=command.get(2).asList())
                            {
                                if (bOpt->size()>3)
                                {
                                    Vector x(3);
                                    string eye=bOpt->get(0).asString();
                                    x[0]=bOpt->get(1).asDouble();
                                    x[1]=bOpt->get(2).asDouble();
                                    x[2]=bOpt->get(3).asDouble();

                                    Vector px;
                                    if (loc->projectPoint(eye,x,px))
                                    {
                                        reply.addVocab(ack);
                                        reply.addList().read(px);
                                        return true;
                                    }
                                }
                            }
                        }
                        else if ((type==createVocab('3','D')) && (command.size()>3))
                        {
                            int subType=command.get(2).asVocab();
                            if (subType==createVocab('m','o','n','o'))
                            {
                                if (Bottle *bOpt=command.get(3).asList())
                                {
                                    if (bOpt->size()>3)
                                    {
                                        string eye=bOpt->get(0).asString();
                                        double u=bOpt->get(1).asDouble();
                                        double v=bOpt->get(2).asDouble();
                                        double z=bOpt->get(3).asDouble();

                                        Vector x;
                                        if (loc->projectPoint(eye,u,v,z,x))
                                        {
                                            reply.addVocab(ack);
                                            reply.addList().read(x);
                                            return true;
                                        }
                                    }
                                }
                            }
                            else if (subType==createVocab('s','t','e','r'))
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
                                            reply.addList().read(x);
                                            return true;
                                        }
                                    }
                                }
                            }
                            else if (subType==createVocab('p','r','o','j'))
                            {
                                if (Bottle *bOpt=command.get(3).asList())
                                {
                                    if (bOpt->size()>6)
                                    {
                                        Vector plane(4);
                                        string eye=bOpt->get(0).asString();
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
                                            reply.addList().read(x);
                                            return true;
                                        }
                                    }
                                }
                            }
                            else if (subType==createVocab('a','n','g'))
                            {
                                if (Bottle *bOpt=command.get(3).asList())
                                {
                                    if (bOpt->size()>3)
                                    {
                                        Vector ang(3);
                                        string type=bOpt->get(0).asString();
                                        ang[0]=CTRL_DEG2RAD*bOpt->get(1).asDouble();
                                        ang[1]=CTRL_DEG2RAD*bOpt->get(2).asDouble();
                                        ang[2]=CTRL_DEG2RAD*bOpt->get(3).asDouble();

                                        Vector x=loc->get3DPoint(type,ang);
                                        reply.addVocab(ack);
                                        reply.addList().read(x);
                                        return true;
                                    }
                                }
                            }
                        }
                        else if ((type==createVocab('a','n','g')) && (command.size()>2))
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
                                    reply.addList().read(ang);
                                    return true;
                                }
                            }
                        }
                        else if (type==createVocab('p','i','d'))
                        {
                            Bottle options;
                            loc->getPidOptions(options);

                            reply.addVocab(ack);
                            reply.addList()=options;
                            return true;
                        }
                        else if (type==createVocab('i','n','f','o'))
                        {
                            Bottle info;
                            if (getInfo(info))
                            {
                                reply.addVocab(ack);
                                reply.addList()=info;
                                return true;
                            }
                        }
                        else if (type==createVocab('t','w','e','a'))
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
                case createVocab('s','e','t'):
                {
                    if (command.size()>2)
                    {
                        int type=command.get(1).asVocab();
                        if (type==createVocab('T','n','e','c'))
                        {
                            double execTime=command.get(2).asDouble();
                            ctrl->setTneck(execTime);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (type==createVocab('T','e','y','e'))
                        {
                            double execTime=command.get(2).asDouble();
                            ctrl->setTeyes(execTime);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (type==createVocab('v','o','r'))
                        {
                            Vector gain=eyesRefGen->getCounterRotGain();
                            gain[0]=command.get(2).asDouble();
                            eyesRefGen->setCounterRotGain(gain);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (type==createVocab('o','c','r'))
                        {
                            Vector gain=eyesRefGen->getCounterRotGain();
                            gain[1]=command.get(2).asDouble();
                            eyesRefGen->setCounterRotGain(gain);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (type==createVocab('s','a','c','c'))
                        {
                            commData.saccadesOn=(command.get(2).asInt()>0);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (type==createVocab('s','i','n','h'))
                        {
                            double period=command.get(2).asDouble();
                            commData.saccadesInhibitionPeriod=period;
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (type==createVocab('s','a','c','t'))
                        {
                            double angle=command.get(2).asDouble();
                            commData.saccadesActivationAngle=angle;
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (type==createVocab('n','t','o','l'))
                        {
                            double angle=command.get(2).asDouble();
                            slv->setNeckAngleUserTolerance(angle);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (type==createVocab('t','r','a','c'))
                        {
                            bool mode=(command.get(2).asInt()>0);
                            ctrl->setTrackingMode(mode);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (type==createVocab('s','t','a','b'))
                        {
                            bool mode=(command.get(2).asInt()>0);
                            reply.addVocab(ctrl->setGazeStabilization(mode)?ack:nack);
                            return true;
                        }
                        else if (type==createVocab('p','i','d'))
                        {
                            if (Bottle *bOpt=command.get(2).asList())
                            {
                                loc->setPidOptions(*bOpt);
                                reply.addVocab(ack);
                                return true;
                            }
                        }
                        else if (type==createVocab('t','w','e','a'))
                        {
                            if (Bottle *bOpt=command.get(2).asList())
                            {
                                reply.addVocab(tweakSet(*bOpt)?ack:nack);
                                return true;
                            }
                        }
                    }

                    break;
                }

                //-----------------
                case createVocab('l','o','o','k'):
                {
                    if (command.size()>2)
                    {
                        int type=command.get(1).asVocab();
                        if (type==createVocab('3','D'))
                        {
                            if (Bottle *bOpt=command.get(2).asList())
                            {
                                if (bOpt->size()>2)
                                {
                                    Vector x(3);
                                    x[0]=bOpt->get(0).asDouble();
                                    x[1]=bOpt->get(1).asDouble();
                                    x[2]=bOpt->get(2).asDouble();

                                    if (ctrl->look(x))
                                    {
                                        reply.addVocab(ack);
                                        return true;
                                    }
                                }
                            }
                        }
                        else if (type==createVocab('m','o','n','o'))
                        {
                            if (Bottle *bOpt=command.get(2).asList())
                            {
                                if (bOpt->size()>3)
                                {
                                    string eye=bOpt->get(0).asString();
                                    double u=bOpt->get(1).asDouble();
                                    double v=bOpt->get(2).asDouble();
                                    double z;

                                    bool ok=false;
                                    if (bOpt->get(3).isDouble())
                                    {
                                        z=bOpt->get(3).asDouble();
                                        ok=true;
                                    }
                                    else if ((bOpt->get(3).asString()=="ver") && (bOpt->size()>4))
                                    {
                                        double ver=bOpt->get(4).asDouble();
                                        z=loc->getDistFromVergence(ver);
                                        ok=true;
                                    }

                                    if (ok)
                                    {
                                        Vector x;
                                        if (loc->projectPoint(eye,u,v,z,x))
                                        {
                                            if (ctrl->look(x))
                                            {
                                                reply.addVocab(ack);
                                                return true;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        else if (type==createVocab('s','t','e','r'))
                        {
                            if (Bottle *bOpt=command.get(2).asList())
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
                                        if (ctrl->look(x))
                                        {
                                            reply.addVocab(ack);
                                            return true;
                                        }
                                    }
                                }
                            }
                        }
                        else if (type==createVocab('a','n','g'))
                        {
                            if (Bottle *bOpt=command.get(2).asList())
                            {
                                if (bOpt->size()>3)
                                {
                                    Vector ang(3);
                                    string type=bOpt->get(0).asString();
                                    ang[0]=CTRL_DEG2RAD*bOpt->get(1).asDouble();
                                    ang[1]=CTRL_DEG2RAD*bOpt->get(2).asDouble();
                                    ang[2]=CTRL_DEG2RAD*bOpt->get(3).asDouble();

                                    Vector x=loc->get3DPoint(type,ang);
                                    if (ctrl->look(x))
                                    {
                                        reply.addVocab(ack);
                                        return true;
                                    }
                                }
                            }
                        }
                    }

                    break;
                }

                //-----------------
                case createVocab('s','t','o','p'):
                {
                    ctrl->stopControl();
                    reply.addVocab(ack);
                    return true;
                }

                //-----------------
                case createVocab('s','t','o','r'):
                {
                    int id;
                    storeContext(&id);
                    reply.addVocab(ack);
                    reply.addInt(id);
                    return true;
                }

                //-----------------
                case createVocab('r','e','s','t'):
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
                case createVocab('d','e','l'):
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
                case createVocab('b','i','n','d'):
                {
                    if (command.size()>2)
                    {
                        int joint=command.get(1).asVocab();
                        if (joint==createVocab('p','i','t','c'))
                        {
                            double min=command.get(2).asDouble();
                            double max=command.get(3).asDouble();
                            slv->bindNeckPitch(min,max);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (joint==createVocab('r','o','l','l'))
                        {
                            double min=command.get(2).asDouble();
                            double max=command.get(3).asDouble();
                            slv->bindNeckRoll(min,max);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (joint==createVocab('y','a','w'))
                        {
                            double min=command.get(2).asDouble();
                            double max=command.get(3).asDouble();
                            slv->bindNeckYaw(min,max);
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (joint==createVocab('e','y','e','s'))
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
                case createVocab('c','l','e','a'):
                {
                    if (command.size()>1)
                    {
                        int joint=command.get(1).asVocab();
                        if (joint==createVocab('p','i','t','c'))
                        {
                            slv->clearNeckPitch();
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (joint==createVocab('r','o','l','l'))
                        {
                            slv->clearNeckRoll();
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (joint==createVocab('y','a','w'))
                        {
                            slv->clearNeckYaw();
                            reply.addVocab(ack);
                            return true;
                        }
                        else if (joint==createVocab('e','y','e','s'))
                        {
                            eyesRefGen->clearEyes();
                            reply.addVocab(ack);
                            return true;
                        }
                    }

                    break;
                }

                //-----------------
                case createVocab('r','e','g','i'):
                {
                    if (command.size()>1)
                    {
                        int type=command.get(1).asVocab();
                        if (type==createVocab('o','n','g','o'))
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
                case createVocab('u','n','r','e'):
                {
                    if (command.size()>1)
                    {
                        int type=command.get(1).asVocab();
                        if (type==createVocab('o','n','g','o'))
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
                case createVocab('l','i','s','t'):
                {
                    if (command.size()>1)
                    {
                        int type=command.get(1).asVocab();
                        if (type==createVocab('o','n','g','o'))
                        {
                            reply.addVocab(ack);
                            reply.addList()=ctrl->listMotionOngoingEvents();
                            return true;
                        }
                    }

                    break;
                }

                //-----------------
                case createVocab('s','u','s','p'):
                {
                    ctrl->suspend();
                    eyesRefGen->suspend();
                    slv->suspend();
                    reply.addVocab(ack);
                    return true;
                }

                //-----------------
                case createVocab('r','u','n'):
                {
                    slv->resume();
                    eyesRefGen->resume();
                    ctrl->resume();
                    reply.addVocab(ack);
                    return true;
                }

                //-----------------
                case createVocab('s','t','a','t'):
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
    void dispose()
    {
        if (loc!=nullptr)
            loc->stop();

        if (eyesRefGen!=nullptr)
            eyesRefGen->stop();

        if (slv!=nullptr)
            slv->stop();

        if (ctrl!=nullptr)
            ctrl->stop();

        if (drvTorso!=nullptr)
            drvTorso->close();

        if (drvHead!=nullptr)
            drvHead->close();

        if (mas_client.isValid())
            mas_client.close();

        if (commData.port_xd!=nullptr)
            if (!commData.port_xd->isClosed())
                commData.port_xd->close();
        if (rpcPort.asPort().isOpen())
            rpcPort.close();

        // this switch-off order does matter !!
        delete commData.port_xd;
        delete loc;
        delete eyesRefGen;
        delete slv;
        delete ctrl;
        delete drvTorso;
        delete drvHead;

        contextMap.clear();
    }

    /************************************************************************/
    bool interruptModule() override
    {
        interrupting=true;

        if (commData.port_xd!=nullptr)
            commData.port_xd->interrupt();
        rpcPort.interrupt();

        return true;
    }

    /************************************************************************/
    bool close() override
    {
        dispose();
        return true;
    }

    /************************************************************************/
    double getPeriod() override
    {
        return 1.0;
    }

    /************************************************************************/
    bool updateModule() override
    {
        if (doSaveTweakFile)
        {
            lock_guard<mutex> lg(mutexTweak);
            saveTweakFile();
            doSaveTweakFile=false;
        }

        return true;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setDefaultContext("iKinGazeCtrl");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }

    GazeModule mod;
    return mod.runModule(rf);
}
