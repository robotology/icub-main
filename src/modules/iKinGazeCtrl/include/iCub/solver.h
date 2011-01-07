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

#ifndef __SOLVER_H__
#define __SOLVER_H__

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iCub/ctrl/pids.h>

#include <iCub/gazeNlp.h>
#include <iCub/utils.h>
#include <iCub/localizer.h>
#include <iCub/controller.h>

#define EYEPINVREFGEN_GAIN                  12.5
#define NECKSOLVER_ACTIVATIONDELAY          0.25    // [s]
#define NECKSOLVER_ACTIVATIONANGLE_JOINTS   0.1     // [deg]
#define NECKSOLVER_ACTIVATIONANGLE_TRA      2.5     // [deg]
#define NECKSOLVER_ACTIVATIONANGLE_SAG      2.5     // [deg]
#define NECKSOLVER_RESTORINGANGLE_TRA       5.0     // [deg]
#define NECKSOLVER_RESTORINGANGLE_SAG       5.0     // [deg]
#define MINALLOWED_VERGENCE                 0.5     // [deg]

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


// The thread launched by the application which computes
// the eyes target position relying on the pseudoinverse
// method.
class EyePinvRefGen : public RateThread
{
protected:
    iCubHeadCenter            *neck;
    iCubEye                   *eyeL,      *eyeR;
    iKinChain                 *chainNeck, *chainEyeL, *chainEyeR;
    iCubInertialSensor         inertialSensor;
    PolyDriver                *drvTorso,  *drvHead;
    IEncoders                 *encTorso,  *encHead;
    exchangeData              *commData;
    xdPort                    *port_xd;
    Integrator                *I;
    iKinLink *alignLnkLeft1,  *alignLnkLeft2;
    iKinLink *alignLnkRight1, *alignLnkRight2;

    BufferedPort<Vector>      *port_inertial;

    unsigned int period;
    string robotName;
    string localName;
    string configFile;
    bool Robotable;
    bool genOn;
    bool VOR;
    int nJointsTorso;
    int nJointsHead;
    double eyeTiltMin;
    double eyeTiltMax;
    double Ts;
    
    Vector fbTorso;
    Vector fbHead;
    Vector qd,fp;
    Matrix eyesJ;
    Vector gyro;

    Vector getFpVelocityDueToNeckRotation(const Vector &fp);

public:
    EyePinvRefGen(PolyDriver *_drvTorso, PolyDriver *_drvHead,
                  exchangeData *_commData, const string &_robotName,
                  const string &_localName, const string &_configFile,
                  const double _eyeTiltMin, const double _eyeTiltMax,
                  const bool _VOR, unsigned int _period);

    void set_xdport(xdPort *_port_xd) { port_xd=_port_xd; }
    void enable()                     { genOn=true;       }
    void disable()                    { genOn=false;      }
    bool getGyro(Vector &data);

    virtual bool threadInit();
    virtual void afterStart(bool s);
    virtual void run();
    virtual void threadRelease();
    virtual void suspend();
    virtual void resume();
    virtual void stopControl();
};


// The thread launched by the application which is
// in charge of inverting the head kinematic relying
// on IPOPT computation.
class Solver : public RateThread
{
protected:    
    iCubHeadCenter            *neck;    
    iCubEye                   *eyeL,      *eyeR;
    iKinChain                 *chainNeck, *chainEyeL, *chainEyeR;
    iCubInertialSensor         inertialSensor;
    GazeIpOptMin              *invNeck;
    PolyDriver                *drvTorso,  *drvHead;
    IEncoders                 *encTorso,  *encHead;
    exchangeData              *commData;
    EyePinvRefGen             *eyesRefGen;
    Localizer                 *loc;
    Controller                *ctrl;
    xdPort                    *port_xd;
    iKinLink *alignLnkLeft1,  *alignLnkLeft2;
    iKinLink *alignLnkRight1, *alignLnkRight2;    

    string localName;
    string configFile;
    unsigned int period;
    bool Robotable;
    int nJointsTorso;
    int nJointsHead;
    double eyeTiltMin;
    double eyeTiltMax;
    double Ts;

    Vector fbTorso;
    Vector fbHead;
    Vector neckPos;
    Vector gazePos;
    Vector gDefaultDir;
    Vector fbTorsoOld;
    Vector fbHeadOld;

    double neckPitchMin;
    double neckPitchMax;
    double neckRollMin;
    double neckRollMax;
    double neckYawMin;
    double neckYawMax;

    void   updateAngles();
    Vector getGravityDirection(const Vector &gyro);

public:
    Solver(PolyDriver *_drvTorso, PolyDriver *_drvHead, exchangeData *_commData,
           EyePinvRefGen *_eyesRefGen, Localizer *_loc, Controller *_ctrl,
           const string &_localName, const string &_configFile, const double _eyeTiltMin,
           const double _eyeTiltMax, unsigned int _period);

    // Returns a measure of neck angle required to reach the target
    Vector neckTargetRotAngles(const Vector &xd);    

    void bindNeckPitch(const double min_deg, const double max_deg);
    void bindNeckRoll(const double min_deg, const double max_deg);
    void bindNeckYaw(const double min_deg, const double max_deg);
    void getCurNeckPitchRange(double &min_deg, double &max_deg) const;
    void getCurNeckRollRange(double &min_deg, double &max_deg) const;
    void getCurNeckYawRange(double &min_deg, double &max_deg) const;
    void clearNeckPitch();
    void clearNeckRoll();
    void clearNeckYaw();

    virtual bool threadInit();
    virtual void afterStart(bool s);
    virtual void run();
    virtual void threadRelease();
    virtual void suspend();
    virtual void resume();
};


#endif


