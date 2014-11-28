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

#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/pids.h>
#include <iCub/gazeNlp.h>
#include <iCub/utils.h>
#include <iCub/localizer.h>
#include <iCub/controller.h>

#define EYEPINVREFGEN_GAIN                  12.5    // [-]
#define SACCADES_VEL                        1000.0  // [deg/s]
#define SACCADES_INHIBITION_PERIOD          0.2     // [s]
#define SACCADES_ACTIVATION_ANGLE           10.0    // [deg]
#define GYRO_BIAS_STABILITY                 5.0     // [deg/s]
#define NECKSOLVER_ACTIVATIONDELAY          0.25    // [s]
#define NECKSOLVER_ACTIVATIONANGLE_JOINTS   0.1     // [deg]
#define NECKSOLVER_ACTIVATIONANGLE          2.5     // [deg]
#define NECKSOLVER_RESTORINGANGLE           5.0     // [deg]

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


// The thread launched by the application which computes
// the eyes target position relying on the pseudoinverse
// method.
class EyePinvRefGen : public GazeComponent, public RateThread
{
protected:
    iCubHeadCenter       *neck;
    iCubInertialSensor   *imu;
    iKinChain            *chainNeck, *chainEyeL, *chainEyeR;    
    PolyDriver           *drvTorso, *drvHead;
    exchangeData         *commData;
    Controller           *ctrl;
    xdPort               *port_xd;
    Integrator           *I;

    double                orig_eye_tilt_min;
    double                orig_eye_tilt_max;
    double                orig_eye_pan_min;
    double                orig_eye_pan_max;

    BufferedPort<Vector> port_inertial;
    Mutex mutex;

    unsigned int period;
    bool saccadesOn;
    bool saccadeUnderWayOld;
    bool genOn;
    int nJointsTorso;
    int nJointsHead;
    double eyesBoundVer;
    int    saccadesRxTargets;
    double saccadesClock;
    double saccadesInhibitionPeriod;
    double saccadesActivationAngle;
    double eyesHalfBaseline;
    double Ts;
    
    Matrix orig_lim,lim;
    Vector fbTorso;
    Vector fbHead;
    Vector qd,fp;
    Matrix eyesJ;
    Vector gyro;
    Vector counterRotGain;

    Vector getEyesCounterVelocity(const Matrix &eyesJ, const Vector &fp);

public:
    EyePinvRefGen(PolyDriver *_drvTorso, PolyDriver *_drvHead, exchangeData *_commData,
                  Controller *_ctrl, const bool _saccadesOn, const Vector &_counterRotGain,
                  const unsigned int _period);

    void   set_xdport(xdPort *_port_xd)                     { port_xd=_port_xd;                }
    void   enable()                                         { genOn=true;                      }
    void   disable()                                        { genOn=false;                     }
    Vector getCounterRotGain() const                        { return counterRotGain;           }
    void   setSaccades(const bool sw)                       { saccadesOn=sw;                   }
    bool   isSaccadesOn() const                             { return saccadesOn;               }
    void   setSaccadesInhibitionPeriod(const double period) { saccadesInhibitionPeriod=period; }    
    void   setSaccadesActivationAngle(const double angle)   { saccadesActivationAngle=angle;   }
    double getSaccadesInhibitionPeriod() const              { return saccadesInhibitionPeriod; }
    double getSaccadesActivationAngle() const               { return saccadesActivationAngle;  }
    double getEyesBoundVer() const                          { return eyesBoundVer;             }
    void   setCounterRotGain(const Vector &gain);
    void   minAllowedVergenceChanged();
    bool   getGyro(Vector &data);
    bool   bindEyes(const double ver);
    bool   clearEyes();
    void   manageBindEyes(const double ver);
    bool   threadInit();
    void   afterStart(bool s);
    void   run();
    void   threadRelease();
    void   suspend();
    void   resume();
};


// The thread launched by the application which is
// in charge of inverting the head kinematic relying
// on IPOPT computation.
class Solver : public GazeComponent, public RateThread
{
protected:    
    iCubHeadCenter     *neck;
    iCubInertialSensor *imu;
    iKinChain          *chainNeck, *chainEyeL, *chainEyeR;    
    GazeIpOptMin       *invNeck;
    PolyDriver         *drvTorso, *drvHead;
    exchangeData       *commData;
    EyePinvRefGen      *eyesRefGen;
    Localizer          *loc;
    Controller         *ctrl;
    xdPort             *port_xd;
    Mutex               mutex;

    unsigned int period;
    bool solveRequest;
    int nJointsTorso;
    int nJointsHead;
    double neckAngleUserTolerance;
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
    Vector computeTargetUserTolerance(const Vector &xd);

public:
    Solver(PolyDriver *_drvTorso, PolyDriver *_drvHead, exchangeData *_commData,
           EyePinvRefGen *_eyesRefGen, Localizer *_loc, Controller *_ctrl,
           const unsigned int _period);

    // Returns a measure of neck angle required to reach the target
    double neckTargetRotAngle(const Vector &xd);
    void   bindNeckPitch(const double min_deg, const double max_deg);
    void   bindNeckRoll(const double min_deg, const double max_deg);
    void   bindNeckYaw(const double min_deg, const double max_deg);
    void   getCurNeckPitchRange(double &min_deg, double &max_deg);
    void   getCurNeckRollRange(double &min_deg, double &max_deg);
    void   getCurNeckYawRange(double &min_deg, double &max_deg);
    void   clearNeckPitch();
    void   clearNeckRoll();
    void   clearNeckYaw();
    double getNeckAngleUserTolerance();
    void   setNeckAngleUserTolerance(const double angle);    
    bool   threadInit();
    void   afterStart(bool s);
    void   run();
    void   threadRelease();
    void   suspend();
    void   resume();
};


#endif


