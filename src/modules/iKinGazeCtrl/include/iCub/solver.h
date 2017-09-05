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

#ifndef __SOLVER_H__
#define __SOLVER_H__

#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/pids.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/gazeNlp.h>
#include <iCub/utils.h>
#include <iCub/localizer.h>
#include <iCub/controller.h>

#define EYEPINVREFGEN_GAIN                  12.5    // [-]
#define SACCADES_VEL                        1000.0  // [deg/s]
#define SACCADES_INHIBITION_PERIOD          0.2     // [s]
#define SACCADES_ACTIVATION_ANGLE           10.0    // [deg]
#define NECKSOLVER_ACTIVATIONDELAY          0.25    // [s]
#define NECKSOLVER_ACTIVATIONANGLE_JOINTS   0.5     // [deg/s]
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
    ExchangeData         *commData;
    Controller           *ctrl;
    Integrator           *I;
    Mutex                 mutex;

    double                orig_eye_tilt_min;
    double                orig_eye_tilt_max;
    double                orig_eye_pan_min;
    double                orig_eye_pan_max;
    
    unsigned int period;
    bool   saccadeUnderWayOld;
    bool   genOn;
    int    nJointsTorso;
    int    nJointsHead;
    int    saccadesRxTargets;
    double saccadesClock;
    double eyesHalfBaseline;
    double Ts;
    
    Matrix orig_lim,lim;
    Vector fbTorso;
    Vector fbHead;
    Vector qd,fp;
    Matrix eyesJ;
    Vector counterRotGain;

    Vector getEyesCounterVelocity(const Matrix &eyesJ, const Vector &fp);

public:
    EyePinvRefGen(PolyDriver *_drvTorso, PolyDriver *_drvHead, ExchangeData *_commData,
                  Controller *_ctrl, const Vector &_counterRotGain, const unsigned int _period);
    virtual ~EyePinvRefGen();

    void   enable()  { genOn=true;  }
    void   disable() { genOn=false; }
    Vector getCounterRotGain();
    void   setCounterRotGain(const Vector &gain);
    void   minAllowedVergenceChanged();
    bool   bindEyes(const double ver);
    bool   clearEyes();
    void   manageBindEyes(const double ver);
    bool   threadInit();
    void   threadRelease();
    void   afterStart(bool s);
    void   run();
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
    ExchangeData       *commData;
    EyePinvRefGen      *eyesRefGen;
    Localizer          *loc;
    Controller         *ctrl;    
    Mutex               mutex;

    unsigned int period;
    int nJointsTorso;
    int nJointsHead;
    double neckAngleUserTolerance;
    double Ts;

    Vector fbTorso;
    Vector fbHead;
    Vector neckPos;
    Vector gazePos;

    AWLinEstimator *torsoVel;

    double neckPitchMin;
    double neckPitchMax;
    double neckRollMin;
    double neckRollMax;
    double neckYawMin;
    double neckYawMax;

    void   updateAngles();
    Vector computeTargetUserTolerance(const Vector &xd);

public:
    Solver(PolyDriver *_drvTorso, PolyDriver *_drvHead, ExchangeData *_commData,
           EyePinvRefGen *_eyesRefGen, Localizer *_loc, Controller *_ctrl,
           const unsigned int _period);
    virtual ~Solver();

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
    double getNeckAngleUserTolerance() const;
    void   setNeckAngleUserTolerance(const double angle);    
    bool   threadInit();
    void   threadRelease();
    void   afterStart(bool s);
    void   run();
    void   suspend();
    void   resume();
};


#endif


