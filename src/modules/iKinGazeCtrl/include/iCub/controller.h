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

#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <string>
#include <vector>
#include <set>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/ctrl/pids.h>
#include <iCub/utils.h>

#define GAZECTRL_SWOFFCOND_DISABLESLOT      10      // [-]
#define GAZECTRL_MOTIONDONE_NECK_QTHRES     0.500   // [deg]
#define GAZECTRL_MOTIONDONE_EYES_QTHRES     0.100   // [deg]
#define GAZECTRL_CRITICVER_STABILIZATION    4.0     // [deg]

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


// The thread launched by the application which is
// in charge of computing the control commands.
class Controller : public GazeComponent, public RateThread
{
protected:
    iCubHeadCenter     *neck;
    iCubInertialSensor *imu;
    iKinChain          *chainNeck, *chainEyeL, *chainEyeR;
    PolyDriver         *drvTorso,  *drvHead;
    IControlMode2      *modHead;
    IPositionControl2  *posHead;
    IVelocityControl2  *velHead;
    IPositionDirect    *posNeck;    
    ExchangeData       *commData;

    minJerkVelCtrl     *mjCtrlNeck;
    minJerkVelCtrl     *mjCtrlEyes;
    Integrator         *IntState;
    Integrator         *IntPlan;
    Integrator         *IntStabilizer;

    BufferedPort<Vector> port_x;
    BufferedPort<Vector> port_q;
    BufferedPort<Bottle> port_event;
    BufferedPort<Bottle> port_debug;
    Stamp txInfo_x;
    Stamp txInfo_q;
    Stamp txInfo_pose;
    Stamp txInfo_event;
    Stamp txInfo_debug;

    Mutex mutexRun;
    Mutex mutexChain;
    Mutex mutexCtrl;
    Mutex mutexData;
    Mutex mutexLook;
    Event eventLook;
    unsigned int period;
    bool unplugCtrlEyes;
    bool ctrlInhibited;
    bool motionDone;
    bool reliableGyro;
    bool stabilizeGaze;
    int nJointsTorso;
    int nJointsHead;
    double ctrlActiveRisingEdgeTime;
    double saccadeStartTime;    
    double printAccTime;
    double neckTime;
    double eyesTime;
    double pathPerc;
    double min_abs_vel;
    double startupMinVer;
    double q_stamp;
    double Ts;

    Matrix lim;
    Vector qddeg,qdeg,vdeg;
    Vector v,vNeck,vEyes;
    Vector q0,qd,qdNeck,qdEyes;
    Vector fbTorso,fbHead,fbNeck,fbEyes;
    vector<int> neckJoints,eyesJoints;
    vector<int> jointsToSet;

    multiset<double> motionOngoingEvents;
    multiset<double> motionOngoingEventsCurrent;

    Vector computedxFP(const Matrix &H, const Vector &v, const Vector &w, const Vector &x_FP);
    Vector computeNeckVelFromdxFP(const Vector &fp, const Vector &dfp);
    Vector computeEyesVelFromdxFP(const Vector &dfp);
    bool   areJointsHealthyAndSet();
    void   setJointsCtrlMode();
    void   stopLimb(const bool execStopPosition=true);
    void   notifyEvent(const string &event, const double checkPoint=-1.0);
    void   motionOngoingEventsHandling();
    void   motionOngoingEventsFlush();
    void   stopControlHelper();

public:
    Controller(PolyDriver *_drvTorso, PolyDriver *_drvHead, ExchangeData *_commData,
               const double _neckTime, const double _eyesTime,
               const double _min_abs_vel, const unsigned int _period);
    virtual ~Controller();

    void   findMinimumAllowedVergence();
    void   minAllowedVergenceChanged();
    void   resetCtrlEyes();
    void   doSaccade(const Vector &ang, const Vector &vel);
    bool   look(const Vector &x);
    void   stopControl();    
    void   printIter(Vector &xd, Vector &fp, Vector &qd, Vector &q, Vector &v, double printTime);
    bool   threadInit();
    void   threadRelease();
    void   afterStart(bool s);
    void   run();
    void   suspend();
    void   resume();
    double getTneck() const;
    double getTeyes() const;
    void   setTneck(const double execTime);
    void   setTeyes(const double execTime);
    bool   isMotionDone();
    void   setTrackingMode(const bool f);
    bool   getTrackingMode() const;
    bool   setGazeStabilization(const bool f);
    bool   getGazeStabilization() const;
    bool   getDesired(Vector &des);
    bool   getVelocity(Vector &vel);
    bool   getPose(const string &poseSel, Vector &x, Stamp &stamp);
    bool   registerMotionOngoingEvent(const double checkPoint);
    bool   unregisterMotionOngoingEvent(const double checkPoint);
    Bottle listMotionOngoingEvents();
};


#endif


