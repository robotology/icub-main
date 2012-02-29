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

#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <string>

#include <yarp/os/Port.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/ctrl/pids.h>
#include <iCub/utils.h>

#define GAZECTRL_MOTIONDONE_QTHRES      2e-1    // [deg]
#define GAZECTRL_MOTIONSTART_XTHRES     1e-3    // [m]

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


// The thread launched by the application which is
// in charge of computing the velocities profile.
class Controller : public RateThread
{
protected:
    iCubHeadCenter   *neck;
    iCubEye          *eyeL,      *eyeR;
    iKinChain        *chainNeck, *chainEyeL, *chainEyeR;
    PolyDriver       *drvTorso,  *drvHead;
    IEncoders        *encTorso,  *encHead;
    IPositionControl *posHead;
    IVelocityControl *velHead;
    exchangeData     *commData;
    xdPort           *port_xd;

    minJerkVelCtrl   *mjCtrlNeck;
    minJerkVelCtrl   *mjCtrlEyes;
    Integrator       *Int;

    Port port_x;
    Port port_q;

    Semaphore mutexChain;
    Semaphore mutexCtrl;
    string robotName;
    string localName;
    string camerasFile;
    unsigned int period;
    bool unplugCtrlEyes;
    bool Robotable;
    bool headV2;
    int nJointsTorso;
    int nJointsHead;
    double printAccTime;
    double neckTime;
    double eyesTime;
    double eyeTiltMin;
    double eyeTiltMax;
    double minAbsVel;
    double Ts;

    Vector qddeg,qdeg,vdeg,xd,fp;
    Vector v,vNeck,vEyes,vdegOld;
    Vector qd,qdNeck,qdEyes;
    Vector fbTorso,fbHead,fbNeck,fbEyes;

    void findMinimumAllowedVergence(const Matrix &lim);

public:
    Controller(PolyDriver *_drvTorso, PolyDriver *_drvHead, exchangeData *_commData,
               const string &_robotName, const string &_localName, const string &_camerasFile,
               const double _neckTime, const double _eyesTime, const double _eyeTiltMin,
               const double _eyeTiltMax, const double _minAbsVel, const bool _headV2,
               const unsigned int _period);

    void   resetCtrlEyes();
    void   doSaccade(const Vector &ang, const Vector &vel);
    void   stopLimbsVel();
    void   set_xdport(xdPort *_port_xd) { port_xd=_port_xd; }
    void   printIter(Vector &xd, Vector &fp, Vector &qd, Vector &q, Vector &v, double printTime);
    bool   threadInit();
    void   afterStart(bool s);
    void   run();
    void   threadRelease();
    void   suspend();
    void   resume();
    double getTneck() const;
    double getTeyes() const;
    void   setTneck(const double execTime);
    void   setTeyes(const double execTime);
    bool   isMotionDone() const;
    void   setTrackingMode(const bool f);
    bool   getTrackingMode() const;
    bool   getDesired(Vector &des) const;
    bool   getVelocity(Vector &vel) const;
    bool   getPose(const string &poseSel, Vector &x);
};


#endif


