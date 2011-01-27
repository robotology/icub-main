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

#include <yarp/os/Port.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/ctrl/pids.h>

#include <iCub/utils.h>

#define GAZECTRL_MOTIONDONE_QTHRES      0.1     // [deg]
#define GAZECTRL_MOTIONSTART_XTHRES     1e-3    // [m]

using namespace std;
using namespace yarp;
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
    iCubHeadCenter            *neck;
    iCubEye                   *eyeL,      *eyeR;
    iKinChain                 *chainNeck, *chainEyeL, *chainEyeR;
    PolyDriver                *drvTorso,  *drvHead;
    IEncoders                 *encTorso,  *encHead;
    IVelocityControl          *velHead;
    exchangeData              *commData;
    xdPort                    *port_xd;
    iKinLink *alignLnkLeft1,  *alignLnkLeft2;
    iKinLink *alignLnkRight1, *alignLnkRight2;

    minJerkVelCtrl            *mjCtrlNeck;
    minJerkVelCtrl            *mjCtrlEyes;
    Integrator                *Int;

    Port port_x;
    Port port_q;

    Semaphore mutex;
    string robotName;
    string localName;
    string configFile;
    unsigned int period;
    bool Robotable;
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

public:
    Controller(PolyDriver *_drvTorso, PolyDriver *_drvHead, exchangeData *_commData,
               const string &_robotName, const string &_localName, const string &_configFile,
               const double _neckTime, const double _eyesTime, const double _eyeTiltMin,
               const double _eyeTiltMax, const double _minAbsVel, unsigned int _period);

    void stopLimbsVel();
    void set_xdport(xdPort *_port_xd) { port_xd=_port_xd; }
    void printIter(Vector &xd, Vector &fp, Vector &qd, Vector &q,
                   Vector &v, double printTime);

    virtual bool   threadInit();
    virtual void   afterStart(bool s);
    virtual void   run();
    virtual void   threadRelease();
    virtual void   suspend();
    virtual void   resume();
    virtual double getTneck() const;
    virtual double getTeyes() const;
    virtual void   setTneck(const double execTime);
    virtual void   setTeyes(const double execTime);
    virtual bool   isMotionDone() const;
    virtual void   setTrackingMode(const bool f);
    virtual bool   getTrackingMode() const;
    virtual bool   getDesired(Vector &des) const;
    virtual bool   getVelocity(Vector &vel) const;
    virtual bool   getPose(const string &poseSel, Vector &x);
};


#endif


