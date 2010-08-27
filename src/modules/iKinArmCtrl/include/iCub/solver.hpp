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

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iostream>
#include <iomanip>
#include <string>

#include <iCub/iKin/iKinInv.h>
#include <iCub/iKin/iKinIpOpt.h>

#include <iCub/utils.hpp>

#define SHOULDER_MAXABDUCTION   (100.0*CTRL_DEG2RAD)

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


// This inherited class handles the constraints to be 
// enforced on iCub shoulders joints to protect cables.
class iCubShoulderConstr : public iKinLinIneqConstr
{
public:

    iCubShoulderConstr(unsigned int dofs, double lower_bound_inf,
                       double upper_bound_inf);
};


// This class handles solver callbacks.
// The use of callback can speed up the reaching task,
// but also may cause the controller to steer the arm
// in a deadlock while IPOPT is still running.
class slvCallback : public iKinIterateCallback
{
private:
    slvCallback(const slvCallback&);
    slvCallback &operator=(const slvCallback&);

protected:
    exchangeData *commData;

public:
    slvCallback(exchangeData *_commData) : commData(_commData) { }

    virtual void exec(Vector xd, Vector q) { commData->setDesired(xd,q); }
};


// The thread launched by the application which is
// in charge of inverting the arm kinematic relying
// on IPOPT computation.
class Solver : public RateThread
{
protected:
    iCubArm              *arm;
    iKinChain            *chain;
    iKinIpOptMin         *slv;
    iCubShoulderConstr   *shouConstr;
    PolyDriver           *drvTorso, *drvArm;
    IControlLimits       *limTorso, *limArm;
    IEncoders            *encTorso, *encArm;
    exchangeData         *commData;
    slvCallback          *slvCallbackObj;

    xdPort               *port_xd;
    BufferedPort<Vector> *port_qd;    

    string localName;
    unsigned int period;
    unsigned int ctrlPose;
    unsigned int ctrlTorso;
    bool Robotable;
    int  nJointsTorso;
    int  nJointsArm;
    int  nJoints;

    Vector q0;
    Vector qTorso_old;
    Vector xd_old;
    Vector fb;

public:
    Solver(PolyDriver *_drvTorso, PolyDriver *_drvArm, exchangeData *_commData,
           const string &_localName, const string &partName, unsigned int _ctrlTorso,
           unsigned int _ctrlPose, unsigned int _period);

    void setStart();

    virtual bool threadInit();
    virtual void afterStart(bool s);
    virtual void run();
    virtual void threadRelease();
    virtual void suspend();
    virtual void resume();
};


#endif


