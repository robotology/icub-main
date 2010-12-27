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

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iostream>
#include <iomanip>
#include <string>

#include <iCub/iKin/iKinInv.h>

#include <iCub/utils.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace iCub::iKin;


// The thread launched by the application which is
// in charge of computing the velocities profile.
class Controller : public RateThread
{
protected:
    iCubArm              *arm;
    iKinChain            *chain;
    MultiRefMinJerkCtrl  *ctrl;
    PolyDriver           *drvTorso, *drvArm;
    IControlLimits       *limTorso, *limArm;
    IEncoders            *encTorso, *encArm;
    IVelocityControl     *velTorso, *velArm;
    exchangeData         *commData;    

    BufferedPort<Vector> *port_x;
    BufferedPort<Vector> *port_q;
    BufferedPort<Vector> *port_v;

    string localName;
    unsigned int period;
    unsigned int ctrlPose;
    unsigned int ctrlTorso;
    double execTime;
    double Ts;    
    bool Robotable;
    int  nJointsTorso;
    int  nJointsArm;
    int  nJoints;

    Vector fb;
    Vector vOld;

public:
    Controller(PolyDriver *_drvTorso, PolyDriver *_drvArm, exchangeData *_commData,
               const string &_localName, const string &partName, double _execTime,
               unsigned int _ctrlTorso, unsigned int _ctrlPose, unsigned int _period);

    void stopLimbsVel();

    virtual bool threadInit();
    virtual void afterStart(bool s);
    virtual void run();
    virtual void threadRelease();
    virtual void suspend();
    virtual void resume();
};


#endif


