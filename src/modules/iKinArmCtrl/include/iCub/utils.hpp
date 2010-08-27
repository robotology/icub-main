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

#ifndef __UTILS_H__
#define __UTILS_H__

#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iCub/iKin/iKinInv.h>
#include <iCub/iKin/iKinIpOpt.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


// This inherited class handles the incoming
// target arm pose (xyz + axis/angle).
// 
// Since it accepts a bottle, it is possible to 
// issue the command "yarp read /sender /ctrlName/xd:i"
// and type the target pose manually.
class xdPort : public BufferedPort<Bottle>
{
public:
    xdPort(const Vector &xd0)         { xd=xd0;    }
    Vector &get_xd()                  { return xd; }
    void    set_xd(const Vector &_xd) { xd=_xd;    }

protected:
    Vector xd;

    virtual void onRead(Bottle &b);
};


// This class handles the data exchange
// between Solver and Controller objects.
class exchangeData
{
protected:
    Semaphore mutex1;
    Semaphore mutex2;

    Vector xd;
    Vector qd;
    Vector q;

public:
    exchangeData() {  }

    void   setDesired(const Vector &_xd, const Vector &_qd);
    void   getDesired(Vector &_xd, Vector &_qd);
    void   set_q(const Vector &_q);
    Vector get_q();
};


// Creates iCub arm
iCubArm *createArm(const string &partName, bool ctrlTorso);


// Reads encoders values, handling all control's conditions.
// If torso joints are not controlled, update corresponding chain's angles.
// Returns true if communication with robot is stable, false otherwise.
bool getFeedback(Vector &fb, iKinChain *chain, IEncoders *encTorso,
                 IEncoders *encArm, int nJointsTorso, int nJointsArm,
                 bool ctrlTorso);


#endif


