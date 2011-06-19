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

#include <yarp/os/Thread.h>
#include <yarp/os/Event.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <string>

#include <iCub/gazeNlp.h>

#define MINALLOWED_VERGENCE     0.5     // [deg]
#define ALMOST_ZERO             1e-6

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


// This inherited class handles the incoming
// fixation point xyz coordinates.
// 
// Since it accepts a bottle, it is possible to 
// issue the command "yarp read /sender /ctrlName/xd:i"
// and type the target position manually.
//
// Moreover, the possibility to delay the received
// target is handled as well.
class xdPort : public BufferedPort<Bottle>,
               public Thread
{
protected:
    void  *slv;

    Semaphore mutex_0;
    Semaphore mutex_1;
    Event     syncEvent;
    Vector    xd;
    Vector    xdDelayed;
    bool      isNew;
    bool      isNewDelayed;
    bool      closing;

    virtual void onRead(Bottle &b);
    virtual void run();

public:
    xdPort(const Vector &xd0, void *_slv);
    ~xdPort();

    bool   &get_new()        { return isNew;        }
    bool   &get_newDelayed() { return isNewDelayed; }
    void    set_xd(const Vector &_xd);
    Vector  get_xd();
    Vector  get_xdDelayed();
};


// This class handles the data exchange
// between Solvers and Controller objects.
class exchangeData
{
protected:
    Semaphore mutex[8];

    Vector xd,qd;
    Vector x,q,torso;
    Vector v,counterv;
    Matrix S;
    bool   isCtrlActive;
    bool   canCtrlBeDisabled;

public:
    exchangeData();

    void    resize_v(const int sz, const double val);
    void    resize_counterv(const int sz, const double val);

    void    set_xd(const Vector &_xd);
    void    set_qd(const Vector &_qd);
    void    set_qd(const int i, const double val);
    void    set_x(const Vector &_x);
    void    set_q(const Vector &_q);
    void    set_torso(const Vector &_torso);
    void    set_v(const Vector &_v);
    void    set_counterv(const Vector &_counterv);
    void    set_fpFrame(const Matrix &_S);

    Vector  get_xd();
    Vector  get_qd();
    Vector  get_x();
    Vector  get_q();
    Vector  get_torso();
    Vector  get_v();
    Vector  get_counterv();
    Matrix  get_fpFrame();
                                                  
    bool   &get_isCtrlActive()      { return isCtrlActive;      }
    bool   &get_canCtrlBeDisabled() { return canCtrlBeDisabled; }
};


// Allocates Projection Matrix Prj for the camera read from configFile
// type is in {"CAMERA_CALIBRATION_LEFT","CAMERA_CALIBRATION_RIGHT"}
// Returns true if correctly configured
bool getCamPrj(const string &configFile, const string &type, Matrix **Prj);


// Allocates the two aligning links read from configFile
// type is in {"ALIGN_KIN_LEFT","ALIGN_KIN_RIGHT"}
// Returns true if correctly configured
bool getAlignLinks(const string &configFile, const string &type,
                   iKinLink **link1, iKinLink **link2);


// Aligns head joints bounds with current onboard bounds.
// Returns a matrix containing the actual limits
Matrix alignJointsBounds(iKinChain *chain, PolyDriver *drvTorso, PolyDriver *drvHead,
                         const double eyeTiltMin, const double eyeTiltMax);


// Copies joints bounds from first chain to second chain
void copyJointsBounds(iKinChain *ch1, iKinChain *ch2);


// Updates torso blocked joints values within the chain
void updateTorsoBlockedJoints(iKinChain *chain, const Vector &fbTorso);


// Updates neck blocked joints values within the chain
void updateNeckBlockedJoints(iKinChain *chain, const Vector &fbNeck);


// Reads encoders values.
// Returns true if communication with robot is stable, false otherwise.
bool getFeedback(Vector &fbTorso, Vector &fbHead, IEncoders *encTorso, IEncoders *encHead);


#endif


