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

#ifndef __UTILS_H__
#define __UTILS_H__

#include <string>
#include <algorithm>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <iCub/gazeNlp.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


// This class handles the incoming fixation point
// xyz coordinates.
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

    Mutex  mutex_0;
    Mutex  mutex_1;
    Event  syncEvent;
    Vector xd;
    Vector xdDelayed;    
    bool   isNew;
    bool   isNewDelayed;
    bool   locked;
    bool   closing;
    int    rx;

    void onRead(Bottle &b);
    void run();

public:
    xdPort(const Vector &xd0, void *_slv);
    ~xdPort();

    void    lock()           { locked=true;         }
    void    unlock()         { locked=false;        }
    bool    islocked() const { return locked;       }
    int     get_rx() const   { return rx;           }
    bool   &get_new()        { return isNew;        }
    bool   &get_newDelayed() { return isNewDelayed; }    
    void    set_xd(const Vector &_xd);
    Vector  get_xd();
    Vector  get_xdDelayed();
};


// This class handles the data exchange among components.
class ExchangeData
{
protected:
    Mutex  mutex[9];
    
    Vector xd,qd;
    Vector x,q,torso;
    Vector v,counterv;
    Matrix S;
    Vector imu;
    bool   isCtrlActive;
    bool   canCtrlBeDisabled;
    bool   saccadeUnderway;
    double minAllowedVergence;
    double x_stamp;

public:
    ExchangeData();

    void    resize_v(const int sz, const double val);
    void    resize_counterv(const int sz, const double val);

    void    set_xd(const Vector &_xd);
    void    set_qd(const Vector &_qd);
    void    set_qd(const int i, const double val);
    void    set_x(const Vector &_x);
    void    set_x(const Vector &_x, const double stamp);
    void    set_q(const Vector &_q);
    void    set_torso(const Vector &_torso);
    void    set_v(const Vector &_v);
    void    set_counterv(const Vector &_counterv);
    void    set_fpFrame(const Matrix &_S);
    void    set_imu(const Vector &_imu);

    Vector  get_xd();
    Vector  get_qd();
    Vector  get_x();
    Vector  get_x(double &stamp);
    Vector  get_q();
    Vector  get_torso();
    Vector  get_v();
    Vector  get_counterv();
    Matrix  get_fpFrame();
    Vector  get_imu();

    bool   &get_isCtrlActive()       { return isCtrlActive;       }
    bool   &get_canCtrlBeDisabled()  { return canCtrlBeDisabled;  }
    bool   &get_isSaccadeUnderway()  { return saccadeUnderway;    }
    double &get_minAllowedVergence() { return minAllowedVergence; }

    // data members that do not need protection
    string         robotName;
    string         localStemName;
    double         eyeTiltMin;
    double         eyeTiltMax;
    double         head_version;
    bool           verbose;
    bool           tweakOverwrite;
    ResourceFinder rf_cameras;
    ResourceFinder rf_tweak;
    string         tweakFile;
    bool           debugInfoEnabled;
};


// This class handles the incoming IMU data
class IMUPort : public BufferedPort<Vector>
{
protected:
    ExchangeData *commData;
    void onRead(Vector &imu);

public:
    IMUPort();
    void setExchangeData(ExchangeData *commData);
};


// This class defines gaze components such as
// controller, localizer, solver ...
class GazeComponent
{
protected:
    iCubEye *eyeL;
    iCubEye *eyeR;

public:
    virtual bool getExtrinsicsMatrix(const string &type, Matrix &M);
    virtual bool setExtrinsicsMatrix(const string &type, const Matrix &M);
    virtual void minAllowedVergenceChanged() { }
};


// Saturate value val between min and max.
inline double sat(const double val, const double min, const double max)
{
    return std::min(std::max(val,min),max);
}


// Allocates Projection Matrix Prj for the camera read from camerasFile;
// type is in {"CAMERA_CALIBRATION_LEFT","CAMERA_CALIBRATION_RIGHT"}.
// Returns true if correctly configured.
bool getCamPrj(const ResourceFinder &rf, const string &type, Matrix **Prj, const bool verbose=false);


// Allocates the two aligning matrices read from camerasFile;
// type is in {"ALIGN_KIN_LEFT","ALIGN_KIN_RIGHT"}.
// Returns true if correctly configured.
bool getAlignHN(const ResourceFinder &rf, const string &type, iKinChain *chain, const bool verbose=false);


// Aligns head joints bounds with current onboard bounds.
// Returns a matrix containing the actual limits.
Matrix alignJointsBounds(iKinChain *chain, PolyDriver *drvTorso, PolyDriver *drvHead,
                         const double eyeTiltMin, const double eyeTiltMax);


// Copies joints bounds from first chain to second chain.
void copyJointsBounds(iKinChain *ch1, iKinChain *ch2);


// Updates torso blocked joints values within the chain.
void updateTorsoBlockedJoints(iKinChain *chain, const Vector &fbTorso);


// Updates neck blocked joints values within the chain.
void updateNeckBlockedJoints(iKinChain *chain, const Vector &fbNeck);


// Reads encoders values.
// Returns true if communication with robot is stable, false otherwise.
bool getFeedback(Vector &fbTorso, Vector &fbHead, PolyDriver *drvTorso,
                 PolyDriver *drvHead, ExchangeData *commData, double *timeStamp=NULL);

// Computes the velocity of the fixation point given:
// 
// H: the frame in which the measurements v and w are provided
// v: the 3x1 translational velocity of the frame [m/s]
// w: the 3x1 rotational velocity of the frame [rad/s]
// x_FP: the current 3x1 fixation point [m]
// 
// dx_FP: the output 6x1 velocity of the fixation point 
// 
// Returns true if the task was successful.
bool computedxFP(const Matrix &H, const Vector &v, const Vector &w,
                 const Vector &x_FP, Vector &dx_FP);

// Computes the neck velocity given the fixation point velocity.
// Returns true if the task was successful.
bool computeNeckVelFromdxFP(iKinChain *chainNeck, const Vector &x_FP,
                            const Vector &dx_FP, Vector &dq_neck);

// Computes the eyes velocity given the fixation point velocity.
// Returns true if the task was successful.
bool computeEyesVelFromdxFP(iKinChain *chainEyeL, iKinChain *chainEyeR,
                            const Vector &x_FP, const Vector &dx_FP,
                            Vector &dq_eyes);

#endif


