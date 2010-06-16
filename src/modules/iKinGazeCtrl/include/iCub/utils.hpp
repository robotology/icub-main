
#ifndef __UTILS_H__
#define __UTILS_H__

#include <yarp/os/Stamp.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <string>

#include <iCub/gazeNlp.hpp>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace ctrl;
using namespace iKin;


// This inherited class handles the incoming
// fixation point xyz coordinates.
// 
// Since it accepts a bottle, it is possible to 
// issue the command "yarp read /sender /ctrlName/xd:i"
// and type the target pose manually.
class xdPort : public BufferedPort<Bottle>
{
protected:
    Vector xd;
    bool   isNew;

    virtual void onRead(Bottle &b);

public:
    xdPort(const Vector &xd0)
    { 
        xd=xd0;
        isNew=false;
    }

    bool   &get_new() { return isNew; }
    Vector &get_xd()  { return xd;    }

    void set_xd(const Vector &_xd)
    {
        xd=_xd;
        isNew=true;
    }
};


// This class handles the data exchange
// between Solvers and Controller objects.
class exchangeData
{
protected:
    Vector xd,qd;
    Vector x,q,torso;
    Vector v,compv;
    Matrix S;
    bool   isCtrlActive;
    bool   canCtrlBeDisabled;

public:
    exchangeData()
    {
        isCtrlActive=false;
        canCtrlBeDisabled=true;
    }

    Vector &get_xd()                { return xd;                }
    Vector &get_qd()                { return qd;                }
    Vector &get_x()                 { return x;                 }
    Vector &get_q()                 { return q;                 }
    Vector &get_torso()             { return torso;             }
    Vector &get_v()                 { return v;                 }
    Vector &get_compv()             { return compv;             }
    Matrix &get_fpFrame()           { return S;                 }
    bool   &get_isCtrlActive()      { return isCtrlActive;      }
    bool   &get_canCtrlBeDisabled() { return canCtrlBeDisabled; }
};


// This class handles callbacks for neck solver
class neckCallback : public iKinIterateCallback
{
private:
    neckCallback(const neckCallback&);
    neckCallback &operator=(const neckCallback&);

protected:
    exchangeData *commData;

public:
    neckCallback(exchangeData *_commData) : commData(_commData) { }
    virtual void exec(Vector xd, Vector q);
};


// This class handles callbacks for eyes solver
class eyesCallback : public iKinIterateCallback
{
private:
    eyesCallback(const eyesCallback&);
    eyesCallback &operator=(const eyesCallback&);

protected:
    exchangeData *commData;

public:
    eyesCallback(exchangeData *_commData) : commData(_commData) { }
    virtual void exec(Vector xd, Vector q);
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
Matrix alignJointsBounds(iKinChain *chain, IControlLimits *limTorso, IControlLimits *limHead,
                         const double eyeTiltMin, const double eyeTiltMax);


// Copies joints bounds from first chain to second chain
void copyJointsBounds(iKinChain *ch1, iKinChain *ch2);


// Updates torso blocked joints values within the chain
void updateTorsoBlockedJoints(iKinChain *chain, Vector &fbTorso);


// Updates neck blocked joints values within the chain
void updateNeckBlockedJoints(iKinChain *chain, Vector &fbNeck);


// Reads encoders values.
// Returns true if communication with robot is stable, false otherwise.
bool getFeedback(Vector &fbTorso, Vector &fbHead, IEncoders *encTorso, IEncoders *encHead);


#endif


