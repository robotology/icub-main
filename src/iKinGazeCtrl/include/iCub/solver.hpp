
#ifndef __SOLVER_H__
#define __SOLVER_H__

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iCub/pids.h>

#include <deque>

#include <iCub/gazeNlp.hpp>
#include <iCub/utils.hpp>
#include <iCub/localizer.hpp>
#include <iCub/controller.hpp>

#define EYEPINVREFGEN_GAIN              10.0
#define NECKSOLVER_ACTIVATIONANGLE_TRA  15.0
#define NECKSOLVER_ACTIVATIONANGLE_SAG  5.0
#define NECKSOLVER_MOVEDTORSOQUEUSIZE   5

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace ctrl;
using namespace iKin;


// The thread launched by the application which computes
// the eyes target position relying on the pseudoinverse
// method.
class EyePinvRefGen : public RateThread
{
protected:
    iCubHeadCenter *neck;
    iCubEye        *eyeL,      *eyeR;
    iKinChain      *chainNeck, *chainEyeL, *chainEyeR;
    PolyDriver     *drvTorso,  *drvHead;
    IControlLimits *limTorso,  *limHead;
    IEncoders      *encTorso,  *encHead;
    exchangeData   *commData;
    xdPort         *port_xd;
    Integrator     *I;

    iKinLink *alignLnkLeft1,  *alignLnkLeft2;
    iKinLink *alignLnkRight1, *alignLnkRight2;

    BufferedPort<Vector> *port_inertial;

    unsigned int period;
    string robotName;
    string localName;
    string inertialName;
    string configFile;
    bool Robotable;
    bool genOn;
    int nJointsTorso;
    int nJointsHead;
    double eyeTiltMin;
    double eyeTiltMax;
    double Ts;
    
    Vector fbTorso;
    Vector fbHead;
    Vector qd,fp;
    Matrix eyesJ;
    Vector gyro;

public:
    EyePinvRefGen(PolyDriver *_drvTorso, PolyDriver *_drvHead,
                  exchangeData *_commData, const string &_robotName,
                  const string &_localName, const string &_inertialName,
                  const string &_configFile, const double _eyeTiltMin,
                  const double _eyeTiltMax, unsigned int _period);

    void set_xdport(xdPort *_port_xd) { port_xd=_port_xd; }
    void enable()                     { genOn=true;       }
    void disable()                    { genOn=false;      }

    virtual bool threadInit();
    virtual void afterStart(bool s);
    virtual void run();
    virtual void threadRelease();
    virtual void suspend();
    virtual void resume();
    virtual void stopControl();
};


// The thread launched by the application which is
// in charge of inverting the head kinematic relying
// on IPOPT computation.
class Solver : public RateThread
{
protected:
    iCubHeadCenter   *neck;
    iCubEye          *eyeL,      *eyeR;
    iKinChain        *chainNeck, *chainEyeL, *chainEyeR;
    GazeIpOptMin     *invNeck,   *invEyes;
    PolyDriver       *drvTorso,  *drvHead;
    IControlLimits   *limTorso,  *limHead;
    IEncoders        *encTorso,  *encHead;
    exchangeData     *commData;
    neckCallback     *neckCallbackObj;
    eyesCallback     *eyesCallbackObj;
    EyePinvRefGen    *eyesRefGen;
    Localizer        *loc;
    Controller       *ctrl;
    xdPort           *port_xd;

    iKinLink *alignLnkLeft1,  *alignLnkLeft2;
    iKinLink *alignLnkRight1, *alignLnkRight2;

    string localName;
    string configFile;
    unsigned int period;
    bool Robotable;
    int nJointsTorso;
    int nJointsHead;
    double eyeTiltMin;
    double eyeTiltMax;
    double Ts;

    Vector xdOld;
    Vector fbTorso;
    Vector fbHead;
    Vector neckPos;
    Vector gazePos;

    double neckPitchMin;
    double neckPitchMax;
    double neckYawMin;
    double neckYawMax;

    deque<Vector> fbTorsoOld;

    unsigned int alignNeckCnt;

public:
    Solver(PolyDriver *_drvTorso, PolyDriver *_drvHead, exchangeData *_commData,
           EyePinvRefGen *_eyesRefGen, Localizer *_loc, Controller *_ctrl,
           const string &_localName, const string &_configFile, const double _eyeTiltMin,
           const double _eyeTiltMax, unsigned int _period);

    // Returns a measure of neck angle required to reach the target
    void   updateAngles();
    Vector neckTargetRotAngles(const Vector &xd);

    void blockNeckPitch(const double val_deg);
    void blockNeckYaw(const double val_deg);
    void clearNeckPitch();
    void clearNeckYaw();

    virtual bool threadInit();
    virtual void afterStart(bool s);
    virtual void run();
    virtual void threadRelease();
    virtual void suspend();
    virtual void resume();
};


#endif


