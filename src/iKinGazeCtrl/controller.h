
#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iCub/pids.h>
#include <iCub/trajectoryGenerator.h>

#include "utils.h"

#define INTARGET_TOL    1e-3

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace ctrl;
using namespace iKin;


// The thread launched by the application which is
// in charge of computing the velocities profile.
class Controller : public RateThread
{
protected:
    iCubEye               *eyeLim;
    iKinChain             *chainEyeLim;
    PolyDriver            *drvTorso,  *drvHead;
    IControlLimits        *limTorso,  *limHead;
    IEncoders             *encTorso,  *encHead;
    IVelocityControl      *velHead;
    exchangeData          *commData;

    BufferedPort<Vector>  *port_qd;
    BufferedPort<Vector>  *port_x;
    BufferedPort<Vector>  *port_q;
    BufferedPort<Vector>  *port_v;

    minJerkTrajGen *genTrajNeck;
    minJerkTrajGen *genTrajEyes;
    Integrator     *Int;

    string robotName;
    string localName;
    unsigned int period;
    bool Robotable;
    int nJointsTorso;
    int nJointsHead;
    double printAccTime;
    double neckTime;
    double eyesTime;
    double Ts;
    double tOld;

    Vector qddeg,qdeg,vdeg,xd,fp;
    Vector v,vNeck,vEyes,vdegOld;
    Vector qd,qdNeck,qdEyes;
    Vector fbTorso,fbHead,fbNeck,fbEyes;

public:
    Controller(PolyDriver *_drvTorso, PolyDriver *_drvHead, exchangeData *_commData,
               const string &_robotName, const string &_localName, double _neckTime,
               double _eyesTime, unsigned int _period);

    void stopLimbsVel();
    void suspend();
    void resume();
    void printIter(Vector &xd, Vector &fp, Vector &qd, Vector &q,
                   Vector &v, double printTime);

    virtual bool threadInit();
    virtual void afterStart(bool s);
    virtual void run();
    virtual void threadRelease();
};


#endif


