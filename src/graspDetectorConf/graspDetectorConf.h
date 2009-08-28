// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <ace/OS.h>
#include <ace/Log_Msg.h>
#include <math.h>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <yarp/String.h> 
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>

#include <gsl/gsl_vector.h>
     


using namespace yarp::math;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp;

#define PI 3.14
#define N_DATA 300

class graspDetector: public RateThread
{
public:
    graspDetector(PolyDriver*, BufferedPort<Bottle>*, int );
    ~graspDetector();
    bool threadInit();
    void stop();
    void run();
    bool startMovement(double, double, int);
    void startCollect(Bottle);
    void stopCollect();
    bool endedMovement(Vector &s);
    void buildPattern();
    bool getPattern(Vector&, double&, double&);
    
private:
    PolyDriver *Arm_dd;
	
    IPositionControl *pos;
    IVelocityControl *vel;
    IEncoders *enc;
    IPidControl *pid;
    IAmplifierControl *amp;
    IControlLimits *lim;

    int joint;
    int nJoints;
    int collect;
    int collectCounter;
    
    BufferedPort<Bottle> *analogPort;
    Bottle *lastBottle;
    Vector index;
    Matrix D;
    Vector span; 
    Vector lambda;
    double maxError;
    double minError;
};


