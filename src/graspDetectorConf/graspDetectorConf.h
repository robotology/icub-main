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
#define N_DATA 3000

class graspDetector: public RateThread
{
public:
    graspDetector(BufferedPort<Bottle>*, int );
    ~graspDetector();
    bool threadInit();
    void stop();
    void run();
    bool startCollect(Bottle);
    void stopCollect();
    bool endedMovement();
    void buildPattern();
    bool getPattern(Vector&, double&, double&);
    
private:
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


