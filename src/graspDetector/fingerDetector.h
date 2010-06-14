// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <math.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <yarp/os/RateThread.h>

/* itoa example */
#include <stdio.h>
#include <stdlib.h>

using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::os;

class fingerDetector: public RateThread
{
public:
    fingerDetector( int );
    ~fingerDetector();
    bool threadInit();
    void setIndex(Bottle);
    void setModel(Bottle, Bottle, double, double, double, double);
    void copyAnalog(Bottle *);
    void stop();
    void run();
    
private:
    Vector index;
    Vector fingerAnalog;
    Vector q0;
    Vector q1;
    double min;
    double max;
    double minT;
    double maxT;
public:
    double status; //true if grasping (model not respected) 
};


