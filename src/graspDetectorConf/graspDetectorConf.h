// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_graspDetectorConf
 *
 * \defgroup icub_graspDetectorConfClasses graspDetectorConfClasses
 *
 *
 * A basic module for configuring the 
 * \ref icub_graspDetector module.
 * \author Francesco Nori
 *
 * Copyright (C) 2008 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/graspDetectorConf/main.cpp.
 */

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

/**
* \ingroup icub_graspDetectorConfClasses
*
* A base class for building a description
* of the hand joints when no object is
* grapsed.
*/
class graspDetectorConf: public RateThread
{
public:
    /**
    * Constructor. 
    * @param p is the port where the analog input is expected
    * @param r is the rate at which data will be collected
    */
    graspDetectorConf(BufferedPort<Bottle>* p, int r);
    ~graspDetectorConf();
    bool threadInit();
    /**
    * Stops the thread.
    */
    void stop();
    /**
    * Starts the thread (will do nothing until a startCollect()
    * is instantiated). 
    */
    void run();
    /**
    * Starts collecting data.
    * @param analogIndex the indices of the joint to be considered
    */
    bool startCollect(Bottle analogIndex);
    /**
    * Stops collecting data.
    */
    void stopCollect();
    bool endedMovement();
    /**
    * Process the data and build a (linear) description
    * of the acquired data.
    */
    void buildPattern();
    /**
    * Gives a (linear) description of the acquired data
    * @param l (output) the linear parameters of the model
    * @param m (output) the minimum distance from the model
    * @param M (output) the maximum distance from the model
    */
    bool getPattern(Vector& l, double& m, double& M);
    
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


