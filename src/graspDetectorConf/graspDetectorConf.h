// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_graspDetectorConf
 *
 * \defgroup icub_graspDetectorConfClasses graspDetectorConfClasses
 *
 *
 * A basic module for configuring the \ref icub_graspDetector module.
 *
 * Gives a (linear) description of the acquired data \f$ q \in R^{n} \f$. 
 * The model is represented as the intersection of \f$ k \f$ linear spaces
 * whose dimensionality is \f$ n-1 \f$. The number of planes \f$ k \f$ is determined by
 * the dimensionality \f$ n \f$ of the input data (represented in the index vector)
 * so as to resutl in one dimensional vector space.
 * In particular, \f$ k=n-1 \f$ (e.g. if \f$ n=2 \f$ then \f$ k=1 \f$, 
 * if \f$ n=3 \f$ then \f$ k=2 \f$). Given, \f$ q = \left[ q_1 \dots  q_n\right] \in R^{n} \f$
 * the \f$ n-1 \f$ planes are described by the following set of equations:
 * \f[ \left\{  \begin{array}{l} k_{21} q_1 + k_{22} q_2 + 0 \cdot q_3 + \dots +  0 \cdot q_n  = 1 \\ \vdots \\ k_{n1} q_1 + 0 \cdot q_2 + \dots + 0 \cdot q_{n-1} +  k_{nn} q_n  = 1 \end{array} \right. .  \f]
 * Parameters \f$ k_{ij} \f$ are estimated with a least squares solution and are
 * then mapped into the following linear model:
 * \f[ q = q_0 + q_1 \cdot t , \f]
 * according to the following equations:
 * \f[ q_0 = \left[ \begin{array}{l}  0 \\ \frac{1}{k_{22}} \\ \vdots \\\frac{1}{k_{nn}} \end{array} \right],  \quad q_1 = \left[ \begin{array}{l}  1 \\ -\frac{k_{21}}{k_{22}} \\ \vdots \\ -\frac{k_{n1}}{k_{nn}}  \end{array} \right].  \f]
 * Given a dataset \f$ q^1 \dots q^N \f$, points of minimum distance are computed
 * as follows:
 * \f[ t^*_j = \arg \min_{t} \left\| q_0 +  q_1 \cdot t - q^j \right\| \quad j = 1 \dots N, \f]
 * which corresponds to:
 * \f[ q^{*,j} = q_0 +  q_1 \cdot t^*_j \quad j = 1 \dots N. \f]
 * Accordingly, the span of allowed values for \f$ t \f$ is dtermined as follows:
 * \f[ t_{max} = \max_{j = 1 \dots N} t^*_j \quad t_{min} = \min_{j = 1 \dots N} t^*_j. \f]
 * Similarly, the maximum and the minimum distance from the model is computed as:
 * \f[ q_{max} = \max_{j = 1 \dots N} \left| q^{*,j} - q^j \right| \quad q_{min} = \min_{j = 1 \dots N} \left| q^{*,j} - q^j \right|. \f]
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
#include <gsl/gsl_blas.h>

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
* grasped.
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
    *
    * @param q0 (output) the linear parameters of the model
    * @param q1 (output) the linear parameters of the model
    * @param m (output) the value of \f$ q_{min}\f$
    * @param M (output) the value of \f$ q_{max}\f$
    * @param t (output) the value of \f$ t_{min}\f$
    * @param T (output) the value of \f$ t_{max}\f$
    */
    bool getPattern(Vector& q0, Vector& q1, double& m, double& M, double& t, double& T);
    
private:
    int joint;
    int nJoints;
    int collect;
    int collectCounter;
    
    BufferedPort<Bottle> *analogPort;
    Bottle *lastBottle;
    Vector index;
    Matrix D;
    Vector q0,q1;
    double maxError;
    double minError;
    double maxT;
    double minT;
};


