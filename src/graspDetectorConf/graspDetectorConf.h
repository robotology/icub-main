// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_graspDetectorConf
 *
 * \defgroup icub_graspDetectorConfThread graspDetectorConfThread
 *
 *
 * A basic thread for configuring the \ref icub_graspDetector module. The thread
 * collects data at a give rate and processes this information fitting a linear
 * model to the collected data. 
 *
 * We here give a detailed description of the procedure used to fit the linear
 * model to the collected data. In order to be as general as possible, we
 * assume that the data are generic \f$ n \f$ dimensional
 * vectors, here indicated \f$ q \in R^{n} \f$ (for the thumb, index and middle
 * distal joints \f$ n=2 \f$ since these joints are obtained by the coordinated
 * movement of the middle and distal phalanxes; for the ring and little fingers
 * we typically have \f$ n=6 \f$ since there is a unique motor actuating the two 
 * fingers proximal, middle and distal phalanxes). The parametric linear model 
 * used to fit the data is:
 *
 * \f[ r: \quad q = q_0 + q_1 \cdot t , \quad t \in \left[ t_{min}, t_{max} \right] \f]
 *
 * where \f$ t \f$ is the free parameter to be chosen in \f$ \left[ t_{min}, 
 * t_{max} \right] \f$. Another possible representation for this linear model
 * can be obtained by observing that it represents a line \f$ r \f$ (i.e. a one dimensional subspace)
 * embedded in a \f$ n \f$ dimensional vector space. Therefore, it can be 
 * represented as the intersection of planes. The general implicit equation of a 
 * plane \f$ \pi \f$ in \f$ R^{n} \f$ is:
 *
 * \f[ \pi: \quad a_1 q_1 + a_2 q_2 + \dots +  a_n \cdot q_n  = c. \f]
 *
 * Since this plane is a \f$ n - 1 \f$ dimensional subspace, the intersection
 * of \f$ k \f$ planes is a \f$ n - k \f$ dimensional subspace. Therefore, a line \f$ r \f$
 * (i.e. a one dimensional subspace) can be represented by intersecting \f$ n - 1 \f$
 * planes:
 *
 * \f[ r: \quad \left\{  \begin{array}{l} a_{2,1} \cdot q_1 + a_{2,2} \cdot q_2 + a_{2,3} \cdot q_3 + \dots +  a_{2,n} \cdot q_n  = c_2 \\ \vdots \\ a_{n,1} \cdot q_1 + a_{n,2} \cdot q_2 + \dots + a_{n,n} \cdot q_n  = c_{n} \end{array} \right. .  \f]
 *
 * However, the representation of a line by means of the coefficients \f$ a_{1,1} \f$, \f$ \dots \f$, \f$ a_{n-1,n} \f$ is redundant. It can be shown that an equivalent
 *
 * non-redundant implicit representation is the following:
 *
 * \f[ r: \left\{  \begin{array}{l} a_{2,1} \cdot q_1 + q_2 + 0 \cdot q_3 + \dots +  0 \cdot q_n  = c_2 \\ \vdots \\ a_{n,1} \cdot q_1 + 0 \cdot q_2 + \dots + 0 \cdot q_{n-1} + q_n  = c_{n} \end{array} \right. .  \f]
 *
 * which corresponds to the following parametric model:
 *
 * \f[ r: \quad q = q_0 + q_1 \cdot t \f]
 *
 * with \f$ t = q_1 \f$ and:
 *
 * \f[ q_0 = \left[ \begin{array}{l}  0 \\ c_{2} \\ \vdots \\ c_{n} \end{array} \right],  \quad q_1 = \left[ \begin{array}{l}  1 \\ -a_{2,1} \\ \vdots \\ -a_{n,1}  \end{array} \right].  \f]
 *
 * Given a data set \f$ q^1 \dots q^N \f$, the parameters \f$ a_{i,1} \f$ and \f$ c_{i} \f$ are 
 * estimated by solving the following least squares optimization:
 *
 * \f[ \min_{\begin{array}{l} a_{2,1}, \dots, a_{n,1} \\ c_2, \dots, c_n \end{array}} \sum_{i=1}^N \sum_{j=2}^n \left\| a_{j,1} q_1^i + q_j^i - c_j \right\|^2 \f]
 *
 * These optimal parameters are then converted to \f$ q_0 \f$  and \f$ q_1 \f$ by means 
 * of the formula above. Moreover, points of minimum distance are computed as follows:
 *
 * \f[ t^*_j = \arg \min_{t} \left\| q_0 +  q_1 \cdot t - q^j \right\| \quad j = 1 \dots N, \f]
 *
 * which corresponds to:
 *
 * \f[ q^{*,j} = q_0 +  q_1 \cdot t^*_j \quad j = 1 \dots N. \f]
 *
 * Accordingly, the span of allowed values for \f$ t \f$ is determined as follows:
 *
 * \f[ t_{max} = \max_{j = 1 \dots N} t^*_j \quad t_{min} = \min_{j = 1 \dots N} t^*_j. \f]
 *
 * Similarly, the maximum and the minimum distance from the model is computed as:
 *
 * \f[ q_{max} = \max_{j = 1 \dots N} \left| q^{*,j} - q^j \right| \quad q_{min} = \min_{j = 1 \dots N} \left| q^{*,j} - q^j \right|. \f]
 *
 * \author Francesco Nori
 *
 * Copyright (C) 2008 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/graspDetectorConf/graspDetctorConf.h.
 */

#include <math.h>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

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

#define PI 3.14
#define N_DATA 3000

/**
* \ingroup icub_graspDetectorConfThread
*
* A thread for collecting hand position data and building from this 
* data a description of the hand joints configuration when no object is
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
    * @param void (input) does not take any input parameter
    */
    void buildPattern();
    /**
    * Retrieve the estimated model.
    * @param q0 (output) the linear parameters of the model
    * @param q1 (output) the linear parameters of the model
    * @param m (output) the value of \f$ q_{min}\f$
    * @param M (output) the value of \f$ q_{max}\f$
    * @param t (output) the value of \f$ t_{min}\f$
    * @param T (output) the value of \f$ t_{max}\f$
    */
    void getPattern(Vector& q0, Vector& q1, double& m, double& M, double& t, double& T);
    
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


