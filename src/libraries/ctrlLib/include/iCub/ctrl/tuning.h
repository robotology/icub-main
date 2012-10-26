/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

/**
 * \defgroup Tuning Tuning
 *  
 * @ingroup ctrlLib
 *
 * Classes for PID tuning
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __TUNING_H__
#define __TUNING_H__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/pids.h>
#include <iCub/ctrl/kalman.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>


namespace iCub
{

namespace ctrl
{

/**
* \ingroup Tuning
*
* Online DC Motor Parameters Estimator. 
*  
* Estimate the gain \f$ K \f$ and the mechanical time constant
* \f$ \tau \f$ of the following DC motor transfer function with 
*     voltage \f$ V \f$ as input and angular position \f$ \theta
*     \f$ as output: \n
*     \f$ \frac{\theta}{V}=\frac{K}{1+s\cdot\tau} \cdot
*         \frac{1}{s}. \f$ \n
* The employed algorithm makes use of an online Extended Kalman
* Filter. 
*/
class OnlineDCMotorParametersEstimator
{
protected:
    yarp::sig::Matrix A;
    yarp::sig::Matrix F;
    yarp::sig::Vector B;
    yarp::sig::Matrix C;
    yarp::sig::Matrix Ct;    
    yarp::sig::Matrix Q;
    yarp::sig::Matrix P;
    yarp::sig::Vector x;
    yarp::sig::Vector _x;
    double R;
    double Ts;

public:
    /**
     * Default constructor.
     */
    OnlineDCMotorParametersEstimator();

    /**
     * Initialize the estimation. 
     *  
     * @param Ts the estimator sample time given in seconds. 
     * @param Q Process noise covariance. 
     * @param R Measurement noise covariance. 
     * @param P0 Initial condition for estimated error covariance. 
     * @param x0 A 4x1 vector containing respectively the initial 
     *           conditions for position, velocity, \f$ \tau \f$ and
     *           \f$ K. \f$
     *  
     * @return true/false on success/failure. 
     */
    bool init(const double Ts, const double Q, const double R,
              const double P0, const yarp::sig::Vector &x0);

    /**
     * Estimate the state vector given the current input and the 
     * current measurement. 
     * 
     * @param u Current input. 
     * @param y Current measurement. 
     * 
     * @return Estimated state vector composed of position, 
     *         velocity, \f$ \tau \f$ and \f$ K. \f$
     */
    yarp::sig::Vector estimate(const double u, const double y);

    /**
     * Return the estimated state.
     * 
     * @return Estimated state vector composed of position, 
     *         velocity, \f$ \tau \f$ and \f$ K. \f$
     */
    yarp::sig::Vector get_x() const { return _x; }
};


/**
* \ingroup Tuning
*
* Online Stiction Estimator. 
*  
* Estimate the positive and negative stiction values. 
*/
class OnlineStictionEstimator : public yarp::os::RateThread
{
protected:
    yarp::dev::IControlMode   *imod;
    yarp::dev::IControlLimits *ilim;
    yarp::dev::IEncoders      *ienc;
    yarp::dev::IPidControl    *ipid;

    yarp::os::Semaphore        mutex;
    yarp::sig::Vector          gamma;
    yarp::sig::Vector          theta;
                              
    AWLinEstimator             velEst;
    AWQuadEstimator            accEst;
    parallelPID               *pid;
    Integrator                 intErr;
    minJerkTrajGen             trajGen;

    int    joint;
    double t0,T;
    double x_min,x_max;
    double x_pos,x_vel,x_acc;
    double kp,ki,kd;
    double vel_thres,e_thres;
    double tg,xd_pos;    
    bool   adapt,adaptOld,done;

    enum {rising, falling} state;

    bool threadInit();
    void run();
    void threadRelease();

public:
    /**
     * Default constructor.
     */
    OnlineStictionEstimator();

    /**
     * Initialize the estimation. 
     *  
     * @param driver the device driver to control the robot part.
     * @param options the configuration options. 
     *  
     * @note Available options are: 
     *  
     * @b Ts <double>: specify the estimator sample time given in 
     *    seconds.
     *  
     * @b joint <int>: specify the joint to be controlled. 
     *  
     * @b T <double>: specify the period in seconds of the reference
     *    waveform used for tracking.
     *  
     * @b kp <double>: specify the proportional term of the 
     *    high-level controller for tracking purpose.
     *  
     * @b ki <double>: specify the integral term of the high-level 
     *    controller for tracking purpose.
     *  
     * @b kd <double>: specify the derivative term of the 
     *    high-level controller for tracking purpose.
     *  
     * @b vel_thres <double>: specify the velocity threshold used to 
     *    identify the estimation time windows. The estimation is
     *    carried out whenever |vel|<vel_thres.
     *  
     * @b e_thres <double>: specify the error threshold above which 
     *    keep updating the stiction values. The estimation is
     *    carried out until |e_mean|>e_thres, where e_mean accounts
     *    for the integral average of the error computed within the
     *    estimation time window.
     *  
     * @b gamma (<double> <double>): specify the gains used for 
     *    updating the stiction positive and negative values,
     *    respectively.
     *  
     * @b theta (<double> <double>): specify the initial stiction 
     *    positive and negative values, respectively.
     *  
     * @return true/false on success/failure. 
     */
    bool configure(yarp::dev::PolyDriver &driver, const yarp::os::Property &options);

    /**
     * Retrieve the estimation. 
     *  
     * @return Current positive and negative stiction values given 
     *         as components of a 2x1 vector.
     */
    yarp::sig::Vector getEstimation();

    /**
     * Check the current estimation status.
     *  
     * @return true when |e_mean|<e_thres.
     */
    bool isDone() const { return done; }
};


/**
* \ingroup Tuning
*
* Online P Compensator Design.
*/
class OnlinePCompensatorDesign
{
protected:

public:
};

}

}

#endif



