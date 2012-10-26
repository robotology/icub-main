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

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/dev/PolyDriver.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/kalman.h>
#include <iCub/ctrl/pids.h>


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
     * @return Estimated state.
     */
    yarp::sig::Vector get_x() const { return x; }
};


/**
* \ingroup Tuning
*
* Online Stiction Estimator.
*/
class OnlineStictionEstimator
{
protected:

public:
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



