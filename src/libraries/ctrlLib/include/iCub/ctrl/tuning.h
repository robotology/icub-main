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
 * Classes for Compensators tuning
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
#include <iCub/ctrl/filters.h>
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
* \f$ \tau \f$ of the following DC motor second order transfer 
*     function with voltage \f$ V \f$ as input and angular
*     position \f$ \theta \f$ as output:
*  
* \f$ \theta/V=K/\left(1+s\tau\right) \cdot 1/s. \f$
*  
* The employed algorithm makes use of an online Extended Kalman
* Filter. 
*/
class OnlineDCMotorEstimator
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
    OnlineDCMotorEstimator();

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
     * Initialize the internal state.
     *  
     * @param P0 Initial condition for estimated error covariance.  
     * @param x0 A 4x1 vector containing respectively the initial 
     *           conditions for position, velocity, \f$ \tau \f$ and
     *           \f$ K. \f$
     */
    bool init(const double P0, const yarp::sig::Vector &x0);

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

    /**
     * Return the estimated error covariance.
     * 
     * @return Estimated error covariance.
     */
    yarp::sig::Matrix get_P() const { return P; }

    /**
     * Return the system parameters.
     * 
     * @return vector containing \f$ \tau \f$ and \f$ K. \f$ 
     */
    yarp::sig::Vector get_parameters() const { return _x.subVector(2,3); }
};


/**
* \ingroup Tuning
*
* Online Stiction Estimator. 
*  
* Estimate the positive and negative stiction values. \n 
* During the experiment, the joint is controlled by a high-level 
* pid controller that commands directly the voltage in order to 
* track a time varying reference position. During the 
* transitions, the stiction values are estimated. 
*/
class OnlineStictionEstimator : public yarp::os::RateThread
{
protected:
    yarp::dev::IControlMode   *imod;
    yarp::dev::IControlLimits *ilim;
    yarp::dev::IEncoders      *ienc;
    yarp::dev::IPidControl    *ipid;

    yarp::os::Semaphore mutex;
    yarp::os::Event     doneEvent;
    yarp::sig::Vector   gamma;
    yarp::sig::Vector   stiction;
    yarp::sig::Vector   done;
                              
    AWLinEstimator   velEst;
    AWQuadEstimator  accEst;
    parallelPID     *pid;
    Integrator       intErr;
    minJerkTrajGen   trajGen;

    int    joint;
    double dpos_dV;
    double t0,T;
    double x_min,x_max;
    double x_pos,x_vel,x_acc;
    double kp,ki,kd;
    double vel_thres,e_thres;
    double tg,xd_pos;    
    bool   adapt,adaptOld;
    bool   configured;

    enum
    {
        rising,
        falling
    } state;

    bool threadInit();
    void run();
    void threadRelease();

    // prevent user from calling them directly
    bool start();
    void stop();

public:
    /**
     * Default constructor.
     */
    OnlineStictionEstimator();

    /**
     * Configure the estimation. 
     *  
     * @param driver the device driver to control the robot part.
     * @param options the configuration options. 
     *  
     * Available options are: 
     *  
     * @b joint <int>: specify the joint to be controlled. 
     *  
     * @b Ts <double>: specify the estimator sample time given in 
     *    seconds.
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
     * @b stiction (<double> <double>): specify the initial stiction
     *    positive and negative values, respectively.
     *  
     * @return true/false on success/failure. 
     */
    virtual bool configure(yarp::dev::PolyDriver &driver, const yarp::os::Property &options);

    /**
     * Check the configuration status.
     *  
     * @return true iff configured successfully.
     */
    virtual bool isConfigured() const { return configured; }

    /**
     * Start off the estimation procedure.
     *  
     * @return true iff started successfully.
     */
    virtual bool startEstimation() { return start(); }

    /**
     * Check the current estimation status.
     *  
     * @return true when |e_mean|<e_thres.
     */
    virtual bool isDone();

    /**
     * Wait until the condition |e_mean|<e_thres is met.
     *  
     * @return true iff the condition is met, false if the process 
     *         is stopped before.
     */
    virtual bool waitUntilDone();

    /**
     * Stop the estimation procedure.
     */
    virtual void stopEstimation() { stop(); }

    /**
     * Retrieve the estimation. 
     *  
     * @param results Current positive and negative stiction values 
     *                given as components of a 2x1 vector.
     *  
     * @return true/false on success/failure.
     */
    virtual bool getResults(yarp::sig::Vector &results);

    /**
     * Destructor.
     */
    virtual ~OnlineStictionEstimator() { }
};


/**
* \ingroup Tuning
*
* Online Compensator Design. 
*  
* Tune in an online fashion a controller for a DC motor plant 
* identified by means of \ref OnlineDCMotorEstimator .
*  
* The design of the controller is such that the compensated 
* closed-loop system behaves like the following second order 
* dynamics: 
*  
* \f$ \omega_n^2/\left(s^2+2\zeta\omega_ns+\omega_n^2\right) \f$ 
*  
* This class has three operative modes: one for the plant 
* estimantion, one for the plant validation and one for the 
* stiction estimation. 
*/
class OnlineCompensatorDesign : public yarp::os::RateThread
{
protected:
    OnlineDCMotorEstimator  plant;
    OnlineStictionEstimator stiction;
    Filter                  predictor;

    yarp::dev::IControlMode   *imod;
    yarp::dev::IControlLimits *ilim;
    yarp::dev::IEncoders      *ienc;
    yarp::dev::IPidControl    *ipid;

    yarp::os::Semaphore mutex;
    yarp::os::Event     doneEvent;
    yarp::os::Port      port;

    yarp::sig::Vector x0;
    double            P0;

    int    joint;
    double t0,dpos_dV;
    double x_min,x_max,x_tg;
    double max_time,max_pwm;
    bool   pwm_pos;
    bool   configured;

    enum
    {
        plant_estimation,
        plant_validation,
        stiction_estimation
    } mode;

    void controlJoint(double &enc, double &u);
    bool threadInit();    
    void run();
    void threadRelease();

    // prevent user from calling them directly
    bool start();
    void stop();

public:
    /**
     * Default constructor.
     */
    OnlineCompensatorDesign();

    /**
     * Configure the design.
     *  
     * @param driver the device driver to control the robot part.
     * @param options the configuration options. 
     *  
     * Available options are to be given within the following 
     * groups: 
     *  
     * <b>[general]</b> 
     *  
     * @b joint <int>: specify the joint to be controlled. 
     *  
     * @b port <string>: if given, specify the name of a yarp port
     *    to open in order to stream out relevant information.
     *  
     * <b>[plant_estimation]</b>
     *  
     * @b Ts <double>: specify the estimator sample time given in 
     *    seconds.
     *  
     * @b Q <double>: specify the process noise covariance. 
     *  
     * @b R <double>: specify the measurement noise covariance. 
     *  
     * @b P0 <double>: specify the initial error covariance. 
     *  
     * @b tau <double>: specify the initial mechanical time constant 
     *    given in seconds.
     *  
     * @b K <double>: specify the initial plant gain. 
     *  
     * @b max_pwm <double>: specify the amplitude of the square 
     *    voltage waveform applied to the joint for identification
     *    purpose.
     *  
     * <b>[stiction_estimation]</b>
     *  
     * see \ref OnlineStictionEstimator for a detailed description 
     * of available options. 
     *  
     * @note the @b joint option is here overidden by the one 
     *       specified within the [general] group.
     *  
     * @return true/false on success/failure. 
     */
    virtual bool configure(yarp::dev::PolyDriver &driver, const yarp::os::Property &options);

    /**
     * Check the configuration status.
     *  
     * @return true iff configured successfully.
     */
    virtual bool isConfigured() const { return configured; }

    /**
     * Start off the plant estimation procedure. 
     *  
     * @param options property containing the estimantion options. 
     *                Available otions are: (@b max_time <double>)
     *                specifies the maximum amount of time for the
     *                experiment.
     *  
     * @note if active, the yarp port streams out, respectively, the
     *       commanded voltage, the actual encoder value and the 4D
     *       internal state of the estimator.
     *  
     * @return true iff started successfully.
     */
    virtual bool startPlantEstimation(const yarp::os::Property &options);

    /**
     * Start off the plant validation procedure. 
     *  
     * @param options property containing the validation options. 
     *                Available otions are: (@b max_time <double>)
     *                specifies the maximum amount of time for the
     *                experiment; (@b tau <double>) specifies the
     *                mechanical time constant of the plant to be
     *                validated; (@b K <double>) specifies the plant
     *                gain to be validated.
     *  
     * @note if active, the yarp port streams out, respectively, the 
     *       commanded voltage, the actual encoder value and the
     *       predicted plant response. Zero-padding allows being
     *       consistent with the data size.
     *  
     * @return true iff started successfully.
     */
    virtual bool startPlantValidation(const yarp::os::Property &options);

    /**
     * Start off the stiction estimation procedure. 
     *  
     * @param options property containing the estimantion options. 
     *                Available otions are: (@b max_time <double>)
     *                specifies the maximum amount of time for the
     *                experiment.
     *  
     * @note if active, the yarp port streams out the stiction 
     *       values.
     *  
     * @return true iff started successfully.
     */
    virtual bool startStictionEstimation(const yarp::os::Property &options);

    /**
     * Check the status of the current ongoing operation.
     *  
     * @return true iff ongoing operation is finished.
     */
    virtual bool isDone();

    /**
     * Wait until the current ongoing operation is accomplished.
     *  
     * @return true iff ongoing operation is finished.
     */
    virtual bool waitUntilDone();

    /**
     * Stop any ongoing operation.
     */
    virtual void stopOperation() { stop(); }

    /**
     * Retrieve the results of the current ongoing operation.
     *  
     * @param results property object containing the results 
     *                depending on the current ongoing operation:
     *                while estimating the plant results is (@b tau
     *                <double>) (@b K <double>); while validating
     *                the plant results is (@b position <double>);
     *                while estimating the stiction values results
     *                is (@b stiction (<double> <double>)).
     *  
     * @return true/false on success/failure. 
     */
    virtual bool getResults(yarp::os::Property &results);

    /**
     * Destructor.
     */
    virtual ~OnlineCompensatorDesign();
};

}

}

#endif



