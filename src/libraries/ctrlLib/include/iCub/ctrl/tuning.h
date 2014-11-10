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
 * \defgroup Tuning Tuning of Compensators
 *  
 * @ingroup ctrlLib
 *
 * Classes for Compensators tuning.
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
    double uOld;
    double Ts;
    double R;

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
* Estimate the up and down stiction values. \n 
* During the experiment, the joint is controlled by a high-level 
* pid controller that commands directly the voltage in order to 
* track a time varying reference position. The stiction values 
* are estimated during the rising and falling edge transitions. 
*/
class OnlineStictionEstimator : public yarp::os::RateThread
{
protected:
    yarp::dev::IControlMode2  *imod;
    yarp::dev::IControlLimits *ilim;
    yarp::dev::IEncoders      *ienc;
    yarp::dev::IPidControl    *ipid;

    yarp::os::Mutex    mutex;
    yarp::os::Event    doneEvent;
    yarp::sig::Vector  gamma;
    yarp::sig::Vector  stiction;
    yarp::os::Property info;
    yarp::sig::Vector  done;

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
    double Kp,Ki,Kd;
    double vel_thres,e_thres;
    double tg,xd_pos;
    double stiction_limit;
    bool   adapt,adaptOld;
    bool   configured;

    enum
    {
        rising,
        falling
    } state;

    void applyStictionLimit();
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
     * @b Kp <double>: specify the proportional term of the 
     *    high-level controller for tracking purpose.
     *  
     * @b Ki <double>: specify the integral term of the high-level 
     *    controller for tracking purpose.
     *  
     * @b Kd <double>: specify the derivative term of the 
     *    high-level controller for tracking purpose.
     *  
     * @b vel_thres <double>: specify the velocity threshold (in 
     *    degrees) used to identify the estimation time windows. The
     *    estimation is carried out whenever |vel|<vel_thres.
     *  
     * @b e_thres <double>: specify the error threshold (in degrees)
     *    above which keep updating the stiction values. The
     *    estimation is carried out until |e_mean|>e_thres, where
     *    e_mean accounts for the integral average of the error
     *    computed within the estimation time window. Value given in
     *    degrees.
     *  
     * @b gamma (<double> <double>): specify the gains used for 
     *    updating the stiction up and down values, respectively.
     *  
     * @b stiction (<double> <double>): specify the initial stiction
     *    up and down values, respectively.
     *  
     * @return true/false on success/failure. 
     */
    virtual bool configure(yarp::dev::PolyDriver &driver, const yarp::os::Property &options);

    /**
     * Reconfigure the estimation after first initialization.
     *  
     * @param options the configuration options. 
     * @return true/false on success/failure. 
     */
    virtual bool reconfigure(const yarp::os::Property &options);

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
    virtual bool startEstimation() { return RateThread::start(); }

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
    virtual void stopEstimation() { RateThread::stop(); }

    /**
     * Retrieve the estimation. 
     *  
     * @param results Current up and down stiction values given as 
     *                components of a 2x1 vector.
     *  
     * @return true/false on success/failure.
     */
    virtual bool getResults(yarp::sig::Vector &results);

    /**
     * Retrieve useful information about the estimation experiment. 
     *  
     * @param info the property containing the info. Available info 
     *             are: (@b voltage <double>) which specifies the
     *             commanded voltage; (@b reference <double>) which
     *             specifies the position reference; (@b position
     *             <double>) which specifies the actual encoder
     *             value.
     *  
     * @return true/false on success/failure.
     */
    virtual bool getInfo(yarp::os::Property &info);

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
* The design of the controller is such that the properties of 
* the compensated closed-loop system comply with specifications 
* given in terms of bandwidth and disturbance rejection. 
*  
* This class has four operative modes: one for the plant 
* estimation, one for the plant validation, one for the stiction
* estimation and one for validating the controller's design.
*/
class OnlineCompensatorDesign : public yarp::os::RateThread
{
protected:
    OnlineDCMotorEstimator  plant;
    OnlineStictionEstimator stiction;
    Kalman                  predictor;

    yarp::dev::IControlMode2    *imod;
    yarp::dev::IControlLimits   *ilim;
    yarp::dev::IEncoders        *ienc;
    yarp::dev::IPositionControl *ipos;
    yarp::dev::IPidControl      *ipid;
    yarp::dev::Pid              *pidCur;
    yarp::dev::Pid               pidOld;
    yarp::dev::Pid               pidNew;

    yarp::os::Mutex mutex;
    yarp::os::Event doneEvent;
    yarp::os::BufferedPort<yarp::sig::Vector> port;

    yarp::sig::Vector x0;
    yarp::sig::Vector meanParams;
    int               meanCnt;
    double            P0;

    int    joint;
    double t0,t1;
    double x_min,x_max,x_tg;
    double max_time,max_pwm,dpos_dV;
    double switch_timeout;
    int    measure_update_ticks;
    int    measure_update_cnt;    
    bool   controller_validation_ref_square;
    double controller_validation_ref_period;
    double controller_validation_ref_sustain_time;
    int    controller_validation_cycles_to_switch;
    int    controller_validation_num_cycles;
    bool   controller_validation_stiction_yarp;
    double controller_validation_stiction_up;
    double controller_validation_stiction_down;
    bool   pwm_pos;
    bool   configured;

    enum
    {
        plant_estimation,
        plant_validation,
        stiction_estimation,
        controller_validation
    } mode;

    void commandJoint(double &enc, double &u);
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
     * Tune the controller once given the plant characteristics. The
     * design requirements for the closed loop system are given in 
     * terms of bandwidth and properties for disturbance rejection.
     *  
     * The plant is assumed to be in the form: 
     *  
     * \f$ K/\left(s \cdot \left(1+s\tau\right)\right). \f$ 
     *  
     * The controller is in the form: 
     *  
     * \f$ K_p + K_i/s. \f$ 
     *  
     * The tuning is symbolic and by no means affects the gains of 
     * the low-level controller. 
     * 
     * @param options property object containing the plant 
     *                characteristics as well as the design
     *                requirements: (@b tau <double>) (@b K
     *                <double>) (@b f_c <double>) (@b T_dr
     *                <double>); (@b type <string>) specifies the
     *                controller's architecture which can be "P" or
     *                "PI".
     * @param results property containing the design outcome in 
     *             terms of \f$ K_p, K_i \f$ controller's
     *             parameters. The property's tags are respectively:
     *             <b>Kp</b>, <b>Ki</b>.
     * @return true/false on success/failure.
     *  
     * @note When designing a <i>P</i> controller the user is 
     *       required to specify the gain crossover frequency
     *       <b>f_c</b> in Hz that represents the frequency at which
     *       the open loop response has a unity-gain, corresponding
     *       roughly to the closed-loop cut-off frequency regulating
     *       the control bandwidth. \n
     *       When designing a <i>PI</i> controller the integral part
     *       is employed separately for disturbance rejection in
     *       that it tries to cancel out a step-wise disturbance in
     *       a time window specified by <b>T_dr</b> parameter, given
     *       in seconds.
     *  
     * @return true/false on success/failure. 
     */
    virtual bool tuneController(const yarp::os::Property &options, yarp::os::Property &results);    

    /**
     * Start off the plant estimation procedure. 
     *  
     * @param options property containing the estimation options. 
     *                Available otions are: (@b max_time <double>)
     *                specifies the maximum amount of time for the
     *                experiment; (@b switch_timeout <double>) if
     *                greater than 0.0 specifies the timeout for
     *                voltage switching logic.
     *  
     * @note if active, the yarp port streams out, respectively, the
     *       mode id 0, the commanded voltage, the actual encoder
     *       value and the 4D internal state of the estimator as
     *       well as the averaged parameters.
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
     *                experiment; (@b switch_timeout <double>) if
     *                greater than 0.0 specifies the timeout for
     *                voltage switching logic; (@b tau <double>)
     *                specifies the mechanical time constant of the
     *                plant to be validated; (@b K <double>)
     *                specifies the plant gain to be validated; (@b
     *                measure_update_ticks <int>) specifies how many
     *                sample ticks to take before updating the
     *                measurement in the Kalman filter, if <= 0 then
     *                no update is performed; (@b Q <double>) (@b R
     *                <double>) (@b P0 <double>) are the well known
     *                Kalman quantities for the noise statistics.
     *  
     * @note if active, the yarp port streams out, respectively, the
     *       mode id 1, the commanded voltage, the actual encoder
     *       value and the predicted plant response which includes
     *       the estimated position and velocity. Zero-padding
     *       allows being compliant with the data size used for the
     *       plant estimation mode.
     *  
     * @return true iff started successfully.
     */
    virtual bool startPlantValidation(const yarp::os::Property &options);

    /**
     * Start off the stiction estimation procedure. 
     *  
     * @param options property containing the estimation options. 
     *                Available otions are: (@b max_time <double>)
     *                specifies the maximum amount of time for the
     *                experiment; all the other options used while
     *                configuring the stiction estimator.
     *  
     * @note if active, the yarp port streams out, respectively, the
     *       mode id 2, the commanded voltage, the actual encoder
     *       value, the position reference and the the stiction
     *       values. Zero-padding allows being compliant with the
     *       data size used for plant estimation mode.
     *  
     * @return true iff started successfully.
     */
    virtual bool startStictionEstimation(const yarp::os::Property &options);

    /**
     * Start off the controller validation procedure. The specified 
     * controller is put to test against the controller currently 
     * set within the firmware. The validation experiment foresees 
     * cycles of rising and falling transitions in the reference 
     * trajectory. The control is therefore continuously switched 
     * between the current and the new controller.
     *  
     * Once the validation has been carried out, then the low-level
     * controller's paremeters are restored to their previous 
     * values. 
     *  
     * @param options property containing the validation options. 
     *                Available otions are: (@b max_time <double>)
     *                specifies the maximum amount of time for the
     *                experiment; (@b Kp <double>) (@b Ki <double>)
     *                (@b Kd <double>) (@b tau_d <double>) (@b scale
     *                <int>) specify the controller's gains; (@b
     *                stiction (<double><double>)) specifies the
     *                stiction values; (@b stiction_compensation
     *                <string>) specifies whether the compensation
     *                is managed by the "firmware" (default) or the
     *                "middleware"; (@b ref_type <string>) specifies
     *                the waveform of the position reference
     *                ("square"|"min-jerk"); (@b ref_period
     *                <double>) specifies the period of the
     *                reference; (@b ref_sustain_time <double>)
     *                specifies how log (in seconds) the reference
     *                should be kept at the set-point before
     *                switching to next value (meaningful for
     *                min-jerk reference type); (@b cycles_to_switch
     *                <int>) specifies the number of cycles during
     *                which one controller is tested before the
     *                switch.
     *  
     * @note if active, the yarp port streams out, respectively, the
     *       mode id 3, the commanded voltage, the actual encoder
     *       value, the position reference and a flag accounting for
     *       the old pid behavior (0) or the new pid behavior (1).
     *       Zero-padding allows being compliant with the data size
     *       used for plant estimation mode.
     *  
     * @return true iff started successfully.
     */
    virtual bool startControllerValidation(const yarp::os::Property &options);

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
    virtual void stopOperation() { RateThread::stop(); }

    /**
     * Retrieve the results of the current ongoing operation.
     *  
     * @param results property object containing the results 
     *                depending on the current ongoing operation:
     *                while estimating the plant, results is (@b tau
     *                <double>) (@b K <double>) (@b tau_mean
     *                <double>) (K_mean <double>); while validating
     *                the plant, results is (@b position <double>)
     *                (@b velocity <double>); while estimating the
     *                stiction values, results is (@b stiction
     *                (<double> <double>)); while validating the
     *                controller, results is (@b voltage <double>)
     *                (@b reference <double>) (@b position
     *                <double>) (@b pid <string>), where @b pid is
     *                "old"|"new".
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



