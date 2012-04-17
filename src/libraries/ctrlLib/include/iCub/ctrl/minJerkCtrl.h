/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * website: www.robotcub.org
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
 * \defgroup minJerkCtrl minJerkCtrl
 *  
 * @ingroup ctrlLib
 *
 * Classes for Minimum-Jerk Control
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __MINJERKCTRL_H__
#define __MINJERKCTRL_H__

#include <string>
#include <deque>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <iCub/ctrl/filters.h>


namespace iCub
{

namespace ctrl
{

/**
* \ingroup minJerkCtrl
*
* Abstract class for minimum-jerk controllers with velocity 
* commands.
*/
class minJerkVelCtrl
{
protected:
    virtual void computeCoeffs() = 0;

public:
    /**
    * Computes the velocity command.
    * @param _T the current execution time.
    * @param e the error between the desired position and the 
    *          feedback.
    * @return the velocity command.
    */
    virtual yarp::sig::Vector computeCmd(const double _T, const yarp::sig::Vector &e) = 0;

    /**
    * Resets the controller to a given value.
    * @param u0 the initial output of the controller.
    */
    virtual void reset(const yarp::sig::Vector &u0) = 0;

    /**
    * Destructor. 
    */
    virtual ~minJerkVelCtrl() { }
};


/**
* \ingroup minJerkCtrl
*
* Implements a minimum-jerk controller with velocity commands in
* the assumption that the plant can be modelled as a pure 
* integrator 1/s. 
*/
class minJerkVelCtrlForIdealPlant : public minJerkVelCtrl
{
private:
    // Default constructor: not implemented.
    minJerkVelCtrlForIdealPlant();

protected:
    ctrl::Filter *F;

    double Ts;
    double T;
    int dim;

    virtual void computeCoeffs();

public:
    /**
    * Constructor. 
    * @param _Ts is the controller sample time in seconds. 
    * @param _dim is the controller's dimension 
    */
    minJerkVelCtrlForIdealPlant(const double _Ts, const int _dim);

    /**
    * Computes the velocity command.
    * @param _T the current execution time.
    * @param e the error between the desired position and the 
    *          feedback.
    * @return the velocity command.
    */
    virtual yarp::sig::Vector computeCmd(const double _T, const yarp::sig::Vector &e);

    /**
    * Resets the controller to a given value.
    * @param u0 the initial output of the controller.
    */
    virtual void reset(const yarp::sig::Vector &u0);

    /**
    * Destructor. 
    */
    virtual ~minJerkVelCtrlForIdealPlant();
};


/**
* \ingroup minJerkCtrl
*
* Implements a minimum-jerk controller with velocity commands 
* assuming a non ideal plant represented with a pure integrator 
* followed by a system with one zero and two poles (real or 
* underdamped), such that the overall transfer function is 
* (1/s)*(Kp*(1+Tz*s)/(1+2*Zeta*Tw*s+(Tw*s)^2)).
*/
class minJerkVelCtrlForNonIdealPlant : public minJerkVelCtrl
{
private:
    // Default constructor: not implemented.
    minJerkVelCtrlForNonIdealPlant();

protected:
    yarp::sig::Vector Kp;
    yarp::sig::Vector Tz;
    yarp::sig::Vector Tw;
    yarp::sig::Vector Zeta;
    std::deque<ctrl::Filter*> F;

    double Ts;
    double T;
    int dim;

    virtual void computeCoeffs();

public:
    /**
    * Constructor. 
    * @param _Ts is the controller sample time in seconds. 
    * @param _dim is the controller's dimension 
    */
    minJerkVelCtrlForNonIdealPlant(const double _Ts, const int _dim);

    /**
    * Computes the velocity command.
    * @param _T the current execution time.
    * @param e the error between the desired position and the 
    *          feedback.
    * @return the velocity command.
    */
    virtual yarp::sig::Vector computeCmd(const double _T, const yarp::sig::Vector &e);

    /**
    * Resets the controller to a given value.
    * @param u0 the initial output of the controller.
    */
    virtual void reset(const yarp::sig::Vector &u0);

    /** 
    * Allows user to assign values to plant parameters.
    * @param parameters contains the set of plant parameters for 
    *                   each dimension in form of a Property object.
    *  
    * Available parameters are: 
    *  
    * \b dimension_# < list>: example (dimension_2 ((Kp 1.0) (Tw 
    *    0.1) ...)), specifies the Kp, Tz, Tw and Zeta parameters
    *    for a given dimension of the plant ("dimension_2" in the
    *    example). Dimensions are 0-based numbers.
    * @param entryTag specifies an entry tag different from 
    *                 "dimension".
    * @param ordering if a not empty Bottle is provided, the 
    *                 ordering is not 0,1,2...dim-1 but the one
    *                 specified by the bottle content.
    */
    virtual void setPlantParameters(const yarp::os::Property &parameters,
                                    const std::string &entryTag="dimension",
                                    const yarp::os::Bottle &ordering=yarp::os::Bottle());

    /** 
    * Allows user to retrieve plant parameters.
    * @param parameters contains the set of plant parameters for 
    *                   each dimension in form of a Property object.
    *  
    * Available parameters are: 
    *  
    * \b dimension_# < list>: example (dimension_2 ((Kp 1.0) (Tw 
    *    0.1) ...)), specifies the Kp, Tz, Tw and Zeta parameters
    *    for a given dimension of the plant ("dimension_2" in the
    *    example). Dimensions are 0-based numbers.
    * @param entryTag specifies an entry tag different from 
    *                 "dimension".
    */
    virtual void getPlantParameters(yarp::os::Property &parameters,
                                    const std::string &entryTag="dimension");

    /**
    * Destructor. 
    */
    virtual ~minJerkVelCtrlForNonIdealPlant();
};


/**
* \ingroup minJerkCtrl
*
* Generator of approximately minimum jerk trajectories.
* The min jerk trajectory is approximated using a 3rd order LTI 
* dynamical system (for more details see <a 
* href="http://pasa.liralab.it/pasapdf/699_Pattacini_etal2010.pdf">Pattacini2010</a>). 
* Position, velocity and acceleration trajectories are computed.
* The main advantage with respect to the standard polynomial form
* is that if the reference value yd changes there is no need to 
* recompute the filter coefficients.
*/
class minJerkTrajGen
{
protected:
    Filter* posFilter;          // filter used to compute the position
    Filter* velFilter;          // filter used to compute the velocity
    Filter* accFilter;          // filter used to compute the acceleration

    yarp::sig::Vector pos;      // current position
    yarp::sig::Vector vel;      // current velocity
    yarp::sig::Vector acc;      // current acceleration

    double Ts;                  // sample time in seconds
    double T;                   // trajectory reference time in seconds
    unsigned int dim;           // dimension of the controlled variable

    void computeCoeffs();       // compute the filter coefficients

public:
    /**
    * Constructor.
    * @param _dim number of variables
    * @param _Ts sample time in seconds
    * @param _T trajectory reference time (90% of steady-state value in t=_T, transient extinguished for t>=1.5*_T)
    */
    minJerkTrajGen(const unsigned int &_dim, const double &_Ts, const double &_T);

    /**
    * Constructor with initial value.
    * @param _y0 initial value of the trajectory
    * @param _Ts sample time in seconds
    * @param _T trajectory reference time (90% of steady-state value in t=_T, transient extinguished for t>=1.5*_T)
    */
    minJerkTrajGen(const yarp::sig::Vector& y0, const double &_Ts, const double &_T);

    /**
    * Copy constructor.
    */
    minJerkTrajGen(const minJerkTrajGen& o);

    /**
    * Assignment operator.
    */
    minJerkTrajGen& operator=(const minJerkTrajGen& z);

    /**
    * Destructor. 
    */
    ~minJerkTrajGen();

    /**
    * Initialize the trajectory.
    * @param y0 initial value of the trajectory
    */
    void init(const yarp::sig::Vector& y0);

    /**
    * Compute the next position, velocity and acceleration.
    * @param yd desired final value of the trajectory
    */
    void computeNextValues(const yarp::sig::Vector &yd);

    /**
    * Get the current position.
    */
    const yarp::sig::Vector& getPos() const { return pos; }
    
    /**
    * Get the current velocity.
    */
    const yarp::sig::Vector& getVel() const { return vel; }

    /**
    * Get the current acceleration.
    */
    const yarp::sig::Vector& getAcc() const { return acc; }

    /**
    * Get the trajectory reference time in seconds
    * (90% of steady-state value in t=_T, transient extinguished for t>=1.5*_T).
    */
    const double& getT() const { return T; }

    /**
    * Get the sample time in seconds.
    */
    const double& getTs() const { return Ts; }

    /**
    * Set the trajectory reference time
    * (90% of steady-state value in t=_T, transient extinguished for t>=1.5*_T).
    * @param _T trajectory reference time in seconds
    * @return true if operation succeeded, false otherwise
    */
    bool setT(const double &_T);

    /**
    * Set the sample time.
    * @param _Ts sample time in seconds
    * @return true if operation succeeded, false otherwise
    */
    bool setTs(const double &_Ts);
};


}

}

#endif



