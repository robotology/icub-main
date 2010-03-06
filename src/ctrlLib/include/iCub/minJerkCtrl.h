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

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <iCub/filters.h>


namespace ctrl
{

/**
* \ingroup minJerkCtrl
*
* Implements a minimum-jerk controller with velocity command in 
* the assumption that the plant can be idealized as a pure 
* integrator 1/s. 
*/
class minJerkVelCtrl
{
private:
    // Default constructor: not implemented.
    minJerkVelCtrl();

protected:
    yarp::sig::Vector num;
    yarp::sig::Vector den;
    ctrl::Filter *F;

    double Ts;
    double T;

    virtual void computeCoeffs();

public:
    /**
    * Constructor. 
    * @param _Ts is the controller sample time in seconds. 
    * @param x0 is the initial position. 
    */
    minJerkVelCtrl(const double _Ts, const yarp::sig::Vector &x0);

    /**
    * Computes the command.
    * @param _T the current execution time.
    * @param e the error between the desired position and the 
    *          feedback.
    * @return the command.
    */
    virtual yarp::sig::Vector computeCmd(const double _T, const yarp::sig::Vector &e);

    /**
    * Resets the internal state. 
    * @param fb the new state. 
    */
    virtual void reset(const yarp::sig::Vector &fb);

    /**
    * Destructor. 
    */
    ~minJerkVelCtrl();
};

}

#endif



