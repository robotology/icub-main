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
    int dim;

    virtual void computeCoeffs();

public:
    /**
    * Constructor. 
    * @param _Ts is the controller sample time in seconds. 
    * @param _dim is the controller's dimension 
    */
    minJerkVelCtrl(const double _Ts, const int _dim);

    /**
    * Computes the command.
    * @param _T the current execution time.
    * @param e the error between the desired position and the 
    *          feedback.
    * @return the command.
    */
    virtual yarp::sig::Vector computeCmd(const double _T, const yarp::sig::Vector &e);

    /**
    * Destructor. 
    */
    ~minJerkVelCtrl();
};

}

}

#endif



