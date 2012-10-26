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
*/
class OnlineDCMotorParametersEstimator
{
protected:

public:
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



