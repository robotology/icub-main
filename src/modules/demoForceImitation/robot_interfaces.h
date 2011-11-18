/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Marco Randazzo
  * email: marco.randazzo@iit.it
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

#ifndef ROBOT_INTERFACES_H
#define ROBOT_INTERFACES_H

#include <yarp/dev/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/skinDynLib/common.h>

class robot_interfaces
{
    public:

    yarp::dev::IPositionControl        *ipos[7];
    yarp::dev::ITorqueControl        *itrq[7];
    yarp::dev::IImpedanceControl    *iimp[7];
    yarp::dev::IControlMode            *icmd[7];
    yarp::dev::IEncoders            *ienc[7];
    yarp::dev::IPidControl            *ipid[7];
    yarp::dev::IVelocityControl        *ivel[7];
    yarp::dev::IAmplifierControl    *iamp[7];

    yarp::os::Property                   options[7];
    yarp::dev::PolyDriver           *dd[7];

    robot_interfaces();
    void init();
};

#endif

