/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Marco Randazzo
  * email: marco.randazzo@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  *http://www.robotcub.org/icub/license/gpl.txt
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
#include <string>

#define LEFT_ARM  0
#define RIGHT_ARM 1
#define LEFT_LEG  2
#define RIGHT_LEG 3
#define TORSO     4

class robot_interfaces
{
    public:

    yarp::dev::IPositionControl     *ipos[5];
    yarp::dev::ITorqueControl       *itrq[5];
    yarp::dev::IImpedanceControl    *iimp[5];
    yarp::dev::IControlMode2        *icmd[5];
    yarp::dev::IInteractionMode     *iint[5];
    yarp::dev::IEncoders            *ienc[5];
    yarp::dev::IPidControl          *ipid[5];
    yarp::dev::IVelocityControl     *ivel[5];
    yarp::dev::IAmplifierControl    *iamp[5];

    yarp::os::Property              options[5];
    yarp::dev::PolyDriver           *dd[5];

    robot_interfaces();
    void init(std::string robotName="icub");
};

#endif

