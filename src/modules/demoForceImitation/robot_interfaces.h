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
#include <map>
#include <vector>

class robot_interfaces
{
    public:

    std::map<iCub::skinDynLib::BodyPart, yarp::dev::IPositionControl*>     ipos;
    std::map<iCub::skinDynLib::BodyPart, yarp::dev::ITorqueControl*>       itrq;
    std::map<iCub::skinDynLib::BodyPart, yarp::dev::IImpedanceControl*>    iimp;
    std::map<iCub::skinDynLib::BodyPart, yarp::dev::IControlMode2*>        icmd;
    std::map<iCub::skinDynLib::BodyPart, yarp::dev::IInteractionMode*>     iint;
    std::map<iCub::skinDynLib::BodyPart, yarp::dev::IEncoders*>            ienc;
    std::map<iCub::skinDynLib::BodyPart, yarp::dev::IPidControl*>          ipid;
    std::map<iCub::skinDynLib::BodyPart, yarp::dev::IVelocityControl*>     ivel;
    std::map<iCub::skinDynLib::BodyPart, yarp::dev::IAmplifierControl*>    iamp;

    std::map<iCub::skinDynLib::BodyPart, yarp::os::Property>               options;
    std::map<iCub::skinDynLib::BodyPart, yarp::dev::PolyDriver*>           dd;

    // *** CONSTRUCTORS ***
    /** 
     * Default constructor: it considers 5 body parts: arms, legs and torso.
    **/
    robot_interfaces();
    robot_interfaces(iCub::skinDynLib::BodyPart bp1);
    robot_interfaces(iCub::skinDynLib::BodyPart bp1, iCub::skinDynLib::BodyPart bp2);
    robot_interfaces(iCub::skinDynLib::BodyPart bp1, iCub::skinDynLib::BodyPart bp2, iCub::skinDynLib::BodyPart bp3);
    robot_interfaces(iCub::skinDynLib::BodyPart bp1, iCub::skinDynLib::BodyPart bp2, iCub::skinDynLib::BodyPart bp3, 
        iCub::skinDynLib::BodyPart bp4);
    robot_interfaces(iCub::skinDynLib::BodyPart bp1, iCub::skinDynLib::BodyPart bp2, iCub::skinDynLib::BodyPart bp3, 
        iCub::skinDynLib::BodyPart bp4, iCub::skinDynLib::BodyPart bp5);
    robot_interfaces(iCub::skinDynLib::BodyPart *bps, int size);

    bool init();

private:
    std::vector<iCub::skinDynLib::BodyPart> bodyParts;   // list of the body parts for which the interfaces are open    
};

#endif

