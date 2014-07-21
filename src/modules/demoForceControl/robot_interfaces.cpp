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

#include "robot_interfaces.h"
#include <string>

using namespace yarp::os;
using namespace yarp::dev;

robot_interfaces::robot_interfaces()
{
    for (int i=0; i<5; i++)
    {
        ipos[i]=0;
        itrq[i]=0;
        iimp[i]=0;
        icmd[i]=0;
        ienc[i]=0;
        ipid[i]=0;
        ivel[i]=0;
        iamp[i]=0;
        iint[i]=0;
        dd[i]=0;
    }
}

void robot_interfaces::init(std::string robot)
{
    std::string part;
    std::string localPort;
    std::string remotePort;

    for (int i=0; i<5; i++)
    {
        switch (i) 
        {
            case LEFT_ARM:     part = "left_arm";   break;
            case RIGHT_ARM:    part = "right_arm";  break;
            case LEFT_LEG:     part = "left_leg";   break;
            case RIGHT_LEG:    part = "right_leg";  break;
            case TORSO:        part = "torso";      break;
        }

        localPort  = "/demoForceControl/" + part;
        remotePort = "/" + robot + "/" + part;
        options[i].put("robot",robot.c_str());
        options[i].put("part",part.c_str());
        options[i].put("device","remote_controlboard");
        options[i].put("local",localPort.c_str());
        options[i].put("remote",remotePort.c_str());

        dd[i] = new PolyDriver(options[i]);
        if(!dd[i] || !(dd[i]->isValid()))
        {
            fprintf(stderr,"Problems instantiating the device driver %d\n", i);
            continue;
        }

        bool ok = true;
        ok = ok & dd[i]->view(ipos[i]);
        ok = ok & dd[i]->view(itrq[i]);
        ok = ok & dd[i]->view(iimp[i]);
        ok = ok & dd[i]->view(icmd[i]);
        ok = ok & dd[i]->view(ivel[i]);
        ok = ok & dd[i]->view(ienc[i]);
        ok = ok & dd[i]->view(ipid[i]);
        ok = ok & dd[i]->view(iamp[i]);
        ok = ok & dd[i]->view(iint[i]);
    }
}


