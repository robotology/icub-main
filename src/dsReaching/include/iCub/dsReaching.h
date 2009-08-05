// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2007 Micha Hersch, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   <firstname.secondname>@robotcub.org
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
// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef __DSREACHING_H__
#define __DSREACHING_H__

#include "iCub/Reaching.h"
#include "iCub/Timer.h"
#include <yarp/os/all.h>

#define NB_JOINTS 4
#define PORT_CONTROL



using namespace yarp::os;

#ifndef PORT_CONTROL
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
using namespace yarp::dev;
#endif



/**
 * Module that produces reaching movements using the shoulder and elbow joints
 *
 */


class dsReaching:public Module{

 protected:  

 

    /**
     * The core of this class, attractor based dynamical system
     */
  Reaching reach;
  /**
   * port where 3d target coordinates are sent
   */
  BufferedPort<Bottle> targetPort;

  /**
   * port commands are sent.
   * Accepted commands include:
   * Pause
   * Resume
   * Reset <angle_list>
   */

  BufferedPort<Bottle> commandPort;

#ifdef PORT_CONTROL
  /**
   * port where the modules outputs the arm position
   */
  BufferedPort<Bottle> outputPort;
  
#else
    PolyDriver *armDriver;
    IPositionControl *posCtl;
#endif
    Timer targetTimer;
    Timer commandTimer;
    Timer outputTimer;

    bool paused;
protected:

    bool readCommandPort();
    void readTargetPort();
#ifndef PORT_CONTROL
    bool initMotorControl();
#endif
 public:
    dsReaching();
  bool readPorts();
  void postOutput();
  bool updateModule();
  bool open(Searchable& config);
  bool close();
    //  bool configure(Searchable& config);

};

#endif
