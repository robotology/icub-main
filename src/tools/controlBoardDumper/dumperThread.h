// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Francesco Nori
 * email:  francesco.nori@iit.it
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

#include <string>
#include <cmath>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <yarp/os/RateThread.h>

#include "genericControlBoardDumper.h"

class boardDumperThread: public RateThread
{
public:
  void setDevice(PolyDriver *board_d, PolyDriver *debug_d, int rate, std::string portPrefix, std::string dataToDump, bool logOnDisk);
  boardDumperThread();
  ~boardDumperThread();
  bool threadInit();
  void setThetaMap(int *, int);
  void threadRelease();
  void run();
  void setGetter(GetData *);
    
private:
  PolyDriver *board_dd;
  PolyDriver *debug_dd;
  GetData *getter;
  Stamp stmp;

  std::string portName;
  Port *port;
  FILE * logFile;
  bool   logToFile;

  IPositionControl *pos;
  IVelocityControl *vel;
  IEncoders *enc;
  IMotorEncoders *imotenc;
  IPidControl *pid;
  IAmplifierControl *amp;
  IControlLimits *lim;
  ITorqueControl *trq;
  IControlMode   *cmod;
  IInteractionMode *imod;
  IMotor          *imot;

  int numberOfJoints;
  double *data;

  int numberOfJointsRead;
  double *dataRead;
  int    *dataMap;

};

