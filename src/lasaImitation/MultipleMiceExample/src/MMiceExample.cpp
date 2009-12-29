// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 Eric Sauser, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   eric.sauser@a3.epfl.ch
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

#include <unistd.h>
#include <iostream>
#include "MultipleMiceDriver/MMiceDeviceDriver.h"

//using namespace yarp::dev;
using namespace std;

int main(int argc, char *argv[]){


  MMiceDeviceDriver mdriver;

  // 3D Mice example
  mdriver.open();
  mdriver.SetBaseMiceName("3Dconnexion SpaceNavigator");
  mdriver.SetMode(MMiceDeviceDriver::MMM_RELTOABS);

  MMiceDeviceDriver mdriver2;

  // Touchpads example
  mdriver2.open();
  mdriver2.SetBaseMiceName("Cirque Corporation USB GlidePoint");

  while(1){
    mdriver2.Update();
    if(mdriver2.HasMouseChanged(0)){
      cout << "Touchpad touched..."<<endl; 
      //mdriver2.PrintSummary2();
    }
    mdriver.Update();
    if(mdriver.HasMouseChanged(0)){
      cout << "3D mouse touched..."<<endl;
      //mdriver.PrintSummary2();
    }
    usleep(10000);
  }

  return 0; 

}

