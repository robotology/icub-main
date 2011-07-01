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
#include "controlBoardDumper.h"

/*
 * Copyright (C) 2006 Francesco Nori
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


void controlBoardDumper::setDevice(PolyDriver *Arm_d, int rate, ConstString portPrefix, ConstString dataToDump)
{
  // open ports
  Arm_dd=Arm_d;
  //getter = NULL;

  bool ok;
  ok  = Arm_dd->view(pos);
  ok &= Arm_dd->view(vel);
  ok &= Arm_dd->view(enc);
  ok &= Arm_dd->view(pid);
  ok &= Arm_dd->view(amp);
  ok &= Arm_dd->view(lim);
  ok &= Arm_dd->view(trq);

  if (!ok)
    printf("Problems acquiring interfaces\n");
  else
    printf("Control board was accessed succesfully!\n");

  pos->getAxes(&numberOfJoints);
  fprintf(stderr, "Number of axes is: %d \n", numberOfJoints);

  //initialize used variables
  data = new double [numberOfJoints];

  port = new Port;

  ConstString portName = portPrefix + dataToDump;
  port->open(portName.c_str());

  this->setRate(rate);
}

controlBoardDumper::~controlBoardDumper()
{    

}

void controlBoardDumper::setThetaMap(int *map, int n)
{ 
  fprintf(stderr, "Setting the map dimension %d \n", n);
  numberOfJointsRead = n;
  dataRead = new double [numberOfJointsRead];
  dataMap = new int [numberOfJointsRead];
  for (int i = 0; i < numberOfJointsRead; i++)
    {
      //fprintf(stderr, "map is %d \n", map[i]);
      dataMap[i] = map[i];
      dataRead[i] = 1.0;
    }
}

void controlBoardDumper::setGetter(GetData *g)
{
  getter = g;
}

bool controlBoardDumper::threadInit()
{

  return 1;
}

controlBoardDumper::controlBoardDumper():RateThread(500)
{

}

void controlBoardDumper::threadRelease()
{
  fprintf(stderr, "Closing thread \n");
  //Arm_dd->close();
  Time::delay(.1);
  fprintf(stderr, "Closing ports \n");
  port->close();
}

void controlBoardDumper::run()
{
  //printf("Entering the main thread\n");
  //enc->getEncoders(data);
  //Bottle bData;
  //for (int i = 0; i < numberOfJointsRead; i++)
  //  {
  //    dataRead[i] = data[dataMap[i]];
  //    bData.addDouble(dataRead[i]);
  //  }
  //port->write(bData);

  if (getter)
    {
      //printf("Getter is getting something\n");
      getter -> getData(data);

      //fprintf(stderr, "Time is %lf \n", stmp.getTime());

      Bottle bData;
      for (int i = 0; i < numberOfJointsRead; i++)
	{
	  //printf("%.2f \n", data[dataMap[i]]);
	  dataRead[i] = data[dataMap[i]];
	  bData.addDouble(dataRead[i]);
	}
      if (getter->getStamp(stmp)) 
	{
	  if (stmp.isValid())
	    port->setEnvelope(stmp);
	  else
	    {
            //stmp.update();
            stmp=Stamp(-1,0.0);
	      port->setEnvelope(stmp);
	    }
	}
      else
	fprintf(stderr, "controlBoardDumper::warning. Trying to get a stamp without a proper IPreciselyTimed defined. \n");
      
      port->write(bData);
    }
}  
