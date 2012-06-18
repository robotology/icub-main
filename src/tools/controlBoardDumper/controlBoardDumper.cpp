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


void controlBoardDumper::setDevice(PolyDriver *board_d, PolyDriver *debug_d, int rate, ConstString portPrefix, ConstString dataToDump)
{
  // open ports
  board_dd=board_d;
  debug_dd=debug_d;

  //getter = NULL;

  bool ok;
  ok  = board_d->view(pos);
  ok &= board_d->view(vel);
  ok &= board_d->view(enc);
  ok &= board_d->view(pid);
  ok &= board_d->view(amp);
  ok &= board_d->view(lim);
  ok &= board_d->view(trq);
  ok &= debug_d->view(idbg);

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
    getter   = 0;
    board_dd = 0;
    debug_dd = 0;
    port     = 0;
    pos      = 0;
    vel      = 0;
    enc      = 0;
    pid      = 0;
    amp      = 0;
    lim      = 0;
    trq      = 0;
    idbg     = 0;
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
