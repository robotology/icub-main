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

#include "genericControlBoardDumper.h"

GetData::GetData()
{
  myIstmp=NULL;
}

bool GetData::getStamp(Stamp &stmp)
{
  if (myIstmp != NULL)
    {
      stmp = myIstmp->getLastInputStamp();
      return true;
    }
  else
    {
      return false;
    }
}

void GetData::setStamp(IPreciselyTimed *iStmp)
{
  myIstmp=iStmp;
}

void GetEncs::setInterface(IEncoders *i)
{
    iencs = i;
}

bool GetEncs::getData(double *e)
{
  //fprintf(stderr, "Entering getEncs\n");
  if (iencs)
    {
      while(!iencs->getEncoders(e))
	fprintf(stderr, "Getting bad encoders! \n");
      return 1;
    }
  else
    {
      fprintf(stderr, "Interface is not ready! \n");
      return 0;
    }
}

void GetSpeeds::setInterface(IEncoders *i)
{
    iencs = i;
}

bool GetSpeeds::getData(double *e)
{
  //fprintf(stderr, "Entering getSpeeds\n");
  if (iencs)
    {
      iencs->getEncoderSpeeds(e);
      return 1;
    }
  else
    return 0;
}

void GetAccs::setInterface(IEncoders *i)
{
    iencs = i;
}

bool GetAccs::getData(double *e)
{
  //fprintf(stderr, "Entering getAccs\n");
  if (iencs)
    {
      iencs->getEncoderAccelerations(e);
      return 1;
    }
  else
    return 0;
}

void GetPosErrs::setInterface(IPidControl *i)
{
    ipid = i;
}

bool GetPosErrs::getData(double *e)
{
  //fprintf(stderr, "Entering getPosErrs\n");
  if (ipid)
    {
      ipid->getErrors(e);
      return 1;
    }
  else
    return 0;
}

void GetOuts::setInterface(IPidControl *i)
{
    ipid = i;
}

bool GetOuts::getData(double *e)
{
  //fprintf(stderr, "Entering getOuts\n");
  if (ipid)
    {
      ipid->getOutputs(e);
      return 1;
    }
  else
    return 0;
}

void GetCurrs::setInterface(IAmplifierControl *i)
{
    iamp = i;
}

bool GetCurrs::getData(double *e)
{
  //fprintf(stderr, "Entering getOuts\n");
  if (iamp)
    {
      iamp->getCurrents(e);
      return 1;
    }
  else
    return 0;
}

void GetTrqs::setInterface(ITorqueControl *i)
{
    itrq = i;
}

bool GetTrqs::getData(double *e)
{
  //fprintf(stderr, "Entering getTrqs\n");
  if (itrq)
    {
	  itrq->getTorques(e);
      return 1;
    }
  else
    return 0;
}

void GetTrqErrs::setInterface(ITorqueControl *i)
{
    itrq = i;
}

bool GetTrqErrs::getData(double *e)
{
  //fprintf(stderr, "Entering getTrqErrs\n");
  if (itrq)
    {
	  itrq->getTorqueErrors(e);
      return 1;
    }
  else
    return 0;
}

