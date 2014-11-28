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

void GetPidRefs::setInterface(IPidControl *i)
{
    ipid = i;
}

bool GetPidRefs::getData(double *e)
{
  //fprintf(stderr, "Entering getPosErrs\n");
  if (ipid)
    {
	  ipid->getReferences(e);
      return 1;
    }
  else
    return 0;
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

void GetControlModes::setInterface(IControlMode2 *i, int n_joints)
{
    icmd = i;
    nj=n_joints;
}

bool GetControlModes::getData(double *e)
{
  //fprintf(stderr, "Entering getTrqs\n");
  int tmp [50];
  if (icmd)
    {
      icmd->getControlModes(tmp);
      for (int i=0; i<nj; i++)
      {
          if      (tmp[i] == VOCAB_CM_POSITION)        e[i] = 0;
          else if (tmp[i] == VOCAB_CM_POSITION_DIRECT) e[i] = 1;
          else if (tmp[i] == VOCAB_CM_VELOCITY)        e[i] = 2;
          else if (tmp[i] == VOCAB_CM_MIXED)           e[i] = 3;
          else if (tmp[i] == VOCAB_CM_TORQUE)          e[i] = 4;
          else if (tmp[i] == VOCAB_CM_OPENLOOP)        e[i] = 5;
          else if (tmp[i] == VOCAB_CM_IDLE)            e[i] = 6;
          else                                         e[i] =-1;
      }
      return 1;
    }
  else
    return 0;
}

void GetInteractionModes::setInterface(IInteractionMode *i, int n_joints)
{
    iint = i;
    nj=n_joints;
}

bool GetInteractionModes::getData(double *e)
{
  //fprintf(stderr, "Entering getTrqs\n");
  yarp::dev::InteractionModeEnum tmp[50];
  if (iint)
    {
      iint->getInteractionModes(tmp);
      for (int i=0; i<nj; i++)
      {
          if      (tmp[i] == VOCAB_IM_STIFF)     e[i] = 0;
          else if (tmp[i] == VOCAB_IM_COMPLIANT) e[i] = 1;
          else                                   e[i] =-1;
      }
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

void GetTrqRefs::setInterface(ITorqueControl *i)
{
    itrq = i;
}

bool GetTrqRefs::getData(double *e)
{
  //fprintf(stderr, "Entering getTrqRefs\n");
  if (itrq)
    {
        itrq->getRefTorques(e);
      return 1;
    }
  else
    return 0;
}

void GetRotorPosition::setInterface(IDebugInterface *i)
{
    idbg = i;
}

bool GetRotorPosition::getData(double *e)
{
  //fprintf(stderr, "Entering getTrqs\n");
  if (idbg)
    {
//      for (int i=0; i<6; i++) idbg->getRotorPosition(i,&(e[i]));
      idbg->getRotorPositions(e);
      return 1;
    }
  else
    return 0;
}

void GetRotorSpeed::setInterface(IDebugInterface *i)
{
    idbg = i;
}

bool GetRotorSpeed::getData(double *e)
{
  //fprintf(stderr, "Entering getTrqErrs\n");
  if (idbg)
    {
//      for (int i=0; i<6; i++) idbg->getRotorSpeed(i,&(e[i]));
      idbg->getRotorSpeeds(e);
      return 1;
    }
  else
    return 0;
}

void GetRotorAcceleration::setInterface(IDebugInterface *i)
{
    idbg = i;
}

bool GetRotorAcceleration::getData(double *e)
{
  //fprintf(stderr, "Entering getTrqs\n");
  if (idbg)
    {
//      for (int i=0; i<6; i++) idbg->getRotorAcceleration(i,&(e[i]));
      idbg->getRotorAccelerations(e);
      return 1;
    }
  else
    return 0;
}

