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

void GetErrs::setInterface(IPidControl *i)
{
    ipid = i;
}

bool GetErrs::getData(double *e)
{
  //fprintf(stderr, "Entering getErrs\n");
  if (ipid)
    {
      ipid->getErrors(e);
      return 1;
    }
  else
    return 0;
}

void GetOuts::setInterface(IOpenLoopControl *i)
{
    iolc = i;
}

bool GetOuts::getData(double *e)
{
  //fprintf(stderr, "Entering getOuts\n");
  if (iolc)
    {
      iolc->getOutputs(e);
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
