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

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PreciselyTimed.h>

using namespace yarp::dev;
using namespace yarp::os;

class GetData
{
private:
  IPreciselyTimed* myIstmp;
public:
  GetData();
  virtual bool getData(double *) = 0;
  bool getStamp(Stamp &);
  void setStamp(IPreciselyTimed*);
  
};

class GetTemps : public GetData
{
public:
  void setInterface (IMotor *);
  virtual bool getData(double *);

  IMotor *imot;
};

class GetMotEncs : public GetData
{
public:
  void setInterface (IMotorEncoders *);
  virtual bool getData(double *);

  IMotorEncoders *imotencs;
};

class GetMotSpeeds : public GetData
{
public:
  void setInterface (IMotorEncoders *);
  virtual bool getData(double *);

  IMotorEncoders *imotencs;
};

class GetMotAccs : public GetData
{
public:
  void setInterface (IMotorEncoders *);
  virtual bool getData(double *);

  IMotorEncoders *imotencs;
};

class GetEncs : public GetData
{
public:
  void setInterface (IEncoders *);
  virtual bool getData(double *);

  IEncoders *iencs;
};

class GetPidRefs : public GetData
{
public:
  void setInterface (IPidControl *);
  virtual bool getData(double *);

  IPidControl *ipid;
};

class GetSpeeds : public GetData
{
public:
  void setInterface (IEncoders *);
  virtual bool getData(double *);

  IEncoders *iencs;
};

class GetAccs : public GetData
{
public:
  void setInterface (IEncoders *);
  virtual bool getData(double *);

  IEncoders *iencs;
};

class GetPosErrs : public GetData
{
public:
  void setInterface (IPidControl *);
  virtual bool getData(double *);

  IPidControl *ipid;
};

class GetOuts : public GetData
{
public:
  void setInterface (IPidControl *);
  virtual bool getData(double *);

  IPidControl *ipid;
};

class GetCurrs : public GetData
{
public:
  void setInterface (IAmplifierControl *);
  virtual bool getData(double *);

  IAmplifierControl *iamp;
};

class GetTrqs : public GetData
{
public:
  void setInterface (ITorqueControl *);
  virtual bool getData(double *);

  ITorqueControl *itrq;
};

class GetControlModes : public GetData
{
public:
  void setInterface (IControlMode *, int joints);
  virtual bool getData(double *);

  IControlMode *icmd;
  int nj;
};

class GetInteractionModes : public GetData
{
public:
  void setInterface (IInteractionMode *, int joints);
  virtual bool getData(double *);

  IInteractionMode *iint;
  int nj;
};

class GetTrqErrs : public GetData
{
public:
  void setInterface (IPidControl *);
  virtual bool getData(double *);

  IPidControl *ipid;
};

class GetTrqRefs : public GetData
{
public:
  void setInterface (ITorqueControl *);
  virtual bool getData(double *);

  ITorqueControl *itrq;
};

class GetMotPwm : public GetData
{
public:
  void setInterface (IAmplifierControl *);
  virtual bool getData(double *);
  int n_joint_part;

  IAmplifierControl *iamp;
};