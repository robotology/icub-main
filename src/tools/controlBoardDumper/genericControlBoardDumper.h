#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PreciselyTimed.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp;

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

class GetEncs : public GetData
{
public:
  void setInterface (IEncoders *);
  virtual bool getData(double *);

  IEncoders *iencs;
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

class GetErrs : public GetData
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
