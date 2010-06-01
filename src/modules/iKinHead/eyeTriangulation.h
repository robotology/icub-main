#include <gsl/gsl_math.h>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <string>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/filters.h>
#include <iCub/ctrl/kalman.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iKin;
using namespace ctrl;


class eyeTriangulation:public RateThread
{
protected:
  bool xConstant;
  bool enableKalman;
  int kalState;
  double kalTimer;
  double Ts;

  string ctrlName;
  string robotName;

  iKinLink  *alignLnkLeft1,  *alignLnkLeft2;
  iKinLink  *alignLnkRight1, *alignLnkRight2;
  iKinChain *chainRightEye,  *chainLeftEye;
  iCubEye   *rightEye,       *leftEye;

  Matrix T_RoLe;
  Matrix T_RoRe;
  Matrix T_LeRo;
  Matrix T_ReRo;
  Matrix Pr;
  Matrix Pl;
  Matrix invPr;
  Matrix invPl;
  unsigned int nl;
  unsigned int nr;
  Vector q;
  Vector qr;
  Vector ql;
  Vector xr;
  Vector xl;

  Matrix A;
  Vector b;

  RateLimiter *rlim;
  Kalman      *kal;

  Matrix kalA,kalH,kalQ,kalR;
  Matrix kalP0;
  Vector kalx0;

  BufferedPort<Bottle> xlrPort;
  BufferedPort<Bottle> qPortTorso;
  BufferedPort<Bottle> qPortHead;
  BufferedPort<Vector> XPort;

  Bottle qBottleTorso;
  Bottle qBottleHead;

  double eyeDistL;
  double eyeDistR;

  bool getAlignLinks(const ResourceFinder&, const string&, iKinLink**, iKinLink**);
  bool qrGet(Vector);
  bool qlGet(Vector);
  bool xCheckBottleFormat(Bottle*, Vector&, Vector&);  
  
public:
  eyeTriangulation(const ResourceFinder&, Matrix, Matrix, bool, unsigned int, const string&, const string&);
  virtual bool threadInit();
  virtual void afterStart(bool);
  virtual void run();
  virtual void threadRelease();

  void xInit(Vector, Vector);
  void xSetConstant(Vector, Vector);
  void xDisConstant();
  bool xExecReq(const Bottle &req, Bottle &reply);
};


