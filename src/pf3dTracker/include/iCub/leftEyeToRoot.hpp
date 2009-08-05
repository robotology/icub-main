#include <iostream>
#include <string>
#include <sstream>
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/all.h>

#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/math/Math.h>
// #ifdef _CH_
// #pragma package <opencv>
// #endif
// #ifndef _EiC
// #include "cv.h"
// #include "highgui.h"
// #endif

#include <iCub/iKinFwd.h>
using namespace std;
using namespace yarp::os;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;
using namespace yarp::math;
using namespace iKin;






class LeftEyeToRoot : public Module
{

private:
 ConstString _inputHeadPortName;
 ConstString _inputTorsoPortName;
 ConstString _inputBallPositionPortName;
 ConstString _outputBallPositionPortName;
 BufferedPort<Bottle> _inputHeadPort;
 BufferedPort<Bottle> _inputTorsoPort;
 BufferedPort<Bottle> _inputBallPositionPort;
 BufferedPort<Bottle> _outputBallPositionPort;

 bool positionConsideredSafe;
 bool receivedHead;
 bool receivedTorso;
 bool receivedBallPosition;

 iCubEye *eye;
 iKinChain chainEyeL;
 Vector v;
 Matrix transformation;
 Vector eyeBallPosition;
 Vector rootBallPosition;

 double head0,head1,head2,head3,head4,head5;
 double torso0,torso1,torso2;
 double ballPositionInX,ballPositionInY,ballPositionInZ,ballPositionInGood;

 int counter;
public:

LeftEyeToRoot(); //constructor
~LeftEyeToRoot(); //destructor

virtual bool open(Searchable& config); //member to set the object up.
virtual bool close();                  //member to close the object.
virtual bool updateModule();           //member that is repeatedly called by YARP, to give this object a chance to do something.

};


