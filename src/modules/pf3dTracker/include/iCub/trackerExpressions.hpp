#include <iostream>
#include <string>
#include <sstream>
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/Drivers.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;





class TrackerExpressions : public Module
{

private:
 ConstString _inputPortName;
 ConstString _outputPortName;
 BufferedPort<Bottle> _inputPort;
 BufferedPort<Bottle> _outputPort;
 int _counter;

public:

TrackerExpressions(); //constructor
~TrackerExpressions(); //destructor

virtual bool open(Searchable& config); //member to set the object up.
virtual bool close();                  //member to close the object.
virtual bool interruptModule();        //member to close the object.
virtual bool updateModule();           //member that is repeatedly called by YARP, to give this object a chance to do something.

};


