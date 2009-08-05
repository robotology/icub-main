#include <stdio.h>
#include <string.h>
#include <yarp/os/Port.h>

#include <ace/OS.h>
#include <ace/Log_Msg.h>
#include <yarp/os/Property.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/String.h> 
#include <yarp/os/Thread.h>

//using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp;

class collectorPort:public Thread
{
private:

  Port *portMiddle;
  Port *portOutput;

  bool continueCommunication;

  char *middlePortName;
  char *clientPortName;
  char *serverPortName;
public:
  collectorPort();
  void open(const char *middle, const char *client, const char *server, const char* output);
  void close();
  void run();
};
