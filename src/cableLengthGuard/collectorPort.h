#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>

using namespace yarp::os;
using namespace yarp;

class collectorPort:public Thread
{
private:
  Semaphore mutex;
  BufferedPort<Bottle> *port;
  double *theta;
  bool   *received;
  int nJnts;
  public:
  collectorPort(int n);
  void open(const char *name);
  void close();
  void run();
  bool get(double &th, int i);
  void reset();
};
