// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

//#include <yarp/os/RateThread.h>
#include "fingerDetector.h"
#include <yarp/dev/PreciselyTimed.h>

using namespace yarp::os;

class graspDetector: public RateThread
{
 public:
    graspDetector(int, fingerDetector **, Port *, BufferedPort<Bottle>*, int );
    ~graspDetector();
    bool threadInit();
    void threadRelease();
    void run();
    double *s;

 private:
    int nFingers;
    fingerDetector  **fd;
    Port *sp;
    BufferedPort<Bottle> *analogPort;
};
