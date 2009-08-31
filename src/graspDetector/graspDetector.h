// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

//#include <yarp/os/RateThread.h>
#include "fingerDetector.h"

using namespace yarp::os;

class graspDetector: public RateThread
{
 public:
    graspDetector(int, fingerDetector **, int );
    ~graspDetector();
    bool threadInit();
    void stop();
    void run();

 private:
    int nFingers;
    fingerDetector  **fd;
};
