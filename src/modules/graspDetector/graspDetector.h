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
