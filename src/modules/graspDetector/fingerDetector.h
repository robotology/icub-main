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

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <math.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <yarp/os/RateThread.h>

/* itoa example */
#include <stdio.h>
#include <stdlib.h>

using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::os;

class fingerDetector: public RateThread
{
public:
    fingerDetector( int );
    ~fingerDetector();
    bool threadInit();
    void setIndex(Bottle);
    void setModel(Bottle, Bottle, double, double, double, double);
    void copyAnalog(Bottle *);
    void stop();
    void run();
    
private:
    Vector index;
    Vector fingerAnalog;
    Vector q0;
    Vector q1;
    double min;
    double max;
    double minT;
    double maxT;
public:
    double status; //true if grasping (model not respected) 
};


