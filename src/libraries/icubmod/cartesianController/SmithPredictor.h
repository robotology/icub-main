/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#ifndef __SMITHPREDICTOR_H__
#define __SMITHPREDICTOR_H__

#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

#include <iCub/ctrl/pids.h>
#include <iCub/ctrl/filters.h>
#include <iCub/iKin/iKinFwd.h>

#include <deque>


// this class implements the Smith Predictor block
// which accounts for the communication delays that
// exist between the velocity commands and their
// execution on the robot
class SmithPredictor
{
protected:
    iCub::ctrl::Integrator *I;
    std::deque<iCub::ctrl::Filter*> F;
    std::deque<std::deque<double>*> tappedDelays;
    bool enabled;

    void dealloc();

public:
    SmithPredictor();
    ~SmithPredictor();
    void configure(yarp::os::Property &options, iCub::iKin::iKinChain &chain);
    void restart(const yarp::sig::Vector &y0);
    yarp::sig::Vector computeCmd(const yarp::sig::Vector &u);
};

#endif

