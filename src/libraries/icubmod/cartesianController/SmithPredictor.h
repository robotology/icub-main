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
#include <iCub/iKin/iKinFwd.h>

#include <deque>

class SmithPredictor
{
protected:
    iCub::ctrl::Integrator *I;
    yarp::sig::Vector gains;
    std::deque<std::deque<double> > tappedDelays;
    bool enabled;

public:
    SmithPredictor();
    ~SmithPredictor();
    void configure(yarp::os::Property &options, iCub::iKin::iKinChain &chain);
    void init(const yarp::sig::Vector &y0);
    yarp::sig::Vector compute(const yarp::sig::Vector &u);
};

#endif

