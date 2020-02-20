/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#ifndef __PERCEPTIVEMODELS_PORTS_H__
#define __PERCEPTIVEMODELS_PORTS_H__

#include <mutex>

#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>


namespace iCub
{

namespace perception
{

class Port : public yarp::os::BufferedPort<yarp::os::Bottle>
{
protected:
    std::mutex mtx;
    yarp::os::Bottle bottle;

    void onRead(yarp::os::Bottle &bottle);

public:
    Port();
    yarp::os::Value getValue(const int index);
};


}

}

#endif


