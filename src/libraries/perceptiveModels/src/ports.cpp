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

#include <iCub/perception/private/ports.h>
#include <yarp/os/Value.h>

using namespace yarp::os;
using namespace iCub::perception;


/************************************************************************/
iCub::perception::Port::Port()
{
    useCallback();
}


/************************************************************************/
void iCub::perception::Port::onRead(Bottle &bottle)
{
    mutex.lock();
    this->bottle=bottle;
    mutex.unlock();
}


/************************************************************************/
void iCub::perception::Port::interrupt()
{
    disableCallback();
    mutex.unlock();
    BufferedPort<Bottle>::interrupt();
}


/************************************************************************/
Value iCub::perception::Port::getValue(const int index)
{
    Value ret;

    mutex.lock();
    if (index<bottle.size())
        ret=Value(bottle.get(index).asDouble());
    mutex.unlock();

    return ret;
}




