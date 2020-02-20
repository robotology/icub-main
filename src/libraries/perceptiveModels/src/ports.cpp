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

using namespace std;
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
    lock_guard<mutex> lck(mtx);
    this->bottle=bottle;
}


/************************************************************************/
Value iCub::perception::Port::getValue(const int index)
{
    lock_guard<mutex> lck(mtx);

    Value ret;
    if ((index>=0) && (index<bottle.size()))
        ret=Value(bottle.get(index).asDouble());
    return ret;
}




