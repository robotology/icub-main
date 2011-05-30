/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
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

#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/perception/perceptiveModels.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::perception;


/************************************************************************/
SensorInterface::SensorInterface(void *interface, const string &type, const int idx)
{
    this->inteface=inteface;
    this->type=type;
    this->idx=idx;
}


/************************************************************************/
bool SensorInterface::getInput(Value &val)
{
    if (type=="ipos")
    {
        Vactor data();
        static_cast<IPositionControl*>(interface)->getEncoders(data.value());
        return true;
    }
    else if (type=="ivel")
    {
        return true;
    }
    else
        return false;
}


