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

#include <ace/Assert.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/sig/Vector.h>

#include <iCub/perception/perceptiveModels.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::perception;


/************************************************************************/
bool SensorInterface::configure(void *implementation, const Property &options)
{
    Property &opt=const_cast<Property&>(options);

    ACE_ASSERT(implementation!=NULL);
    ACE_ASSERT(options.check("type"));
    ACE_ASSERT(options.check("size"));
    ACE_ASSERT(options.check("idx"));

    this->implementation=implementation;
    type=options.find("type").asString().c_str();
    size=options.find("size").asInt();
    idx=options.find("idx").asInt();

    return configured=true;
}


/************************************************************************/
bool SensorInterface::getInput(Value &val) const
{
    if (!configured)
        return false;

    Vactor vect(size);

    if (type=="pos")
    {        
        static_cast<IPositionControl*>(implementation)->getEncoders(vect.data());
        val=Value(vect[idx]);

        return true;
    }
    else if (type=="vel")
    {
        static_cast<IVelocityControl*>(implementation)->getEncoders(vect.data());
        val=Value(vect[idx]);

        return true;
    }
    else
        return false;
}


/************************************************************************/
SensorPort::SensorPort()
{
    val=Value;
}


/************************************************************************/
bool SensorPort::configure(void *implementation, const Property &options)
{
    Property &opt=const_cast<Property&>(options);

    ACE_ASSERT(implementation!=NULL);
    ACE_ASSERT(options.check("idx"));

    this->implementation=implementation;
    idx=options.find("idx").asInt();

    return configured=true;
}


/************************************************************************/
bool SensorPort::getInput(Value &val) const
{
    if (!configured)
        return false;

    Bottle *data=static_cast<BufferedPort<Bottle>*>(implementation)->read(false);
    if (data!=NULL)
        this->val=data->get(idx);

    val=this->val;

    return true;
}




