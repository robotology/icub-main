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

#include <iCub/perception/sensors.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::perception;


/************************************************************************/
Sensor::Sensor()
{
    name="";
    source=NULL;
    configured=false;    
}


/************************************************************************/
void SensorInterface::configure(void *source, const Property &options)
{
    Property &opt=const_cast<Property&>(options);

    ACE_ASSERT(source!=NULL);
    ACE_ASSERT(options.check("name"));
    ACE_ASSERT(options.check("type"));
    ACE_ASSERT(options.check("size"));
    ACE_ASSERT(options.check("idx"));

    this->source=source;
    name=options.find("name").asString().c_str();
    type=options.find("type").asString().c_str();
    size=options.find("size").asInt();
    idx=options.find("idx").asInt();

    configured=true;
}


/************************************************************************/
bool SensorInterface::getInput(Value &in) const
{
    if (!configured)
        return false;    

    if (type=="pos")
    {
        Vactor vect(size);
        static_cast<IPositionControl*>(source)->getEncoders(vect.data());
        in=Value(vect[idx]);

        return true;
    }
    else
        return false;
}


/************************************************************************/
void SensorPort::configure(void *source, const Property &options)
{
    Property &opt=const_cast<Property&>(options);

    ACE_ASSERT(source!=NULL);
    ACE_ASSERT(options.check("name"));
    ACE_ASSERT(options.check("idx"));

    this->source=source;
    name=options.find("name").asString().c_str();
    idx=options.find("idx").asInt();

    configured=true;
}


/************************************************************************/
bool SensorPort::getInput(Value &in) const
{
    if (configured)
    {
        Bottle *data=static_cast<BufferedPort<Bottle>*>(source)->read(false);
        if (data!=NULL)
            val=data->get(idx);

        in=val;

        return true;
    }
    else
        return false;
}




