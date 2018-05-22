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

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/sig/Vector.h>

#include <iCub/perception/private/ports.h>
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
bool SensorEncoders::configure(void *source, const Property &options)
{
    if ((source==NULL) || !options.check("name") ||
        !options.check("size") || !options.check("index"))
        return false;

    this->source=source;
    name=options.find("name").asString();
    size=options.find("size").asInt();
    index=options.find("index").asInt();

    return configured=true;
}


/************************************************************************/
bool SensorEncoders::getOutput(Value &in) const
{
    if (!configured)
        return false;    

    Vector vect(size);
    static_cast<IEncoders*>(source)->getEncoders(vect.data());
    in=Value(vect[index]);

    return true;
}


/************************************************************************/
bool SensorPort::configure(void *source, const Property &options)
{
    if ((source==NULL) || !options.check("name") || !options.check("index"))
        return false;

    this->source=source;
    name=options.find("name").asString();
    index=options.find("index").asInt();

    return configured=true;
}


/************************************************************************/
bool SensorPort::getOutput(Value &in) const
{
    if (configured)
    {
        in=static_cast<iCub::perception::Port*>(source)->getValue(index);
        return true;
    }
    else
        return false;
}




