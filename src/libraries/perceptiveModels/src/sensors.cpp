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
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
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
    size=options.find("size").asInt32();
    index=options.find("index").asInt32();

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
bool SensorEncoderArrays::configure(void *source, const Property &options)
{
    if ((source==NULL) || !options.check("name") || !options.check("num_arrays") ||
        !options.check("index_array") || !options.check("index_element"))
        return false;

    this->source=source;
    name=options.find("name").asString();
    num_arrays=options.find("num_arrays").asInt32();
    index_array=options.find("index_array").asInt32();
    index_element=options.find("index_element").asInt32();

    return configured=true;
}


/************************************************************************/
bool SensorEncoderArrays::getOutput(Value &in) const
{
    auto* iencarray=static_cast<IEncoderArrays*>(source);
    if (!configured)
        return false;
    
    if ((index_array<0) || (index_array>=iencarray->getNrOfEncoderArrays()))
        return false;

    if ((index_element<0) || (index_element>=iencarray->getEncoderArraySize(index_array)))
        return false;

    if (iencarray->getEncoderArrayStatus(index_array)!=MAS_OK)
        return false;

    double stamp;
    Vector vect(iencarray->getEncoderArraySize(index_array));
    iencarray->getEncoderArrayMeasure(index_array,vect,stamp);
    in=Value(vect[index_element]);

    return true;
}


/************************************************************************/
bool SensorPort::configure(void *source, const Property &options)
{
    if ((source==NULL) || !options.check("name") || !options.check("index"))
        return false;

    this->source=source;
    name=options.find("name").asString();
    index=options.find("index").asInt32();

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




