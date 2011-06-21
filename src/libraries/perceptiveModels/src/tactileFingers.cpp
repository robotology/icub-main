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

#include <stdio.h>
#include <stdarg.h>
#include <sstream>

#include <yarp/os/Network.h>

#include <iCub/ctrl/math.h>
#include <iCub/perception/private/ports.h>
#include <iCub/perception/tactileFingers.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::perception;


/************************************************************************/
bool TactileFinger::fromProperty(const Property &options)
{
    Property &opt=const_cast<Property&>(options);
    if (!opt.check("name"))
        return false;

    sensors.clear();
    callbacks.clear();
    neighbors.clear();

    name=opt.find("name").asString().c_str();
    directLogic=(opt.check("logic",Value("direct")).asString()=="direct");

    return true;
}


/************************************************************************/
void TactileFinger::toProperty(Property &options) const
{
    options.clear();
    options.put("name",name.c_str());
    options.put("logic",directLogic?"direct":"inverse");
}


/************************************************************************/
bool TactileFinger::calibrate(const Property &options)
{
    // not available
    return false;
}


/************************************************************************/
bool TactileFinger::getSensorsData(Value &data) const
{    
    Vector in(12);
    for (int j=0; j<12; j++)
    {
        ostringstream tag;
        tag<<"In_"<<j;

        map<string,Sensor*>::const_iterator In=sensors.find(tag.str());
        if (In==sensors.end())
            return false;

        Value val_in; In->second->getInput(val_in);
        in[j]=val_in.asDouble();
    }

    Property prop;
    Value i; i.fromString(("("+string(in.toString().c_str())+")").c_str());
    prop.put("in",i);
    data.fromString(("("+string(prop.toString().c_str())+")").c_str());

    return true;
}


/************************************************************************/
bool TactileFinger::extractSensorsData(Vector &in) const
{
    Value data;
    if (getSensorsData(data))
    {
        Property prop(data.asList()->toString().c_str());
        Bottle *b=prop.find("in").asList(); in.resize(b->size());
        for (int i=0; i<in.length(); i++)
            in[i]=b->get(i).asDouble();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool TactileFinger::getOutput(Value &out) const
{
    Vector i;
    if (!extractSensorsData(i))
        return false;

    double ret=0.0;
    for (int j=0; j<i.length(); j++)
        ret=std::max(ret,directLogic?(255.0-i[j]):i[j]);

    out=Value(ret);

    return true;
}


/************************************************************************/
TactileFingersModel::TactileFingersModel()
{
    port=new iCub::perception::Port;
    configured=false;
}


/************************************************************************/
int TactileFingersModel::printMessage(const int level, const char *format, ...) const
{
    if (verbose>=level)
    {
        fprintf(stdout,"*** %s: ",name.c_str());
    
        va_list ap;
        va_start(ap,format);
        int ret=vfprintf(stdout,format,ap);
        va_end(ap);
        
        return ret;
    }
    else
        return -1;
}


/************************************************************************/
bool TactileFingersModel::fromProperty(const Property &options)
{
    Property &opt=const_cast<Property&>(options);
    if (!opt.check("name") || !opt.check("type"))
    {
        printMessage(1,"missing mandatory options \"name\" and/or \"type\"\n");
        return false;
    }

    if (configured)
        close();

    name=opt.find("name").asString().c_str();
    type=opt.find("type").asString().c_str();
    robot=opt.check("robot",Value("icub")).asString().c_str();
    verbose=opt.check("verbose",Value(0)).asInt();

    port->open(("/"+name+"/"+type+"_hand:i").c_str());
    string skinPortName(("/"+robot+"/skin/"+type+"_hand").c_str());
    if (!Network::connect(skinPortName.c_str(),port->getName().c_str(),"udp"))
    {
        printMessage(1,"unable to connect to %s\n",skinPortName.c_str());
        close();
        return false;
    }

    printMessage(1,"configuring port-based sensors ...\n");
    void *pPort=static_cast<void*>(port);
    for (int j=0; j<60; j++)
    {
        ostringstream tag;
        tag<<"In_"<<(j%12);

        Property prop;
        prop.put("name",tag.str().c_str());
        prop.put("index",j);

        if (!sensPort[j].configure(pPort,prop))
        {
            printMessage(1,"some errors occured\n");
            close();
            return false;
        }
    }

    printMessage(1,"configuring fingers ...\n");
    Property thumb(opt.findGroup("thumb").toString().c_str());
    Property index(opt.findGroup("index").toString().c_str());
    Property middle(opt.findGroup("middle").toString().c_str());
    Property ring(opt.findGroup("ring").toString().c_str());
    Property little(opt.findGroup("little").toString().c_str());

    bool fingers_ok=true;
    fingers_ok&=fingers[0].fromProperty(thumb);
    fingers_ok&=fingers[1].fromProperty(index);
    fingers_ok&=fingers[2].fromProperty(middle);
    fingers_ok&=fingers[3].fromProperty(ring);
    fingers_ok&=fingers[4].fromProperty(little);

    if (!fingers_ok)
    {
        printMessage(1,"some errors occured\n");
        close();
        return false;
    }

    printMessage(1,"attaching sensors to fingers ...\n");
    for (int j=0; j<12; j++)
    {
        fingers[0].attachSensor(sensPort[j+4*12]);
        fingers[1].attachSensor(sensPort[j]);
        fingers[2].attachSensor(sensPort[j+12]);
        fingers[3].attachSensor(sensPort[j+2*12]);
        fingers[4].attachSensor(sensPort[j+3*12]);
    }

    attachNode(fingers[0]);
    attachNode(fingers[1]);
    attachNode(fingers[2]);
    attachNode(fingers[3]);
    attachNode(fingers[4]);

    printMessage(1,"configuration complete\n");
    return configured=true;
}


/************************************************************************/
void TactileFingersModel::toProperty(Property &options) const
{
    options.clear();

    if (configured)
    {
        Property prop[5];
        fingers[0].toProperty(prop[0]);
        fingers[1].toProperty(prop[1]);
        fingers[2].toProperty(prop[2]);
        fingers[3].toProperty(prop[3]);
        fingers[4].toProperty(prop[4]);

        options.put("name",name.c_str());
        options.put("type",type.c_str());
        options.put("robot",robot.c_str());
        options.put("verbose",verbose);
        options.put("thumb",prop[0].toString().c_str());
        options.put("index",prop[1].toString().c_str());
        options.put("middle",prop[2].toString().c_str());
        options.put("ring",prop[3].toString().c_str());
        options.put("little",prop[4].toString().c_str());
    }
}


/************************************************************************/
bool TactileFingersModel::calibrate(const Property &options)
{
    printMessage(1,"configuration not available\n");
    return false;
}


/************************************************************************/
bool TactileFingersModel::getOutput(Value &out) const
{
    if (configured)
    {
        Value val[5];
        fingers[0].getOutput(val[0]);
        fingers[1].getOutput(val[1]);
        fingers[2].getOutput(val[2]);
        fingers[3].getOutput(val[3]);
        fingers[4].getOutput(val[4]);
        
        Bottle bOut; Bottle &ins=bOut.addList();
        ins.addDouble(val[0].asDouble());
        ins.addDouble(val[1].asDouble());
        ins.addDouble(val[2].asDouble());
        ins.addDouble(val[3].asDouble());
        ins.addDouble(val[4].asDouble());

        out.fromString(bOut.toString().c_str());

        return true;
    }
    else
        return false;
}


/************************************************************************/
void TactileFingersModel::close()
{
    printMessage(1,"closing ...\n");
    port->interrupt();
    port->close();

    nodes.clear();

    configured=false;
}


/************************************************************************/
TactileFingersModel::~TactileFingersModel()
{
    close();
    delete port;
}



