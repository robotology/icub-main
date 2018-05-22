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

#include <sstream>
#include <iomanip>
#include <algorithm>

#include <iCub/ctrl/math.h>
#include <iCub/perception/private/ports.h>
#include <iCub/perception/private/models.h>
#include <iCub/perception/tactileFingers.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::perception;


/************************************************************************/
bool TactileFinger::fromProperty(const Property &options)
{
    if (!options.check("name"))
        return false;

    sensors.clear();
    callbacks.clear();
    neighbors.clear();

    name=options.find("name").asString();
    directLogic=(options.check("logic",Value("direct")).asString()=="direct");
    outputGain=options.check("output_gain",Value(1.0)).asDouble();

    if ((name=="thumb") || (name=="index") || (name=="middle") ||
        (name=="ring")  || (name=="little"))
        return true;
    else
        return false;
}


/************************************************************************/
void TactileFinger::toProperty(Property &options) const
{
    options.clear();
    options.put("name",name);
    options.put("logic",directLogic?"direct":"inverse");
    options.put("output_gain",outputGain);
}


/************************************************************************/
bool TactileFinger::toStream(ostream &str) const
{
    str<<"name        "<<name<<endl;
    str<<"logic       "<<(directLogic?"direct":"inverse")<<endl;
    str<<"output_gain "<<outputGain<<endl;

    if (str.fail())
        return false;
    else
        return true;
}


/************************************************************************/
bool TactileFinger::calibrate(const Property &options)
{
    yWarning("TactileFinger: calibration not available");
    return true;
}


/************************************************************************/
bool TactileFinger::isCalibrated() const
{
    yWarning("TactileFinger: calibration not available");
    return true;
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

        Value val_in; In->second->getOutput(val_in);
        in[j]=val_in.asDouble();
    }

    Property prop;
    Bottle b;

    b.addList().read(in);
    prop.put("in",b.get(0));

    b.addList().read(prop);
    data=b.get(1);

    return true;
}


/************************************************************************/
bool TactileFinger::extractSensorsData(Vector &in) const
{
    bool ret=false;

    Value data;
    if (getSensorsData(data))
    {
        if (Bottle *b1=data.asList())
        {
            if (Bottle *b2=b1->find("in").asList())
            {
                in.resize(b2->size()); 
                for (size_t i=0; i<in.length(); i++)
                    in[i]=b2->get(i).asDouble();

                ret=true;
            }
        }
    }

    return ret;
}


/************************************************************************/
bool TactileFinger::getOutput(Value &out) const
{
    Vector i;
    if (!extractSensorsData(i))
        return false;

    double ret=0.0;
    for (size_t j=0; j<i.length(); j++)
        ret=std::max(ret,directLogic?(255.0-i[j]):i[j]);

    out=Value(outputGain*ret);

    return true;
}


/************************************************************************/
TactileFingersModel::TactileFingersModel()
{
    port=new iCub::perception::Port;
    configured=false;
}


/************************************************************************/
bool TactileFingersModel::fromProperty(const Property &options)
{
    if (!options.check("name") || !options.check("type"))
    {
        printMessage(log::error,1,"missing mandatory options \"name\" and/or \"type\"");
        return false;
    }

    if (configured)
        close();

    name=options.find("name").asString();
    type=options.find("type").asString();
    robot=options.check("robot",Value("icub")).asString();
    carrier=options.check("carrier",Value("udp")).asString();
    compensation=(options.check("compensation",Value("false")).asString()=="true");
    verbosity=options.check("verbosity",Value(0)).asInt();

    port->open("/"+name+"/"+type+"_hand:i");
    string skinPortName("/"+robot+"/skin/"+type+"_hand");
    if (compensation)
        skinPortName+="_comp";

    if (!Network::connect(skinPortName,port->getName(),carrier))
    {
        printMessage(log::error,1,"unable to connect to %s",skinPortName.c_str());
        close();
        return false;
    }

    printMessage(log::info,1,"configuring port-based sensors ...");
    void *pPort=static_cast<void*>(port);
    for (int j=0; j<60; j++)
    {
        ostringstream tag;
        tag<<"In_"<<(j%12);

        Property prop;
        prop.put("name",tag.str());
        prop.put("index",j);

        if (!sensPort[j].configure(pPort,prop))
        {
            printMessage(log::error,1,"some errors occured");
            close();
            return false;
        }
    }

    printMessage(log::info,1,"configuring fingers ...");
    Property thumb(options.findGroup("thumb").toString().c_str());
    Property index(options.findGroup("index").toString().c_str());
    Property middle(options.findGroup("middle").toString().c_str());
    Property ring(options.findGroup("ring").toString().c_str());
    Property little(options.findGroup("little").toString().c_str());

    bool fingers_ok=true;
    fingers_ok&=fingers[0].fromProperty(thumb);
    fingers_ok&=fingers[1].fromProperty(index);
    fingers_ok&=fingers[2].fromProperty(middle);
    fingers_ok&=fingers[3].fromProperty(ring);
    fingers_ok&=fingers[4].fromProperty(little);

    if (!fingers_ok)
    {
        printMessage(log::error,1,"some errors occured");
        close();
        return false;
    }

    printMessage(log::info,1,"attaching sensors to fingers ...");
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

    printMessage(log::info,1,"configuration complete");
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

        string thumb="(thumb ";
        thumb+=prop[0].toString();
        thumb+=")";

        string index="(index ";
        index+=prop[1].toString();
        index+=")";

        string middle="(middle ";
        middle+=prop[2].toString();
        middle+=")";

        string ring="(ring ";
        ring+=prop[3].toString();
        ring+=")";

        string little="(little ";
        little+=prop[4].toString();
        little+=")";

        options.fromString(thumb+index+middle+ring+little);
        options.put("name",name);
        options.put("type",type);
        options.put("robot",robot);
        options.put("compensation",compensation?"true":"false");
        options.put("verbosity",verbosity);
    }
}


/************************************************************************/
bool TactileFingersModel::toStream(ostream &str) const
{
    if (configured)
    {
        str<<"name         "<<name<<endl;
        str<<"type         "<<type<<endl;
        str<<"robot        "<<robot<<endl;
        str<<"compensation "<<(compensation?"true":"false")<<endl;
        str<<"verbosity    "<<verbosity<<endl;

        str<<endl;
        str<<"[thumb]"<<endl;
        fingers[0].toStream(str);

        str<<endl;
        str<<"[index]"<<endl;
        fingers[1].toStream(str);

        str<<endl;
        str<<"[middle]"<<endl;
        fingers[2].toStream(str);

        str<<endl;
        str<<"[ring]"<<endl;
        fingers[3].toStream(str);

        str<<endl;
        str<<"[little]"<<endl;
        fingers[4].toStream(str);

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool TactileFingersModel::calibrate(const Property &options)
{
    printMessage(log::warning,1,"calibration not available");
    return true;
}


/************************************************************************/
bool TactileFingersModel::isCalibrated() const
{
    printMessage(log::warning,1,"calibration not available");
    return true;
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

        out=bOut.get(0);
        return true;
    }
    else
        return false;
}


/************************************************************************/
void TactileFingersModel::close()
{
    printMessage(log::info,1,"closing ...");

    if (!port->isClosed())
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


