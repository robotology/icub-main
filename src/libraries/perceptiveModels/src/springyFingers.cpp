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

#include <assert.h>

#include <iCub/ctrl/math.h>
#include <iCub/perception/springyFingers.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace iCub::learningmachine;
using namespace iCub::perception;


/************************************************************************/
SpringyFinger::SpringyFinger()
{
    threshold=0.0;
}


/************************************************************************/
bool SpringyFinger::fromProperty(const Property &options)
{
    lssvm.reset();

    Property &opt=const_cast<Property&>(options);
    assert(opt.check("name"));
    name=opt.find("name").asString().c_str();
    threshold=opt.check("thres",Value(0.0)).asDouble();

    scaler.setLowerBoundIn(0.0);
    scaler.setUpperBoundIn(360.0);
    lssvm.setDomainSize(1);

    if ((name=="thumb") || (name=="index") || (name=="middle"))
        lssvm.setCoDomainSize(2);
    else if ((name=="ring") || (name=="little"))
        lssvm.setCoDomainSize(3);
    else
        return false;

    if (opt.check("scaler"))
        scaler.fromString(opt.find("scaler").asString().c_str());

    if (opt.check("lssvm"))
        lssvm.fromString(opt.find("lssvm").asString().c_str());

    return true;
}


/************************************************************************/
void SpringyFinger::toProperty(Property &options) const
{
    options.put("name",name.c_str());
    options.put("thres",threshold);
    options.put("scaler",scaler.toString().c_str());
    options.put("lssvm",lssvm.toString().c_str());
}


/************************************************************************/
bool SpringyFinger::getData(Vector &in, Vector &out) const
{
    map<string,Sensor*>::const_iterator In_0=sensors.find("In_0");
    map<string,Sensor*>::const_iterator Out_0=sensors.find("Out_0");
    map<string,Sensor*>::const_iterator Out_1=sensors.find("Out_1");
    map<string,Sensor*>::const_iterator Out_2=sensors.find("Out_2");

    bool ok=true;
    ok&=In_0!=sensors.end();
    ok&=Out_0!=sensors.end();
    ok&=Out_1!=sensors.end();

    if (lssvm.getCoDomainSize()>2)
        ok&=Out_2!=sensors.end();

    if (!ok)
        return false;

    Value val_in, val_out(3);
    In_0->second->getInput(val_in);
    Out_0->second->getInput(val_out[0]);
    Out_1->second->getInput(val_out[1]);

    in.resize(lssvm.getDomainSize());
    in[0]=val_in.asDouble();

    out.resize(lssvm.getCoDomainSize());
    out[0]=val_out[0].asDouble();
    out[1]=val_out[1].asDouble();

    if (lssvm.getCoDomainSize()>2)
    {
        Out_2->second->getInput(val_out[2]);
        out[2]=val_out[2].asDouble();
    }

    return true;
}


/************************************************************************/
bool SpringyFinger::getOutput(Value &out) const
{
    Vector i,o;
    if (!getData(i,o))
        return false;

    i[0]=scaler.transform(i[0].asDouble());
    Vector pred=lssvm.predict(i);

    for (int j=0; j<pred.length(); j++)
        pred[j]=scaler.unTransform(pred[j]);

    Property prop;
    prop.put("prediction",Value(pred.toString().c_str()));
    prop.put("out",Value(norm(o-pred)>thres?1:0));

    out=Value(prop.toString().c_str());

    return true;
}


/************************************************************************/
bool SpringyFinger::calibrate(const Property &options)
{
    Property &opt=const_cast<Property&>(options);

    if (opt.check("reset"))
        lssvm.reset();

    if (opt.check("feed"))
    {
        Vector in,out;
        if (getData(in,out))
        {
            in[0]=scaler.transform(in[0]);
            for (int i=0; i<out.length(); i++)
                out[i]=scaler.transform(out[i]);

            lssvm.feedSample(in,out);
        }
        else
            return false;
    }

    if (opt.check("train"))
        lssvm.train();

    return true;
}



