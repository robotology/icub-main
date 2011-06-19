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

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <iCub/ctrl/math.h>
#include <iCub/perception/springyFingers.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace iCub::learningmachine;
using namespace iCub::perception;


/************************************************************************/
bool SpringyFinger::fromProperty(const Property &options)
{
    sensors.clear();
    callbacks.clear();
    neighbors.clear();
    lssvm.reset();

    Property &opt=const_cast<Property&>(options);
    assert(opt.check("name"));
    name=opt.find("name").asString().c_str();

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
    options.clear();
    options.put("name",name.c_str());
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

    Value val_in, val_out[3];
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

    i[0]=scaler.transform(i[0]);
    Vector pred=lssvm.predict(i);

    for (int j=0; j<pred.length(); j++)
        pred[j]=scaler.unTransform(pred[j]);

    out=Value(norm(o-pred));

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


/************************************************************************/
SpringyFingersModel::SpringyFingersModel()
{
    configured=false;
}


/************************************************************************/
bool SpringyFingersModel::fromProperty(const Property &options)
{
    if (configured)
        close();

    Property &opt=const_cast<Property&>(options);
    assert(opt.check("name"));
    assert(opt.check("type"));
    name=opt.find("name").asString().c_str();
    type=opt.find("type").asString().c_str();
    robot=opt.check("robot",Value("icub")).asString().c_str();

    string part_motor=string("/"+type+"_arm");
    string part_analog=string("/"+type+"_hand");

    Property prop;
    prop.put("robot",robot.c_str());
    prop.put("remote",("/"+robot+part_motor).c_str());
    prop.put("local",("/"+name+part_motor).c_str());
    if (!driver.open(prop))
        return false;

    port.open(("/"+name+part_analog+"/analog:i").c_str());
    if (!Network::connect(port.getName().c_str(),("/"+robot+part_analog+"/analog:o").c_str(),"udp"))
    {
        port.close();
        return false;
    }

    IEncoders *ienc; driver.view(ienc);
    int nAxes; ienc->getAxes(&nAxes);

    // configure interface-based sensors
    Property propGen;
    propGen.put("name","In_0");
    propGen.put("size",nAxes);

    Property propThumb=propGen;  propThumb.put( "index",10);
    Property propIndex=propGen;  propIndex.put( "index",12);
    Property propMiddle=propGen; propMiddle.put("index",14);
    Property propRing=propGen;   propRing.put(  "index",15);
    Property propLittle=propGen; propLittle.put("index",15);

    void *pIF=static_cast<void*>(ienc);
    sensIF[0].configure(pIF,propThumb);
    sensIF[1].configure(pIF,propIndex);
    sensIF[2].configure(pIF,propMiddle);
    sensIF[3].configure(pIF,propRing);
    sensIF[4].configure(pIF,propLittle);

    // configure port-based sensors
    Property thumb_mp(  "(name Out_0) (index 1)" );
    Property thumb_ip(  "(name Out_1) (index 2)" );
    Property index_mp(  "(name Out_0) (index 4)" );
    Property index_ip(  "(name Out_1) (index 5)" );
    Property middle_mp( "(name Out_0) (index 7)" );
    Property middle_ip( "(name Out_1) (index 8)" );
    Property ring_mp(   "(name Out_0) (index 9)" );
    Property ring_pip(  "(name Out_1) (index 10)");
    Property ring_dip(  "(name Out_2) (index 11)");
    Property little_mp( "(name Out_0) (index 12)");
    Property little_pip("(name Out_1) (index 13)");
    Property little_dip("(name Out_2) (index 14)");

    void *pPort=static_cast<void*>(&port);
    sensPort[0].configure(&pPort,thumb_mp);
    sensPort[1].configure(&pPort,thumb_ip);
    sensPort[2].configure(&pPort,index_mp);
    sensPort[3].configure(&pPort,index_ip);
    sensPort[4].configure(&pPort,middle_mp);
    sensPort[5].configure(&pPort,middle_ip);
    sensPort[6].configure(&pPort,ring_mp);
    sensPort[7].configure(&pPort,ring_pip);
    sensPort[8].configure(&pPort,ring_dip);
    sensPort[9].configure(&pPort,little_mp);
    sensPort[10].configure(&pPort,little_pip);
    sensPort[11].configure(&pPort,little_dip);

    // configure fingers
    Property thumb(opt.findGroup("thumb").toString().c_str());
    Property index(opt.findGroup("index").toString().c_str());
    Property middle(opt.findGroup("middle").toString().c_str());
    Property ring(opt.findGroup("ring").toString().c_str());
    Property little(opt.findGroup("little").toString().c_str());

    fingers[0].fromProperty(thumb);
    fingers[1].fromProperty(index);
    fingers[2].fromProperty(middle);
    fingers[3].fromProperty(ring);
    fingers[4].fromProperty(little);

    // attach sensors to fingers
    fingers[0].attachSensor(sensIF[0]);
    fingers[0].attachSensor(sensPort[0]);
    fingers[0].attachSensor(sensPort[1]);

    fingers[1].attachSensor(sensIF[1]);
    fingers[1].attachSensor(sensPort[2]);
    fingers[1].attachSensor(sensPort[3]);

    fingers[2].attachSensor(sensIF[2]);
    fingers[2].attachSensor(sensPort[4]);
    fingers[2].attachSensor(sensPort[5]);

    fingers[3].attachSensor(sensIF[3]);
    fingers[3].attachSensor(sensPort[6]);
    fingers[3].attachSensor(sensPort[7]);
    fingers[3].attachSensor(sensPort[8]);

    fingers[4].attachSensor(sensIF[4]);
    fingers[4].attachSensor(sensPort[9]);
    fingers[4].attachSensor(sensPort[10]);
    fingers[4].attachSensor(sensPort[11]);

    return configured=true;
}


/************************************************************************/
void SpringyFingersModel::toProperty(Property &options) const
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
        options.put("thumb",prop[0].toString().c_str());
        options.put("index",prop[1].toString().c_str());
        options.put("middle",prop[2].toString().c_str());
        options.put("ring",prop[3].toString().c_str());
        options.put("little",prop[4].toString().c_str());
    }
}


/************************************************************************/
bool SpringyFingersModel::calibrate(const Property &options)
{
    if (configured)
    {
        IControlLimits   *ilim; driver.view(ilim);
        IEncoders        *ienc; driver.view(ienc);
        IPositionControl *ipos; driver.view(ipos);

        // latch the hand configuration
        int nAxes; ienc->getAxes(&nAxes);
        Vector q0(nAxes),qmin(nAxes),qmax(nAxes);
        ienc->getEncoders(q0.data());

        // steer the hand to a suitable starting configuration
        for (int j=7; j<nAxes; j++)
        {            
            ilim->getLimits(j,&qmin[j],&qmax[j]);
            ipos->setRefAcceleration(j,1e9);
            ipos->setRefSpeed(j,30.0);
            ipos->positionMove(j,qmin[j]);
        }

        // proceed with the calibration
        Property &opt=const_cast<Property&>(options);
        string tag=opt.check("finger",Value("all")).asString().c_str();
        if (tag=="thumb")
        {
            calibrateFinger(fingers[0],10,qmin,qmax);
        }
        else if (tag=="index")
        {
            calibrateFinger(fingers[1],12,qmin,qmax);
        }
        else if (tag=="middle")
        {
            calibrateFinger(fingers[2],14,qmin,qmax);
        }
        else if (tag=="ring")
        {
            calibrateFinger(fingers[3],15,qmin,qmax);
        }
        else if (tag=="little")
        {
            calibrateFinger(fingers[4],15,qmin,qmax);
        }
        else if (tag=="all")
        {
            calibrateFinger(fingers[0],10,qmin,qmax);
            calibrateFinger(fingers[1],12,qmin,qmax);
            calibrateFinger(fingers[2],14,qmin,qmax);
            calibrateFinger(fingers[3],15,qmin,qmax);
            calibrateFinger(fingers[4],15,qmin,qmax);
        }
        else
            return false;

        // steer the hand back to the original configuration
        for (int j=7; j<nAxes; j++)
            ipos->positionMove(j,q0[j]);

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool SpringyFingersModel::getOutput(Value &out) const
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
void SpringyFingersModel::calibrateFinger(SpringyFinger &finger, const int joint,
                                          const Vector &qmin, const Vector &qmax)
{
    double margin=0.1*(qmax[joint]-qmin[joint]);
    double min=qmin[joint]+margin;
    double max=qmax[joint]-margin;

    IEncoders        *ienc; driver.view(ienc);
    IPositionControl *ipos; driver.view(ipos);
    double *val=&min;
    double timeout=(max-min)/20.0;

    Property reset("(reset)");
    Property feed("(feed)");
    Property train("(train)");

    finger.calibrate(reset);

    for (int i=0; i<5; i++)
    {
        ipos->positionMove(joint,*val);

        bool done=false;
        double t0=Time::now();
        while (!done)
        {
            finger.calibrate(feed);

            double fb;
            ienc->getEncoder(joint,&fb);
            done=(fabs(*val-fb)<5.0)||(Time::now()-t0>timeout);

            Time::delay(0.02);
        }

        if (val==&min)
            val=&max;
        else
            val=&min;
    }

    finger.calibrate(train);
}


/************************************************************************/
void SpringyFingersModel::close()
{
    if (configured)
    {
        driver.close();

        port.interrupt();
        port.close();

        configured=false;
    }
}


/************************************************************************/
SpringyFingersModel::~SpringyFingersModel()
{
    close();
}



