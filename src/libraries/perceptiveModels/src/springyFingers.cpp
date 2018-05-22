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
#include <cmath>
#include <limits>
#include <deque>
#include <set>

#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
#include <iCub/perception/private/ports.h>
#include <iCub/perception/private/models.h>
#include <iCub/perception/springyFingers.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::learningmachine;
using namespace iCub::perception;


/************************************************************************/
bool SpringyFinger::fromProperty(const Property &options)
{
    if (!options.check("name"))
        return false;

    sensors.clear();
    callbacks.clear();
    neighbors.clear();
    lssvm.reset();

    name=options.find("name").asString();

    scaler.setLowerBoundIn(0.0);
    scaler.setUpperBoundIn(255.0);
    scaler.setLowerBoundOut(0.0);
    scaler.setUpperBoundOut(1.0);

    lssvm.setDomainSize(1);
    lssvm.setC(1e4);
    lssvm.getKernel()->setGamma(500.0);

    double defaultCalibVel;
    if ((name=="thumb") || (name=="index") || (name=="middle"))
    {
        lssvm.setCoDomainSize(2);
        defaultCalibVel=30.0;
    }
    else if ((name=="ring") || (name=="little"))
    {
        lssvm.setCoDomainSize(3);
        defaultCalibVel=60.0;
    }
    else
        return false;    

    calibratingVelocity=options.check("calib_vel",Value(defaultCalibVel)).asDouble();
    outputGain=options.check("output_gain",Value(1.0)).asDouble();
    calibrated=(options.check("calibrated",Value("false")).asString()=="true");

    if (options.check("scaler"))
    {
        Bottle *pB=options.find("scaler").asList();
        scaler.fromString(pB->toString());
    }

    if (options.check("lssvm"))
    {
        Bottle *pB=options.find("lssvm").asList();
        lssvm.fromString(pB->toString());
    }

    return true;
}


/************************************************************************/
void SpringyFinger::toProperty(Property &options) const
{
    options.clear();
    options.put("name",name);
    options.put("calib_vel",calibratingVelocity);
    options.put("output_gain",outputGain);
    options.put("calibrated",calibrated?"true":"false");
    options.put("scaler","("+scaler.toString()+")");
    options.put("lssvm","("+lssvm.toString()+")");
}


/************************************************************************/
bool SpringyFinger::toStream(ostream &str) const
{
    str<<"name        "<<name<<endl;
    str<<"calib_vel   "<<calibratingVelocity<<endl;
    str<<"output_gain "<<outputGain<<endl;
    str<<"calibrated  "<<(calibrated?"true":"false")<<endl;
    str<<"scaler      "<<"("+scaler.toString()+")"<<endl;
    str<<"lssvm       "<<"("+lssvm.toString()+")"<<endl;

    return !str.fail();
}


/************************************************************************/
bool SpringyFinger::getSensorsData(Value &data) const
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
    In_0->second->getOutput(val_in);
    Out_0->second->getOutput(val_out[0]);
    Out_1->second->getOutput(val_out[1]);

    Vector in(lssvm.getDomainSize());
    in[0]=val_in.asDouble();

    Vector out(lssvm.getCoDomainSize());
    out[0]=val_out[0].asDouble();
    out[1]=val_out[1].asDouble();

    if (lssvm.getCoDomainSize()>2)
    {
        Out_2->second->getOutput(val_out[2]);
        out[2]=val_out[2].asDouble();
    }

    Property prop;
    Bottle b;

    b.addList().read(in);
    prop.put("in",b.get(0));

    b.addList().read(out);
    prop.put("out",b.get(1));

    b.addList().read(prop);
    data=b.get(2);

    return true;
}


/************************************************************************/
bool SpringyFinger::extractSensorsData(Vector &in, Vector &out) const
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
            }

            if (Bottle *b2=b1->find("out").asList())
            {
                out.resize(b2->size());
                for (size_t i=0; i<out.length(); i++)
                    out[i]=b2->get(i).asDouble();

                ret=true;
            }
        }
    }

    return ret;
}


/************************************************************************/
bool SpringyFinger::getOutput(Value &out) const
{
    Vector i,o;
    if (!extractSensorsData(i,o))
        return false;

    i[0]=scaler.transform(i[0]);
    Vector pred=lssvm.predict(i).getPrediction();

    for (size_t j=0; j<pred.length(); j++)
        pred[j]=scaler.unTransform(pred[j]);

    out=Value(outputGain*norm(o-pred));

    return true;
}


/************************************************************************/
bool SpringyFinger::calibrate(const Property &options)
{
    if (options.check("reset"))
        lssvm.reset();

    if (options.check("feed"))
    {
        Vector in,out;
        if (extractSensorsData(in,out))
        {            
            in[0]=scaler.transform(in[0]);
            for (size_t i=0; i<out.length(); i++)
                out[i]=scaler.transform(out[i]);

            lssvm.feedSample(in,out);
        }
        else
            return false;
    }

    if (options.check("train"))
    {
        lssvm.train();
        calibrated=true;
    }

    return true;
}


/************************************************************************/
SpringyFingersModel::SpringyFingersModel()
{
    port=new iCub::perception::Port;
    configured=false;
}


/************************************************************************/
bool SpringyFingersModel::fromProperty(const Property &options)
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
    verbosity=options.check("verbosity",Value(0)).asInt();

    string part_motor=string(type+"_arm");
    string part_analog=string(type+"_hand");

    Property prop;
    prop.put("device","remote_controlboard");
    prop.put("remote","/"+robot+"/"+part_motor);
    prop.put("local","/"+name+"/"+part_motor);
    if (!driver.open(prop))
        return false;
    
    port->open("/"+name+"/"+part_analog+"/analog:i");
    string analogPortName("/"+robot+"/"+part_analog+"/analog:o");
    if (!Network::connect(analogPortName,port->getName(),carrier))
    {
        printMessage(log::error,1,"unable to connect to %s",analogPortName.c_str());
        close();
        return false;
    }

    IEncoders *ienc; driver.view(ienc);
    int nAxes; ienc->getAxes(&nAxes);

    printMessage(log::info,1,"configuring interface-based sensors ...");
    Property propGen;
    propGen.put("name","In_0");
    propGen.put("size",nAxes);

    Property propThumb=propGen;  propThumb.put( "index",10);
    Property propIndex=propGen;  propIndex.put( "index",12);
    Property propMiddle=propGen; propMiddle.put("index",14);
    Property propRing=propGen;   propRing.put(  "index",15);
    Property propLittle=propGen; propLittle.put("index",15);

    bool sensors_ok=true;
    void *pEncs=static_cast<void*>(ienc);
    sensors_ok&=sensEncs[0].configure(pEncs,propThumb);
    sensors_ok&=sensEncs[1].configure(pEncs,propIndex);
    sensors_ok&=sensEncs[2].configure(pEncs,propMiddle);
    sensors_ok&=sensEncs[3].configure(pEncs,propRing);
    sensors_ok&=sensEncs[4].configure(pEncs,propLittle);

    printMessage(log::info,1,"configuring port-based sensors ...");
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

    void *pPort=static_cast<void*>(port);
    sensors_ok&=sensPort[0].configure(pPort,thumb_mp);
    sensors_ok&=sensPort[1].configure(pPort,thumb_ip);
    sensors_ok&=sensPort[2].configure(pPort,index_mp);
    sensors_ok&=sensPort[3].configure(pPort,index_ip);
    sensors_ok&=sensPort[4].configure(pPort,middle_mp);
    sensors_ok&=sensPort[5].configure(pPort,middle_ip);
    sensors_ok&=sensPort[6].configure(pPort,ring_mp);
    sensors_ok&=sensPort[7].configure(pPort,ring_pip);
    sensors_ok&=sensPort[8].configure(pPort,ring_dip);
    sensors_ok&=sensPort[9].configure(pPort,little_mp);
    sensors_ok&=sensPort[10].configure(pPort,little_pip);
    sensors_ok&=sensPort[11].configure(pPort,little_dip);

    if (!sensors_ok)
    {
        printMessage(log::error,1,"some errors occured");
        close();
        return false;
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
    fingers[0].attachSensor(sensEncs[0]);
    fingers[0].attachSensor(sensPort[0]);
    fingers[0].attachSensor(sensPort[1]);

    fingers[1].attachSensor(sensEncs[1]);
    fingers[1].attachSensor(sensPort[2]);
    fingers[1].attachSensor(sensPort[3]);

    fingers[2].attachSensor(sensEncs[2]);
    fingers[2].attachSensor(sensPort[4]);
    fingers[2].attachSensor(sensPort[5]);

    fingers[3].attachSensor(sensEncs[3]);
    fingers[3].attachSensor(sensPort[6]);
    fingers[3].attachSensor(sensPort[7]);
    fingers[3].attachSensor(sensPort[8]);

    fingers[4].attachSensor(sensEncs[4]);
    fingers[4].attachSensor(sensPort[9]);
    fingers[4].attachSensor(sensPort[10]);
    fingers[4].attachSensor(sensPort[11]);

    attachNode(fingers[0]);
    attachNode(fingers[1]);
    attachNode(fingers[2]);
    attachNode(fingers[3]);
    attachNode(fingers[4]);

    printMessage(log::info,1,"configuration complete");
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
        options.put("verbosity",verbosity);
    }
}


/************************************************************************/
bool SpringyFingersModel::toStream(ostream &str) const
{
    if (configured)
    {
        str<<"name      "<<name<<endl;
        str<<"type      "<<type<<endl;
        str<<"robot     "<<robot<<endl;
        str<<"verbosity "<<verbosity<<endl;

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
bool SpringyFingersModel::calibrate(const Property &options)
{
    if (configured)
    {
        Value fng=options.find("finger");
        if (fng.isNull())
        {
            printMessage(log::error,1,"unspecified option \"finger\"");
            return false;
        }

        IControlMode     *imod; driver.view(imod);
        IControlLimits   *ilim; driver.view(ilim);
        IEncoders        *ienc; driver.view(ienc);
        IPositionControl *ipos; driver.view(ipos);

        int nAxes; ienc->getAxes(&nAxes);
        Vector qmin(nAxes),qmax(nAxes),vel(nAxes),acc(nAxes);

        printMessage(log::info,1,"steering the hand to a suitable starting configuration");
        for (int j=7; j<nAxes; j++)
        {
            imod->setControlMode(j,VOCAB_CM_POSITION);
            ilim->getLimits(j,&qmin[j],&qmax[j]);            

            ipos->getRefAcceleration(j,&acc[j]);
            ipos->getRefSpeed(j,&vel[j]);
            
            ipos->setRefAcceleration(j,std::numeric_limits<double>::max());
            ipos->setRefSpeed(j,60.0);
            ipos->positionMove(j,(j==8)?qmax[j]:qmin[j]);   // thumb in opposition
        }

        printMessage(log::info,1,"proceeding with the calibration");
        bool ok=true;

        if (fng.isString())
        {
            string tag=options.check("finger",Value("all")).asString();
            if (tag=="thumb")
            {
                calibrateFinger(fingers[0],10,qmin[10],qmax[10]);
            }
            else if (tag=="index")
            {
                calibrateFinger(fingers[1],12,qmin[12],qmax[12]);
            }
            else if (tag=="middle")
            {
                calibrateFinger(fingers[2],14,qmin[14],qmax[14]);
            }
            else if (tag=="ring")
            {
                calibrateFinger(fingers[3],15,qmin[15],qmax[15]);
            }
            else if (tag=="little")
            {
                calibrateFinger(fingers[4],15,qmin[15],qmax[15]);
            }
            else if ((tag=="all") || (tag=="all_serial"))
            {
                calibrateFinger(fingers[0],10,qmin[10],qmax[10]);
                calibrateFinger(fingers[1],12,qmin[12],qmax[12]);
                calibrateFinger(fingers[2],14,qmin[14],qmax[14]);
                calibrateFinger(fingers[3],15,qmin[15],qmax[15]);
                calibrateFinger(fingers[4],15,qmin[15],qmax[15]);
            }
            else if (tag=="all_parallel")
            {
                Bottle b;
                Bottle &bl=b.addList();
                bl.addString("thumb");
                bl.addString("index");
                bl.addString("middle");
                bl.addString("ring");
                bl.addString("little");

                fng=b.get(0);
            }
            else
                ok=false;
        }

        if (fng.isList())
        {
            deque<CalibThread*> db;
            set<string> singletons;

            Bottle *items=fng.asList();
            for (int i=0; i<items->size(); i++)
            {
                string tag=items->get(i).asString();
                if (tag=="thumb")
                {
                    if (singletons.find(tag)==singletons.end())
                    {
                        CalibThread *thr=new CalibThread; 
                        thr->setInfo(this,fingers[0],10,qmin[10],qmax[10]);
                        thr->start();
                        db.push_back(thr);
                        singletons.insert(tag);
                    }
                }
                else if (tag=="index")
                {
                    if (singletons.find(tag)==singletons.end())
                    {
                        CalibThread *thr=new CalibThread;
                        thr->setInfo(this,fingers[1],12,qmin[12],qmax[12]);
                        thr->start();
                        db.push_back(thr);
                        singletons.insert(tag);
                    }
                }
                else if (tag=="middle")
                {
                    if (singletons.find(tag)==singletons.end())
                    {
                        CalibThread *thr=new CalibThread;
                        thr->setInfo(this,fingers[2],14,qmin[14],qmax[14]);
                        thr->start();
                        db.push_back(thr);
                        singletons.insert(tag);
                    }
                }
                else if (tag=="ring")
                {
                    if (singletons.find(tag)==singletons.end())
                    {
                        CalibThread *thr=new CalibThread;
                        thr->setInfo(this,fingers[3],15,qmin[15],qmax[15]);
                        thr->start();
                        db.push_back(thr);
                        singletons.insert(tag);
                    }
                }
                else if (tag=="little")
                {
                    if (singletons.find(tag)==singletons.end())
                    {
                        CalibThread *thr=new CalibThread;
                        thr->setInfo(this,fingers[4],15,qmin[15],qmax[15]);
                        thr->start();
                        db.push_back(thr);
                        singletons.insert(tag);
                    }
                }
            }

            if (db.size()>0)
            {
                size_t i=0;
                while (db.size()>0)
                {
                    if (db[i]->isDone())
                    {
                        db[i]->stop();
                        delete db[i];
                        db.erase(db.begin()+i);
                    }

                    Time::delay(0.1);
                    if (++i>=db.size())
                        i=0;
                }
            }
            else
                ok=false;
        }
        
        if (!fng.isString() && !fng.isList())
            ok=false;

        if (!ok)
            printMessage(log::error,1,"unknown finger request %s",fng.toString().c_str());

        for (int j=7; j<nAxes; j++)
        {
            ipos->setRefAcceleration(j,acc[j]);
            ipos->setRefSpeed(j,vel[j]);
        }

        return ok;
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

        out=bOut.get(0);
        return true;
    }
    else
        return false;
}


/************************************************************************/
void SpringyFingersModel::calibrateFinger(SpringyFinger &finger, const int joint,
                                          const double min, const double max)
{
    printMessage(log::info,1,"calibrating finger %s ...",finger.getName().c_str());
    double margin=0.1*(max-min);
    double _min=min+margin;
    double _max=max-margin;
    double tol_min=5.0;
    double tol_max=5.0;
    double *val=&_min;
    double *tol=&tol_min;
    double timeout=2.0*(_max-_min)/finger.getCalibVel();

    mutex.lock();
    IControlMode     *imod; driver.view(imod);
    IEncoders        *ienc; driver.view(ienc);
    IPositionControl *ipos; driver.view(ipos);
    mutex.unlock();

    // workaround
    if ((finger.getName()=="ring") || (finger.getName()=="little"))
    {
        _min=30.0;
        _max=180.0;
        tol_min=20.0;
        tol_max=50.0;
    }

    Property reset("(reset)");
    Property feed("(feed)");
    Property train("(train)");

    finger.calibrate(reset);

    mutex.lock();
    imod->setControlMode(joint,VOCAB_CM_POSITION);
    ipos->setRefSpeed(joint,finger.getCalibVel());
    mutex.unlock();

    for (int i=0; i<5; i++)
    {
        mutex.lock();
        ipos->positionMove(joint,*val);
        mutex.unlock();

        bool done=false;
        double fbOld=std::numeric_limits<double>::max();
        double t0=Time::now();
        while (!done)
        {
            Time::delay(0.01);

            mutex.lock();
            double fb; ienc->getEncoder(joint,&fb);
            mutex.unlock();

            if (fabs(fb-fbOld)>0.5)
            {
                printMessage(log::no_info,2,"feeding finger %s",finger.getName().c_str());
                finger.calibrate(feed);
            }

            done=(fabs(*val-fb)<*tol)||(Time::now()-t0>timeout);
            fbOld=fb;
        }

        if (val==&_min)
        {
            val=&_max;
            tol=&tol_max;
        }
        else
        {
            val=&_min;
            tol=&tol_min;
        }
    }

    printMessage(log::info,1,"training finger %s ...",finger.getName().c_str());    
    finger.calibrate(train);
    printMessage(log::info,1,"finger %s trained!",finger.getName().c_str());
}


/************************************************************************/
bool SpringyFingersModel::isCalibrated() const
{
    return (fingers[0].isCalibrated()&&
            fingers[1].isCalibrated()&&
            fingers[2].isCalibrated()&&
            fingers[3].isCalibrated()&&
            fingers[4].isCalibrated());
}


/************************************************************************/
void SpringyFingersModel::close()
{
    printMessage(log::info,1,"closing ...");

    if (driver.isValid())
        driver.close();

    if (!port->isClosed())
        port->close();

    nodes.clear();

    configured=false;
}


/************************************************************************/
SpringyFingersModel::~SpringyFingersModel()
{
    close();
    delete port;
}



