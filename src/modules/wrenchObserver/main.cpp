/**
@ingroup icub_module

\defgroup wrenchObserver wrenchObserver
 
Estimates the external forces and torques acting at the end effector
through a model based estimation of the robot dynamics
 
Copyright (C) 2008 RobotCub Consortium
 
Author: Matteo Fumagalli
 
Date: first release 27/05/2010 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description

This module estimates the external wrench acting at the end
effector of the iCub limbs, through a model based compensation 
of the 6-axis force/torque (FT) sensor's measurements, which are 
acquired through an input YARP port and provides them to an 
output YARP ports.
The estimation is perfomed relying on rigid body dynamics using CAD 
parameters. 
The intrinsic offsets of the sensors, which are due to the stresses
generated during mounting, are defined by the first FT data. In the 
future it will also be given the possibility to set the offsets of 
the sensors.
The model of the sensor measurements consider a fixed base, with z-axis 
pointing upwards. The estimation of the external wrench applied at the 
end-effector of the limb has the same orientation of the fixed base frame.
 
\section lib_sec Libraries 
- YARP libraries. 
- ctrlLib library. 
- iKin library.
- iDyn library.  

\section parameters_sec Parameters

--name \e name 
- The parameter \e name identifies the module's name; all the 
  open ports will be tagged with the prefix <name>/. If not
  specified \e /ftObs is assumed.

  --robot
- The parameter \e robot identifies the robot that is used. This parameter defines the
  prefix of the ports of the device. As default \e icub is used. 

--part  
- The parameter \e part identifies the part of the robot which is used. All the opened 
  ports will deal with the part which is defined. the default value is \e left_arm 

--rate \e r 
- The parameter \e r identifies the rate the thread will work. If not
  specified \e 100ms is assumed. The minimum suggested rate is \e 20ms.

\section portsa_sec Ports Accessed
The port the service is listening to.

\section portsc_sec Ports Created
 
- \e <name>/<part>/FT:i (e.g. /ftObs/right_arm/FT:i) receives the input data 
  vector.
 
- \e <name>/<part>/contact:o (e.g. /ftObs/right_arm/contact:o) provides the estimated 
  end-effector wrench.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section conf_file_sec Configuration Files
None
 
\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example
By launching the following command: 
 
\code 
wrenchObserver --name ftObs --robot icub --part right_arm --rate 50  
\endcode 
 
the module will create the listening port /ftObs/right_arm/FT:i for 
the acquisition of data vector coming for istance from the right arm analog port.
At the same time it will provide the estimated 
external wrench at the end-effector to /ftObs/right_arm/contact:o port. (use --help option to 
see). 
 
Try now the following: 
 
\code 
yarp connect /icub/right_arm/analog:o /ftObs/right_arm/FT:i
\endcode 
 
\author Matteo Fumagalli

This file can be edited at \in src/wrenchObserver/main.cpp.
*/ 

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/ctrl/ctrlMath.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iFC.h>

#include <iostream>
#include <iomanip>
#include <string.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace ctrl;
using namespace iDyn;
using namespace std;


// class inverseDynamics: class for reading from Vrow and providing for FT on an output port
class inverseDynamics: public RateThread
{
private:
    PolyDriver *dd;
    PolyDriver *tt;
    IEncoders  *iencs;
    IEncoders  *tencs;

    string part;

    Vector *ft;
    BufferedPort<Vector> *port_FT;
    BufferedPort<Vector> *port_Contact;
    bool first;
    Stamp info;

    AWLinEstimator  *linEst;
    AWQuadEstimator *quadEst;

    int ctrlJnt;
    iDynLimb      *limb;
    iDynChain     *chain;
    iDynInvSensor *sens;

    iFB         *FTB;
    iFTransform *sensor;
    int sensorLink;

    Matrix HS, HSC, IS;
    double ms;

    Vector encoders;
    Vector encodersT;

    Vector q;
    Vector dq;
    Vector d2q;

    Vector w0,dw0,d2p0,Fend,Mend;
    Vector F_measured, F_iDyn, F_offset, FT;
    double time;

    Vector evalVel(const Vector &x)
    {
        AWPolyElement el;
        el.data=x;
        el.time=Time::now();

        return linEst->estimate(el);
    }

    Vector evalAcc(const Vector &x)
    {
        AWPolyElement el;
        el.data=x;
        el.time=Time::now();

        return quadEst->estimate(el);
    }

public:
    inverseDynamics(int _rate, PolyDriver *_dd, PolyDriver *_tt, BufferedPort<Vector> *_port_FT,
                    const string &_part, const string &_name) : RateThread(_rate), dd(_dd), tt(_tt)
    {        
        part = _part.c_str();
        first = true;
        port_FT = _port_FT;
        port_Contact=new BufferedPort<Vector>;
        string fwdSlash = "/";
        string port = fwdSlash+_name;
        port += (fwdSlash+part.c_str());
        port += "/contact:o";
        port_Contact->open(port.c_str());

        dd->view(iencs);
        if (tt)
            tt->view(tencs);

        if (tt)
        {
            if (part=="left_arm")
                limb = new iCubArmDyn("left");
            else
                limb = new iCubArmDyn("right");
        }
        else
        {
            if (part=="left_leg")
                limb = new iCubLegDyn("left");
            else
                limb = new iCubLegDyn("right");
        }

        chain = limb->asChain();

        HS.resize(4,4);  HS.eye();
        HSC.resize(4,4); HSC.eye();
        IS.resize(3,3);  IS.zero();
        ms = 0.0;

        if (tt)
        {
            sensorLink = 5;

            if (part=="left_arm")
                sens = new iDynInvSensorArm(limb->asChain(),"left",NE_DYNAMIC);
            else
                sens = new iDynInvSensorArm(limb->asChain(),"right",NE_DYNAMIC);
        }
        else
        {
            sensorLink = 2;

            if (part=="left_leg")
                sens = new iDynInvSensorLeg(limb->asChain(),"left",NE_DYNAMIC);
            else
                sens = new iDynInvSensorLeg(limb->asChain(),"right",NE_DYNAMIC);
        }

        sensor = new iFTransform(HS.submatrix(0,2,0,2),HS.submatrix(0,2,0,3).getCol(3));

        FTB = new iFB(sensorLink);
        FTB->attach(chain);
        FTB->attach(sensor);

        linEst =new AWLinEstimator(16,1.0);
        quadEst=new AWQuadEstimator(25,1.0);

        int jnt1=0;
        int jnt2=0;

        iencs->getAxes(&jnt1);
        encoders.resize(jnt1);

        if (tt)
        {
            tencs->getAxes(&jnt2);
            encodersT.resize(jnt2);
        }

        int jnt=jnt1+jnt2;

        q.resize(jnt);
        dq.resize(jnt);
        d2q.resize(jnt);

        q=dq=d2q=0.0;

        limb->setAng(q);
        limb->setDAng(dq);
        limb->setD2Ang(d2q);
        limb->prepareNewtonEuler(NE_DYNAMIC);

        w0.resize(3);
        dw0.resize(3);
        d2p0.resize(3);
        Fend.resize(3);
        Mend.resize(3);
        w0=dw0=d2p0=Fend=Mend=0.0;
        d2p0(0)=9.81;

        F_measured.resize(6);
        F_iDyn.resize(6);
        F_offset.resize(6);
        FT.resize(6);

        F_measured.zero();
        F_iDyn.zero();
        F_offset.zero();
        FT.zero();

        limb->initNewtonEuler(w0,dw0,d2p0,Fend,Mend);

        time = Time::now();
    }

    bool threadInit()
    {       
        return true;
    }

    void run()
    {   
        iencs->getEncoders(encoders.data());

        if (part == "left_arm" || part == "right_arm")
        {
            tencs->getEncoders(encodersT.data());
            for (int i=0;i<3;i++)
                q(i) = encodersT(2-i);
            for (int i=3;i<q.length();i++)
                q(i) = encoders(i-3);
        }
        else
        {
            for (int i=1;i<q.length();i++)
                q(i) = encoders(i);
        }

        Vector dq = evalVel(q);
        Vector d2q = evalAcc(q);

        limb->setAng(CTRL_DEG2RAD * q);
        limb->setDAng(CTRL_DEG2RAD * dq);
        limb->setD2Ang(CTRL_DEG2RAD * d2q);

        limb->computeNewtonEuler(w0,dw0,d2p0,Fend,Mend);
        sens->computeSensorForceMoment();
        F_iDyn = -1.0 * sens->getSensorForceMoment();

        ft = port_FT->read(false);

        if (ft!=0)
        {
            F_measured = *ft;

            if (first)
            {
                F_offset = F_measured-F_iDyn;
                first = false;
            }

            FT = FTB->getFB(F_measured - F_offset - F_iDyn);
        }
        else
        {
            if (!first)
                FT = FTB->getFB();
            else
            {
                FT=0.0;
                F_measured=0.0;
                F_offset=0.0;
            }
        }

        port_Contact->prepare() = FT;
        port_Contact->setEnvelope(info);
        port_Contact->write();
        time = Time::now();
    }

    void threadRelease()
    {
        if (sens)
        {
            delete sens;
            sens = 0;
        }

        if (limb)
        {
            delete limb;
            limb = 0;
        }

        if (linEst)
        {
            delete linEst;
            linEst = 0;
        }

        if (quadEst)
        {
            delete quadEst;
            quadEst = 0;
        }

        if (ft)
        {
            delete ft;
            ft = 0;
        }

        if (port_Contact)
        {
            port_Contact->interrupt();
            port_Contact->close();

            delete port_Contact;
            port_Contact = 0;
        }
    }   
};


class wrenchObserver: public RFModule
{
private:
    Property OptionsLimb;
    Property OptionsTorso;
    inverseDynamics *inv_dyn;
    string handlerPortName;
    Port handlerPort;      //a port to handle messages 

    PolyDriver *dd;
    PolyDriver *tt;
    BufferedPort<Vector>* port_FT;

public:
    wrenchObserver()
    {
        inv_dyn = 0;
        dd=0;
        tt=0;
    }

    virtual bool createDriver(PolyDriver *_dd)
    {
        // Creating Driver for Limb...
        if (!_dd || !(_dd->isValid()))
        {
            fprintf(stderr,"It is not possible to instantiate the device driver\nreturning...");
            return 0;
        }

        IEncoders *encs;

        bool ok = true;
        ok = ok & _dd->view(encs);
        if (!ok)
        {
            fprintf(stderr,"ERROR: one or more devices has not been viewed\nreturning...");
            return false;
        }

        return true;
    }

    bool configure(ResourceFinder &rf)
    {
        string PortName;
        string local;
        string part;
        string robot;
        string fwdSlash = "/";
        Bottle tmp;
        int rate = 100;
        tmp=0;

        string name;
        if (rf.check("name"))
            name = rf.find("name").asString();
        else name = "ftObs";
        PortName = (PortName+fwdSlash+name);

        //---------------------ROBOT-----------------------------//
        ConstString robotName=rf.find("robot").asString();
        if (rf.check("robot"))
        {
            //PortName=fwdSlash+rf.find("robot").asString().c_str();
            robot = rf.find("robot").asString().c_str();
        }
        else
        {
            fprintf(stderr,"Device not found\n");
            //PortName=fwdSlash+"icub";
            robot = "icub";
        }

        //---------------------PART-----------------------------//
        ConstString partName=rf.find("part").asString();
        if (rf.check("part"))
        {
            PortName=PortName+fwdSlash+rf.find("part").asString().c_str();
            part = rf.find("part").asString().c_str();
        }
        else
        {
            fprintf(stderr,"Could not find part in the config file\n");
            return false;
        }
        //---------------------RATE-----------------------------//
        if (rf.check("rate"))
        {
            rate = rf.find("rate").asInt();
            fprintf(stderr,"rateThread working at %d ms\n", rate);
        }
        else
        {
            fprintf(stderr,"Could not find rate in the config file\nusing 100ms as default");
            rate = 100;
            return false;
        }

        //---------------------PORT--------------------------//
        port_FT=new BufferedPort<Vector>;
        port_FT->open((PortName+"/FT:i").c_str());

        //---------------------DEVICES--------------------------//
        if (part=="left_arm" || part=="right_arm")
        {
            OptionsTorso.put("robot",robot.c_str());
            OptionsTorso.put("part","torso");
            OptionsTorso.put("device","remote_controlboard");
            OptionsTorso.put("local",(fwdSlash+name+"/torso/client").c_str());
            OptionsTorso.put("remote","/icub/torso");

            tt = new PolyDriver(OptionsTorso);
            if (!createDriver(tt))
            {
                fprintf(stderr,"ERROR: unable to create limb device driver...quitting\n");
                return false;
            }
            else
                fprintf(stderr,"device driver created\n");
        }

        OptionsLimb.put("robot",robot.c_str());
        OptionsLimb.put("part",part.c_str());
        OptionsLimb.put("device","remote_controlboard");
        OptionsLimb.put("local",(fwdSlash+name+fwdSlash+part+"/client").c_str());
        OptionsLimb.put("remote",(fwdSlash+robot+fwdSlash+part).c_str());
        dd = new PolyDriver(OptionsLimb);
        if (!createDriver(dd))
        {
            fprintf(stderr,"ERROR: unable to create limb device driver...quitting\n");
            return false;
        }

        //--------------------------THREAD--------------------------
        if (part=="left_arm" || part=="right_arm")
        {
            inv_dyn = new inverseDynamics(rate,  dd, tt, port_FT, part.c_str(), name.c_str());
            fprintf(stderr,"ft thread istantiated...\n");
            inv_dyn->start();
            fprintf(stderr,"thread started\n");
        }
        else if (part=="left_leg" || part=="right_leg")
        {
            inv_dyn = new inverseDynamics(rate,  dd, NULL, port_FT, part.c_str(), name.c_str());
            fprintf(stderr,"ft thread istantiated...\n");
            inv_dyn->start();
            fprintf(stderr,"thread started\n");
        }
        else
        {    
            fprintf(stderr,"unable to istantiate thread!!!");
            return false;
        }

        return true;
    }

    bool close()
    {
        fprintf(stderr,"closing... \n");

        if (inv_dyn)
            inv_dyn->stop();

        if (inv_dyn)
        {
            delete inv_dyn;
            inv_dyn=0;
        }

        if (port_FT)
        {
            port_FT->interrupt();
            port_FT->close();

            delete port_FT;
            port_FT=0;
        }

        if (dd)
        {
            delete dd;
            dd=0;
        }

        return true;
    }

    double getPeriod()  { return 1.0;  }
    bool updateModule() { return true; }
};


int main(int argc, char * argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--context context: where to find the called resource (referred to $ICUB_ROOT/app: default wrechObserver/conf)" << endl;
        cout << "\t--from       from: the name of the file.ini to be used for calibration"                                        << endl;
        cout << "\t--rate       rate: the period used by the module. default 100ms (not less than 15ms)"                          << endl;
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    wrenchObserver obs;

    return obs.runModule(rf);
}

