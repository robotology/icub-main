/** 
\defgroup demoReachManager_IIT_ISR demoReachManager_IIT_ISR
 
@ingroup icub_module  
 
The manager module for the Joint Reaching Demo developed by IIT 
and ISR. 

Copyright (C) 2009 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
This module collects the 3-d object positions estimated by the 
particle filter and sends data to the head and arm controllers 
in order to gaze at and reach for the target. 
 
\note A video on iCub performing grasp-priming can be seen at
      http://eris.liralab.it/misc/icubvideos/reaching_IIT_ISR.wmv

\section lib_sec Libraries 
- YARP libraries. 

\section parameters_sec Parameters
None. 
 
\section portsa_sec Ports Accessed
iCubInterface is supposed to be running. 
 
\section portsc_sec Ports Created 
 
- \e /demoReachManager_IIT_ISR/trackTarget:i receives the 3-d 
  position to track.
 
- \e /demoReachManager_IIT_ISR/cmdHead:o sends out commands to 
  the iKinGazeCtrl module in order to control the gaze.
 
- \e /demoReachManager_IIT_ISR/cmdHand:o sends out commands to 
  the iKinArmCtrl module in order to control the arm for the
  reaching.
 
- \e /demoReachManager_IIT_ISR/rpc remote procedure 
    call. Recognized remote commands:
    -'quit' quit the module
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files
The configuration file passed through the option \e --from
should look like as follows:
 
\code 
[general] 
robot	icub            // the robot name 
part	right_arm       // the arm under control
period  10              // the thread period

[left_arm]
// the offset [m] to be added to the desired position 
offset		0.0 -0.13 0.0 
// the hand orientation [rad] given in axis-angle representation
orientation 0.064485 0.707066 0.704201 3.140572 

[right_arm]
offset		0.0 0.13 0.0
orientation	-0.012968 -0.721210 0.692595 2.917075 
\endcode 

\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <gsl/gsl_math.h>

#include <iostream>
#include <iomanip>
#include <string>
                                                             
using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;


class managerThread : public RateThread
{
protected:
    ResourceFinder &rf;

    string name;
    string robot;
    string part;

    BufferedPort<Vector> *inportTrackTarget;
    BufferedPort<Vector> *outportCmdHead;
    BufferedPort<Vector> *outportCmdHand;

    PolyDriver *drvTorso;
    IEncoders  *encTorso;

    Vector targetPos;
    Vector targetOffset;
    Vector handOrient;
    Vector torso;

    Matrix R,Rx,Ry,Rz;

    bool targetNew;
    bool goHand;

    double Ts;
    int    slot;

public:
    managerThread(const string &_name, ResourceFinder &_rf, unsigned int period) : 
                  RateThread(period), name(_name), rf(_rf)
    {
        Ts=getRate()/1000.0;
        slot=0;

        targetPos.resize(3);
        targetOffset.resize(3);
        handOrient.resize(4);
        torso.resize(3);

        targetPos=0.0;

        R=Rx=Ry=Rz=eye(3,3);

        targetNew=false;
        goHand=false;

        drvTorso=NULL;
    }

    virtual bool threadInit()
    {
        Bottle &bGeneral=rf.findGroup("general");
        if (bGeneral.isNull())
        {
            cout<<"general part is missing!"<<endl;
            return false;
        }

        robot=bGeneral.check("robot",Value("icub")).asString();
        part=bGeneral.check("part",Value("left_arm")).asString();

        Bottle &bPart=rf.findGroup(part.c_str());
        if (bPart.isNull())
        {
            cout<<part<<" part is missing!"<<endl;
            return false;
        }

        if (bPart.check("offset"))
        {
            Bottle &bOffset=bPart.findGroup("offset");
            if (bOffset.size()-1==3)
                for (int i=0; i<bOffset.size()-1; i++)
                    targetOffset[i]=bOffset.get(1+i).asDouble();
            else
            {
                cout<<"option size != 3"<<endl;
                return false;
            }
        }
        else
        {
            cout<<"offset option is missing!"<<endl;
            return false;
        }
        
        if (bPart.check("orientation"))
        {
            Bottle &bOrientation=bPart.findGroup("orientation");
            if (bOrientation.size()-1==4)
                for (int i=0; i<bOrientation.size()-1; i++)
                    handOrient[i]=bOrientation.get(1+i).asDouble();
            else
            {
                cout<<"option size != 4"<<endl;
                return false;
            }
        }
        else
        {
            cout<<"orientation option is missing!"<<endl;
            return false;
        }

        string slash="/";
        Property optTorso("(device remote_controlboard)");
        optTorso.put("remote",(slash+robot+"/torso").c_str());
        optTorso.put("local",(name+"/torso").c_str());

        drvTorso=new PolyDriver(optTorso);
        if (!drvTorso->isValid())
        {
            delete drvTorso;    
            return false;
        }

        drvTorso->view(encTorso);

        inportTrackTarget=new BufferedPort<Vector>;
        outportCmdHead   =new BufferedPort<Vector>;
        outportCmdHand   =new BufferedPort<Vector>;

        inportTrackTarget->open((name+"/trackTarget:i").c_str());
        outportCmdHead->open((name+"/cmdHead:o").c_str());
        outportCmdHand->open((name+"/cmdHand:o").c_str());

        return true;
    }

    /******************************************************/
    virtual void run()
    {
        getSensorData();

        commandHead();

        doReach();

        targetNew=false;

        if (++slot>=4)
            slot=0;
    }

    virtual void threadRelease()
    {
        inportTrackTarget->interrupt();
        outportCmdHead->interrupt();
        outportCmdHand->interrupt();

        inportTrackTarget->close();
        outportCmdHead->close();
        outportCmdHand->close();

        delete inportTrackTarget;
        delete outportCmdHead;
        delete outportCmdHand;

        if (drvTorso)
            delete drvTorso;
    }

    void getSensorData()
    {
        if (encTorso->getEncoders(torso.data()))
            R=rotx(torso[1])*roty(-torso[2])*rotz(-torso[0]);

        if (Vector *targetPosNew=inportTrackTarget->read(false))
        {    
            targetPos=*targetPosNew;
            targetNew=true;
            goHand=true;
        }
    }

    void commandHead()
    {
        if (targetNew)
        {
            outportCmdHead->prepare()=targetPos;
            outportCmdHead->write();
        }        
    }

    void doReach()
    {
        if (slot==0 && goHand)
        {
            Vector y=R.transposed()*(targetPos+targetOffset);
            limitRange(y);
            y=R*y;

            Vector &x=outportCmdHand->prepare();
            x.resize(7);

            for (int i=0; i<3; i++)
                x[i]=y[i];

            for (int i=0; i<4; i++)
                x[3+i]=handOrient[i];

            outportCmdHand->write();
        }
    }
    
    void limitRange(Vector &x)
    {               
        x[0]=x[0]>-0.1 ? -0.1 : x[0];       
    }

    Matrix &rotx(const double theta)
    {
        double t=(M_PI/180.0)*theta;
        double c=cos(t);
        double s=sin(t);

        Rx(1,1)=Rx(2,2)=c;
        Rx(1,2)=-s;
        Rx(2,1)=s;

        return Rx;
    }

    Matrix &roty(const double theta)
    {
        double t=(M_PI/180.0)*theta;
        double c=cos(t);
        double s=sin(t);

        Ry(0,0)=Ry(2,2)=c;
        Ry(0,2)=s;
        Ry(2,0)=-s;

        return Ry;
    }

    Matrix &rotz(const double theta)
    {
        double t=(M_PI/180.0)*theta;
        double c=cos(t);
        double s=sin(t);

        Rz(0,0)=Rz(1,1)=c;
        Rz(0,1)=-s;
        Rz(1,0)=s;

        return Rz;
    }
};


class managerModule: public RFModule
{
protected:
    managerThread *thr;    
    Port           rpcPort;

public:
    managerModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        int period;

        Bottle &general=rf.findGroup("general");
        if (!general.isNull())
            period=rf.check("period",Value((int)10)).asInt();
        else
        {
            cout<<"general part is missing!"<<endl;
            return false;
        }

        thr=new managerThread(getName().c_str(),rf,period);
        if (!thr->start())
        {
            delete thr;    
            return false;
        }

        rpcPort.open(getName("/rpc"));
        attach(rpcPort);

        return true;
    }

    virtual bool close()
    {
        rpcPort.interrupt();
        rpcPort.close();

        thr->stop();
        delete thr;

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};


int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("demoReach_IIT_ISR/conf");
    rf.setDefaultConfigFile("config.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    managerModule mod;
    mod.setName("/demoReachManager_IIT_ISR");

    return mod.runModule(rf);
}




