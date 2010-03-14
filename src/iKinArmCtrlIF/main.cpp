/** 
\defgroup iKinArmCtrlIF iKinArmCtrlIF
 
@ingroup icub_module  
 
The YARP interface version of \ref iKinArmCtrl.
 
Copyright (C) 2009 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
 
This module relies on the YARP ICartesianControl interface to 
implement the same functionalities of iKinArmCtrl module. 
Please refer to \ref iKinArmCtrl for a detailed description.
 
The main differences with respect to \ref iKinArmCtrl are: 
-# \ref iKinArmCtrlIF controls joint by joint individually, 
   hence one can use the non-controlled joints concurrently
   without conflict.
-# \ref iKinArmCtrlIF does not have a <i>simulation</i> mode 
 since it is deeply connected to the robot. Nonethelesse, you
 can still use it together with the \ref icub_Simulation "iCub
 Simulator".
 
\section lib_sec Libraries 
- YARP libraries. 

\section parameters_sec Parameters
--modName \e name 
- The parameter \e name identifies the module's name; all the 
  open ports will be tagged with the prefix
  /<modName>/<part>/. If not specified \e iKinArmCtrlIF is
  assumed.
 
--ctrlName \e name 
- The parameter \e name identifies the cartesian controller's 
  name to connect to. If not specified \e
  icub/cartesianController is assumed.
 
--part \e type 
- The parameter \e type selects the robot's arm to work with. It
  can be \e right_arm or \e left_arm; if not specified
  \e right_arm is assumed.
 
--T \e time
- specify the task execution time in seconds; by default \e time
  is 2.0 seconds. Note that this just an approximation of
  execution time since there exists a controller running
  underneath.
 
--DOF8
- enable the control of torso yaw joint. 
 
--DOF9
- enable the control of torso yaw/pitch joints. 
 
--DOF10
- enable the control of torso yaw/roll/pitch joints. 
 
--onlyXYZ  
- disable orientation control. 
 
\section portsa_sec Ports Accessed
 
Assumes that \ref icub_iCubInterface (with ICartesianControl 
interface implemented) is running. 
 
\section portsc_sec Ports Created 
 
The module creates the ports required for the communication with
the robot (through interfaces) and the following ports: 
 
- \e /<ctrlName>/<part>/xd:i receives the target end-effector 
  pose. It accepts 7 double (also as a Bottle object): 3 for xyz
  and 4 for orientation in axis/angle mode.

- \e /<ctrlName>/<part>/rpc remote procedure call. 
    Recognized remote commands:
    -'quit' quit the module

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files
None. 
 
\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>

#include <gsl/gsl_math.h>

#ifdef USE_ICUB_MOD
    #include "drivers.h"
#endif

#include <iostream>
#include <iomanip>
#include <string>

#define MAX_TORSO_PITCH     30.0    // [deg]
#define EXECTIME_THRESDIST  0.3     // [m]
#define PRINT_STATUS_PER    1.0     // [s]

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


class CtrlThread: public RateThread
{
protected:
    ResourceFinder      &rf;
    PolyDriver          *client;
    ICartesianControl   *arm;
    BufferedPort<Bottle> port_xd;

    bool ctrlCompletePose;
    string remoteName;
    string localName;

    Vector xd;
    Vector od;

    double defaultExecTime;
    double t0;

public:
    CtrlThread(unsigned int _period, ResourceFinder &_rf,
               string _remoteName, string _localName) :
               RateThread(_period),     rf(_rf),
               remoteName(_remoteName), localName(_localName) { }

    virtual bool threadInit()
    {
        // get params from the RF
        if (rf.check("onlyXYZ"))
            ctrlCompletePose=false;
        else
            ctrlCompletePose=true;

        // open the client
        Property option("(device cartesiancontrollerclient)");
        option.put("remote",remoteName.c_str());
        option.put("local",localName.c_str());

        client=new PolyDriver;
        if (!client->open(option))
        {
            delete client;    
            return false;
        }

        // open the view
        client->view(arm);

        // set trajectory time
        defaultExecTime=rf.check("T",Value(2.0)).asDouble();

        // set torso dofs
        Vector newDof, curDof;
        arm->getDOF(curDof);
        newDof=curDof;

        if (rf.check("DOF10"))
        {    
            // torso joints completely enabled
            newDof[0]=1;
            newDof[1]=1;
            newDof[2]=1;

            limitTorsoPitch();
        }
        else if (rf.check("DOF9"))
        {    
            // torso yaw and pitch enabled
            newDof[0]=1;
            newDof[1]=0;
            newDof[2]=1;

            limitTorsoPitch();
        }
        else if (rf.check("DOF8"))
        {                
            // only torso yaw enabled
            newDof[0]=0;
            newDof[1]=0;
            newDof[2]=1;
        }
        else
        {    
            // torso joints completely disabled
            newDof[0]=0;
            newDof[1]=0;
            newDof[2]=0;
        }

        arm->setDOF(newDof,curDof);

        // set tracking mode
        arm->setTrackingMode(false);

        // init variables
        while (true)
            if (arm->getPose(xd,od))
                break;

        // open ports
        port_xd.open((localName+"/xd:i").c_str());

        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            cout<<"Thread started successfully"<<endl;
        else
            cout<<"Thread did not start"<<endl;

        t0=Time::now();
    }

    virtual void run()
    {
        if (Bottle *b=port_xd.read(false))
        {                
            if (b->size()>=3)
            {                                
                for (int i=0; i<3; i++)
                    xd[i]=b->get(i).asDouble();

                if (ctrlCompletePose && b->size()>=7)
                {    
                    for (int i=0; i<4; i++)
                        od[i]=b->get(3+i).asDouble();
                }

                const double execTime=calcExecTime(xd);

                if (ctrlCompletePose)
                    arm->goToPose(xd,od,execTime);
                else
                    arm->goToPosition(xd,execTime);
            }
        }

        printStatus();
    }

    virtual void threadRelease()
    {    
        arm->stopControl();
        delete client;

        port_xd.interrupt();
        port_xd.close();
    }

    double norm(const Vector &v)
    {
        return sqrt(dot(v,v));
    }

    void limitTorsoPitch()
    {
        int axis=0; // pitch joint
        double min, max;

        arm->getLimits(axis,&min,&max);
        arm->setLimits(axis,min,MAX_TORSO_PITCH);
    }

    double calcExecTime(const Vector &xd)
    {
        Vector x,o;
        arm->getPose(x,o);

        if (norm(xd-x)<EXECTIME_THRESDIST)
            return defaultExecTime;
        else
            return 1.5*defaultExecTime;
    }

    void printStatus()
    {
        double t=Time::now();

        if (t-t0>=PRINT_STATUS_PER)
        {
            Vector x,o,xdhat,odhat,qdhat;

            arm->getPose(x,o);
            arm->getDesired(xdhat,odhat,qdhat);
            double e_x=norm(xdhat-x);

            cout<< "xd          [m] = "<<xd.toString()   <<endl;
            cout<< "xdhat       [m] = "<<xdhat.toString()<<endl;
            cout<< "x           [m] = "<<x.toString()    <<endl;
            cout<< "norm(e_x)   [m] = "<<e_x             <<endl;

            if (ctrlCompletePose)
            {
                double e_o=norm(odhat-o);

                cout<< "od        [rad] = "<<od.toString()   <<endl;
                cout<< "odhat     [rad] = "<<odhat.toString()<<endl;
                cout<< "o         [rad] = "<<o.toString()    <<endl;
                cout<< "norm(e_o) [rad] = "<<e_o             <<endl;
            }

            cout<<endl;

            t0=t;
        }
    }
};



class CtrlModule: public RFModule
{
protected:
    CtrlThread *thr;
    Port        rpcPort;

public:
    CtrlModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        string slash="/";
        string modName;
        string ctrlName;
        string partName;
        string remoteName;
        string localName;

        Time::turboBoost();

        // get params from the RF
        modName=rf.check("modName",Value("iKinArmCtrlIF")).asString();
        ctrlName=rf.check("ctrlName",Value("icub/cartesianController")).asString();
        partName=rf.check("part",Value("right_arm")).asString();

        remoteName=slash+ctrlName+slash+partName;
        localName=slash+modName+slash+partName;

        thr=new CtrlThread(20,rf,remoteName,localName);
        if (!thr->start())
        {
            delete thr;
            return false;
        }

        rpcPort.open((localName+"/rpc").c_str());
        attach(rpcPort);

        return true;
    }

    virtual bool close()
    {
        thr->stop();
        delete thr;

        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};



int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--modName  name: module name (default iKinArmCtrlIF)"                              << endl;
        cout << "\t--ctrlName name: controller name to connect to (default icub/cartesianController)" << endl;
        cout << "\t--part     type: robot arm type, left_arm or right_arm (default: right_arm)"       << endl;
        cout << "\t--T        time: specify the task execution time in seconds (default: 2.0)"        << endl;
        cout << "\t--DOF10        : control the torso yaw/roll/pitch as well"                         << endl;
        cout << "\t--DOF9         : control the torso yaw/pitch as well"                              << endl;
        cout << "\t--DOF8         : control the torso yaw as well"                                    << endl;
        cout << "\t--onlyXYZ      : disable orientation control"                                      << endl;

        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

#ifdef USE_ICUB_MOD
    DriverCollection dev;
#endif

    CtrlModule mod;

    return mod.runModule(rf);
}



