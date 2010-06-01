/** 
\defgroup iKinArmView iKinArmView
 
@ingroup icub_tools 
 
A viewer for the iCub arm implemented in MATLAB.

Copyright (C) 2008 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description 
 
This module handles the communication between YARP based program 
such as \ref iKinArmCtrl "iKinArmCtrl" and the GUI developed in
MATLAB with the purpose to display the actual iCub arm 
configuration. It requires the user just to connect some ports 
whose description is detailed hereafter. 
 
The GUI will look like this: 
\image html iKinArmView.jpg 
 
Installation notes: 
 
Within CMake please assign the following two paths (example for 
Windows is given): 

- INCLUDE_DIRS   => ${MATLAB_ROOT}/extern/include
- LIBRARIES_DIRS => ${MATLAB_ROOT}/extern/lib/win32/microsoft 
 
For Windows users only: if you did not register MATLAB as COM 
server during installation, on the command line you can issue 
the following: matlab /regserver 
 
For Linux users only: make sure that \e csh shell is installed 
and MATLAB is in the path. 
 
\section lib_sec Libraries 
- YARP libraries. 
- MATLAB installed. 

\section parameters_sec Parameters
--name \e name 
- The parameter \e name identifies the module's name; all the 
  open ports will be tagged with the prefix <name>/. If not
  specified \e /iKinArmView is assumed.
 
--arm \e arm 
- The parameter \e arm selects the type of iCub arm to display. 
  It can be \e right or \e left; if not specified
  \e right is assumed.
 
--visibility \e switch 
- The parameter \e switch set MATLAB session visibility on/off 
  (off by default).
  
\section portsa_sec Ports Accessed
The ports the module is connected to.

\section portsc_sec Ports Created 
 
- \e <name>/xd:o the port where the target end-effector pose 
  selected through the graphical interface is sent. The pose
  comprises 7 double: 3 for xyz and 4 for orientation in
  angle/axis mode.
 
- \e <name>/q:i receives the joints angles in order to display 
  the arm configuration (Vector of 10 double: 3 torso_angles + 7
  arm_angles). The order for torso_angles is the one defined by
  kinematic chain (reversed order).
 
- \e <name>/rpc remote procedure call port useful to shut down 
  the module remotely by sending to this port the 'quit'
  command.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files
None. 
 
\section tested_os_sec Tested OS
Windows and Linux. 
 
\section example_sec Example
Try with the following: 
 
\code 
on terminal 1: iKinArmCtrl 
 
on terminal 2: iKinArmView 
 
on terminal 3: 
- yarp connect /iKinArmCtrl/q:o  /iKinArmView/q:i 
- yarp connect /iKinArmView/xd:o /iKinArmCtrl/xd:i 
\endcode 
 
\author Ugo Pattacini

This file can be edited at 
\in src/iKinArmView/main.cpp. 
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>

// MATLAB Engine Library
#include "engine.h"

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;


bool runViewer(Engine *ep, const string &armType);


class rxPort : public BufferedPort<Bottle>
{
public:
    rxPort(unsigned int _dim) : dim(_dim) { curr.resize(dim); curr=0.0; }
    Vector &get_curr()                    { return curr;                }

private:
    unsigned int dim;
    Vector curr;

    virtual void onRead(Bottle &b)
    {
        for (unsigned int i=0; i<dim; i++)
            curr[i]=b.get(i).asDouble();
    }
};



class GatewayThread : public RateThread
{
private:
    rxPort               *port_q;
    BufferedPort<Vector> *port_xd;
    unsigned int          dim;

    int    period;
    string visibility;
    string portName;
    string armType;

    Engine  *ep;
    mxArray *mlBuffer;
    mxArray *mlIdx;
    unsigned int N,idx;

    Vector xdOld;

    double t0;

public:
    GatewayThread(int _period, const string &_portName, const string &_armType,
                  const string &_visibility) :
                  RateThread(_period), period(_period), portName(_portName),
                  armType(_armType), visibility(_visibility)
    {
        dim=10;
        N  =500;
        idx=1;

        xdOld.resize(7);
        xdOld=0.0;

        port_q=NULL;
        port_xd=NULL;

        ep=NULL;
        mlBuffer=NULL;
        mlIdx=NULL;
    }

    virtual bool threadInit()
    {
        port_q=new rxPort(dim);
        port_q->useCallback();
        string n1=portName+"/q:i";
        port_q->open(n1.c_str());

        port_xd=new BufferedPort<Vector>();
        string n2=portName+"/xd:o";
        port_xd->open(n2.c_str());

    #ifdef WIN32
        int retstatus;
        if (!(ep=engOpenSingleUse(NULL,NULL,&retstatus)))
    #else
        if (!(ep=engOpen("\0")))
    #endif
        {
            cerr << "Opening MATLAB engine failed!" << endl;
            dispose();
            return false;
        }

        if (visibility=="on")
            engSetVisible(ep,1);
        else
            engSetVisible(ep,0);

        engEvalString(ep,"cd([getenv('ICUB_ROOT') '/app/iKinView/scripts'])");

        if (!runViewer(ep,armType))
        {
            cerr << "Unable to locate MATLAB script" << endl;
            dispose();
            return false;
        }

        if (!(mlBuffer=mxCreateDoubleMatrix(N,dim+1,mxREAL)))
        {
            cerr << "Unable to create mxMatrix" << endl;
            dispose();
            return false;
        }

        if (!(mlIdx=mxCreateDoubleScalar(idx)))
        {
            cerr << "Unable to create mxMatrix" << endl;
            dispose();
            return false;
        }

        cout << "Starting main thread..." << endl;

        return true;
    }

    virtual void afterStart(bool s)
    {
        t0=Time::now();

        if (s)
            cout << "Thread started successfully" << endl;
        else
            cout << "Thread did not start" << endl;
    }

    virtual void run()
    {
        mxArray *ml_xd=engGetVariable(ep,"xd");

        if (ml_xd)
        {
            mwSize n=mxGetNumberOfElements(ml_xd);

            if (n>=7)
            {
                double *_xd=mxGetPr(ml_xd);
                Vector xd(7);

                xd[0]=_xd[0];
                xd[1]=_xd[1];
                xd[2]=_xd[2];
                xd[3]=_xd[3];
                xd[4]=_xd[4];
                xd[5]=_xd[5];
                xd[6]=_xd[6];

                if (!(xd==xdOld))
                {
                    Vector &xdP=port_xd->prepare();
                    xdP=xd;
                    port_xd->write();
                }

                xdOld=xd;
            }

            mxDestroyArray(ml_xd);
        }

        double t=Time::now()-t0;
        Vector q=port_q->get_curr();

        *(double*)mxGetPr(mlIdx)=idx;
        double *ptr=(double*)mxGetPr(mlBuffer);

        ptr[idx-1]=t;
        for (unsigned int i=0; i<dim; i++)
            ptr[(idx-1)+(i+1)*N]=q[i];

        if (engPutVariable(ep,"Buffer",mlBuffer))
            cerr << "Unable to update MATLAB workspace" << endl;        

        if (engPutVariable(ep,"idx",mlIdx))
            cerr << "Unable to update MATLAB workspace" << endl;

        if (++idx>N)
            idx=1;
    }

    virtual void threadRelease()
    {
        dispose();
    }

    void dispose()
    {
        if (port_q)
        {
            port_q->interrupt();
            port_q->close();
            delete port_q;
        }

        if (port_xd)
        {
            port_xd->interrupt();
            port_xd->close();
            delete port_xd;
        }

        if (mlBuffer)
            mxDestroyArray(mlBuffer);

        if (mlIdx)
            mxDestroyArray(mlIdx);

        if (ep)
            engClose(ep);
    }
};



class GatewayModule: public RFModule
{
private:
    GatewayThread *thread;
    Port           rpcPort;
    string         portName;    

public:
    GatewayModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        string armType;
        string visibility;

        Time::turboBoost();

        if (rf.check("name"))
            portName=rf.find("name").asString();
        else
            portName="/iKinArmView";

        if (rf.check("arm"))
            armType=rf.find("arm").asString();
        else
            armType="right";

        if (rf.check("visibility"))
            visibility=rf.find("visibility").asString();
        else
            visibility="off";

        thread=new GatewayThread(20,portName,armType,visibility);
        if (!thread->start())
        {
            delete thread;
            return false;
        }

        string rpcPortName=portName+"/rpc";
        rpcPort.open(rpcPortName.c_str());
        attach(rpcPort);

        return true;
    }

    virtual bool close()
    {
        thread->stop();
        delete thread;

        rpcPort.interrupt();
        rpcPort.close();        

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};



bool runViewer(Engine *ep, const string &armType)
{
    char ret_string[10];
    char ok[]="ok";

    string command="if exist('iKinArmView.m','file'), disp('ok'); iKinArmView('";
    command+=armType;
    command+="'), else disp('err'); end";

    engOutputBuffer(ep,&ret_string[0],10);
    engEvalString(ep,command.c_str());
    engOutputBuffer(ep,NULL,0);
    
    if (strpbrk(ret_string,ok))
        return true;
    else        
        return false;
}



int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--name         name: port name (default /iKinArmView)"                   << endl;
        cout << "\t--arm           arm: iCub arm left or right (default: right)"            << endl;
        cout << "\t--visibility switch: set MATLAB session visibility on/off (default off)" << endl;

        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    GatewayModule mod;

    return mod.runModule(rf);
}


