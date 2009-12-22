/** 
\defgroup iKinGazeView iKinGazeView
 
@ingroup icub_module 
 
A viewer for the iCub gaze implemented in MATLAB.

Copyright (C) 2008 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description 
 
This module handles the communication between YARP based program 
such as \ref iKinGazeCtrl "iKinGazeCtrl" and the GUI developed 
in MATLAB with the purpose to display the actual iCub gaze 
configuration (head posture and lines of sight). It requires the
user just to connect some ports whose description is detailed 
hereafter. 
 
The GUI will look like this: 
\image html iKinGazeView.jpg 
 
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
- \ref iKin "iKin" library (IPOPT is not used, so the dependence
  can be switched off from within CMake).
- MATLAB installed. 

\section parameters_sec Parameters
--name \e name 
- The parameter \e name identifies the module's name; all the 
  open ports will be tagged with the prefix <name>/. If not
  specified \e /iKinGazeView is assumed.
 
--config \e file 
- The parameter \e file specifies the file name used for 
  aligning eyes kinematic to image planes (see below).
 
--visibility \e switch 
- The parameter \e switch set MATLAB session visibility on/off 
  (off by default). 
  
\section portsa_sec Ports Accessed
The ports the module is connected to.

\section portsc_sec Ports Created 
 
- \e <name>/xd:o the port where the target to gaze at, selected
  through the graphical interface, is sent. The target comprises
  3 double for xyz position.
 
- \e <name>/q:i receives the joints angles in order to display 
  the gaze configuration (Vector of 9 double: 3 torso_angles + 6
  head_angles). The order for torso_angles is the one defined by
  kinematic chain (reversed order).
 
- \e <name>/rpc remote procedure call port useful to shut down 
  the module remotely by sending to this port the 'quit'
  command.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files 
The configuration file passed through the option \e --config 
should contain the fields required to reconstruct the 
rototranslation matrices which align the kinematic eye axis with
optical axis.
 
Example 
 
\code 
[LEFT]
R      0.0324596 -0.0329982 0.0229948               // [rad]
T      8.9604e-009 1.51339e-007 9.28864e-007        // [m]

[RIGHT]
R      0.0872497 -0.0872643 -0.0546534              // [rad]
T      -4.21262e-007 4.47732e-008 -9.24493e-007     // [m]
\endcode 
 
\section tested_os_sec Tested OS
Windows and Linux. 
 
\section example_sec Example
Try with the following: 
 
\code 
on terminal 1: iKinGazeCtrl 
 
on terminal 2: iKinGazeView 
 
on terminal 3: 
- yarp connect /iKinGazeCtrl/q:o  /iKinGazeView/q:i 
- yarp connect /iKinGazeView/xd:o /iKinGazeCtrl/xd:i 
\endcode 
 
\author Ugo Pattacini

This file can be edited at 
\in src/iKinGazeView/main.cpp. 
*/ 

#include <ace/OS.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

#include <iCub/iKinFwd.h>

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
using namespace yarp::math;


bool runViewer(Engine *ep);


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
    string portName;
    string configFile;
    string visibility;

    Matrix alignHL;
    Matrix alignHR;

    Engine  *ep;
    mxArray *mlBuffer;
    mxArray *mlIdx;
    unsigned int N,idx;

    double t0;

    Vector xdOld;

public:
    GatewayThread(int _period, const string &_portName, const string &_configFile,
                  const string &_visibility) : RateThread(_period), period(_period),
                  portName(_portName), configFile(_configFile), visibility(_visibility)
    {
        dim=9;
        N  =500;
        idx=1;

        alignHL=alignHR=eye(4,4);        

        if (configFile.size())
        {
            Property par;
            par.fromConfigFile(configFile.c_str());
    
            getAlignH(par,"LEFT",alignHL);
            getAlignH(par,"RIGHT",alignHR);
        }

        xdOld.resize(3);
        xdOld=0.0;
    }

    void getAlignH(Property &par, const string &type, Matrix &H)
    {
        Bottle parType=par.findGroup(type.c_str());
        string error="unable to find aligning parameters for "+type+" eye";

        if (parType.size())
        {
            Bottle R=parType.findGroup("R");
            Bottle T=parType.findGroup("T");

            if (R.size()>=4 && T.size()>=4)
            {
                Vector v(4); v=0.0;
                v[0]=R.get(1).asDouble();
                v[1]=R.get(2).asDouble();
                v[2]=R.get(3).asDouble();
                double m=norm(v);
                if (m)
                    v=(1/m)*v;
                v[3]=m;

                H=axis2dcm(v);
                H(0,3)=T.get(1).asDouble();
                H(1,3)=T.get(2).asDouble();
                H(2,3)=T.get(3).asDouble();
            }
            else
                cerr << error << endl;
        }
        else
            cerr << error << endl;
    }

    virtual bool threadInit()
    {
    #ifdef WIN32
        int retstatus;
        if (!(ep=engOpenSingleUse(NULL,NULL,&retstatus)))
    #else
        if (!(ep=engOpen("\0")))
    #endif
        {
            cerr << "Opening MATLAB engine failed!" << endl;
            return false;
        }

        if (visibility=="on")
            engSetVisible(ep,1);
        else
            engSetVisible(ep,0);

        engEvalString(ep,"cd([getenv('ICUB_ROOT') '/app/iKinView/scripts'])");

        // copy the aligning matrices in the workspace 
        if (mxArray *mlH=mxCreateDoubleMatrix(4,4,mxREAL))
        {
            double *ptr=(double*)mxGetPr(mlH);

            for (unsigned int i=0; i<4; i++)
                for (unsigned int j=0; j<4; j++)
                    ptr[i+4*j]=alignHL(i,j);

            if (engPutVariable(ep,"alignHL",mlH))
                cerr << "Unable to update MATLAB workspace" << endl;

            for (unsigned int i=0; i<4; i++)
                for (unsigned int j=0; j<4; j++)
                    ptr[i+4*j]=alignHR(i,j);

            if (engPutVariable(ep,"alignHR",mlH))
                cerr << "Unable to update MATLAB workspace" << endl;

            mxDestroyArray(mlH);
        }
        else
        {
            cerr << "Unable to create mxMatrix" << endl;
            engClose(ep);
            return false;
        }

        if (!runViewer(ep))
        {
            cerr << "Unable to locate MATLAB script" << endl;
            engClose(ep);
            return false;
        }

        if (!(mlBuffer=mxCreateDoubleMatrix(N,dim+1,mxREAL)))
        {
            cerr << "Unable to create mxMatrix" << endl;
            engClose(ep);
            return false;
        }

        if (!(mlIdx=mxCreateDoubleScalar(idx)))
        {
            cerr << "Unable to create mxMatrix" << endl;
            engClose(ep);
            return false;
        }

        port_q=new rxPort(dim);
        port_q->useCallback();
        string n1=portName+"/q:i";
        port_q->open(n1.c_str());

        port_xd=new BufferedPort<Vector>();
        string n2=portName+"/xd:o";
        port_xd->open(n2.c_str());

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

            if (n>=3)
            {
                double *_xd=mxGetPr(ml_xd);
                Vector xd(3);

                xd[0]=_xd[0];
                xd[1]=_xd[1];
                xd[2]=_xd[2];

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
        port_xd->interrupt();
        port_q->interrupt();

        port_xd->close();
        port_q->close();

        delete port_q;
        delete port_xd;

        mxDestroyArray(mlBuffer);
        mxDestroyArray(mlIdx);

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
        string configFile;
        string visibility;

        Time::turboBoost();

        if (rf.check("name"))
            portName=rf.find("name").asString();
        else
            portName="/iKinGazeView";

        if (rf.check("config"))
            if ((configFile=rf.findFile(rf.find("config").asString()))=="")
                return false;
        else
            configFile.clear();

        if (rf.check("visibility"))
            visibility=rf.find("visibility").asString();
        else
            visibility="off";

        thread=new GatewayThread(20,portName,configFile,visibility);
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



bool runViewer(Engine *ep)
{
    char ret_string[10];
    char ok[]="ok";

    string command="if exist('iKinGazeView.m','file'), disp('ok'); iKinGazeView, else disp('err'); end";

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
        cout << "\t--name         name: port name (default /iKinGazeView)"                     << endl;
        cout << "\t--config       file: file name for aligning eyes kinematic to image planes" << endl;
        cout << "\t--visibility switch: set MATLAB session visibility on/off (default off)"    << endl;

        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    GatewayModule mod;

    return mod.runModule(rf);
}


