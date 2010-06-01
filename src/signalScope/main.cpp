/** 
\defgroup signalScope signalScope
 
@ingroup icub_guis
 
A MATLAB scope which displays signal acquired form a YARP port 
along with its FFT. 

Copyright (C) 2008 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description 
 
A MATLAB script launched by this module displays data acquired 
from YARP port. Two type of plot are provided: the stripchart 
and the spectrogram. 
 
The Scope will look like this: 
\image html signalScope.jpg 
 
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
- The parameter \e name identifies the module's name; if not 
  specified /signalScope is assumed.
 
--channel \e chn 
- The parameter \e chn identifies the vector element to be 
  acquired (first element by default).
 
--Fs \e Fs 
- The parameter \e Fs specifies the sample rate in Hz (50 Hz by 
  default).
 
--width \e w 
- The parameter \e w specifies the time window in seconds to be 
  displayed (10 s by default).
 
--Nfft \e N 
- The parameter \e N specifies the FFT length (1024 by default).
  Note that the update period of the spectrogram is given by
  Nfft/Fs seconds.
 
--visibility \e switch 
- The parameter \e switch set MATLAB session visibility on/off 
  (off by default).
  
\section portsa_sec Ports Accessed
The ports the module is connected to.

\section portsc_sec Ports Created 
 
- \e <name>:i (e.g. /signalScope:i) receives the input data 
  vector.
 
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
 
- signalScope --Fs 100
- yarp connect /icub/right_arm/state:o /signalScope:i 
\endcode 
 
\author Ugo Pattacini

This file can be edited at 
\in src/signalScope/main.cpp. 
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
#include <deque>

// MATLAB Engine Library
#include "engine.h"

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;


bool checkScope(Engine *ep);
void openScope(Engine *ep, int Fs, int width, int Nfft);
void updateStrip(Engine *ep, const string &val);
void updateSpectr(Engine *ep, const string &val);


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
    rxPort               *inputPort;
    BufferedPort<Bottle>  rpcPort;

    int    chn;
    int    Fs;
    int    width;
    int    Nfft;
    string visibility;
    string portName;

    Engine  *ep;
    mxArray *mlValStrip;
    mxArray *mlValSpectr;
    double  *ptrStrip;
    double  *ptrSpectr;

    unsigned int NStrip,   NSpectr;
    unsigned int cntStrip, cntSpectr;

public:
    GatewayThread(const string &_portName, const int _chn, const int _Fs,
                  const int _width, const int _Nfft, const string &_visibility) :
                  RateThread(1000/_Fs), portName(_portName), chn(_chn), Fs(_Fs),
                  width(_width), Nfft(_Nfft), visibility(_visibility)
    {
        NStrip=(unsigned int)(0.1*Fs+1.0);
        NStrip=NStrip<2 ? 2 : NStrip;

        NSpectr=Nfft;

        cntStrip=cntSpectr=0;
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

        engEvalString(ep,"cd([getenv('ICUB_ROOT') '/app/signalScope/scripts'])");

        if (!checkScope(ep))
        {
            cerr << "Unable to locate MATLAB scripts" << endl;
            engClose(ep);
            return false;
        }

        openScope(ep,Fs,width,Nfft);

        if (!(mlValStrip=mxCreateDoubleMatrix(NStrip,1,mxREAL)))
        {
            cerr << "Unable to create mxMatrix" << endl;
            engClose(ep);
            return false;
        }

        if (!(mlValSpectr=mxCreateDoubleMatrix(NSpectr,1,mxREAL)))
        {
            cerr << "Unable to create mxMatrix" << endl;
            engClose(ep);
            return false;
        }

        ptrStrip =(double*)mxGetPr(mlValStrip);
        ptrSpectr=(double*)mxGetPr(mlValSpectr);

        inputPort=new rxPort(chn--);
        inputPort->useCallback();
        inputPort->open((portName+":i").c_str());

        cout << "Starting main thread..." << endl;

        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            cout << "Thread started successfully" << endl;
        else
            cout << "Thread did not start" << endl;
    }

    virtual void run()
    {
        string valStrip="valStrip";
        string valSpectr="valSpectr";

        Vector v=inputPort->get_curr();
        ptrStrip[cntStrip]=v[chn];
        ptrSpectr[cntSpectr]=v[chn];

        if (++cntStrip>=NStrip)
        {
            if (engPutVariable(ep,valStrip.c_str(),mlValStrip))
                cerr << "Unable to update MATLAB workspace" << endl;

            updateStrip(ep,valStrip.c_str());

            cntStrip=0;
        }

        if (++cntSpectr>=NSpectr)
        {
            if (engPutVariable(ep,valSpectr.c_str(),mlValSpectr))
                cerr << "Unable to update MATLAB workspace" << endl;

            updateSpectr(ep,valSpectr.c_str());

            cntSpectr=0;
        }
    }

    virtual void threadRelease()
    {
        inputPort->interrupt();
        inputPort->close();

        delete inputPort;

        mxDestroyArray(mlValStrip);
        mxDestroyArray(mlValSpectr);

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
        string visibility;
        int chn, Fs, width, Nfft;

        Time::turboBoost();

        if (rf.check("name"))
            portName=rf.find("name").asString();
        else
            portName="/signalScope";

        if (rf.check("channel"))
            chn=rf.find("channel").asInt();
        else
            chn=1;

        if (rf.check("Fs"))
            Fs=rf.find("Fs").asInt();
        else
            Fs=50;

        if (rf.check("width"))
            width=rf.find("width").asInt();
        else
            width=10;

        if (rf.check("Nfft"))
            Nfft=rf.find("Nfft").asInt();
        else
            Nfft=1024;

        if (rf.check("visibility"))
            visibility=rf.find("visibility").asString();
        else
            visibility="off";

        thread=new GatewayThread(portName,chn,Fs,width,Nfft,visibility);
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



bool checkScope(Engine *ep)
{
    char ret_string[10];
    char ok[]="ok";

    string command="if exist('stripchart.m','file') && exist('specgramscope.m','file')";
    command+=", disp('ok'), else disp('err'); end";

    engOutputBuffer(ep,&ret_string[0],10);
    engEvalString(ep,command.c_str());
    engOutputBuffer(ep,NULL,0);

    if (strpbrk(ret_string,ok))
        return true;
    else        
        return false;
}



void openScope(Engine *ep, int Fs, int width, int Nfft)
{
    char cFs[10], cwidth[10], cNfft[10];

    sprintf(cFs,"%d",Fs);
    sprintf(cwidth,"%d",width);
    sprintf(cNfft,"%d",Nfft);

    string sFs(cFs), swidth(cwidth), sNfft(cNfft);

    string command="figure('Color','w'); hax1=subplot(211); hax2=subplot(212);";
    command+="stripchart(hax1,"+sFs+","+swidth+");";
    command+="specgramscope(hax2,"+sFs+","+sNfft+");";

    engEvalString(ep,command.c_str());
}



void updateStrip(Engine *ep, const string &val)
{
    string command="stripchart(hax1,"+val+");";
    engEvalString(ep,command.c_str());
}



void updateSpectr(Engine *ep, const string &val)
{
    string command="specgramscope(hax2,"+val+");";
    command+="lim=ylim(hax2); ylim(hax2,[0 lim(2)])";
    engEvalString(ep,command.c_str());
}



int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--name         name: port name (default /signalScope)"                       << endl;
        cout << "\t--channel       chn: element to be displayed within the vector (default 1)"  << endl;
        cout << "\t--Fs             Fs: sample rate in Hz (default 50)"                         << endl;
        cout << "\t--width           w: display the most recent w seconds of data (default 10)" << endl;
        cout << "\t--Nfft            N: FFT length (default 1024)"                              << endl;
        cout << "\t--visibility switch: set MATLAB session visibility on/off (default off)"     << endl;

        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return false;

    GatewayModule mod;

    return mod.runModule(rf);
}


