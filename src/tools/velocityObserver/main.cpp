/**
@ingroup icub_tools

\defgroup velocityObserver velocityObserver
 
Estimates the first and second derivatives of incoming data 
vector through a least-squares based algorithm on a adpative 
window
 
Copyright (C) 2008 RobotCub Consortium
 
Author: Ugo Pattacini 
 
Date: first release 24/10/2008 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description

This module computes the first derivative (velocity) and the 
second derivative (acceleration) of vector acquired through an 
input YARP port and provides them to output YARP ports. The 
estimation is perfomed relying on a least-squares algorithm 
(<a 
href="http://ieeexplore.ieee.org/iel5/87/19047/00880606.pdf">PDF</a>) 
which finds the best linear or quadratic regressor upon a 
certain window of the input samples. The window's length is 
adaptable, i.e. it varies according the smoothness of the input 
signal. Therefore, the more smooth is the input signal the 
larger will be the window and the delay introduced by the 
computation will be negligible. For fast change in the input 
signal, the window's length is reduced in order to best capture 
the derivative and to limit the latency. 
 
Note that no knowledge of the sampling time is required since to
perform the computation the module relies on the time
information stored by the sender within the envelope of the 
message. If the envelope is not valid, then the Time Stamp is 
the reference time of the machine where the module is running 
(i.e. the arrival time): of course in this case the computation 
will be much less precise due to the variable YARP communication
latencies. 
 
The figure below shows a snapshot of the estimated velocity and 
acceleration along with a comparison between the proposed 
algorithm and a simple moving average filter: 
\image html velocityObserver.jpg
 
\section lib_sec Libraries 
- YARP libraries. 
- ctrlLib library. 

\section parameters_sec Parameters
--name \e name 
- The parameter \e name identifies the module's name; all the 
  open ports will be tagged with the prefix <name>/. If not
  specified \e /velObs is assumed.
 
--lenVel \e N 
- The parameter \e N identifies the maximum window length for 
  buffering the input data (used for the first derivative
  estimation). Therefore, the window on which the least-squares
  algorithm operates on will automatically adapts its length
  from a minimum of 2 to a maximum of N samples. The greater is
  N, the more filtered is the derivative.
 
--lenAcc \e N 
- The same as above but for the second derivative's estimation.
 
--thrVel \e D 
- With this option it is possibile to specify the maximum 
  deviation threshold for the algorithm (used for the first derivative
  estimation). During the validation phase, indeed, the window's
  length is considered valid if all the computed values from the
  linear regressor lie wihin this threshold wrt their
  corresponding sample values. If not the window's length is
  reduced by one step. Each instance of the computation a trial
  on a window's length enlarged by one step wrt to the current
  one is performed. Therefore, iteration by iteration, the
  window's length can be adapted by only one step change.

--thrAcc \e D 
- The same as above but for the second derivative's estimation.
 
\section portsa_sec Ports Accessed
The port the service is listening to.

\section portsc_sec Ports Created
 
- \e <name>/pos:i (e.g. /velObs/pos:i) receives the input data 
  vector.
 
- \e <name>/vel:o (e.g. /velObs/vel:o) provides the estimated 
  first derivatives.

- \e <name>/acc:o (e.g. /velObs/acc:o) provides the estimated 
  second derivatives.
 
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
Linux and Windows.

\section example_sec Example
By launching the following command: 
 
\code 
velocityObserver --name /jointVel --lenVel 20 --thrVel 2.0 
\endcode 
 
the module will create the listening port /jointVel/pos:i for 
the acquisition of data vector coming for instance from one of 
the icub ports. At the same time it will provide the estimated 
derivatives to /jointVel/vel:o /jointVel/acc:o ports. Here a 
value of 20 samples is chosen for the velocity maximum window's 
length and the velocity maximum permitted tolerance is 2.0; for 
the acceleration default values are used (use --help option to 
see). 
 
Try now the following: 
 
\code 
yarp connect /icub/right_arm/state:o /jointVel/pos:i
\endcode 
 
\author Ugo Pattacini

This file can be edited at \in src/velocityObserver/main.cpp.
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include <iostream>
#include <iomanip>
#include <string>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace ctrl;


// A class which handles the incoming data.
// The estimated derivatives are returned at once
// since they are computed within the onRead method.
class dataCollector : public BufferedPort<Bottle>
{
private:
    BufferedPort<Vector> &port_vel;
    BufferedPort<Vector> &port_acc;
    AWLinEstimator       *linEst;
    AWQuadEstimator      *quadEst;

    unsigned int NVel,NAcc;
    double DVel,DAcc;

    virtual void onRead(Bottle &b)
    {
        Stamp info;
        BufferedPort<Bottle>::getEnvelope(info);

        size_t sz=b.size();
        Vector x(sz);

        for (unsigned int i=0; i<sz; i++)
            x[i]=b.get(i).asDouble();

        AWPolyElement el;
        el.data=x;

        // for the esteem the time stamp
        // is required. If not present within the
        // packet, the actual machine time is 
        // attached to it.
        if (info.isValid())
            el.time=info.getTime();
        else
            el.time=Time::now();

        port_vel.prepare()=linEst->estimate(el);       
        port_acc.prepare()=quadEst->estimate(el);

        // the outbound packets will carry the same
        // envelope information of the inbound ones.
        port_vel.setEnvelope(info);
        port_acc.setEnvelope(info);
        port_vel.write();
        port_acc.write();
    }

public:
    dataCollector(unsigned int _NVel, double _DVel, BufferedPort<Vector> &_port_vel,
                  unsigned int _NAcc, double _DAcc, BufferedPort<Vector> &_port_acc) :
                  NVel(_NVel), DVel(_DVel), port_vel(_port_vel),
                  NAcc(_NAcc), DAcc(_DAcc), port_acc(_port_acc)
    {
        linEst =new AWLinEstimator(NVel,DVel);
        quadEst=new AWQuadEstimator(NAcc,DAcc);
    }

    ~dataCollector()
    {
        delete linEst;
        delete quadEst;
    }
};



// Usual YARP stuff...
class velObserver: public RFModule
{
private:
    dataCollector        *port_pos;
    BufferedPort<Vector>  port_vel;
    BufferedPort<Vector>  port_acc;
    Port                  rpcPort;

public:
    velObserver() { }

    virtual bool configure(ResourceFinder &rf)
    {
        string portName;
        unsigned int NVel,NAcc;
        double DVel,DAcc;

        Time::turboBoost();

        if (rf.check("name"))
            portName=rf.find("name").asString();
        else
            portName="/velObs";

        if (rf.check("lenVel"))
        {
            NVel=rf.find("lenVel").asInt();

            if (NVel<2)
            {
                cout << "Warning: lenVel cannot be lower than 2 => N=2 is assumed" << endl;    
                NVel=2;
            }
        }
        else
            NVel=16;

        if (rf.check("lenAcc"))
        {
            NAcc=rf.find("lenAcc").asInt();

            if (NAcc<3)
            {
                cout << "Warning: lenAcc cannot be lower than 3 => N=3 is assumed" << endl;    
                NAcc=3;
            }
        }
        else
            NAcc=25;

        if (rf.check("thrVel"))
        {
            DVel=rf.find("thrVel").asDouble();

            if (DVel<0.0)
            {
                cout << "Warning: thrVel cannot be lower than 0.0 => D=0.0 is assumed" << endl;    
                DVel=0.0;
            }
        }
        else
            DVel=1.0;

        if (rf.check("thrAcc"))
        {
            DAcc=rf.find("thrAcc").asDouble();

            if (DAcc<0.0)
            {
                cout << "Warning: thrAcc cannot be lower than 0.0 => D=0.0 is assumed" << endl;    
                DAcc=0.0;
            }
        }
        else
            DAcc=1.0;

        string n1=portName+"/vel:o";
        port_vel.open(n1.c_str());

        string n2=portName+"/acc:o";
        port_acc.open(n2.c_str());

        port_pos=new dataCollector(NVel,DVel,port_vel,NAcc,DAcc,port_acc);
        port_pos->useCallback();
        string n3=portName+"/pos:i";
        port_pos->open(n3.c_str());

        string rpcPortName=portName+"/rpc";
        rpcPort.open(rpcPortName.c_str());
        attach(rpcPort);

        return true;
    }

    virtual bool close()
    {
        port_pos->interrupt();
        port_vel.interrupt();
        port_acc.interrupt();
        rpcPort.interrupt();

        port_pos->close();
        port_vel.close();
        port_acc.close();
        rpcPort.close();

        delete port_pos;

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
        cout << "\t--name   name: observer port name (default /velObs)"                << endl;
        cout << "\t--lenVel    N: velocity window's max length (default: 16)"          << endl;
        cout << "\t--thrVel    D: velocity max deviation threshold (default: 1.0)"     << endl;
        cout << "\t--lenAcc    N: acceleration window's max length (default: 25)"      << endl;
        cout << "\t--thrAcc    D: acceleration max deviation threshold (default: 1.0)" << endl;

        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    velObserver obs;

    return obs.runModule(rf);
}


