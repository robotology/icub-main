/*
 * Copyright (C) 2010 RobotCub Consortium
 * Author: Lorenzo Natale
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/** 
\defgroup ctpService constantTimePositionService
 
@ingroup icub_module 
 
Hidden behind a sophisticated name lies a simple but useful 
service that will allow you to send constant time position
commands to any of the robot ports.

\section intro_sec Description

Sit silently and waits for incoming commands. Receive position
commands requests and sends position commands, but it computes
the reference speeds s.t. commands are all executed in the same
time irrespectively of the current position (the service reads 
the current encoders and compute the reference speed accordingly).

\section ports_sec Ports and Messages

- Input, for receiving commands:
 - /ctpservice/{part}/rpc

- Output, to interface with the robot, usual control board ports:
 - /ctpservice/local/{part}/rpc:o
 - /ctpservice/local/{part}/state:i
 - /ctpservice/local/{part}/command:o

- Output, to interface with the @ref icub_velocityControl 
  module:
 - /ctpservice/{part}/vc:o
 
Here {part} is replaced with the robot part (see --part parameter)

ctpservice is a default value that can be replaced with --name.

Incoming commands are the following:

[ctpn] [time] TIME(seconds) [off] j [pos] list

or 

[ctpq] [time] TIME (seconds) [off] j [pos] list 
 
or 
 
[ctpf] filename (in dataDumper format)
 
Example:
This requires 1 second movement of joints 5,6,7 to 10 10 10 respectively:
[ctpn] [time] 1 [off] 5 [pos] (10 10 10)

\section parameters_sec Parameters
Run as:

ctpService --robot robotname --part part (optionally: --name modulename)

robot: name of robot (e.g. icub)

part: prefix for part name (e.g. right_arm)

name: module name (used to form port names)

\section lib_sec Libraries 
- YARP libraries. 
 
\section tested_os_sec Tested OS
Windows, Linux

Copyright (C) 2010 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.
 
Author: Lorenzo Natale 
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Semaphore.h>

#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <deque>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;

#define VCTP_TIME VOCAB4('t','i','m','e')
#define VCTP_OFFSET VOCAB3('o','f','f')
#define VCTP_CMD_NOW VOCAB4('c','t','p','n')
#define VCTP_CMD_QUEUE VOCAB4('c','t','p','q')
#define VCTP_CMD_FILE VOCAB4('c','t','p','f')
#define VCTP_POSITION VOCAB3('p','o','s')
#define VCTP_WAIT VOCAB4('w','a','i','t')

#define VEL_FILT_SIZE   16
#define VEL_FILT_THRES  1.0

// Class for position control
class ActionItem
{
private:
    int off;
    Vector cmd;
    double time;
public:
    const int &getOffset() const { return off; }
    int &getOffset() { return off; }

    const double &getTime() const { return time; }
    double &getTime() {return time; }

    const Vector &getCmd() const { return cmd; }

    Vector &getCmd() { return cmd; }
};

typedef deque<ActionItem *> ActionList;

class Actions
{
    ActionList actions;

public:
    Actions(){}
    ~Actions(){}

    int size()
    {
        return actions.size();
    }

    void clear()
    {
        for(unsigned int k=0; k<actions.size();k++)
        {
            ActionItem *ret=actions.at(k);
            delete ret;
         }
        actions.clear();
    }

    ActionItem *pop()
    {
        if (actions.empty())
            return 0;

        ActionItem *ret=actions.at(0);
        actions.pop_front();
        return ret;
    }

    void push_back(ActionItem *v)
    {
        actions.push_back(v);
    }
};

class scriptPosPort
{
protected:
    bool verbose;
    bool connected;
    Property          drvOptions;
    PolyDriver       *drv;
    IPositionControl *pos;
    IEncoders        *enc;
    Actions actions;
    Semaphore mutex;
 
    void _send(const ActionItem *x)
    {
        if (!connected)
        {
            cerr<<"Error: not connected to control board skipping"<<endl;
            return;
        }

        int size=x->getCmd().size();
        int offset=x->getOffset();
        double time=x->getTime();
        int nJoints=0;

        enc->getAxes(&nJoints);
        if ((offset+size)>nJoints)
        {
            cerr<<"Error: detected possible overflow, skipping"<<endl;
            cerr<<"For debug --> joints: "<<nJoints<< " off: "<<offset<<" cmd lenght: "<<size<<endl;
            return;
        }

        Vector disp(size);

        if (time==0)
        {
            return;
        }

        for (int i=0; i<disp.length(); i++)
        {
            double q;

           
            if (!enc->getEncoder(offset+i,&q))
            {
                 cerr<<"Error: encoders timed out, cannot rely on encoder feedback, aborted"<<endl;
                 return;
            }
                
            disp[i]=x->getCmd()[i]-q;

            if (disp[i]<0.0)
                disp[i]=-disp[i];
        }

        for (int i=0; i<disp.length(); i++)
        {
            pos->setRefSpeed(offset+i,disp[i]/time);
            pos->positionMove(offset+i,x->getCmd()[i]);
        }

        cout << "Script port: " << const_cast<Vector &>(x->getCmd()).toString() << endl;
    }

public:
    scriptPosPort()
    {
        drvOptions.clear();
        drv=0;
        verbose=1;
        connected=false;
    }

    void send(ActionItem *tmp)
    {
        mutex.wait();
        _send(tmp);
        mutex.post();
    }

    bool configure(const Property &copt)
    {
        bool ret=true;
        Property &options=const_cast<Property &> (copt);

        if (options.check("device"))
            drvOptions.put("device", options.find("device").asString());
        else
            drvOptions.put("device","remote_controlboard");

        string name="/local/";

        string remote=string("/")+string(options.find("robot").asString());
        remote+=name+string(options.find("part").asString());

        string local=string("/")+string(options.find("robot").asString());
        local+=string("/")+string(options.find("part").asString());
   
        drvOptions.put("remote",local.c_str());
        drvOptions.put("local",remote.c_str());


        if (verbose)
        {
            cout << "Driver options:\n" << drvOptions.toString().c_str();
        }

        return ret;
     }

    void queue(ActionItem *item)
    {
        mutex.wait();
        actions.push_back(item);
        mutex.post();
    }

    void sendNow(ActionItem *item)
    {
        mutex.wait();
        actions.clear();
        _send(item);
        mutex.post();
    }

    ActionItem *pop()
    {
        mutex.wait();
        ActionItem *ret=actions.pop();
        mutex.post();
        return ret;
    }

    bool connect()
    {
        drv=new PolyDriver(drvOptions);

        if (drv->isValid())
            connected=drv->view(pos)&&drv->view(enc);
        else
            connected=false;

        if (!connected)
        {    
            delete drv;
            drv=0;
        }

        return connected;
    }

    ~scriptPosPort()
    {
        if (drv)
            delete drv;
    }
};

class WorkingThread: public RateThread
{
private:
    scriptPosPort *posPort;
public:
    WorkingThread(int period=100): RateThread(period)
    {}

    void attachPosPort(scriptPosPort *p)
    {
        if (p)
            posPort=p;
    }

    bool threadInit()
    {
        if (!posPort)
            return false;
        return true;
    }
        
    void run()
    {
        ActionItem *tmp=posPort->pop();
        if (tmp)
        {
            posPort->send(tmp);
            double time=tmp->getTime();
            delete tmp;
            Time::delay(time);
        }
    }
};

class VelocityThread: public Thread
{
private:
    Semaphore       mutex;
    AWLinEstimator  linEst;
    Port           *velPort;
    Port           *velInitPort;
    ResourceFinder *pRF;
    
    ifstream   fin;
    char       line[1024];
    bool       closing;
    bool       firstRun;
    bool       velInit;

    unsigned int filtElemsCnt;

    bool checkInitConnection(Port *p)
    {

        if(p->getOutputCount() > 0)
        {   
            cout << "***** got a connection" << endl;
            return true;
        }
        else
            return false;
    }

    void initVelCtrl(Port *p, ResourceFinder *rf)
    {
        int i = 0;
        bool cont = true;
        char numberId[64];

        while(cont)
        {
            ConstString gainStr = "gain";
            sprintf(numberId, "%d", i);
            gainStr = gainStr + numberId;

            Bottle c,r;
            c.add("gain");
            c.addInt(i);
            if (rf->check(gainStr))
            {
                c.addDouble(rf->find(gainStr).asDouble());
                p->write(c,r);
                i++;
            }
            else
                cont = false;
        }                

        i = 0;
        cont = true;
        while(cont)
        {
            ConstString svelStr = "svel";
            sprintf(numberId, "%d", i);
            svelStr = svelStr + numberId;

            Bottle c,r;
            c.add("svel");
            c.addInt(i);
            if (rf->check(svelStr))
            {
                c.addDouble(rf->find(svelStr).asDouble());
                p->write(c,r);
                i++;
            }
            else
                cont = false;
        }     
    }

    bool readLine(Vector &v, double &time)
    {        
        fin.getline(&line[0],sizeof(line),'\n');
        string str(line);

        if (str.length()!=0)
        {
            Bottle b;
            b.fromString(str.c_str());

            if (b.size()>2)
            {
                time=b.get(1).asDouble();
                v.resize(b.size()-2);

                for (int i=0; i<v.length(); i++)
                    v[i]=b.get(i+2).asDouble();

                return true;
            }
            else
                return false;            
        }
        else
            return false;
    }

    Vector formCommand(const Vector &p, const double t)
    {
        Vector res;

        for (int i=0; i<p.length(); i++)
        {
            res.push_back(i);
            res.push_back(p[i]);
        }

        AWPolyElement el;
        el.data=p;
        el.time=t;

        Vector v=linEst.estimate(el);

        if (++filtElemsCnt>VEL_FILT_SIZE)
        {
            for (int i=0; i<p.length(); i++)
            {
                res.push_back(1000.0+i);
                res.push_back(v[i]);
            }
        }

        return res;
    }

public:
    VelocityThread() : mutex(0), linEst(VEL_FILT_SIZE,VEL_FILT_THRES)
    {
        velInit=false;
        velPort=NULL;
        velInitPort=NULL;
        closing=false;
        firstRun=true;
    }

    void attachRF(ResourceFinder *attachedRF)
    {
        pRF = attachedRF;
    }

    void attachVelPort(Port *p)
    {
        if (p)
            velPort=p;
    }

    void attachVelInitPort(Port *p)
    {
        if (p)
            velInitPort=p;
    }

    bool go(const string &fileName)
    {
        if (velPort==NULL)
            return false;

        if (fin.is_open())
            fin.close();

        fin.open(fileName.c_str());        

        if (fin.is_open())
        {
            fin.seekg(0,ios_base::beg);
            firstRun=true;
            mutex.post();
            return true;
        }
        else
            return false;
    }

    void onStop()
    {
        closing=true;
        mutex.post();
    }

    void threadRelease()
    {
        if (fin.is_open())
            fin.close();
    }

    void run()
    {
        Vector p1,p2;
        double time1,time2,t;
        bool   send=false;

        while (!isStopping())
        {
			if (!velInit)
			{
                if (checkInitConnection(velInitPort))
                {
                    initVelCtrl(velInitPort, pRF);
                    velInit = true;
                }
                else
                    Time::delay(1.0);
			}
			else
			{
    			if (!closing)
    				mutex.wait();
    
    			if (firstRun)
    			{
                    Bottle c,r;
                    c.addString("run");
                    velInitPort->write(c,r);
    				send=readLine(p1,time1);
                    linEst.reset();
                    t=0.0;
                    filtElemsCnt=0;
    				firstRun=false;
    			}            
    
    			if (send)
    			{
    				velPort->write(formCommand(p1,t));
    				send=false;
    			}
    
    			if (readLine(p2,time2))
    			{
                    double dt=time2-time1;
    				Time::delay(dt);
                    t+=dt;
    				p1=p2;
    				time1=time2;
    				send=true;
    				mutex.post();
    			}
    			else
                {
    				fin.close();
                    Bottle c,r;
                    c.addString("susp");
                    velInitPort->write(c,r);
                }
			}
        }
    }
};

class scriptModule: public RFModule
{
protected:
    Port           rpcPort;
    string         name;
    string         contextPath;
    bool           verbose;
    scriptPosPort  posPort;
    Port           velPort;
    Port           velInitPort;
    WorkingThread  thread;
    VelocityThread velThread;

public:
    scriptModule() 
    {
        verbose=true;
    }

    bool handlectpm(const Bottle &cmd, Bottle &reply)
    {
        ActionItem *action;
        bool ret=parsePosCmd(cmd, reply, &action);
        if (ret)
            posPort.sendNow(action);
        return ret;
    }

    bool handle_ctp_queue(const Bottle &cmd, Bottle &reply)
    {
        ActionItem *action;
        bool ret=parsePosCmd(cmd, reply, &action);
        if (ret)
            posPort.queue(action);
        return ret;
    }

    bool handle_ctp_file(const Bottle &cmd, Bottle &reply)
    {
        if (cmd.size()<2)
            return false;
        
        string fileName=contextPath+"/"+cmd.get(1).asString().c_str();
        return velThread.go(fileName);
    }

    bool handle_wait(const Bottle &cmd, Bottle &reply)
    {
        cerr<<"Warning command not implemented yet"<<endl;
        return true;
        //ActionItem *action;
        //bool ret=parseWaitCmd(cmd, reply, &action);
        //if (ret)
        //   posPort.queue(action);
    }

    bool parsePosCmd(const Bottle &cmd, Bottle &reply, ActionItem **action)
    {
        const int expectedMsgSize=7;
        bool parsed=false;
        if (cmd.size()!=expectedMsgSize)
        {
            return false;
        }

        if (cmd.get(1).asVocab()==VCTP_TIME)
        {
            double time=cmd.get(2).asDouble();
            int offset=0;
            Bottle *posCmd=0;
            if (cmd.get(3).asVocab()==VCTP_OFFSET)
            {
                offset=cmd.get(4).asInt();
                if (cmd.get(5).asVocab()==VCTP_POSITION)
                {
                    posCmd=cmd.get(6).asList();
                    //check posCmd!=0
                    parsed=true;
                    if (verbose)
                    {
                        cout<<"Received command:"<<endl;
                        cout<<"Time: " << time << "offset: " << offset <<" Cmd: " << posCmd->toString()<<endl;
                    }
                    *action=new ActionItem;
                    (*action)->getCmd().resize(posCmd->size());
                    for(int k=0;k<posCmd->size();k++)
                        (*action)->getCmd()[k]=posCmd->get(k).asDouble();

                    (*action)->getOffset()=offset;
                    (*action)->getTime()=time;
                    return true;
                }
            }
        }
        return false;
    }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        if (rf.check("name"))
            name=string("/")+rf.find("name").asString().c_str();
        else
            name="/ctpservice";

        contextPath=rf.getContextPath().c_str();

        rpcPort.open((name+string("/")+rf.find("part").asString().c_str()+"/rpc").c_str());
        attach(rpcPort);

        rf.find("part").asString();
        rf.find("robot").asString();

        Property portProp;
        portProp.put("part", rf.find("part"));
        portProp.put("robot", rf.find("robot"));

        if (!posPort.configure(portProp))
        {
            cerr<<"Error configuring position controller, check parameters"<<endl;
            return false;
        }
        
        if (!posPort.connect())
        {
            cerr<<"Error cannot conenct to remote ports"<<endl;
            return false;
        }

        cout << "***** connect to rpc port and type \"help\" for commands list" << endl;

        thread.attachPosPort(&posPort);
        if (!thread.start())
        {
            cerr<<"Thread did not start, queue will not work"<<endl;
        }

        velPort.open((name+string("/")+rf.find("part").asString().c_str()+"/vc:o").c_str());
        velInitPort.open((name+string("/")+rf.find("part").asString().c_str()+"/vcInit:o").c_str());
        velThread.attachVelPort(&velPort);
        velThread.attachVelInitPort(&velInitPort);
        velThread.attachRF(&rf);
        cout << "***** starting the thread" << endl;
        velThread.start();

        return true;
    }

    virtual bool respond(const Bottle &command, Bottle &reply)
    {
        bool ret;
        if (command.size()!=0)
        {
            switch (command.get(0).asVocab())
            {
                case VOCAB4('h','e','l','p'):
                {                    
                    cout << "Available commands:"          << endl;
                    reply.addVocab(Vocab::encode("ack"));
                    return true;
                }
                case VCTP_CMD_NOW:
                    {
                        ret=handlectpm(command, reply);
                        return ret;
                    }
                case VCTP_CMD_QUEUE:
                    {
                        ret=handle_ctp_queue(command, reply);
                        return ret;
                    }
                case VCTP_CMD_FILE:
                    {
                        ret=handle_ctp_file(command, reply);
                        return ret;
                    }
                case VCTP_WAIT:
                    {
                        ret=handle_wait(command, reply);
                    }
                default:
                    return RFModule::respond(command,reply);
            }
        }
        else
        {
            reply.addVocab(Vocab::encode("nack"));
            return false;
        }
    }

    virtual bool close()
    {
        rpcPort.interrupt();
        rpcPort.close();
        thread.stop();

        velThread.stop();
        velPort.interrupt();
        velPort.close();

        velInitPort.interrupt();
        velInitPort.close();

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};


int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("ctpService/conf");
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--name moduleName: set new module name"      << endl;
        cout << "\t--robot robotname: robot name"      << endl;
        cout << "\t--part partname: robot part name" << endl;
        cout << "\t--from   fileName: input configuration file" << endl;
        cout << "\t--context dirName: resource finder context"  << endl;

        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    scriptModule mod;
 
    return mod.runModule(rf);
}



