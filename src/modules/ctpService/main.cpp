/*
 * Copyright (C) 2010 RobotCub Consortium
 * Author: Lorenzo Natale
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/** 
\defgroup ctpService constantTimePositionService
 
@ingroup icub_module 
 
A simple service that allows you to send constant time position
commands to any of the robot ports.

\section intro_sec Description

Sit silently and waits for incoming commands. Receive position
commands requests and sends position commands, but it computes
the reference speeds so that commands are all executed in the same
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

Execute command. Command will take TIME seconds, the list of position will be sent to the motors
starting from offset j.
 
or 
 
[ctpq] [time] TIME (seconds) [off] j [pos] list 
 
 
Queue commands for execution. Command will take TIME seconds, the list of position will be sent to the motors
starting from offset j
 
or 
 
[ctpf] filename (in yarpdatadumper format)
 
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
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/LogStream.h>

#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <deque>
#include <vector>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;

#define VCTP_TIME       yarp::os::createVocab('t','i','m','e')
#define VCTP_OFFSET     yarp::os::createVocab('o','f','f')
#define VCTP_CMD_NOW    yarp::os::createVocab('c','t','p','n')
#define VCTP_CMD_QUEUE  yarp::os::createVocab('c','t','p','q')
#define VCTP_CMD_FILE   yarp::os::createVocab('c','t','p','f')
#define VCTP_POSITION   yarp::os::createVocab('p','o','s')
#define VCTP_WAIT       yarp::os::createVocab('w','a','i','t')

#define VEL_FILT_SIZE   10
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
    IControlMode     *mode;
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
            cerr<<"For debug --> joints: "<<nJoints<< " off: "<<offset<<" cmd length: "<<size<<endl;
            return;
        }

        Vector disp(size);

        if (time==0)
        {
            return;
        }

        for (size_t i=0; i<disp.length(); i++)
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

        // don't blend together the two "for"
        // since we have to enforce the modes on the whole
        // prior to commanding the joints
        std::vector<int>    joints;
        std::vector<int>    modes;
        std::vector<double> speeds;
        std::vector<double> positions;

        for (size_t i=0; i<disp.length(); i++)
        {
            joints.push_back(offset+i);
            speeds.push_back(disp[i]/time);
            modes.push_back(VOCAB_CM_POSITION);
            positions.push_back(x->getCmd()[i]);
        }

        mode->setControlModes(disp.length(), joints.data(), modes.data());
        yarp::os::Time::delay(0.01);  // give time to update control modes value
        mode->getControlModes(disp.length(), joints.data(), modes.data());
        for (size_t i=0; i<disp.length(); i++)
        {
            if(modes[i] != VOCAB_CM_POSITION)
            {
                yError() << "Joint " << i << " not in position mode";
            }
        }
        pos->setRefSpeeds(disp.length(), joints.data(), speeds.data());
        pos->positionMove(disp.length(), joints.data(), positions.data());

        cout << "Script port: " << x->getCmd().toString() << endl;
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

    bool configure(const Property &options)
    {
        bool ret=true;
        if (options.check("device"))
            drvOptions.put("device", options.find("device").asString());
        else
            drvOptions.put("device","remote_controlboard");

        string name=string(options.find("name").asString());

        string remote=string("/")+string(options.find("robot").asString());
        remote+=string("/")+string(options.find("part").asString());

        string local=name;
        local+=string("/local/")+string(options.find("part").asString());
   
        drvOptions.put("remote",remote);
        drvOptions.put("local",local);

        if (verbose)
        {
            cout << "Driver options:\n" << drvOptions.toString();
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
            connected=drv->view(mode) &&
                      drv->view(pos) && 
                      drv->view(enc);
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

class WorkingThread: public PeriodicThread
{
private:
    scriptPosPort *posPort;
public:
    WorkingThread(int period=100): PeriodicThread((double)period/1000.0)
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
        {
            return false;
        }
    }

    bool initVelCtrl(Port *p, ResourceFinder *rf)
    {
        int i = 0;
        bool cont = true;
        char numberId[64];
        bool ret = true;

        while(cont)
        {
            string gainStr = "gain";
            sprintf(numberId, "%d", i);
            gainStr = gainStr + numberId;

            Bottle c,r;
            c.addString("gain");
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
        if (i==0)
        {
            ret = false;
            cout << " velCtrl: missing 'gain' parameters" << endl;
        }

        i = 0;
        cont = true;
        while(cont)
        {
            string svelStr = "svel";
            sprintf(numberId, "%d", i);
            svelStr = svelStr + numberId;

            Bottle c,r;
            c.addString("svel");
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
        if (i==0)
        {
            ret = false;
            cout << " velCtrl: missing 'svel' parameters" << endl;
        }

        return ret;
    }

    bool readLine(Vector &v, double &time)
    {        
        fin.getline(&line[0],sizeof(line),'\n');
        string str(line);

        if (str.length()!=0)
        {
            Bottle b;
            b.fromString(str);

            if (b.size()>2)
            {
                time=b.get(1).asDouble();
                v.resize(b.size()-2);

                for (size_t i=0; i<v.length(); i++)
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

        for (size_t i=0; i<p.length(); i++)
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
            for (size_t i=0; i<p.length(); i++)
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
            // skip the first two lines
            fin.getline(&line[0],sizeof(line),'\n');
            fin.getline(&line[0],sizeof(line),'\n');
            firstRun=true;
            cout<<"File loaded"<<endl;
            mutex.post();
            return true;
        }
        else
        {
            cout<<"Unable to load file: "<<fileName<<endl;
            return false;
        }
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
                    bool b = initVelCtrl(velInitPort, pRF);
                    if (b)
                    {
                        cout << "velocity control initialized. gain, svel parameters loaded" << endl;
                    }
                    else
                    {
                        cout << "***  velocity control missing init parameters! (gain, svel)" << endl;
                    }
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
                    Vector cmd=formCommand(p1,t);
                    velPort->write(cmd);
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
    ResourceFinder *rf;
    Port            rpcPort;
    string          name;
    bool            verbose;
    scriptPosPort   posPort;
    Port            velPort;
    Port            velInitPort;
    WorkingThread   thread;
    VelocityThread  velThread;

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
        {
            posPort.sendNow(action);
            reply.addVocab(Vocab::encode("ack"));
        }
        
        return ret;
    }

    bool handle_ctp_queue(const Bottle &cmd, Bottle &reply)
    {
        ActionItem *action;
        bool ret=parsePosCmd(cmd, reply, &action);
        if (ret)
        {
            posPort.queue(action);
            reply.addVocab(Vocab::encode("ack"));
        }
        return ret;
    }

    bool handle_ctp_file(const Bottle &cmd, Bottle &reply)
    {
        if (cmd.size()<2)
            return false;

        string fileName=rf->findFile(cmd.get(1).asString());
        bool ret = velThread.go(fileName);
        if (ret)
        {
            reply.addVocab(Vocab::encode("ack"));
        }
        else
        {
            reply.addVocab(Vocab::encode("nack"));
            reply.addString("Unable to load file");
        }
        return ret;
    }

    bool handle_wait(const Bottle &cmd, Bottle &reply)
    {
        cerr<<"Warning command not implemented yet"<<endl;
        reply.addVocab(Vocab::encode("ack"));
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
                    if (!posCmd)
                        return false;

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
        this->rf=&rf;

        if (rf.check("name"))
            name=string("/")+rf.find("name").asString();
        else
            name="/ctpservice";

        rpcPort.open(name+string("/")+rf.find("part").asString()+"/rpc");
        attach(rpcPort);

        Property portProp;
        portProp.put("name", name);
        portProp.put("robot", rf.find("robot"));
        portProp.put("part", rf.find("part"));

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

        velPort.open(name+string("/")+rf.find("part").asString()+"/vc:o");
        velInitPort.open(name+string("/")+rf.find("part").asString()+"/vcInit:o");
        velThread.attachVelPort(&velPort);
        velThread.attachVelInitPort(&velInitPort);
        cout << "Using parameters:" << endl << rf.toString() << endl;
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
                case yarp::os::createVocab('h','e','l','p'):
                {                    
                    cout << "Available commands:"          << endl;
                    cout<<"Queue command:\n";
                    cout<<"[ctpq] [time] seconds [off] j [pos] (list)\n";
                    cout<<"New command, execute now (erase queue):\n";
                    cout<<"[ctpq] [time] seconds [off] j [pos] (list)\n";
                    cout<<"Load sequence from file:\n";
                    cout<<"[ctpf] filename\n";
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
    rf.setDefaultContext("ctpService");
    rf.configure(argc,argv);

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
        return 1;

    scriptModule mod;
    return mod.runModule(rf);
}



