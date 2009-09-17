/** 
\defgroup scriptSequencer scriptSequencer 
 
@ingroup icub_module 
 
A data sequencer over YARP.

Copyright (C) 2008 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
 
A simple sequencer which sends data over YARP according to a 
table loaded from a configuration file. 
Learn from enclosed example how to prepare a configuration file 
:) 
 
Implemented ports are of streaming and position type, but it is 
straightforward to add new types: just inherit from the abstract
father class and define few methods as detailed within the code.
 
Type "help" from within the console for a list of available 
commands. 
Basically, at start-up the thread is suspended; you can 
therefore choose the mode (between the self-explaining "shot" 
and "cyc") and then issue a "run". 
 
\section lib_sec Libraries 
- YARP libraries. 
 
\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini
*/ 

#include <ace/OS.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <deque>

#define SHOT    0x00
#define CYCLE   0x01

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;


typedef enum { stream, pos } portType;

// if you want to add a new type, you need to inherit from scriptPort class
// and define methods for handling options, inserting items in the list, connectig
// and sending data.
// you also need to extend parser capabilities

// Father class
class scriptPort
{
protected:
    portType      type;
    string        name;
    bool          active;
    deque<double> absTime;
    deque<bool>   interpFlag;
    deque<Vector> data;

    int    size;
    int    idx;
    int    numEntries;
    double t;

    bool connected;
    bool completed;

    virtual void send(const Vector &x)=0;

public:
    scriptPort() : size(0), idx(0), numEntries(0), t(0.0), type(stream),
                   active(true), connected(false), completed(false) { }

    virtual void setOptions(Property &options)=0;
    virtual void push_back(const Vector &item)=0;
    virtual bool connect()=0;

    virtual bool acquireData(Bottle &b, const string &modName)
    {        
        if (b.check("Name"))
            setName((string)b.find("Name").asString());
        else
        {
            cout << "Name option is missing!" << endl;
            return false;
        }

        if (b.check("Active"))
            setActive(b.find("Active").asString()=="ON");
        else
        {
            cout << "Active option is missing!" << endl;
            return false;
        }

        if (b.check("NumEntries"))
            numEntries=b.find("NumEntries").asInt();
        else
        {
            cout << "NumEntries option is missing!" << endl;
            return false;
        }

        return true;
    }

    void setType(const portType _type)    { type=_type;        }
    void setName(const string &_name)     { name=_name;        }
    void setActive(const bool activeFlag) { active=activeFlag; }

    void acquireEntries(Bottle &b)
    {
        for (int j=0; j<numEntries; j++)
        {
            char Entry[255];
            sprintf(Entry,"Entry_%d",j);
            Bottle bEntry=b.findGroup(Entry);
            int sz=bEntry.size();

            Vector item(sz-1);
            for (int k=1; k<sz; k++)
                item[k-1]=bEntry.get(k).asDouble();

            push_back(item);
        }
    }

    bool process(double Ts)
    {
        if (!active)
            return true;

        if (idx>=size)
            return true;
        else if (t>=absTime[idx])
        {
            send(data[idx]);

            if (++idx>=size)
                return completed=true;
            else
                return false;
        }
        else if (idx>0)
            if (interpFlag[idx-1])
            {
                Vector x0=data[idx-1];
                Vector x1=data[idx];

                double ratio=(t-absTime[idx-1])/(absTime[idx]-absTime[idx-1]);
                ratio=ratio<1.0 ? ratio : 1.0;

                send(x0+(x1-x0)*ratio);
            }

        t+=Ts;

        return false;
    }

    portType getType()    { return type;      }
    string   getName()    { return name;      }
    bool     getActive()  { return active;    }
    int      getSize()    { return size;      }
    int      getIdx()     { return idx;       }
    bool     isComplete() { return completed; }

    void init()
    {
        t=0.0;
        idx=0;
        completed=false;
    }

    ~scriptPort()
    {
        absTime.clear();
        interpFlag.clear();
        data.clear();
    }
};


// Class to connect to control port
class scriptStreamPort : public scriptPort
{
protected:
    BufferedPort<Vector> *p;
    string srcName, destName;

    void send(const Vector &x)
    {
        if (connected)
        {
            p->prepare()=x;
            p->write();
        }
        
        cout << name << ": " << ((Vector)x).toString() << endl;
    }

public:
    scriptStreamPort() : scriptPort()
    {
        p=new BufferedPort<Vector>;
    }

    bool acquireData(Bottle &b, const string &modName)
    {
        if (!scriptPort::acquireData(b,modName))
            return false;

        string destName;

        if (b.check("Port"))
            destName=b.find("Port").asString();
        else
        {
            cout << "Port option is missing!" << endl;
            return false;
        }

        Property options;
        string sourceName=modName+destName;
        options.put("sourceName",sourceName.c_str());
        options.put("destName",destName.c_str());

        setOptions(options);

        acquireEntries(b);

        return true;
    }

    void setOptions(Property &options)
    {
        if (options.check("sourceName"))
            srcName=options.find("sourceName").asString();

        if (options.check("destName"))
            destName=options.find("destName").asString();
    }

    void push_back(const Vector &item)
    {
        int sz=item.length()-2;
        Vector x(sz);
        for (int i=0; i<sz; i++)
            x[i]=item[2+i];

        absTime.push_back(item[0]);
        interpFlag.push_back((bool)item[1]);
        data.push_back(x);

        size++;
    }

    bool connect()
    {
        p->open(srcName.c_str());
        return connected=Network::connect(srcName.c_str(),destName.c_str());
    }

    ~scriptStreamPort()
    {
        if (!p->isClosed())
        {
            p->interrupt();
            p->close();
        }

        delete p;
    }
};


// Class for position control
class scriptPosPort : public scriptPort
{
protected:
    Property          drvOptions;
    PolyDriver       *drv;
    IPositionControl *pos;
    IEncoders        *enc;
    int offset;

    void send(const Vector &x)
    {
        Vector disp(x.length()-1);

        for (int i=0; i<disp.length(); i++)
        {
            double q;
            
            enc->getEncoder(offset+i,&q);
            disp[i]=x[1+i]-q;

            if (disp[i]<0.0)
                disp[i]=-disp[i];
        }

        for (int i=0; i<disp.length(); i++)
        {
            pos->setRefSpeed(offset+i,disp[i]/x[0]);
            pos->positionMove(offset+i,x[1+i]);
        }

        cout << name << ": " << ((Vector)x).toString() << endl;
    }

public:
    scriptPosPort() : scriptPort()
    {
        drvOptions.clear();
        drv=NULL;
    }

    bool acquireData(Bottle &b, const string &modName)
    {
        if (!scriptPort::acquireData(b,modName))
            return false;

        string remote;
        int offset;

        if (b.check("Part"))
            remote=b.find("Part").asString();
        else
        {
            cout << "Part option is missing!" << endl;
            return false;
        }

        if (b.check("Offset"))
            offset=b.find("Offset").asInt();
        else
        {
            cout << "Offset option is missing!" << endl;
            return false;
        }

        Property options;
        string local=modName+remote;
        options.put("remote",remote.c_str());
        options.put("local",local.c_str());
        options.put("offset",offset);

        setOptions(options);

        acquireEntries(b);

        return true;
    }

    void setOptions(Property &options)
    {
        if (!drvOptions.check("device"))
            drvOptions.put("device","remote_controlboard");

        if (options.check("remote"))
        {
            drvOptions.unput("remote");    
            drvOptions.put("remote",options.find("remote").asString());
        }

        if (options.check("local"))
        {    
            drvOptions.unput("local");
            drvOptions.put("local",options.find("local").asString());
        }

        if (options.check("offset"))
        {    
            drvOptions.unput("offset");
            offset=options.find("offset").asInt();
        }
    }

    void push_back(const Vector &item)
    {
        int sz=item.length()-1;
        Vector x(sz);
        for (int i=0; i<sz; i++)
            x[i]=item[1+i];

        absTime.push_back(item[0]);
        interpFlag.push_back(false);
        data.push_back(x);

        size++;
    }

    bool connect()
    {
        drv=new PolyDriver(drvOptions);

        if (drv->isValid())
            connected=drv->view(pos)&&drv->view(enc);
        else
            connected=false;

        return connected;
    }

    ~scriptPosPort()
    {
        if (drv)
            delete drv;
    }
};


class ParserConfig
{
protected:
    ResourceFinder &rf;
    string modName;
    int period;
    int numPorts;
    deque<scriptPort*> portList;

public:
    ParserConfig(const string &_modName, ResourceFinder &_rf) : modName(_modName), rf(_rf) { }    

    bool acquireData()
    {
        Bottle bGeneral=rf.findGroup("GENERAL");
        if (!bGeneral.size())
        {
            cout << "GENERAL group is missing!" << endl;
            return false;
        }
        if (bGeneral.check("TimeGranularity"))
            period=bGeneral.find("TimeGranularity").asInt();
        else
        {
            cout << "TimeGranularity option is missing!" << endl;
            return false;
        }
        period=period<5 ? 5 : period;

        if (!bGeneral.check("NumStreamPorts") && !bGeneral.check("NumPosPorts"))
        {
            cout << "Either NumStreamPorts or NumPosPorts option is missing!" << endl;
            return false;
        }

        int numStreamPorts=0;
        int numPosPorts=0;

        if (bGeneral.check("NumStreamPorts"))
            numStreamPorts=bGeneral.find("NumStreamPorts").asInt();

        if (bGeneral.check("NumPosPorts"))
            numPosPorts=bGeneral.find("NumPosPorts").asInt();

        numPorts=numStreamPorts+numPosPorts;

        for (int i=0; i<numPorts; i++)
        {
            char Entry[255];
            bool isStreamPort;
            scriptPort *p;

            if (i<numStreamPorts)
            {    
                sprintf(Entry,"STREAMPORT_%d",i);
                isStreamPort=true;
                p=new scriptStreamPort;
                p->setType(stream);
            }
            else
            {    
                sprintf(Entry,"POSPORT_%d",i-numStreamPorts);
                isStreamPort=false;
                p=new scriptPosPort;
                p->setType(pos);
            }

            Bottle portData=rf.findGroup(Entry);

            if (portData.size())
            {
                if (!p->acquireData(portData,modName))
                {
                    cout << "Errors detected while processing " << Entry << endl;
                    return false;
                }
            }
            else
            {
                cout << "Data for " << Entry << " not found!" << endl;
                return false;
            }

            portList.push_back(p);
        }

        return true;
    }

    int         getPeriod()                { return period;       }
    int         getNumPorts()              { return numPorts;     }
    scriptPort &operator[](unsigned int i) { return *portList[i]; }

    ~ParserConfig()
    {
        for (unsigned int i=0; i<portList.size(); i++)
            delete portList[i];

        portList.clear();
    }
};


class scriptThread : public RateThread
{
protected:
    ParserConfig &p;
    unsigned int period;
    double Ts,t0;
    int mode;
    bool running;
    bool trustDeltaT;

    void init()
    {
        for (int i=0; i<p.getNumPorts(); i++)
            p[i].init();

        trustDeltaT=false;
    }

public:
    scriptThread(ParserConfig &_p) :
                 RateThread(10), p(_p), mode(SHOT),
                 running(true), trustDeltaT(false)
    {
        period=p.getPeriod();
        setRate(period);
        Ts=period/1000.0;
    }

    void resume()
    {
        init();
        RateThread::resume();
    }

    void suspend()
    {
        RateThread::suspend();
    }

    void setMode(int _mode)
    {
        mode=_mode;
        init();
        running=true;
    }

    string getMode()
    {
        if (mode==SHOT)
            return "shot";
        else
            return "cyc";
    }

    virtual bool threadInit()
    {
        for (int i=0; i<p.getNumPorts(); i++)
            if (p[i].getActive())
                if (!p[i].connect())
                {
                    cout << "Unable to connect port " << p[i].getName() << endl;
                    return false;
                }

        suspend();

        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
        {    
            cout << "Thread started successfully..." << endl;            
            cout << "...now is suspended" << endl;
        }
        else
            cout << "Thread did not start" << endl;
    }

    virtual void run()
    {
        double t1=Time::now();
        double deltaT=t1-t0;
        t0=t1;

        if (running)
        {    
            int cnt=0;

            for (int i=0; i<p.getNumPorts(); i++)
                if (p[i].process(trustDeltaT?deltaT:Ts))
                    cnt++;

            if (cnt>=p.getNumPorts())
            {
                running=false;
                cout << "Task complete!" << endl;
            }
        }

        if (!running && mode==CYCLE)
        {
            init();
            running=true;
        }
        
        if (!trustDeltaT)
            trustDeltaT=true;        
    }

    virtual void threadRelease() { }
};


class scriptModule: public RFModule
{
protected:
    ParserConfig *p;
    scriptThread *thr;
    Port          rpcPort;

public:
    scriptModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        p=new ParserConfig(getName().c_str(),rf);

        if (!p->acquireData())
        {
            cout << "Errors detected while processing input file!" << endl;
            return false;
        }

        thr=new scriptThread(*p);
        thr->start();

        rpcPort.open(getName("rpc"));
        attach(rpcPort);
        attachTerminal();

        cout << "***** type ""help"" for commands list" << endl;

        return true;
    }

    virtual bool respond(const Bottle &command, Bottle &reply)
    {
        if (command.size())
        {
            switch (command.get(0).asVocab())
            {
                case VOCAB4('h','e','l','p'):
                {                    
                    cout << "Available commands:"          << endl;
                    cout << "mode: query for current mode" << endl;
                    cout << "cyc : go cycling"             << endl;
                    cout << "shot: go for only one run"    << endl;
                    cout << "susp: suspend the thread"     << endl;
                    cout << "run:  start the thread"       << endl;
                    cout << "quit: quit the module"        << endl;
                    reply.addVocab(Vocab::encode("ack"));
                    return true;
                }

                case VOCAB4('m','o','d','e'):
                {                    
                    string mode=thr->getMode();
                    reply.addVocab(Vocab::encode(mode.c_str()));
                    return true;
                }

                case VOCAB3('c','y','c'):
                {                  
                    thr->setMode(CYCLE);
                    reply.addVocab(Vocab::encode("ack"));
                    return true;
                }
        
                case VOCAB4('s','h','o','t'):
                {                    
                    thr->setMode(SHOT);
                    reply.addVocab(Vocab::encode("ack"));
                    return true;
                }

                case VOCAB3('r','u','n'):
                {                    
                    thr->resume();
                    reply.addVocab(Vocab::encode("ack"));
                    return true;
                }
        
                case VOCAB4('s','u','s','p'):
                {                    
                    thr->suspend();
                    reply.addVocab(Vocab::encode("ack"));
                    return true;
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

    // use detachTerminal() instead of close()
    // since attachTerminal has been called
    virtual bool detachTerminal()
    {
        rpcPort.interrupt();
        rpcPort.close();

        thr->stop();
        delete thr;

        delete p;

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
        cout << "\t--name moduleName: set new module name"      << endl;
        cout << "\t--from   fileName: input configuration file" << endl;
        cout << "\t--context dirName: resource finder context"  << endl;

        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    if (!rf.check("from"))
    {
        cout << "Configuration file is missing!" << endl;
        return -1;    
    }

    scriptModule mod;
    mod.setName("/scriptSequencer");

    return mod.runModule(rf);
}



