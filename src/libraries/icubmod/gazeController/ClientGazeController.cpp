// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// Developed by Ugo Pattacini

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/ctrlMath.h>

#include <stdio.h>

#include "ClientGazeController.h"

#define GAZECTRL_DEFAULT_TMO    0.1     // [s]

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace ctrl;


/************************************************************************/
ClientGazeController::ClientGazeController()
{
    portCmdFp=NULL;
    portCmdAng=NULL;
    portCmdMono=NULL;
    portCmdStereo=NULL;
    portStateFp=NULL;
    portStateAng=NULL;
    portStateHead=NULL;
    portRpc=NULL;

    connected=false;
    closed=false;

    timeout=GAZECTRL_DEFAULT_TMO;
    lastFpMsgArrivalTime=0.0;
    lastAngMsgArrivalTime=0.0;

    fixationPoint.resize(3,0.0);
    angles.resize(3,0.0);
}


/************************************************************************/
ClientGazeController::ClientGazeController(Searchable &config)
{
    open(config);
}


ClientGazeController::~ClientGazeController()
{
    close();
}


/************************************************************************/
bool ClientGazeController::open(Searchable &config)
{
    ConstString remote, local;

    if (config.check("remote"))
        remote=config.find("remote").asString();
    else
        return false;

    if (config.check("local"))
        local=config.find("local").asString();
    else
        return false;

    if (config.check("timeout"))
        timeout=config.find("timeout").asDouble();

    portCmdFp=new BufferedPort<Bottle>;
    portCmdFp->open((local+"/xd:o").c_str());

    portCmdAng=new BufferedPort<Bottle>;
    portCmdAng->open((local+"/angles:o").c_str());

    portCmdMono=new BufferedPort<Bottle>;
    portCmdMono->open((local+"/mono:o").c_str());

    portCmdStereo=new BufferedPort<Bottle>;
    portCmdStereo->open((local+"/stereo:o").c_str());

    portStateFp=new BufferedPort<Vector>;
    portStateFp->open((local+"/x:i").c_str());

    portStateAng=new BufferedPort<Vector>;
    portStateAng->open((local+"/angles:i").c_str());

    portStateHead=new BufferedPort<Vector>;
    portStateHead->open((local+"/q:i").c_str());

    portRpc=new Port;
    portRpc->open((local+"/rpc").c_str());

    remote=remote+"/head";
    bool ok=true;

    ok&=Network::connect(portCmdFp->getName().c_str(),(remote+"/xd:i").c_str());
    ok&=Network::connect(portCmdAng->getName().c_str(),(remote+"/angles:i").c_str());
    ok&=Network::connect(portCmdMono->getName().c_str(),(remote+"/mono:i").c_str());
    ok&=Network::connect(portCmdStereo->getName().c_str(),(remote+"/stereo:i").c_str());
    ok&=Network::connect((remote+"/x:o").c_str(),portStateFp->getName().c_str());
    ok&=Network::connect((remote+"/angles:o").c_str(),portStateAng->getName().c_str());
    ok&=Network::connect((remote+"/q:o").c_str(),portStateHead->getName().c_str());
    ok&=Network::connect(portRpc->getName().c_str(),(remote+"/rpc").c_str());

    return connected=ok;
}


/************************************************************************/
bool ClientGazeController::close()
{
    if (closed)
        return true;

    if (portCmdFp)
    {
        portCmdFp->interrupt();
        portCmdFp->close();
        delete portCmdFp;
    }

    if (portCmdAng)
    {
        portCmdAng->interrupt();
        portCmdAng->close();
        delete portCmdAng;
    }

    if (portCmdMono)
    {
        portCmdMono->interrupt();
        portCmdMono->close();
        delete portCmdMono;
    }

    if (portCmdStereo)
    {
        portCmdStereo->interrupt();
        portCmdStereo->close();
        delete portCmdStereo;
    }

    if (portStateFp)
    {
        portStateFp->interrupt();
        portStateFp->close();
        delete portStateFp;
    }

    if (portStateAng)
    {
        portStateAng->interrupt();
        portStateAng->close();
        delete portStateAng;
    }

    if (portStateHead)
    {
        portStateHead->interrupt();
        portStateHead->close();
        delete portStateHead;
    }

    if (portRpc)
    {
        portRpc->interrupt();
        portRpc->close();
        delete portRpc;
    }

    connected=false;

    return closed=true;
}


/************************************************************************/
bool ClientGazeController::setTrackingMode(const bool f)
{
    if (!connected)
        return false;

    Bottle command, reply;

    command.addString("set");
    command.addString("track");
    command.addInt((int)f);

    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }

    return true;
}


/************************************************************************/
bool ClientGazeController::getTrackingMode(bool *f)
{
    if (!connected)
        return false;

    Bottle command, reply;

    command.addString("get");
    command.addString("track");

    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }
    else
    {
        *f=(reply.get(0).asInt()>0);
        return true;
    }
}


/************************************************************************/
bool ClientGazeController::getFixationPoint(Vector &fp)
{
    if (!connected)
        return false;

    double now=Time::now();

    if (Vector *v=portStateFp->read(false))
    {
        fixationPoint=*v;
        lastFpMsgArrivalTime=now;
    }

    fp=fixationPoint;

    return (now-lastFpMsgArrivalTime<timeout);
}


/************************************************************************/
bool ClientGazeController::getAngles(Vector &ang)
{
    if (!connected)
        return false;

    double now=Time::now();

    if (Vector *v=portStateAng->read(false))
    {
        angles=*v;
        lastAngMsgArrivalTime=now;
    }

    ang=angles;

    return (now-lastAngMsgArrivalTime<timeout);
}


/************************************************************************/
bool ClientGazeController::lookAtFixationPoint(const Vector &fp)
{
    if (!connected || fp.length()<3)
        return false;

    Bottle &cmd=portCmdFp->prepare();
    cmd.clear();
    cmd.addDouble(fp[0]);
    cmd.addDouble(fp[1]);
    cmd.addDouble(fp[2]);
    portCmdFp->write();

    return true;
}


/************************************************************************/
bool ClientGazeController::lookAtAbsAngles(const Vector &ang)
{
    if (!connected || ang.length()<3)
        return false;

    Bottle &cmd=portCmdAng->prepare();
    cmd.clear();
    cmd.addString("abs");
    cmd.addDouble(ang[0]);
    cmd.addDouble(ang[1]);
    cmd.addDouble(ang[2]);
    portCmdAng->write();

    return true;
}


/************************************************************************/
bool ClientGazeController::lookAtRelAngles(const Vector &ang)
{
    if (!connected || ang.length()<3)
        return false;

    Bottle &cmd=portCmdAng->prepare();
    cmd.clear();
    cmd.addString("rel");
    cmd.addDouble(ang[0]);
    cmd.addDouble(ang[1]);
    cmd.addDouble(ang[2]);
    portCmdAng->write();

    return true;
}


/************************************************************************/
bool ClientGazeController::lookAtMonoPixel(const int camSel, const Vector &px, const double z)
{
    if (!connected || px.length()<2)
        return false;

    Bottle &cmd=portCmdMono->prepare();
    cmd.clear();
    cmd.addString((camSel)?"right":"left");
    cmd.addDouble(px[0]);
    cmd.addDouble(px[1]);
    cmd.addDouble(z);
    portCmdMono->write();

    return true;
}


/************************************************************************/
bool ClientGazeController::lookAtStereoPixels(const Vector &pxl, const Vector &pxr)
{
    if (!connected || pxl.length()<2 || pxr.length()<2)
        return false;

    Bottle &cmd=portCmdStereo->prepare();
    cmd.clear();
    cmd.addDouble(pxl[0]);
    cmd.addDouble(pxl[1]);
    cmd.addDouble(pxr[0]);
    cmd.addDouble(pxr[1]);
    portCmdStereo->write();

    return true;
}


/************************************************************************/
bool ClientGazeController::getNeckTrajTime(double *t)
{
    if (!connected)
        return false;

    Bottle command, reply;

    command.addString("get");
    command.addString("Tneck");

    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }
    else
    {
        *t=reply.get(0).asDouble();
        return true;
    }
}


/************************************************************************/
bool ClientGazeController::getEyesTrajTime(double *t)
{
    if (!connected)
        return false;

    Bottle command, reply;

    command.addString("get");
    command.addString("Teyes");

    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }
    else
    {
        *t=reply.get(0).asDouble();
        return true;
    }
}


/************************************************************************/
bool ClientGazeController::setNeckTrajTime(const double t)
{
    if (!connected)
        return false;

    Bottle command, reply;

    command.addString("set");
    command.addString("Tneck");
    command.addDouble(t);

    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }
    
    return true;
}


/************************************************************************/
bool ClientGazeController::setEyesTrajTime(const double t)
{
    if (!connected)
        return false;

    Bottle command, reply;

    command.addString("set");
    command.addString("Teyes");
    command.addDouble(t);

    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }
    
    return true;
}


/************************************************************************/
bool ClientGazeController::blockNeckPitch(const double val)
{
    if (!connected)
        return false;

    Bottle command, reply;

    command.addString("block");
    command.addString("pitch");
    command.addDouble(val);

    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }
    
    return true;
}


/************************************************************************/
bool ClientGazeController::blockNeckPitch()
{
    if (!connected)
        return false;

    Bottle command, reply;

    Vector *val=portStateHead->read(true);

    command.addString("block");
    command.addString("pitch");
    command.addDouble((*val)[3]);

    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }
    
    return true;
}


/************************************************************************/
bool ClientGazeController::blockNeckYaw(const double val)
{
    if (!connected)
        return false;

    Bottle command, reply;

    command.addString("block");
    command.addString("yaw");
    command.addDouble(val);

    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }
    
    return true;
}


/************************************************************************/
bool ClientGazeController::blockNeckYaw()
{
    if (!connected)
        return false;

    Bottle command, reply;

    Vector *val=portStateHead->read(true);

    command.addString("block");
    command.addString("yaw");
    command.addDouble((*val)[5]);

    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }
    
    return true;
}


/************************************************************************/
bool ClientGazeController::clearNeckPitch()
{
    if (!connected)
        return false;

    Bottle command, reply;

    command.addString("clear");
    command.addString("pitch");

    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }
    
    return true;
}


/************************************************************************/
bool ClientGazeController::clearNeckYaw()
{
    if (!connected)
        return false;

    Bottle command, reply;

    command.addString("clear");
    command.addString("yaw");

    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }
    
    return true;
}


/************************************************************************/
bool ClientGazeController::checkMotionDone(bool *f)
{
    if (!connected)
        return false;

    Bottle command, reply;

    command.addString("get");
    command.addString("done");

    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }
    else
    {
        *f=(reply.get(0).asInt()>0);
        return true;
    }
}


/************************************************************************/
bool ClientGazeController::stopControl()
{
    if (!connected)
        return false;

    Bottle command, reply;

    command.addString("susp");

    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }
    
    return true;
}


/************************************************************************/
bool ClientGazeController::resumeControl()
{
    if (!connected)
        return false;

    Bottle command, reply;

    command.addString("run");

    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }
    
    return true;
}


