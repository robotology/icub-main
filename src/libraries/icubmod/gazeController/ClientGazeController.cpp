/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// Developed by Ugo Pattacini

#include <stdio.h>
#include <algorithm>
#include <sstream>

#include <yarp/math/Math.h>
#include "ClientGazeController.h"

#define GAZECTRL_CLIENT_VER     1.1
#define GAZECTRL_DEFAULT_TMO    0.1     // [s]
#define GAZECTRL_ACK            Vocab::encode("ack")
#define GAZECTRL_NACK           Vocab::encode("nack")

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


/************************************************************************/
void GazeEventHandler::onRead(Bottle &event)
{
    if (interface!=NULL)
        interface->eventHandling(event);
}


/************************************************************************/
void GazeEventHandler::setInterface(ClientGazeController *interface)
{
    this->interface=interface;
    setStrict();
    useCallback();
}


/************************************************************************/
ClientGazeController::ClientGazeController()
{
    init();
}


/************************************************************************/
ClientGazeController::ClientGazeController(Searchable &config)
{
    init();
    open(config);
}


/************************************************************************/
void ClientGazeController::init()
{
    connected=false;
    closed=false;

    timeout=GAZECTRL_DEFAULT_TMO;
    lastFpMsgArrivalTime=0.0;
    lastAngMsgArrivalTime=0.0;

    fixationPoint.resize(3,0.0);
    angles.resize(3,0.0);

    portEvents.setInterface(this);
}


/************************************************************************/
bool ClientGazeController::open(Searchable &config)
{
    ConstString remote, local, carrier;

    if (config.check("remote"))
        remote=config.find("remote").asString();
    else
        return false;

    if (config.check("local"))
        local=config.find("local").asString();
    else
        return false;

    carrier=config.check("carrier",Value("udp")).asString();

    if (config.check("timeout"))
        timeout=config.find("timeout").asDouble();
    
    portCmdFp.open((local+"/xd:o").c_str());
    portCmdAng.open((local+"/angles:o").c_str());
    portCmdMono.open((local+"/mono:o").c_str());
    portCmdStereo.open((local+"/stereo:o").c_str());
    portStateFp.open((local+"/x:i").c_str());
    portStateAng.open((local+"/angles:i").c_str());
    portStateHead.open((local+"/q:i").c_str());
    portEvents.open((local+"/events:i").c_str());
    portRpc.open((local+"/rpc").c_str());

    bool ok=true;
    ok&=Network::connect(portRpc.getName().c_str(),(remote+"/rpc").c_str());
    if (ok)
    {
        Bottle info;
        getInfoHelper(info);
        if (info.check("server_version"))
        {
            double server_version=info.find("server_version").asDouble();
            if (server_version!=GAZECTRL_CLIENT_VER)
            {
                printf("Error: version mismatch => server(%g) != client(%g); please update accordingly\n",
                       server_version,GAZECTRL_CLIENT_VER);
                return false;
            }
        }
        else
            printf("Warning: unable to retrieve server version; please update the server\n");
    }
    else
    {
        printf("Error: unable to connect to the server rpc port!\n");
        return false;
    }

    ok&=Network::connect(portCmdFp.getName().c_str(),(remote+"/xd:i").c_str(),carrier.c_str());
    ok&=Network::connect(portCmdAng.getName().c_str(),(remote+"/angles:i").c_str(),carrier.c_str());
    ok&=Network::connect(portCmdMono.getName().c_str(),(remote+"/mono:i").c_str(),carrier.c_str());
    ok&=Network::connect(portCmdStereo.getName().c_str(),(remote+"/stereo:i").c_str(),carrier.c_str());
    ok&=Network::connect((remote+"/x:o").c_str(),portStateFp.getName().c_str(),carrier.c_str());
    ok&=Network::connect((remote+"/angles:o").c_str(),portStateAng.getName().c_str(),carrier.c_str());
    ok&=Network::connect((remote+"/q:o").c_str(),portStateHead.getName().c_str(),carrier.c_str());
    ok&=Network::connect((remote+"/events:o").c_str(),portEvents.getName().c_str(),carrier.c_str());

    return connected=ok;
}


/************************************************************************/
bool ClientGazeController::close()
{
    if (closed)
        return true;

    deleteContexts();

    while (eventsMap.size()>0)
        unregisterEvent(*eventsMap.begin()->second);

    portCmdFp.interrupt();
    portCmdAng.interrupt();
    portCmdMono.interrupt();
    portCmdStereo.interrupt();
    portStateFp.interrupt();
    portStateAng.interrupt();
    portStateHead.interrupt();
    portEvents.interrupt();
    portRpc.interrupt();

    portCmdFp.close();
    portCmdAng.close();
    portCmdMono.close();
    portCmdStereo.close();
    portStateFp.close();
    portStateAng.close();
    portStateHead.close();
    portEvents.close();
    portRpc.close();

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

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==GAZECTRL_ACK);
}


/************************************************************************/
bool ClientGazeController::getTrackingMode(bool *f)
{
    if (!connected || (f==NULL))
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("track");

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>1))
    {
        *f=(reply.get(1).asInt()>0);
        return true;
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::getFixationPoint(Vector &fp, Stamp *stamp)
{
    if (!connected)
        return false;

    double now=Time::now();
    if (Vector *v=portStateFp.read(false))
    {
        fixationPoint=*v;
        portStateFp.getEnvelope(fpStamp);
        lastFpMsgArrivalTime=now;
    }

    fp=fixationPoint;
    if (stamp!=NULL)
        *stamp=fpStamp;

    return (now-lastFpMsgArrivalTime<timeout);
}


/************************************************************************/
bool ClientGazeController::getAngles(Vector &ang, Stamp *stamp)
{
    if (!connected)
        return false;

    double now=Time::now();
    if (Vector *v=portStateAng.read(false))
    {
        angles=*v;
        portStateAng.getEnvelope(anglesStamp);
        lastAngMsgArrivalTime=now;
    }

    ang=angles;
    if (stamp!=NULL)
        *stamp=anglesStamp;

    return (now-lastAngMsgArrivalTime<timeout);
}


/************************************************************************/
bool ClientGazeController::lookAtFixationPoint(const Vector &fp)
{
    if (!connected || (fp.length()<3))
        return false;

    Bottle &cmd=portCmdFp.prepare();
    cmd.clear();

    cmd.addDouble(fp[0]);
    cmd.addDouble(fp[1]);
    cmd.addDouble(fp[2]);

    portCmdFp.writeStrict();
    return true;
}


/************************************************************************/
bool ClientGazeController::lookAtAbsAngles(const Vector &ang)
{
    if (!connected || (ang.length()<3))
        return false;

    Bottle &cmd=portCmdAng.prepare();
    cmd.clear();

    cmd.addString("abs");
    cmd.addDouble(ang[0]);
    cmd.addDouble(ang[1]);
    cmd.addDouble(ang[2]);

    portCmdAng.writeStrict();
    return true;
}


/************************************************************************/
bool ClientGazeController::lookAtRelAngles(const Vector &ang)
{
    if (!connected || (ang.length()<3))
        return false;

    Bottle &cmd=portCmdAng.prepare();
    cmd.clear();

    cmd.addString("rel");
    cmd.addDouble(ang[0]);
    cmd.addDouble(ang[1]);
    cmd.addDouble(ang[2]);

    portCmdAng.writeStrict();
    return true;
}


/************************************************************************/
bool ClientGazeController::lookAtMonoPixel(const int camSel, const Vector &px,
                                           const double z)
{
    if (!connected || (px.length()<2))
        return false;

    Bottle &cmd=portCmdMono.prepare();
    cmd.clear();

    cmd.addString((camSel==0)?"left":"right");
    cmd.addDouble(px[0]);
    cmd.addDouble(px[1]);
    cmd.addDouble(z);

    portCmdMono.writeStrict();
    return true;
}


/************************************************************************/
bool ClientGazeController::lookAtMonoPixelWithVergence(const int camSel,
                                                       const Vector &px,
                                                       const double ver)
{
    if (!connected || (px.length()<2))
        return false;

    Bottle &cmd=portCmdMono.prepare();
    cmd.clear();

    cmd.addString((camSel==0)?"left":"right");
    cmd.addDouble(px[0]);
    cmd.addDouble(px[1]);
    cmd.addString("ver");
    cmd.addDouble(ver);

    portCmdMono.writeStrict();
    return true;
}


/************************************************************************/
bool ClientGazeController::lookAtStereoPixels(const Vector &pxl, const Vector &pxr)
{
    if (!connected || (pxl.length()<2) || (pxr.length()<2))
        return false;

    Bottle &cmd=portCmdStereo.prepare();
    cmd.clear();

    cmd.addDouble(pxl[0]);
    cmd.addDouble(pxl[1]);
    cmd.addDouble(pxr[0]);
    cmd.addDouble(pxr[1]);

    portCmdStereo.writeStrict();
    return true;
}


/************************************************************************/
bool ClientGazeController::getNeckTrajTime(double *t)
{
    if (!connected || (t==NULL))
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("Tneck");

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>1))
    {
        *t=reply.get(1).asDouble();
        return true;
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::getEyesTrajTime(double *t)
{
    if (!connected || (t==NULL))
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("Teyes");

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>1))
    {
        *t=reply.get(1).asDouble();
        return true;
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::getVORGain(double *gain)
{
    if (!connected || (gain==NULL))
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("vor");

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>1))
    {
        *gain=reply.get(1).asDouble();
        return true;
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::getOCRGain(double *gain)
{
    if (!connected || (gain==NULL))
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("ocr");

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>1))
    {
        *gain=reply.get(1).asDouble();
        return true;
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::getSaccadesStatus(bool *f)
{
    if (!connected || (f==NULL))
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("sacc");

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>1))
    {
        *f=(reply.get(1).asInt()>0);
        return true;
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::getSaccadesInhibitionPeriod(double *period)
{
    if (!connected || (period==NULL))
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("sinh");

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>1))
    {
        *period=reply.get(1).asDouble();
        return true;
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::getSaccadesActivationAngle(double *angle)
{
    if (!connected || (angle==NULL))
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("sact");

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>1))
    {
        *angle=reply.get(1).asDouble();
        return true;
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::getPose(const string &poseSel, Vector &x, Vector &o,
                                   Stamp *stamp)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("pose");
    command.addString(poseSel.c_str());

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>1))
    {
        if (Bottle *bPose=reply.get(1).asList())
        {
            if (bPose->size()>=7)
            {
                x.resize(3);
                o.resize(bPose->size()-x.length());
        
                for (size_t i=0; i<x.length(); i++)
                    x[i]=bPose->get(i).asDouble();
        
                for (size_t i=0; i<o.length(); i++)
                    o[i]=bPose->get(x.length()+i).asDouble();
        
                if ((reply.size()>2) && (stamp!=NULL))
                {
                    if (Bottle *bStamp=reply.get(2).asList())
                    {
                        Stamp tmpStamp(bStamp->get(0).asInt(),
                                       bStamp->get(1).asDouble());

                        *stamp=tmpStamp;
                    }
                }

                return true;
            }
        }
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::getLeftEyePose(Vector &x, Vector &o, Stamp *stamp)
{
    return getPose("left",x,o,stamp);
}


/************************************************************************/
bool ClientGazeController::getRightEyePose(Vector &x, Vector &o, Stamp *stamp)
{
    return getPose("right",x,o,stamp);
}


/************************************************************************/
bool ClientGazeController::getHeadPose(Vector &x, Vector &o, Stamp *stamp)
{
    return getPose("head",x,o,stamp);
}


/************************************************************************/
bool ClientGazeController::get2DPixel(const int camSel, const Vector &x,
                                      Vector &px)
{
    if (!connected || (x.length()<3))
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("2D");
    Bottle &bOpt=command.addList();
    bOpt.addString((camSel==0)?"left":"right");
    bOpt.addDouble(x[0]);
    bOpt.addDouble(x[1]);
    bOpt.addDouble(x[2]);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>1))
    {
        if (Bottle *bPixel=reply.get(1).asList())
        {
            px.resize(bPixel->size());
            for (size_t i=0; i<px.length(); i++)
                px[i]=bPixel->get(i).asDouble();

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::get3DPoint(const int camSel, const Vector &px,
                                      const double z, Vector &x)
{
    if (!connected || (px.length()<2))
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("3D");
    command.addString("mono");
    Bottle &bOpt=command.addList();
    bOpt.addString((camSel==0)?"left":"right");
    bOpt.addDouble(px[0]);
    bOpt.addDouble(px[1]);
    bOpt.addDouble(z);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>1))
    {
        if (Bottle *bPoint=reply.get(1).asList())
        {
            x.resize(bPoint->size());
            for (size_t i=0; i<x.length(); i++)
                x[i]=bPoint->get(i).asDouble();

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::get3DPointOnPlane(const int camSel, const Vector &px,
                                             const Vector &plane, Vector &x)
{
    if (!connected || (px.length()<2) || (plane.length()<4))
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("3D");
    command.addString("proj");
    Bottle &bOpt=command.addList();
    bOpt.addString((camSel==0)?"left":"right");
    bOpt.addDouble(px[0]);
    bOpt.addDouble(px[1]);
    bOpt.addDouble(plane[0]);
    bOpt.addDouble(plane[1]);
    bOpt.addDouble(plane[2]);
    bOpt.addDouble(plane[3]);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>1))
    {
        if (Bottle *bPoint=reply.get(1).asList())
        {
            x.resize(bPoint->size());
            for (size_t i=0; i<x.length(); i++)
                x[i]=bPoint->get(i).asDouble();

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::get3DPointFromAngles(const int mode, const Vector &ang,
                                                Vector &x)
{
    if (!connected || (ang.length()<3))
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("3D");
    command.addString("ang");
    Bottle &bOpt=command.addList();
    bOpt.addString((mode==0)?"abs":"rel");
    bOpt.addDouble(ang[0]);
    bOpt.addDouble(ang[1]);
    bOpt.addDouble(ang[2]);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>1))
    {
        if (Bottle *bPoint=reply.get(1).asList())
        {
            x.resize(bPoint->size());
            for (size_t i=0; i<x.length(); i++)
                x[i]=bPoint->get(i).asDouble();

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::getAnglesFrom3DPoint(const Vector &x, Vector &ang)
{
    if (!connected || (x.length()<3))
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("ang");
    Bottle &bOpt=command.addList();
    bOpt.addDouble(x[0]);
    bOpt.addDouble(x[1]);
    bOpt.addDouble(x[2]);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>1))
    {
        if (Bottle *bAng=reply.get(1).asList())
        {
            ang.resize(bAng->size());
            for (size_t i=0; i<ang.length(); i++)
                ang[i]=bAng->get(i).asDouble();

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::triangulate3DPoint(const Vector &pxl, const Vector &pxr,
                                              Vector &x)
{
    if (!connected || ((pxl.length()<2) && (pxr.length()<2)))
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("3D");
    command.addString("stereo");
    Bottle &bOpt=command.addList();
    bOpt.addDouble(pxl[0]);
    bOpt.addDouble(pxl[1]);
    bOpt.addDouble(pxr[0]);
    bOpt.addDouble(pxr[1]);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>1))
    {
        if (Bottle *bPoint=reply.get(1).asList())
        {
            x.resize(bPoint->size());
            for (size_t i=0; i<x.length(); i++)
                x[i]=bPoint->get(i).asDouble();

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::getJointsDesired(Vector &qdes)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("des");

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>1))
    {
        if (Bottle *bDes=reply.get(1).asList())
        {
            qdes.resize(bDes->size());
            for (size_t i=0; i<qdes.length(); i++)
                qdes[i]=bDes->get(i).asDouble();

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::getJointsVelocities(Vector &qdot)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("vel");

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>1))
    {
        if (Bottle *bVel=reply.get(1).asList())
        {
            qdot.resize(bVel->size());
            for (size_t i=0; i<qdot.length(); i++)
                qdot[i]=bVel->get(i).asDouble();

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::getStereoOptions(Bottle &options)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("pid");

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>1))
    {
        if (Bottle *bOpt=reply.get(1).asList())
        {
            options=*bOpt;
            return true;
        }
    }

    return false;
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

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }
    
    return (reply.get(0).asVocab()==GAZECTRL_ACK);
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

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }
    
    return (reply.get(0).asVocab()==GAZECTRL_ACK);
}


/************************************************************************/
bool ClientGazeController::setVORGain(const double gain)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addString("set");
    command.addString("vor");
    command.addDouble(gain);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }
    
    return (reply.get(0).asVocab()==GAZECTRL_ACK);
}


/************************************************************************/
bool ClientGazeController::setOCRGain(const double gain)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addString("set");
    command.addString("ocr");
    command.addDouble(gain);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }
    
    return (reply.get(0).asVocab()==GAZECTRL_ACK);
}


/************************************************************************/
bool ClientGazeController::setSaccadesStatus(const bool f)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addString("set");
    command.addString("sacc");
    command.addInt((int)f);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==GAZECTRL_ACK);
}


/************************************************************************/
bool ClientGazeController::setSaccadesInhibitionPeriod(const double period)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addString("set");
    command.addString("sinh");
    command.addDouble(period);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==GAZECTRL_ACK);
}


/************************************************************************/
bool ClientGazeController::setSaccadesActivationAngle(const double angle)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addString("set");
    command.addString("sact");
    command.addDouble(angle);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==GAZECTRL_ACK);
}


/************************************************************************/
bool ClientGazeController::setStereoOptions(const Bottle &options)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addString("set");
    command.addString("pid");
    command.addList()=options;

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==GAZECTRL_ACK);
}


/************************************************************************/
bool ClientGazeController::blockNeckJoint(const string &joint, const double min,
                                          const double max)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addString("bind");
    command.addString(joint.c_str());
    command.addDouble(min);
    command.addDouble(max);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==GAZECTRL_ACK);
}


/************************************************************************/
bool ClientGazeController::blockNeckJoint(const string &joint, const int j)
{
    if (!connected)
        return false;

    Vector *val=portStateHead.read(true);
    return blockNeckJoint(joint,(*val)[j],(*val)[j]);
}


/************************************************************************/
bool ClientGazeController::getNeckJointRange(const string &joint, double *min,
                                             double *max)
{
    if (!connected || (min==NULL) || (max==NULL))
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString(joint.c_str());

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>2))
    {
        *min=reply.get(1).asDouble();
        *max=reply.get(2).asDouble();
        return true;
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::clearJoint(const string &joint)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addString("clear");
    command.addString(joint.c_str());

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==GAZECTRL_ACK);
}


/************************************************************************/
bool ClientGazeController::bindNeckPitch(const double min, const double max)
{
    return blockNeckJoint("pitch",min,max);
}


/************************************************************************/
bool ClientGazeController::blockNeckPitch(const double val)
{
    return blockNeckJoint("pitch",val,val);
}


/************************************************************************/
bool ClientGazeController::blockNeckPitch()
{
    return blockNeckJoint("pitch",3);
}


/************************************************************************/
bool ClientGazeController::bindNeckRoll(const double min, const double max)
{
    return blockNeckJoint("roll",min,max);
}


/************************************************************************/
bool ClientGazeController::blockNeckRoll(const double val)
{
    return blockNeckJoint("roll",val,val);
}


/************************************************************************/
bool ClientGazeController::blockNeckRoll()
{
    return blockNeckJoint("roll",4);
}


/************************************************************************/
bool ClientGazeController::bindNeckYaw(const double min, const double max)
{
    return blockNeckJoint("yaw",min,max);
}


/************************************************************************/
bool ClientGazeController::blockNeckYaw(const double val)
{
    return blockNeckJoint("yaw",val,val);
}


/************************************************************************/
bool ClientGazeController::blockNeckYaw()
{
    return blockNeckJoint("yaw",5);
}


/************************************************************************/
bool ClientGazeController::blockEyes(const double ver)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addString("bind");
    command.addString("eyes");
    command.addDouble(ver);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==GAZECTRL_ACK);
}


/************************************************************************/
bool ClientGazeController::blockEyes()
{
    if (!connected)
        return false;

    Vector *val=portStateHead.read(true);
    return blockEyes((*val)[5]);
}


/************************************************************************/
bool ClientGazeController::getNeckPitchRange(double *min, double *max)
{
    return getNeckJointRange("pitch",min,max);
}


/************************************************************************/
bool ClientGazeController::getNeckRollRange(double *min, double *max)
{
    return getNeckJointRange("roll",min,max);
}


/************************************************************************/
bool ClientGazeController::getNeckYawRange(double *min, double *max)
{
    return getNeckJointRange("yaw",min,max);
}


/************************************************************************/
bool ClientGazeController::getBlockedVergence(double *ver)
{
    if (!connected || (ver==NULL))
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("eyes");

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>2))
    {
        *ver=reply.get(1).asDouble();
        return true;
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::clearNeckPitch()
{
    return clearJoint("pitch");
}


/************************************************************************/
bool ClientGazeController::clearNeckRoll()
{
    return clearJoint("roll");
}


/************************************************************************/
bool ClientGazeController::clearNeckYaw()
{
    return clearJoint("yaw");
}


/************************************************************************/
bool ClientGazeController::clearEyes()
{
    return clearJoint("eyes");
}


/************************************************************************/
bool ClientGazeController::getNeckAngleUserTolerance(double *angle)
{
    if (!connected || (angle==NULL))
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("ntol");

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if ((reply.get(0).asVocab()==GAZECTRL_ACK) && (reply.size()>1))
    {
        *angle=reply.get(1).asDouble();
        return true;
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::setNeckAngleUserTolerance(const double angle)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addString("set");
    command.addString("ntol");
    command.addDouble(angle);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==GAZECTRL_ACK);
}


/************************************************************************/
bool ClientGazeController::checkMotionDone(bool *f)
{
    if (!connected || (f==NULL))
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("done");

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==GAZECTRL_ACK)
    {
        *f=(reply.get(1).asInt()>0);
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ClientGazeController::waitMotionDone(const double period, const double timeout)
{
    bool done=false;
    double t0=Time::now();

    while (!done)
    {
        Time::delay(period);

        if (!checkMotionDone(&done) || ((timeout>0.0) && ((Time::now()-t0)>timeout)))
            return false;
    }

    return true;
}


/************************************************************************/
bool ClientGazeController::checkSaccadeDone(bool *f)
{
    if (!connected || (f==NULL))
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("sdon");

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==GAZECTRL_ACK)
    {
        *f=(reply.get(1).asInt()>0);
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ClientGazeController::waitSaccadeDone(const double period, const double timeout)
{
    bool done=false;
    double t0=Time::now();

    while (!done)
    {
        Time::delay(period);

        if (!checkSaccadeDone(&done) || ((timeout>0.0) && ((Time::now()-t0)>timeout)))
            return false;
    }

    return true;
}


/************************************************************************/
bool ClientGazeController::stopControl()
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addString("stop");

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }
    
    return (reply.get(0).asVocab()==GAZECTRL_ACK);
}


/************************************************************************/
bool ClientGazeController::storeContext(int *id)
{
    if (!connected || (id==NULL))
        return false;

    Bottle command, reply;
    command.addString("stor");

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==GAZECTRL_ACK)
    {
        contextIdList.insert(*id=reply.get(1).asInt());
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ClientGazeController::restoreContext(const int id)
{
    if (!connected || ((contextIdList.find(id)==contextIdList.end()) && (id!=0)))
        return false;

    Bottle command, reply;
    command.addString("rest");
    command.addInt(id);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==GAZECTRL_ACK);
}


/************************************************************************/
bool ClientGazeController::deleteContext(const int id)
{
    if (!connected || ((contextIdList.find(id)==contextIdList.end()) && (id!=0)))
        return false;

    Bottle command, reply;
    command.addString("del");
    command.addList().addInt(id);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==GAZECTRL_ACK)
    {
        contextIdList.erase(id);
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ClientGazeController::deleteContexts()
{
    if (!connected)
        return false;

    if (contextIdList.empty())
        return true;

    Bottle command, reply;
    command.addString("del");
    Bottle &ids=command.addList();
    for (set<int>::iterator itr=contextIdList.begin(); itr!=contextIdList.end(); itr++)
        ids.addInt(*itr);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    contextIdList.clear();

    return (reply.get(0).asVocab()==GAZECTRL_ACK);
}


/************************************************************************/
bool ClientGazeController::getInfoHelper(Bottle &info)
{
    Bottle command, reply;
    command.addString("get");
    command.addString("info");

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==GAZECTRL_ACK)
    {
        if (reply.size()>1)
        {
            if (Bottle *infoPart=reply.get(1).asList())
                info=*infoPart;

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientGazeController::getInfo(Bottle &info)
{
    if (connected)
        return getInfoHelper(info);
    else
        return false;
}


/************************************************************************/
void ClientGazeController::eventHandling(Bottle &event)
{
    string type=event.get(0).asString().c_str();
    double time=event.get(1).asDouble();
    double checkPoint=(type=="motion-ongoing")?event.get(2).asDouble():-1.0;
    map<string,GazeEvent*>::iterator itr;

    // rise the all-events callback
    itr=eventsMap.find("*");
    if (itr!=eventsMap.end())
    {
        if (itr->second!=NULL)
        {
            GazeEvent &Event=*itr->second;
            Event.gazeEventVariables.type=ConstString(type.c_str());
            Event.gazeEventVariables.time=time;

            if (checkPoint>=0.0)
                Event.gazeEventVariables.motionOngoingCheckPoint=checkPoint;

            Event.gazeEventCallback();
        }
    }

    string typeExtended=type;
    if (checkPoint>=0.0)
    {
        ostringstream ss;
        ss<<type<<"-"<<checkPoint;
        typeExtended=ss.str();
    }

    // rise the event specific callback
    itr=eventsMap.find(typeExtended);
    if (itr!=eventsMap.end())
    {
        if (itr->second!=NULL)
        {
            GazeEvent &Event=*itr->second;
            Event.gazeEventVariables.type=ConstString(type.c_str());
            Event.gazeEventVariables.time=time;

            if (checkPoint>=0.0)
                Event.gazeEventVariables.motionOngoingCheckPoint=checkPoint;

            Event.gazeEventCallback();
        }
    }
}


/************************************************************************/
bool ClientGazeController::registerEvent(GazeEvent &event)
{
    if (!connected)
        return false;

    string type=event.gazeEventParameters.type.c_str();
    if (type=="motion-ongoing")
    {
        double checkPoint=event.gazeEventParameters.motionOngoingCheckPoint;

        Bottle command, reply;
        command.addString("register");
        command.addString("ongoing");
        command.addDouble(checkPoint);

        if (!portRpc.write(command,reply))
        {
            printf("Error: unable to get reply from server!\n");
            return false;
        }

        if (reply.get(0).asVocab()!=GAZECTRL_ACK)
            return false;

        ostringstream ss;
        ss<<type<<"-"<<checkPoint;
        type=ss.str();
    }

    eventsMap[type]=&event;
    return true;
}


/************************************************************************/
bool ClientGazeController::unregisterEvent(GazeEvent &event)
{
    if (!connected)
        return false;

    string type=event.gazeEventParameters.type.c_str();
    if (type=="motion-ongoing")
    {
        double checkPoint=event.gazeEventParameters.motionOngoingCheckPoint;

        Bottle command, reply;
        command.addString("unregister");
        command.addString("ongoing");
        command.addDouble(checkPoint);

        if (!portRpc.write(command,reply))
        {
            printf("Error: unable to get reply from server!\n");
            return false;
        }

        if (reply.get(0).asVocab()!=GAZECTRL_ACK)
            return false;

        ostringstream ss;
        ss<<type<<"-"<<checkPoint;
        type=ss.str();
    }

    eventsMap.erase(type);
    return true;
}


/************************************************************************/
bool ClientGazeController::tweakSet(const Bottle &options)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addString("set");
    command.addString("tweak");
    command.addList()=options;

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==GAZECTRL_ACK);
}


/************************************************************************/
bool ClientGazeController::tweakGet(Bottle &options)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addString("get");
    command.addString("tweak");

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==GAZECTRL_ACK)
    {
        if (reply.size()>1)
        {
            if (Bottle *optionsPart=reply.get(1).asList())
                options=*optionsPart;

            return true;
        }
    }

    return false;
}


/************************************************************************/
ClientGazeController::~ClientGazeController()
{
    close();
}


