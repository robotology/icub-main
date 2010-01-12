// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// Developed by Ugo Pattacini

#include <yarp/os/Network.h>
#include <yarp/math/Math.h>

#include <stdio.h>

#include "CommonCartesianController.h"
#include "ClientCartesianController.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


/************************************************************************/
ClientCartesianController::ClientCartesianController()
{
    portCmd  =NULL;
    portState=NULL;
    portRpc  =NULL;

    closed=false;
    gotPose=false;

    pose.resize(7,0.0);
}


/************************************************************************/
ClientCartesianController::ClientCartesianController(Searchable &config)
{
    open(config);
}


ClientCartesianController::~ClientCartesianController()
{
    close();
}


/************************************************************************/
bool ClientCartesianController::open(Searchable &config)
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

    portCmd=new BufferedPort<Bottle>;
    portCmd->open((local+"/command:o").c_str());

    portState=new BufferedPort<Vector>;
    portState->open((local+"/state:i").c_str());

    portRpc=new Port;
    portRpc->open((local+"/rpc:o").c_str());

    bool ok=true;

    ok&=Network::connect(portCmd->getName().c_str(),(remote+"/command:i").c_str());
    ok&=Network::connect((remote+"/state:o").c_str(),portState->getName().c_str());
    ok&=Network::connect(portRpc->getName().c_str(),(remote+"/rpc:i").c_str());

    // check whether the solver is alive and connected
    if (ok)
    {        
        Bottle command, reply;
    
        command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
        command.addVocab(IKINCARTCTRL_VOCAB_OPT_ISSOLVERON);
    
        if (!portRpc->write(command,reply))
        {
            fprintf(stdout,"Error: unable to get reply from server!\n");
            close();

            return false;
        }

        if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
            if (reply.size()>1)
                if (reply.get(1).asVocab()==IKINCARTCTRL_VOCAB_VAL_TRUE)
                    return true;

        fprintf(stdout,"Error: unable to connect to solver!\n");
        close();

        return false;
    }
    else
        return false;
}


/************************************************************************/
bool ClientCartesianController::close()
{
    if (closed)
        return true;

    if (portCmd)
    {
        portCmd->interrupt();
        portCmd->close();
        delete portCmd;
    }

    if (portState)
    {
        portState->interrupt();
        portState->close();
        delete portState;
    }

    if (portRpc)
    {
        portRpc->interrupt();
        portRpc->close();
        delete portRpc;
    }

    return closed=true;
}


/************************************************************************/
bool ClientCartesianController::setTrackingMode(const bool f)
{
    Bottle command, reply;

    // prepare command
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_SET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_MODE);

    if (f)
        command.addVocab(IKINCARTCTRL_VOCAB_VAL_MODE_TRACK);
    else
        command.addVocab(IKINCARTCTRL_VOCAB_VAL_MODE_SINGLE);

    // send command and wait for reply
    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
        return true;
    else
        return false;
}


/************************************************************************/
bool ClientCartesianController::getTrackingMode(bool *f)
{
    Bottle command, reply;

    // prepare command
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_MODE);

    // send command and wait for reply
    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        int mode=reply.get(1).asVocab();

        if (mode==IKINCARTCTRL_VOCAB_VAL_MODE_TRACK)
            *f=true;
        else if (mode==IKINCARTCTRL_VOCAB_VAL_MODE_SINGLE)
            *f=false;
        else
            return false;

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ClientCartesianController::getPose(Vector &x, Vector &o)
{
    // receive from network in streaming mode (non-blocking)
    if (Vector *v=portState->read(false))
    {
        gotPose=true;
        pose=*v;
    }

    x.resize(3);
    o.resize(pose.length()-3);

    for (int i=0; i<3; i++)
        x[i]=pose[i];

    for (int i=0; i<pose.length(); i++)
        o[i]=pose[3+i];

    return gotPose;
}


/************************************************************************/
bool ClientCartesianController::goToPose(const Vector &xd, const Vector &od, const double t)
{
    if (xd.length()<3 || od.length()<4)
        return false;

    Bottle &command=portCmd->prepare();
    command.clear();

    // prepare command
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GO);
    command.addVocab(IKINCARTCTRL_VOCAB_VAL_POSE_FULL);
    command.addDouble(t);
    Bottle &xdesPart=command.addList();

    for (int i=0; i<3; i++)
        xdesPart.addDouble(xd[i]);

    for (int i=0; i<4; i++)
        xdesPart.addDouble(od[i]);    

    // send command
    portCmd->write();

    return true;
}


/************************************************************************/
bool ClientCartesianController::goToPosition(const Vector &xd, const double t)
{
    if (xd.length()<3)
        return false;

    Bottle &command=portCmd->prepare();
    command.clear();

    // prepare command
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GO);
    command.addVocab(IKINCARTCTRL_VOCAB_VAL_POSE_XYZ);
    command.addDouble(t);
    Bottle &xdesPart=command.addList();

    for (int i=0; i<3; i++)
        xdesPart.addDouble(xd[i]);    

    // send command
    portCmd->write();

    return true;
}


/************************************************************************/
bool ClientCartesianController::goToPoseSync(const Vector &xd, const Vector &od, const double t)
{
    if (xd.length()<3 || od.length()<4)
        return false;

    Bottle command, reply;

    // prepare command
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GO);
    command.addVocab(IKINCARTCTRL_VOCAB_VAL_POSE_FULL);
    command.addDouble(t);
    Bottle &xdesPart=command.addList();

    for (int i=0; i<3; i++)
        xdesPart.addDouble(xd[i]);

    for (int i=0; i<4; i++)
        xdesPart.addDouble(od[i]);

    // send command and wait for reply
    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK);
}


/************************************************************************/
bool ClientCartesianController::goToPositionSync(const Vector &xd, const double t)
{
    if (xd.length()<3)
        return false;

    Bottle command, reply;

    // prepare command
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GO);
    command.addVocab(IKINCARTCTRL_VOCAB_VAL_POSE_XYZ);
    command.addDouble(t);
    Bottle &xdesPart=command.addList();

    for (int i=0; i<3; i++)
        xdesPart.addDouble(xd[i]);

    // send command and wait for reply
    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK);
}


/************************************************************************/
bool ClientCartesianController::getDesired(Vector &xdcap, Vector &odcap, Vector &qdcap)
{
    Bottle command, reply;

    // prepare command
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_DES);

    // send command and wait for reply
    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
	{
		if (Bottle *body=reply.get(1).asList())
		{
			// xdcap and odcap part
			if (body->check(Vocab::decode(IKINCARTCTRL_VOCAB_OPT_X)))
            {
                Bottle *xData=body->find(Vocab::decode(IKINCARTCTRL_VOCAB_OPT_X)).asList();
                xdcap.resize(3);
                odcap.resize(4);
            
                for (int i=0; i<xdcap.length(); i++)
                    xdcap[i]=xData->get(i).asDouble();
            
                for (int i=0; i<odcap.length(); i++)
                    odcap[i]=xData->get(xdcap.length()+i).asDouble();
            }
            else
                return false;

			// qdcap part
            if (body->check(Vocab::decode(IKINCARTCTRL_VOCAB_OPT_Q)))
            {
                Bottle *qData=body->find(Vocab::decode(IKINCARTCTRL_VOCAB_OPT_Q)).asList();
                qdcap.resize(qData->size());

                for (int i=0; i<qdcap.length(); i++)
                    qdcap[i]=qData->get(i).asDouble();
            }
            else
                return false;

			return true;
		}
		else
			return false;
	}
    else
        return false;
}


/************************************************************************/
bool ClientCartesianController::getDOF(Vector &curDof)
{
    Bottle command, reply;

    // prepare command
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_DOF);

    // send command and wait for reply
    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        if (Bottle *dofPart=reply.get(1).asList())
        {
            curDof.resize(dofPart->size());
            
            for (int i=0; i<dofPart->size(); i++)
                curDof[i]=dofPart->get(i).asDouble();

            return true;
        }
        else
            return false;
    }
    else
        return false;
}


/************************************************************************/
bool ClientCartesianController::setDOF(const Vector &newDof, Vector &curDof)
{
    Bottle command, reply;

    // prepare command
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_SET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_DOF);
    Bottle &dofPart=command.addList();

    for (int i=0; i<newDof.length(); i++)
        dofPart.addInt((int)newDof[i]);

    // send command and wait for reply
    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {        
        if (Bottle *rcvDof=reply.get(1).asList())
        {                        
            curDof.resize(rcvDof->size());

            for (int i=0; i<rcvDof->size(); i++)
                curDof[i]=rcvDof->get(i).asDouble();

            return true;
        }
        else
            return false;
    }
    else
        return false;
}


/************************************************************************/
bool ClientCartesianController::getLimits(int axis, double *min, double *max)
{
    Bottle command, reply;

    // prepare command
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_LIM);
    command.addInt(axis);

    // send command and wait for reply
    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        if (reply.size()>2)
        {
            *min=reply.get(1).asDouble();
            *max=reply.get(2).asDouble();

            return true;
        }
        else
            return false;
    }
    else
        return false;
}


/************************************************************************/
bool ClientCartesianController::setLimits(int axis, const double min, const double max)
{
    Bottle command, reply;

    // prepare command
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_SET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_LIM);
    command.addInt(axis);
    command.addDouble(min);
    command.addDouble(max);

    // send command and wait for reply
    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
        return true;
    else
        return false;
}


/************************************************************************/
bool ClientCartesianController::getTrajTime(double *t)
{
    Bottle command, reply;

    // prepare command
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_TIME);

    // send command and wait for reply
    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        if (reply.size()>1)
        {
            *t=reply.get(1).asDouble();

            return true;
        }
        else
            return false;
    }
    else
        return false;
}


/************************************************************************/
bool ClientCartesianController::setTrajTime(const double t)
{
    Bottle command, reply;

    // prepare command
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_SET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_TIME);
    command.addDouble(t);

    // send command and wait for reply
    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
        return true;
    else
        return false;
}


/************************************************************************/
bool ClientCartesianController::getInTargetTol(double *tol)
{
    Bottle command, reply;

    // prepare command
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_TOL);

    // send command and wait for reply
    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        if (reply.size()>1)
        {
            *tol=reply.get(1).asDouble();

            return true;
        }
        else
            return false;
    }
    else
        return false;
}


/************************************************************************/
bool ClientCartesianController::setInTargetTol(const double tol)
{
    Bottle command, reply;

    // prepare command
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_SET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_TOL);
    command.addDouble(tol);

    // send command and wait for reply
    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
        return true;
    else
        return false;
}


/************************************************************************/
bool ClientCartesianController::checkMotionDone(bool *f)
{
    Bottle command, reply;

    // prepare command
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_MOTIONDONE);

    // send command and wait for reply
    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        if (reply.size()>1)
        {
            int flag=reply.get(1).asVocab();

            if (flag==IKINCARTCTRL_VOCAB_VAL_TRUE)
                *f=true;
            else if (flag==IKINCARTCTRL_VOCAB_VAL_FALSE)
                *f=false;

            return true;
        }
        else
            return false;
    }
    else
        return false;
}


/************************************************************************/
bool ClientCartesianController::stopControl()
{
    Bottle command, reply;

    // prepare command
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_STOP);

    // send command and wait for reply
    if (!portRpc->write(command,reply))
    {
        fprintf(stdout,"Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
        return true;
    else
        return false;
}



