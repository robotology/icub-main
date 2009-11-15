// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// Developed by Ugo Pattacini

#include <ace/config.h>
#include <ace/Vector_T.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/impl/NameClient.h>
#include <yarp/os/impl/Carriers.h>

#include <stdio.h>

#include <iCub/iKinVocabs.h>
#include <iCub/iKinHlp.h>

#include "CommonCartesianController.h"
#include "ServerCartesianController.h"

#define RES_DSC(p)              ((ACE_Vector<DriverDescriptor>*)p)
#define RES_LIM(p)              ((ACE_Vector<IControlLimits*>*)p)
#define RES_ENC(p)              ((ACE_Vector<IEncoders*>*)p)
#define RES_VEL(p)              ((ACE_Vector<IVelocityControl*>*)p)
#define RES_JNT(p)              ((ACE_Vector<int>*)p)
#define RES_RMP(p)              ((ACE_Vector<int*>*)p)

#define CARTCTRL_DEFAULT_PER    10
#define CARTCTRL_DEFAULT_TOL    5e-3
#define CARTCTRL_MAX_ACCEL      1e9
#define CARTCTRL_CONNECT_TMO    5e3

using namespace yarp;
using namespace yarp::os;
using namespace yarp::os::impl;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iKin;


/************************************************************************/
CartesianCtrlRpcProcessor::CartesianCtrlRpcProcessor(ServerCartesianController *_ctrl)
{
    ctrl=_ctrl;
}


/************************************************************************/
bool CartesianCtrlRpcProcessor::read(ConnectionReader &connection)
{
    Bottle cmd, reply;

    if (!cmd.read(connection))
        return false;

    if (ctrl->respond(cmd,reply))
        if (ConnectionWriter *writer=connection.getWriter())
            reply.write(*writer);

    return true;
}


/************************************************************************/
CartesianCtrlCommandPort::CartesianCtrlCommandPort(ServerCartesianController *_ctrl)
{
    ctrl=_ctrl;
}


/************************************************************************/
void CartesianCtrlCommandPort::onRead(Bottle &command)
{
    if (command.size()>3)
        if  (command.get(0).asVocab()==IKINCARTCTRL_VOCAB_CMD_GO)
        {   
            int pose=command.get(1).asVocab();
            double t=command.get(2).asDouble();
            Bottle *v=command.get(3).asList();
            Vector xd(v->size());

            for (int i=0; i<v->size(); i++)
                xd[i]=v->get(i).asDouble();

            if (pose==IKINCARTCTRL_VOCAB_VAL_POSE_FULL)
                ctrl->goTo(IKINCTRL_POSE_FULL,xd,t);
            else if (pose==IKINCARTCTRL_VOCAB_VAL_POSE_XYZ)
                ctrl->goTo(IKINCTRL_POSE_XYZ,xd,t);
        }
}


/************************************************************************/
ServerCartesianController::ServerCartesianController() :
                           RateThread(CARTCTRL_DEFAULT_PER)
{
    init();
}


/************************************************************************/
ServerCartesianController::ServerCartesianController(Searchable &config) :
                           RateThread(CARTCTRL_DEFAULT_PER)
{
    init();
    open(config);
}


/************************************************************************/
void ServerCartesianController::init()
{
    // initialization
    limb =NULL;
    chain=NULL;
    ctrl =NULL;

    lDsc=NULL;
    lLim=NULL;
    lEnc=NULL;
    lVel=NULL;
    lJnt=NULL;
    lRmp=NULL;

    mutex =NULL;
    txInfo=NULL;

    portSlvIn   =NULL;
    portSlvOut  =NULL;
    portSlvRpc  =NULL;
    portCmd     =NULL;
    portState   =NULL;
    portRpc     =NULL;
    rpcProcessor=NULL;

    attached     =false;
    connected    =false;
    closed       =false;
    trackingMode =false;
    executingTraj=false;
    motionDone   =true;

    connectCnt=0;
    ctrlPose=IKINCTRL_POSE_FULL;
    maxPartJoints=0;
    trajTime=1.0;

    // request high resolution scheduling
    Time::turboBoost();
}


/************************************************************************/
void ServerCartesianController::openPorts()
{
    portSlvIn   =new BufferedPort<Bottle>;
    portSlvOut  =new BufferedPort<Bottle>;
    portSlvRpc  =new Port;
    portCmd     =new CartesianCtrlCommandPort(this);
    portState   =new BufferedPort<Vector>;
    portRpc     =new Port;
    rpcProcessor=new CartesianCtrlRpcProcessor(this);

    portCmd->useCallback();
    portRpc->setReader(*rpcProcessor);

    ConstString prefixName="/";
    prefixName=prefixName+ctrlName;

    portSlvIn->open((prefixName+"/"+slvName+"/in").c_str());
    portSlvOut->open((prefixName+"/"+slvName+"/out").c_str());
    portSlvRpc->open((prefixName+"/"+slvName+"/rpc").c_str());
    portCmd->open((prefixName+"/command:i").c_str());
    portState->open((prefixName+"/state:o").c_str());
    portRpc->open((prefixName+"/rpc:i").c_str());
}


/************************************************************************/
void ServerCartesianController::closePorts()
{
    if (portSlvIn)
    {
        portSlvIn->interrupt();
        portSlvIn->close();
        delete portSlvIn;
    }

    if (portSlvOut)
    {
        portSlvOut->interrupt();
        portSlvOut->close();
        delete portSlvOut;
    }

    if (portSlvRpc)
    {
        portSlvRpc->interrupt();
        portSlvRpc->close();
        delete portSlvRpc;
    }

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

    if (rpcProcessor)
        delete rpcProcessor;
}


/************************************************************************/
bool ServerCartesianController::respond(const Bottle &command, Bottle &reply)
{
    if (command.size())
        switch (command.get(0).asVocab())
        {
            case IKINCARTCTRL_VOCAB_CMD_STOP:
            {   
                stopControl();
                break;
            }

            case IKINCARTCTRL_VOCAB_CMD_GO:
            {
                if (command.size()>3)
                {
                    int pose=command.get(1).asVocab();
                    double t=command.get(2).asDouble();
                    Bottle *v=command.get(3).asList();
                    Vector xd(v->size());
    
                    for (int i=0; i<v->size(); i++)
                        xd[i]=v->get(i).asDouble();

                    bool ret;
    
                    if (pose==IKINCARTCTRL_VOCAB_VAL_POSE_FULL)
                        ret=goTo(IKINCTRL_POSE_FULL,xd,t);
                    else if (pose==IKINCARTCTRL_VOCAB_VAL_POSE_XYZ)
                        ret=goTo(IKINCTRL_POSE_XYZ,xd,t);

                    if (ret)
                        reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                    else
                        reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                }
                else
                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
    
                break;
            }

            case IKINSLV_VOCAB_CMD_GET:
            {
                if (command.size()>1)
                    switch (command.get(1).asVocab())
                    {
                        case IKINCARTCTRL_VOCAB_OPT_MODE:
                        {
                            bool flag;

                            if (getTrackingMode(&flag))
                            {   
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                 
                                if (flag)
                                    reply.addVocab(IKINCARTCTRL_VOCAB_VAL_MODE_TRACK);
                                else
                                    reply.addVocab(IKINCARTCTRL_VOCAB_VAL_MODE_SINGLE);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                            break;
                        }

                        case IKINCARTCTRL_VOCAB_OPT_TIME:
                        {
                            double time;

                            if (getTrajTime(&time))
                            {   
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                reply.addDouble(time);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
    
                            break;
                        }

                        case IKINCARTCTRL_VOCAB_OPT_TOL:
                        {
                            double tol;
    
                            if (getInTargetTol(&tol))
                            {   
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                reply.addDouble(tol);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
    
                            break;
                        }

                        case IKINCARTCTRL_VOCAB_OPT_MOTIONDONE:
                        {
                            bool flag;

                            if (checkMotionDone(&flag))
                            {   
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);

                                if (flag)
                                    reply.addVocab(IKINCARTCTRL_VOCAB_VAL_TRUE);
                                else
                                    reply.addVocab(IKINCARTCTRL_VOCAB_VAL_FALSE);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
    
                            break;
                        }

                        case IKINCARTCTRL_VOCAB_OPT_ISSOLVERON:
                        {
                            reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);

                            if (connected)
                                reply.addVocab(IKINCARTCTRL_VOCAB_VAL_TRUE);
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_VAL_FALSE);
    
                            break;
                        }

                        case IKINCARTCTRL_VOCAB_OPT_LIM:
                        {
                            if (command.size()>2)
                            {
                                int axis=command.get(2).asInt();
                                double min, max;

                                if (getLimits(axis,&min,&max))
                                {
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                    reply.addDouble(min);
                                    reply.addDouble(max);
                                }
                                else
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                            break;
                        }

                        case IKINCARTCTRL_VOCAB_OPT_DOF:
                        {
                            Vector curDof;

                            if (getDOF(curDof))
                            {
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                Bottle &dofPart=reply.addList();
                                    
                                for (int i=0; i<curDof.length(); i++)
                                    dofPart.addInt((int)curDof[i]);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                            break;
                        }

                        case IKINCARTCTRL_VOCAB_OPT_DES:
                        {
                            reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
							Bottle &body=reply.addList();

                            // xdcap part
                            Bottle &xPart=body.addList();
                            xPart.addVocab(IKINCARTCTRL_VOCAB_OPT_X);
                            Bottle &xData=xPart.addList();

                            // populate xdcap list
                            for (int i=0; i<xdes.length(); i++)
                                xData.addDouble(xdes[i]);
    
                            // qdcap part
                            Bottle &qPart=body.addList();
                            qPart.addVocab(IKINCARTCTRL_VOCAB_OPT_Q);
                            Bottle &qData=qPart.addList();

                            // populate qdcap list
                            int cnt=0;
                            for (unsigned int i=0; i<chain->getN(); i++)
                                if ((*chain)[i].isBlocked())
                                    qData.addDouble((180.0/M_PI)*chain->getAng(i));
                                else
                                    qData.addDouble((180.0/M_PI)*qdes[cnt++]);

                            break;
                        }

                        default:
                            reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                    }
                else
                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                break;
            }

            case IKINCARTCTRL_VOCAB_CMD_SET:
            {
                if (command.size()>2)
                    switch (command.get(1).asVocab())
                    {
                        case IKINCARTCTRL_VOCAB_OPT_MODE:
                        {
                            int mode=command.get(2).asVocab();
    
                            if (mode==IKINCARTCTRL_VOCAB_VAL_MODE_TRACK)
                            {    
                                setTrackingMode(true);
                                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            }
                            else if (mode==IKINCARTCTRL_VOCAB_VAL_MODE_SINGLE)
                            {    
                                setTrackingMode(false);
                                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            }
                            else
                                reply.addVocab(IKINSLV_VOCAB_REP_NACK);
    
                            break;
                        }

                        case IKINCARTCTRL_VOCAB_OPT_TIME:
                        {
                            setTrajTime(command.get(2).asDouble());
                            reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            break;
                        }

                        case IKINCARTCTRL_VOCAB_OPT_TOL:
                        {
                            setInTargetTol(command.get(2).asDouble());
                            reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            break;
                        }

                        case IKINCARTCTRL_VOCAB_OPT_LIM:
                        {                            
                            if (command.size()>4)
                            {
                                int axis=command.get(2).asInt();
                                double min=command.get(3).asDouble();
                                double max=command.get(4).asDouble();

                                if (setLimits(axis,min,max))
                                    reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                                else
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                            break;
                        }

                        case IKINCARTCTRL_VOCAB_OPT_DOF:
                        {
                            if (Bottle *b=command.get(2).asList())
                            {
                                Vector newDof(b->size());
                                Vector curDof;

                                for (int i=0; i<b->size(); i++)
                                    newDof[i]=b->get(i).asDouble();

                                if (setDOF(newDof,curDof))
                                {                                                                        
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                    Bottle &dofPart=reply.addList();

                                    for (int i=0; i<curDof.length(); i++)
                                        dofPart.addInt((int)curDof[i]);
                                }
                                else
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);                            

                            break;
                        }

                        default:
                            reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                    }
                else
                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                break;
            }

            default:
                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
        }
    else
        reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

    return true;
}


/************************************************************************/
void ServerCartesianController::stopLimbVel()
{
    for (int i=0; i<numDrv; i++)
        (*RES_VEL(lVel))[i]->stop();
}


/************************************************************************/
void ServerCartesianController::alignJointsBounds()
{
    if (connected)
    {           
        fprintf(stdout,"Getting joints bounds from cartesian solver...\n");
    
        for (unsigned int i=0; i<chain->getN(); i++)
        {
            Bottle command, reply;
            double min, max;

            fprintf(stdout,"joint #%d: ... ",i);

            // prepare command
            command.addVocab(IKINSLV_VOCAB_CMD_GET);
            command.addVocab(IKINSLV_VOCAB_OPT_LIM);
            command.addInt(i);

            // send command to solver and wait for reply
            if (!portSlvRpc->write(command,reply))
            {
                fprintf(stdout,"%s error: unable to get reply from solver!\n",slvName.c_str());
                return;
            }

            if (reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK)
            {
                min=reply.get(1).asDouble();
                max=reply.get(2).asDouble();                        
        
                // align local joint's bounds
                (*chain)[i].setMin((M_PI/180.0)*min);
                (*chain)[i].setMax((M_PI/180.0)*max);

                fprintf(stdout,"[%.1f, %.1f] deg\n",min,max);
            }
            else
                fprintf(stdout,"failed\n");
        }
    }
}


/************************************************************************/
void ServerCartesianController::getFeedback(Vector &_fb)
{
    Vector fbTmp(maxPartJoints);
    int chainCnt=0;
    int _fbCnt=0;

    for (int i=0; i<numDrv; i++)
        if ((*RES_ENC(lEnc))[i]->getEncoders(fbTmp.data()))
            for (int j=0; j<(*RES_JNT(lJnt))[i]; j++)
            {
                double tmp=(M_PI/180.0)*fbTmp[(*RES_RMP(lRmp))[i][j]];

                if ((*chain)[chainCnt].isBlocked())
                    chain->setBlockingValue(chainCnt,tmp);
                else
                    _fb[_fbCnt++]=tmp;

                chainCnt++;
            }
        else for (int j=0; j<(*RES_JNT(lJnt))[i]; j++)
            if (!(*chain)[chainCnt++].isBlocked())
                _fbCnt++;
}


/************************************************************************/
void ServerCartesianController::newController()
{
    // if it already exists, destroy old controller
    if (ctrl)
        delete ctrl;

    // guard
    if (!chain)
        return;

    // update quantities
    fb.resize(chain->getDOF());
    getFeedback(fb);
    velOld.resize(chain->getDOF(),0.0);
    xdes=chain->EndEffPose();
    qdes=chain->getAng();

    // instantiate new controller
    ctrl=new MultiRefMinJerkCtrl(*chain,ctrlPose,chain->getAng(),getRate()/1000.0);

    // set tolerance
    ctrl->setInTargetTol(CARTCTRL_DEFAULT_TOL);

    // set default task execution time
    trajTime=ctrl->set_execTime(trajTime,true);
}


/************************************************************************/
bool ServerCartesianController::getTarget(Vector &_xdes, Vector &_qdes)
{
    if (Bottle *b1=portSlvIn->read(false))
    {
        if (b1->check(Vocab::decode(IKINSLV_VOCAB_OPT_X)))
        {
            Bottle *b2=CartesianHelper::getEndEffectorPoseOption(*b1);
            int l1=b2->size();
            int l2=7;
            int len=l1<l2 ? l1 : l2;

            for (int i=0; i<len; i++)
                _xdes[i]=b2->get(i).asDouble();
        }

        if (b1->check(Vocab::decode(IKINSLV_VOCAB_OPT_Q)))
        {
            Bottle *b2=CartesianHelper::getJointsOption(*b1);
            int l1=b2->size();
            int l2=chain->getDOF();
            int len=l1<l2 ? l1 : l2;

            for (int i=0; i<len; i++)
                _qdes[i]=(M_PI/180.0)*b2->get(i).asDouble();
        }

        return true;
    }
    else
        return false;
}


/************************************************************************/
void ServerCartesianController::sendVelocity(const Vector &v)
{
    int j=0;
    int k=0;
    int cnt=0;

    for (unsigned int i=0; i<chain->getN(); i++)
    {
        if (!(*chain)[i].isBlocked())
        {    
            // send only if changed
            if (v[cnt]!=velOld[cnt])
                (*RES_VEL(lVel))[j]->velocityMove((*RES_RMP(lRmp))[j][k],velOld[cnt]=v[cnt]);

            cnt++;
        }

        if (++k>=(*RES_JNT(lJnt))[j])
        {
            j++;
            k=0;
        }
    }
}


/************************************************************************/
bool ServerCartesianController::threadInit()
{
    fprintf(stdout,"Starting %s at %d ms\n",ctrlName.c_str(),(int)getRate());

    return true;
}


/************************************************************************/
void ServerCartesianController::afterStart(bool s)
{
    fprintf(stdout,"%s ",ctrlName.c_str());

    if (s)
        fprintf(stdout," started successfully\n");
    else
        fprintf(stdout," did not start!\n");
}


/************************************************************************/
void ServerCartesianController::run()
{    
    if (connected)
    {
        // begin of critical code
        mutex->wait();
    
        // get the current target pose
        if (getTarget(xdes,qdes))
            executingTraj=true; // onset of new trajectory
    
        // read the feedback
        getFeedback(fb);
        ctrl->set_q(fb);
        
        if (executingTraj || trackingMode)
        {
            // limb control loop
            ctrl->iterate(xdes,qdes);
    
            // send joints velocities to the robot [deg/s]
            sendVelocity((180.0/M_PI)*ctrl->get_qdot());
    
            // handle the end-trajectory event
            if (ctrl->isInTarget())
            {
                executingTraj=false;
                motionDone   =true;
    
                if (!trackingMode)
                {
                    stopLimbVel();
    
                    // switch the solver status to one shot mode
                    Bottle &b=portSlvOut->prepare();
                    b.clear();
                    CartesianHelper::addModeOption(b,false);
                    portSlvOut->write();
                }
            }
        }
    
        // streams out the end-effector pose
        portState->prepare()=chain->EndEffPose();
        portState->setEnvelope(*txInfo);
        portState->write();
    
        txInfo->update();
    
        // end of critical code
        mutex->post();
    }
    else if ((++connectCnt)*getRate()>CARTCTRL_CONNECT_TMO)
    {
        connectToSolver();
        connectCnt=0;
    }
}


/************************************************************************/
void ServerCartesianController::threadRelease()
{
    fprintf(stdout,"Stopping %s\n",ctrlName.c_str());
    stopLimbVel();
}


/************************************************************************/
bool ServerCartesianController::open(Searchable &config)
{
    fprintf(stdout,"***** Configuring cartesian controller *****\n");

    // GENERAL group
    Bottle &optGeneral=config.findGroup("GENERAL");
    if (optGeneral.isNull())
    {
        fprintf(stdout,"GENERAL group is missing\n");
        close();

        return false;
    }

    fprintf(stdout,"Acquiring options for group GENERAL...\n");

    // scan for required params
    if (optGeneral.check("SolverNameToConnect"))
        slvName=optGeneral.find("SolverNameToConnect").asString();
    else
    {
        fprintf(stdout,"SolverNameToConnect option is missing\n");
        close();

        return false;
    }

    if (optGeneral.check("KinematicPart"))
    {
        kinPart=optGeneral.find("KinematicPart").asString();

        if (kinPart!="arm" && kinPart!="leg")
        {
            fprintf(stdout,"Try to instantiate an unknown kinematic part\n");
            fprintf(stdout,"Available parts are: arm, leg\n");
            close();

            return false;
        }
    }
    else
    {
        fprintf(stdout,"KinematicPart option is missing\n");
        close();

        return false;
    }

    if (optGeneral.check("KinematicType"))
    {
        kinType=optGeneral.find("KinematicType").asString();

        if (kinType!="left" && kinType!="right")
        {
            fprintf(stdout,"Try to instantiate an unknown kinematic type\n");
            fprintf(stdout,"Available types are: left, right\n");
            close();

            return false;
        }
    }
    else
    {
        fprintf(stdout,"KinematicType option is missing\n");
        close();

        return false;
    }

    if (optGeneral.check("NumberOfDrivers"))
    {
        if (!(numDrv=optGeneral.find("NumberOfDrivers").asInt()))
        {
            fprintf(stdout,"NumberOfDrivers shall be positive\n");
            close();

            return false;
        }
    }
    else
    {
        fprintf(stdout,"NumberOfDrivers option is missing\n");
        close();

        return false;
    }

    // optional params
    if (optGeneral.check("ControllerName"))
        ctrlName=optGeneral.find("ControllerName").asString();
    else
    {
        ctrlName="cartController";
        ctrlName=ctrlName+"/";
        ctrlName=ctrlName+kinPart+kinType;
        fprintf(stdout,"default ControllerName assumed: %s",ctrlName.c_str());
    }

    if (optGeneral.check("ControllerPeriod"))
        setRate(optGeneral.find("ControllerPeriod").asInt());

    lDsc=new ACE_Vector<DriverDescriptor>;

    // scan DRIVER groups
    for (int i=0; i<numDrv; i++)
    {
        char entry[255];
        sprintf(entry,"DRIVER_%d",i);

        Bottle &optDrv=config.findGroup(entry);
        if (optDrv.isNull())
        {
            fprintf(stdout,"%s group is missing\n",entry);
            close();

            return false;
        }

        fprintf(stdout,"Acquiring options for group %s...\n",entry);

        DriverDescriptor desc;

        if (optDrv.check("Key"))
            desc.key=optDrv.find("Key").asString();
        else
        {
            fprintf(stdout,"Key option is missing\n");
            close();

            return false;
        }

        if (optDrv.check("JointsOrder"))
        {
            ConstString jointsOrderType=optDrv.find("JointsOrder").asString();

            if (jointsOrderType=="direct")
                desc.jointsDirectOrder=true;
            else if (jointsOrderType=="reversed")
                desc.jointsDirectOrder=false;
            else
            {
                fprintf(stdout,"Try to select an unknown mapping order\n");
                fprintf(stdout,"Available orders are: direct, reversed\n");
                close();

                return false;
            }
        }
        else
        {
            fprintf(stdout,"JointsOrder option is missing\n");
            close();

            return false;
        }

        RES_DSC(lDsc)->push_back(desc);
    }

    // instantiate kinematic object
    if (kinPart=="arm")
        limb=new iCubArm(kinType.c_str());
    else if (kinPart=="leg")
        limb=new iCubLeg(kinType.c_str());

    chain=limb->asChain();

    mutex =new Semaphore(); 
    txInfo=new Stamp(0,Time::now());

    openPorts();

    return true;
}


/************************************************************************/
bool ServerCartesianController::close()
{
    if (closed)
        return true;

    detachAll();

    if (lDsc)
        delete RES_DSC(lDsc);

    if (lLim)
        delete RES_LIM(lLim);

    if (lEnc)
        delete RES_ENC(lEnc);

    if (lVel)
        delete RES_VEL(lVel);

    if (lJnt)
        delete RES_JNT(lJnt);

    if (lRmp)
    {
        for (unsigned int i=0; i<RES_RMP(lRmp)->size(); i++)
            delete[] (*RES_RMP(lRmp))[i];

        delete RES_RMP(lRmp);
    }

    if (limb)
        delete limb;

    if (ctrl)
        delete ctrl;    

    if (mutex)
        delete mutex;

    if (txInfo)
        delete txInfo;

    closePorts();

    return closed=true;
}


/************************************************************************/
bool ServerCartesianController::attachAll(const PolyDriverList &p)
{
    drivers=p;
    int nd=drivers.size();

    fprintf(stdout,"***** Attaching drivers to cartesian controller %s *****\n",ctrlName.c_str());
    fprintf(stdout,"Received list of %d driver(s)\n",nd);

    if (nd!=numDrv)
    {
        fprintf(stdout,"Expected list of %d driver(s)\n",numDrv);
        return false;
    }

    lLim=new ACE_Vector<IControlLimits*>;
    lEnc=new ACE_Vector<IEncoders*>;
    lVel=new ACE_Vector<IVelocityControl*>;
    lJnt=new ACE_Vector<int>;
    lRmp=new ACE_Vector<int*>;

    int remainingJoints=chain->getN();

    for (int i=0; i<numDrv; i++)
    {
        fprintf(stdout,"Acquiring info on driver %s... ",(*RES_DSC(lDsc))[i].key.c_str());

        // check if what we require is present within the given list
        int j;
        for (j=0; j<drivers.size(); j++)
            if ((*RES_DSC(lDsc))[i].key==drivers[j]->key)
                break;

        if (j>=drivers.size())
        {
            fprintf(stdout,"None of provided drivers is of type %s\n",(*RES_DSC(lDsc))[i].key.c_str());
            return false;
        }

        // acquire interfaces and driver's info
        if (drivers[j]->poly->isValid())
        {
            fprintf(stdout,"ok\n");

            IControlLimits   *lim;
            IEncoders        *enc;
            IVelocityControl *vel;
            int               joints;

            drivers[j]->poly->view(lim);
            drivers[j]->poly->view(enc);
            drivers[j]->poly->view(vel);

            enc->getAxes(&joints);

            // this is for allocating vector
            // to read data from the interface
            if (joints>maxPartJoints)
                maxPartJoints=joints;

            // handle chain's bounds
            if (joints>remainingJoints)
                joints=remainingJoints;

            remainingJoints-=joints;

            int *rmpTmp=new int[joints];
            for (int k=0; k<joints; k++)
                rmpTmp[k]=(*RES_DSC(lDsc))[i].jointsDirectOrder ? k : joints-k-1;

            RES_LIM(lLim)->push_back(lim);
            RES_ENC(lEnc)->push_back(enc);
            RES_VEL(lVel)->push_back(vel);
            RES_JNT(lJnt)->push_back(joints);
            RES_RMP(lRmp)->push_back(rmpTmp);
        }
        else
        {
            fprintf(stdout,"error\n");
            return false;
        }
    }

    // exclude acceleration constraints by fixing
    // thresholds at high values
    for (int i=0; i<numDrv; i++)
    {
        Vector maxAcc((*RES_JNT(lJnt))[i]);
        maxAcc=CARTCTRL_MAX_ACCEL;

        (*RES_VEL(lVel))[i]->setRefAccelerations(maxAcc.data());
    }

    // create controller
    newController();    

    // this line shall be put before any
    // call to attached-dependent methods
    attached=true;

    connectToSolver();

    start();

    return true;
}


/************************************************************************/
bool ServerCartesianController::detachAll()
{
    if (isRunning())
        stop();

    return true;
}


/************************************************************************/
bool ServerCartesianController::pingSolver()
{    
    ConstString portSlvName="/";
    portSlvName=portSlvName+slvName+"/in";

    fprintf(stdout,"%s: Checking if cartesian solver %s is alive... ",ctrlName.c_str(),slvName.c_str());

    NameClient &nic=NameClient::getNameClient();
    Address address=nic.queryName(portSlvName.c_str());
    bool ret;

    if (address.isValid())
    {    
        if (OutputProtocol *out=Carriers::connect(address))
        {
            out->close();
            delete out;
        
            ret=true;
        }
        else
            ret=false;
    }
    else
        ret=false;

    fprintf(stdout,"%s\n",ret?"ok":"failed");

    return ret;
}


/************************************************************************/
bool ServerCartesianController::connectToSolver()
{
    if (attached && !connected && pingSolver())
    {        
        fprintf(stdout,"%s: Connecting to cartesian solver %s...\n",ctrlName.c_str(),slvName.c_str());

        ConstString portSlvName="/";
        portSlvName=portSlvName+slvName;

        bool ok=true;

        ok&=Network::connect((portSlvName+"/out").c_str(),portSlvIn->getName().c_str());
        ok&=Network::connect(portSlvOut->getName().c_str(),(portSlvName+"/in").c_str());
        ok&=Network::connect(portSlvRpc->getName().c_str(),(portSlvName+"/rpc").c_str());

        if (!ok)
            return false;

        // this line shall be put before any
        // call to connected-dependent methods
        connected=true;

        // keep solver and controller status aligned at start-up
        Vector curDof, tmpDof;
        getDOF(curDof);
        setDOF(curDof,tmpDof);
        setTrackingMode(false);
        alignJointsBounds();
        
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::goTo(unsigned int _ctrlPose, const Vector &xd, const double t)
{    
    if (connected)
    {
        motionDone=false;
        
        ctrl->set_ctrlPose(ctrlPose=_ctrlPose);

        // update trajectory execution time just if required
        if (t>0.0)
            setTrajTime(t);

        Bottle &b=portSlvOut->prepare();
        b.clear();
    
        // xd part
        CartesianHelper::addTargetOption(b,xd);
        // pose part
        CartesianHelper::addPoseOption(b,ctrlPose);
        // always put solver in continuous mode
        // before commanding a new desired pose
        // in order to compensate for movements
        // of uncontrolled joints
        // correct solver status will be reinstated
        // accordingly at the end of trajectory
        CartesianHelper::addModeOption(b,true);
    
        portSlvOut->write();        
    
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setTrackingMode(const bool f)
{
    if (connected)
    {
        Bottle &b=portSlvOut->prepare();
        b.clear();
        CartesianHelper::addModeOption(b,trackingMode=f);
        portSlvOut->write();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::getTrackingMode(bool *f)
{
    if (attached)
    {
        *f=trackingMode;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::getPose(Vector &x, Vector &o)
{
    if (attached)
    {
        Vector pose=chain->EndEffPose();
    
        x.resize(3);
        o.resize(pose.length()-3);
    
        for (int i=0; i<x.length(); i++)
            x[i]=pose[i];
    
        for (int i=0; i<o.length(); i++)
            o[i]=pose[x.length()+i];
    
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::goToPose(const Vector &xd, const Vector &od, const double t)
{
    if (connected)
    {
        Vector _xd(xd.length()+od.length());
    
        for (int i=0; i<xd.length(); i++)
            _xd[i]=xd[i];
    
        for (int i=0; i<od.length(); i++)
            _xd[xd.length()+i]=od[i];
    
        return goTo(IKINCTRL_POSE_FULL,_xd,t);
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::goToPosition(const Vector &xd, const double t)
{
    if (connected)
        return goTo(IKINCTRL_POSE_XYZ,xd,t);
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::goToPoseSync(const Vector &xd, const Vector &od, const double t)
{
    return goToPose(xd,od,t);
}


/************************************************************************/
bool ServerCartesianController::goToPositionSync(const Vector &xd, const double t)
{
    return goToPosition(xd,t);
}


/************************************************************************/
bool ServerCartesianController::getDesired(Vector &xdcap, Vector &odcap, Vector &qdcap)
{
    if (connected)
    {
        xdcap.resize(3);
        odcap.resize(xdes.length()-3);

        for (int i=0; i<xdcap.length(); i++)
            xdcap[i]=xdes[i];

        for (int i=0; i<odcap.length(); i++)
            odcap[i]=xdes[xdcap.length()+i];

        qdcap.resize(chain->getN());
        int cnt=0;

        for (unsigned int i=0; i<chain->getN(); i++)
            if ((*chain)[i].isBlocked())
                qdcap[i]=(180.0/M_PI)*chain->getAng(i);
            else
                qdcap[i]=(180.0/M_PI)*qdes[cnt++];

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::getDOF(Vector &curDof)
{
    if (connected)
    {
        curDof.resize(chain->getN());
    
        for (unsigned int i=0; i<chain->getN(); i++)
            curDof[i]=!(*chain)[i].isBlocked();
    
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setDOF(const Vector &newDof, Vector &curDof)
{
    if (connected)
    {
        Bottle command, reply;
    
        // begin of critical code
        mutex->wait();    
    
        // prepare command
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_DOF);
        Bottle &dof=command.addList();
        for (int i=0; i<newDof.length(); i++)
            dof.addDouble(newDof[i]);
    
        // send command to solver and wait for reply
        if (!portSlvRpc->write(command,reply))
        {
            fprintf(stdout,"%s error: unable to get reply from solver!\n",slvName.c_str());
            mutex->post();
            return false;
        }
    
        // update chain's links
        // skip the first ack/nack vocab
        Bottle *dofPart=reply.get(1).asList();
        curDof.resize(dofPart->size());
        for (int i=0; i<dofPart->size(); i++)
            if (curDof[i]=dofPart->get(i).asInt())
                chain->releaseLink(i);
            else
                chain->blockLink(i);
            
        // update controller
        newController();
    
        // end of critical code
        mutex->post();
    
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::getLimits(int axis, double *min, double *max)
{
    if (connected)
    {
        if (axis<(int)chain->getN())
        {
            *min=(180.0/M_PI)*(*chain)[axis].getMin();
            *max=(180.0/M_PI)*(*chain)[axis].getMax();

            return true;
        }
        else
            return false;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setLimits(int axis, const double min, const double max)
{
    if (connected)
    {
        Bottle command, reply;

        // prepare command
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_LIM);
        command.addInt(axis);
        command.addDouble(min);
        command.addDouble(max);

        // send command to solver and wait for reply
        if (!portSlvRpc->write(command,reply))
        {
            fprintf(stdout,"%s error: unable to get reply from solver!\n",slvName.c_str());
            return false;
        }

        if (reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK)
        {
            // align local joint's limits
            (*chain)[axis].setMin((M_PI/180.0)*min);
            (*chain)[axis].setMax((M_PI/180.0)*max);

            return true;
        }
        else
            return false;        
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::getTrajTime(double *t)
{
    if (attached)
    {
        *t=trajTime;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setTrajTime(const double t)
{
    if (attached)
    {
        trajTime=ctrl->set_execTime(t,true);
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::getInTargetTol(double *tol)
{
    if (attached)
    {
        *tol=ctrl->getInTargetTol();
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setInTargetTol(const double tol)
{
    if (attached)
    {
        ctrl->setInTargetTol(tol);
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::checkMotionDone(bool *f)
{
    if (attached)
    {
        *f=motionDone;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::stopControl()
{
    if (connected)
    {
        executingTraj=false;
        motionDone   =true;

        stopLimbVel();
        setTrackingMode(false);
    
        return true;
    }
    else
        return false;
}


/************************************************************************/
ServerCartesianController::~ServerCartesianController()
{
    close();
}



