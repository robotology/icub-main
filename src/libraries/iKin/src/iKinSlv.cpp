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

#include <stdio.h>
#include <algorithm>

#include <gsl/gsl_math.h>

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

#include <iCub/iKin/iKinVocabs.h>
#include <iCub/iKin/iKinSlv.h>

#define CARTSLV_DEFAULT_PER                 20      // [ms]
#define CARTSLV_DEFAULT_TOL                 1e-3
#define CARTSLV_WEIGHT_2ND_TASK             0.01
#define CARTSLV_WEIGHT_3RD_TASK             0.01
#define CARTSLV_UNCTRLEDJNTS_THRES          1.0     // [deg]

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


/************************************************************************/
bool RpcProcessor::read(ConnectionReader &connection)
{
    if (!slv->isClosed())
    {
        Bottle cmd, reply;
        if (!cmd.read(connection))
            return false;
    
        slv->respond(cmd,reply);
        if (ConnectionWriter *writer=connection.getWriter())
            reply.write(*writer);
    
        return true;
    }
    else
        return false;
}


/************************************************************************/
InputPort::InputPort(CartesianSolver *_slv)
{
    slv=_slv;

    maxLen=7;
    xd.resize(maxLen,0.0);
    dof.resize(1,0.0);

    pose=IKINCTRL_POSE_FULL;
    contMode=false;
    isNew=false;
    token=0.0;
    pToken=NULL;
}


/************************************************************************/
void InputPort::set_dof(const Vector &_dof)
{
    mutex.lock();
    dof=_dof;
    mutex.unlock();
}


/************************************************************************/
Vector InputPort::get_dof()
{
    mutex.lock();
    Vector _dof=dof;
    mutex.unlock();
    return _dof;
}


/************************************************************************/
Vector InputPort::get_xd()
{
    mutex.lock();
    Vector _xd=xd;
    mutex.unlock();
    return _xd;
}


/************************************************************************/
void InputPort::reset_xd(const Vector &_xd)
{
    mutex.lock();
    xd=_xd;
    mutex.unlock();
    isNew=false;
}


/************************************************************************/
bool InputPort::isNewDataEvent()
{
    if (isNew)
    {
        isNew=false;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool InputPort::handleTarget(Bottle *b)
{
    if (b!=NULL)
    {
        mutex.lock();
        int len=std::min(b->size(),maxLen);
        for (int i=0; i<len; i++)
            xd[i]=b->get(i).asDouble();
        mutex.unlock();

        return isNew=true;
    }
    else
        return false;
}


/************************************************************************/
bool InputPort::handleDOF(Bottle *b)
{
    if (b!=NULL)
    {
        slv->lock();

        mutex.lock();
        dof.resize(b->size());
        for (int i=0; i<b->size(); i++)
            dof[i]=b->get(i).asInt();
        mutex.unlock();

        slv->unlock();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool InputPort::handlePose(const int newPose)
{
    if (newPose==IKINSLV_VOCAB_VAL_POSE_FULL)
    {
        if (pose==IKINCTRL_POSE_XYZ)
            isNew=true;

        pose=IKINCTRL_POSE_FULL;
        return true;
    }
    else if (newPose==IKINSLV_VOCAB_VAL_POSE_XYZ)
    {
        if (pose==IKINCTRL_POSE_FULL)
            isNew=true;

        pose=IKINCTRL_POSE_XYZ;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool InputPort::handleMode(const int newMode)
{
    if (newMode==IKINSLV_VOCAB_VAL_MODE_TRACK)
    {
        slv->lock();
        contMode=true;
        slv->unlock();
        return true;
    }
    else if (newMode==IKINSLV_VOCAB_VAL_MODE_SINGLE)
    {
        slv->lock();
        contMode=false;
        slv->unlock();
        return true;
    }
    else
        return false;
}


/************************************************************************/
void InputPort::onRead(Bottle &b)
{
    bool xdOptIn  =b.check(Vocab::decode(IKINSLV_VOCAB_OPT_XD));
    bool dofOptIn =b.check(Vocab::decode(IKINSLV_VOCAB_OPT_DOF));
    bool poseOptIn=b.check(Vocab::decode(IKINSLV_VOCAB_OPT_POSE));
    bool modeOptIn=b.check(Vocab::decode(IKINSLV_VOCAB_OPT_MODE));

    if (xdOptIn)
    {
        if (CartesianHelper::getTokenOption(b,&token))
            pToken=&token;
        else
            pToken=NULL;
    }

    if (modeOptIn)
        if (!handleMode(b.find(Vocab::decode(IKINSLV_VOCAB_OPT_MODE)).asVocab()))
            printf("%s: got incomplete %s command\n",slv->slvName.c_str(),
                   Vocab::decode(IKINSLV_VOCAB_OPT_MODE).c_str());

    if (dofOptIn)
        if (!handleDOF(b.find(Vocab::decode(IKINSLV_VOCAB_OPT_DOF)).asList()))
            printf("%s: expected %s data\n",slv->slvName.c_str(),
                   Vocab::decode(IKINSLV_VOCAB_OPT_DOF).c_str());

    if (poseOptIn)
        if (!handlePose(b.find(Vocab::decode(IKINSLV_VOCAB_OPT_POSE)).asVocab()))
            printf("%s: got incomplete %s command\n",slv->slvName.c_str(),
                   Vocab::decode(IKINSLV_VOCAB_OPT_POSE).c_str());

    if (slv->handleJointsRestPosition(b.find(Vocab::decode(IKINSLV_VOCAB_OPT_REST_POS)).asList()) ||
        slv->handleJointsRestWeights(b.find(Vocab::decode(IKINSLV_VOCAB_OPT_REST_WEIGHTS)).asList()))
    {
        slv->lock();
        slv->prepareJointsRestTask();
        slv->unlock();
    }

    // shall be the last handling
    if (xdOptIn)
    {    
        if (!handleTarget(b.find(Vocab::decode(IKINSLV_VOCAB_OPT_XD)).asList()))
            printf("%s: expected %s data\n",slv->slvName.c_str(),
                   Vocab::decode(IKINSLV_VOCAB_OPT_XD).c_str());
    }
    else
        printf("%s: missing %s data; it shall be present\n",
               slv->slvName.c_str(),Vocab::decode(IKINSLV_VOCAB_OPT_XD).c_str());
}


/************************************************************************/
void SolverCallback::exec(const Vector &xd, const Vector &q)
{
    slv->send(xd,slv->prt->chn->EndEffPose(),CTRL_RAD2DEG*q,slv->pToken);
}


/************************************************************************/
CartesianSolver::CartesianSolver(const string &_slvName) : RateThread(CARTSLV_DEFAULT_PER)
{          
    // initialization
    slvName=_slvName;
    configured=false;
    closing=false;
    closed=false;
    interrupting=false;
    verbosity=false;
    timeout_detected=false;
    maxPartJoints=0;
    unctrlJointsNum=0;
    ping_robot_tmo=0.0;

    prt=NULL;
    slv=NULL;
    clb=NULL;
    inPort=NULL;
    outPort=NULL;

    // open rpc port
    rpcPort=new Port;
    cmdProcessor=new RpcProcessor(this);
    rpcPort->setReader(*cmdProcessor);
    rpcPort->open(("/"+slvName+"/rpc").c_str());

    // token
    token=0.0;
    pToken=NULL;

    // request high resolution scheduling
    Time::turboBoost();
}


/************************************************************************/
PolyDriver *CartesianSolver::waitPart(const Property &partOpt)
{    
    Property &options=const_cast<Property&>(partOpt);
    string partName=options.find("part").asString().c_str();
    PolyDriver *pDrv=NULL;

    double t0=Time::now();
    while (Time::now()-t0<ping_robot_tmo)
    {
        delete pDrv;

        pDrv=new PolyDriver(options);
        bool ok=pDrv->isValid();

        printf("%s: Checking if %s part is active ... ",
               slvName.c_str(),partName.c_str());

        if (ok)
        {
            printf("ok\n");
            return pDrv;
        }
        else
        {
            double dt=ping_robot_tmo-(Time::now()-t0);
            printf("not yet: still %.1f [s] to timeout expiry\n",dt>0.0?dt:0.0);

            double t1=Time::now();
            while (Time::now()-t1<1.0)
                Time::delay(0.1);
        }

        if (interrupting)
            break;
    }

    return pDrv;
}


/************************************************************************/
void CartesianSolver::alignJointsBounds()
{
    double min, max;
    int cnt=0;

    printf("%s: aligning joints bounds ...\n",slvName.c_str());
    for (int i=0; i<prt->num; i++)
    {
        printf("part #%d: %s\n",i,prt->prp[i].find("part").asString().c_str());
        for (int j=0; j<jnt[i]; j++)
        {               
            lim[i]->getLimits(rmp[i][j],&min,&max);

            printf("joint #%d: [%g, %g] deg\n",cnt,min,max);
            (*prt->chn)[cnt].setMin(CTRL_DEG2RAD*min);
            (*prt->chn)[cnt].setMax(CTRL_DEG2RAD*max);
        
            cnt++;
        }
    }
}


/************************************************************************/
bool CartesianSolver::setLimits(int axis, double min, double max)
{
    int cnt=0;

    for (int i=0; i<prt->num; i++)
    {
        for (int j=0; j<jnt[i]; j++)
        {
            if (cnt++==axis)
            {
                double curMin, curMax;
            
                lim[i]->getLimits(rmp[i][j],&curMin,&curMax);
            
                if ((min>=curMin) && (max<=curMax))
                {
                    (*prt->chn)[axis].setMin(CTRL_DEG2RAD*min);
                    (*prt->chn)[axis].setMax(CTRL_DEG2RAD*max);
            
                    return true;
                }
                else
                    return false;
            }
        }
    }

    return false;
}


/************************************************************************/
void CartesianSolver::countUncontrolledJoints()
{
    unctrlJointsNum=prt->chn->getN()-prt->chn->getDOF();
}


/************************************************************************/
void CartesianSolver::latchUncontrolledJoints(Vector &joints)
{
    if (unctrlJointsNum>0)
    {
        joints.resize(unctrlJointsNum);
        int j=0;
        
        for (unsigned int i=0; i<prt->chn->getN(); i++)
            if ((*prt->chn)[i].isBlocked())
                joints[j++]=prt->chn->getAng(i);
    }
    else
        joints.resize(1);
}


/************************************************************************/
void CartesianSolver::getFeedback(const bool wait)
{
    Vector fbTmp(maxPartJoints);
    int chainCnt=0;

    for (int i=0; i<prt->num; i++)
    {    
        bool ok=enc[i]->getEncoders(fbTmp.data());
        while (wait && !closing && !ok)
        {
            ok=enc[i]->getEncoders(fbTmp.data());
            Time::delay(0.00025*getRate()); // wait for 1/4 of thread period
        }

        if (ok)
        {    
            for (int j=0; j<jnt[i]; j++)
            {
                double tmp=CTRL_DEG2RAD*fbTmp[rmp[i][j]];
            
                if ((*prt->chn)[chainCnt].isBlocked())
                    prt->chn->setBlockingValue(chainCnt,tmp);
                else
                    prt->chn->setAng(chainCnt,tmp);
            
                chainCnt++;
            }
        }
        else
        {    
            printf("%s: timeout detected on part %s!\n",
                   slvName.c_str(),prt->prp[i].find("part").asString().c_str());

            timeout_detected=true;
            chainCnt+=jnt[i];
        }
    }
}


/************************************************************************/
void CartesianSolver::initPos()
{
    lock();
    getFeedback(true);  // wait until all joints are acquired
    unlock();

    latchUncontrolledJoints(unctrlJointsOld);
    inPort->reset_xd(prt->chn->EndEffPose());
}


/************************************************************************/
void CartesianSolver::lock()
{
    mutex.lock();
}


/************************************************************************/
void CartesianSolver::unlock()
{
    mutex.unlock();
}


/************************************************************************/
double CartesianSolver::getNorm(const Vector &v, const string &typ)
{
    double ret=0.0;
    int i1=0;
    int i2=3;

    if (typ=="ang")
    {
        i1=i2;
        i2=v.length();
    }

    for (int i=i1; i<i2; i++)
        ret+=v[i]*v[i];

    return sqrt(ret);
}


/************************************************************************/
void CartesianSolver::waitDOFHandling()
{
    dofEvent.reset();
    dofEvent.wait();
}


/************************************************************************/
void CartesianSolver::postDOFHandling()
{
    dofEvent.signal();
}


/************************************************************************/
void CartesianSolver::fillDOFInfo(Bottle &reply)
{
    for (unsigned int i=0; i<prt->chn->getN(); i++)
        reply.addInt(int(!(*prt->chn)[i].isBlocked()));
}


/************************************************************************/
void CartesianSolver::respond(const Bottle &command, Bottle &reply)
{
    if (command.size())
    {
        int vcb=command.get(0).asVocab();

        if (!configured && vcb!=IKINSLV_VOCAB_CMD_CFG)
            reply.addVocab(IKINSLV_VOCAB_REP_NACK);
        else switch (vcb)
        {
            //-----------------
            case IKINSLV_VOCAB_CMD_GET:
            {
                if (command.size()>1)
                {
                    switch (command.get(1).asVocab())
                    {
                        //-----------------
                        case IKINSLV_VOCAB_OPT_POSE:
                        {
                            reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                    
                            if (ctrlPose==IKINCTRL_POSE_FULL)
                                reply.addVocab(IKINSLV_VOCAB_VAL_POSE_FULL);
                            else
                                reply.addVocab(IKINSLV_VOCAB_VAL_POSE_XYZ);
                    
                            break;
                        }
                    
                        //-----------------
                        case IKINSLV_VOCAB_OPT_PRIO:
                        {
                            reply.addVocab(IKINSLV_VOCAB_REP_ACK);

                            string priority=slv->get_posePriority();
                            if (priority=="position")
                                reply.addVocab(IKINSLV_VOCAB_VAL_PRIO_XYZ);
                            else
                                reply.addVocab(IKINSLV_VOCAB_VAL_PRIO_ANG);

                            break;
                        }

                        //-----------------
                        case IKINSLV_VOCAB_OPT_MODE:
                        {
                            reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                    
                            if (inPort->get_contMode())
                                reply.addVocab(IKINSLV_VOCAB_VAL_MODE_TRACK);
                            else
                                reply.addVocab(IKINSLV_VOCAB_VAL_MODE_SINGLE);
                    
                            break;
                        }
                    
                        //-----------------
                        case IKINSLV_VOCAB_OPT_LIM:
                        {
                            if (command.size()>2)
                            {                    
                                int axis=command.get(2).asInt();
                    
                                if (axis<(int)prt->chn->getN())
                                {
                                    reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                                    reply.addDouble(CTRL_RAD2DEG*(*prt->chn)[axis].getMin());
                                    reply.addDouble(CTRL_RAD2DEG*(*prt->chn)[axis].getMax());
                                }
                                else
                                    reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                            }
                            else
                                reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                    
                            break;
                        }
                    
                        //-----------------
                        case IKINSLV_VOCAB_OPT_VERB:
                        {
                            reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                    
                            if (verbosity)
                                reply.addVocab(IKINSLV_VOCAB_VAL_ON);
                            else
                                reply.addVocab(IKINSLV_VOCAB_VAL_OFF);
                    
                            break;
                        }
                    
                        //-----------------
                        case IKINSLV_VOCAB_OPT_DOF:
                        {
                            reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            Bottle &dofPart=reply.addList();
                            fillDOFInfo(dofPart);
                            break;
                        }
                    
                        //-----------------
                        case IKINSLV_VOCAB_OPT_REST_POS:
                        {
                            reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            handleJointsRestPosition(NULL,&reply);
                            break;
                        }
                    
                        //-----------------
                        case IKINSLV_VOCAB_OPT_REST_WEIGHTS:
                        {
                            reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            handleJointsRestWeights(NULL,&reply);
                            break;
                        }
                    
                        //-----------------
                        case IKINSLV_VOCAB_OPT_TASK2:
                        {
                            reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            Bottle &payLoad=reply.addList();
                            payLoad.addInt(slv->get2ndTaskChain().getN());
            
                            Bottle &posPart=payLoad.addList();
                            for (size_t i=0; i<xd_2ndTask.length(); i++)
                                posPart.addDouble(xd_2ndTask[i]);

                            Bottle &weightsPart=payLoad.addList();
                            for (size_t i=0; i<w_2ndTask.length(); i++)
                                weightsPart.addDouble(w_2ndTask[i]);
            
                            break; 
                        }
            
                        //-----------------
                        case IKINSLV_VOCAB_OPT_CONVERGENCE:
                        {
                            reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            Bottle &payLoad=reply.addList();

                            Bottle &maxIter=payLoad.addList();
                            maxIter.addString("max_iter");
                            maxIter.addInt(slv->getMaxIter());

                            Bottle &tol=payLoad.addList();
                            tol.addString("tol");
                            tol.addDouble(slv->getTol());

                            Bottle &ttol=payLoad.addList();
                            ttol.addString("translationalTol");
                            ttol.addDouble(slv->getTranslationalTol());

                            break;
                        }

                        //-----------------
                        default:
                            reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                    }
                }
                else
                    reply.addVocab(IKINSLV_VOCAB_REP_NACK);
            
                break;
            }
            
            //-----------------
            case IKINSLV_VOCAB_CMD_SET:
            {
                if (command.size()>2)
                {
                    switch (command.get(1).asVocab())
                    {
                        //-----------------
                        case IKINSLV_VOCAB_OPT_POSE:
                        {
                            if (inPort->handlePose(command.get(2).asVocab()))
                                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            else
                                reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                    
                            break;
                        }
                    
                        //-----------------
                        case IKINSLV_VOCAB_OPT_PRIO:
                        {
                            int type=command.get(2).asVocab();
                            if (type==IKINSLV_VOCAB_VAL_PRIO_XYZ)
                            {
                                slv->set_posePriority("position");
                                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            }
                            else if (type==IKINSLV_VOCAB_VAL_PRIO_ANG)
                            {
                                slv->set_posePriority("orientation");
                                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            }
                            else
                                reply.addVocab(IKINSLV_VOCAB_REP_NACK); 

                            break;
                        }

                        //-----------------
                        case IKINSLV_VOCAB_OPT_MODE:
                        {
                            if (inPort->handleMode(command.get(2).asVocab()))
                                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            else
                                reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                    
                            break;
                        }
                    
                        //-----------------
                        case IKINSLV_VOCAB_OPT_LIM:
                        {
                            if (command.size()>4)
                            {
                                int axis=command.get(2).asInt();
                                double min=command.get(3).asDouble();
                                double max=command.get(4).asDouble();
                    
                                if (setLimits(axis,min,max))
                                    reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                                else
                                    reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                            }
                            else
                                reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                    
                            break;
                        }
                    
                        //-----------------
                        case IKINSLV_VOCAB_OPT_VERB:
                        {
                            int sw=command.get(2).asVocab();
                    
                            if (sw==IKINSLV_VOCAB_VAL_ON)
                            {
                                verbosity=true;    
                                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            }
                            else if (sw==IKINSLV_VOCAB_VAL_OFF)
                            {   
                                verbosity=false; 
                                reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                            }
                    
                            break;
                        }
                    
                        //-----------------
                        case IKINSLV_VOCAB_OPT_DOF:
                        {
                            if (inPort->handleDOF(command.get(2).asList()))
                            {                                    
                                waitDOFHandling();  // sleep till dof handling is done
                    
                                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                                Bottle &dofPart=reply.addList();
                                fillDOFInfo(dofPart);
                            }
                            else
                                reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                    
                            break;
                        }
                    
                        //-----------------
                        case IKINSLV_VOCAB_OPT_REST_POS:
                        {
                            Bottle restPart;
                    
                            if (handleJointsRestPosition(command.get(2).asList(),&restPart))
                            {
                                lock();
                                prepareJointsRestTask();
                                unlock();
                    
                                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                                reply.append(restPart);
                            }
                            else
                                reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                    
                            break;
                        }
                    
                        //-----------------
                        case IKINSLV_VOCAB_OPT_REST_WEIGHTS:
                        {
                            Bottle restPart;
                    
                            if (handleJointsRestWeights(command.get(2).asList(),&restPart))
                            {
                                lock();
                                prepareJointsRestTask();
                                unlock();
                    
                                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                                reply.append(restPart);
                            }
                            else
                                reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                    
                            break;
                        }
                    
                        //-----------------
                        case IKINSLV_VOCAB_OPT_TIP_FRAME:
                        {
                            if (Bottle *tipPart=command.get(2).asList())
                            {
                                if (tipPart->size()>=7)
                                {
                                    Vector x(3);
                                    for (size_t i=0; i<x.length(); i++)
                                        x[i]=tipPart->get(i).asDouble();
            
                                    Vector o(4);
                                    for (size_t i=0; i<o.length(); i++)
                                        o[i]=tipPart->get(i+x.length()).asDouble();
            
                                    Matrix HN=axis2dcm(o);
                                    HN(0,3)=x[0];
                                    HN(1,3)=x[1];
                                    HN(2,3)=x[2];
            
                                    lock();
                                    prt->chn->setHN(HN);
                                    unlock();
            
                                    reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                                    break;
                                }
                            }
            
                            reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                            break;
                        }
            
                        //-----------------
                        case IKINSLV_VOCAB_OPT_TASK2:
                        {
                            if (Bottle *payLoad=command.get(2).asList())
                            {
                                if (payLoad->size()>=3)
                                {
                                    int n=payLoad->get(0).asInt();
                                    Bottle *posPart=payLoad->get(1).asList();
                                    Bottle *weightsPart=payLoad->get(2).asList();
            
                                    if ((posPart!=NULL) && (weightsPart!=NULL))
                                    {
                                        if ((posPart->size()>=3) && (weightsPart->size()>=3))
                                        {
                                            for (size_t i=0; i<xd_2ndTask.length(); i++)
                                                xd_2ndTask[i]=posPart->get(i).asDouble();
            
                                            for (size_t i=0; i<w_2ndTask.length(); i++)
                                                w_2ndTask[i]=weightsPart->get(i).asDouble();
            
                                            if (n>=0)
                                                slv->specify2ndTaskEndEff(n);
                                            else
                                                slv->get2ndTaskChain().clear();

                                            reply.addVocab(IKINSLV_VOCAB_REP_ACK); 
                                            break;
                                        }
                                    }
                                }
                            }
           
                            reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                            break;
                        }

                        //-----------------
                        case IKINSLV_VOCAB_OPT_CONVERGENCE:
                        {
                            if (Bottle *payLoad=command.get(2).asList())
                            {
                                int cnt=0;
                                if (payLoad->check("max_iter"))
                                {
                                    slv->setMaxIter(payLoad->find("max_iter").asInt());
                                    cnt++;
                                }

                                if (payLoad->check("tol"))
                                {
                                    slv->setTol(payLoad->find("tol").asDouble());
                                    cnt++;
                                }

                                if (payLoad->check("translationalTol"))
                                {
                                    slv->setTranslationalTol(payLoad->find("translationalTol").asDouble());
                                    cnt++;
                                }

                                reply.addVocab(cnt>0?IKINSLV_VOCAB_REP_ACK:IKINSLV_VOCAB_REP_NACK);
                            }
                            else
                                reply.addVocab(IKINSLV_VOCAB_REP_NACK);

                            break;
                        }

                        //-----------------
                        default:
                            reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                    }
                }
                else
                    reply.addVocab(IKINSLV_VOCAB_REP_NACK);
            
                break;
            }
            
            //-----------------
            case IKINSLV_VOCAB_CMD_ASK:
            {
                Bottle &options=const_cast<Bottle&>(command);
            
                Bottle *b_xd=getTargetOption(options);
                Bottle *b_q=getJointsOption(options);
            
                // some integrity checks
                if (b_xd==NULL)
                {
                    reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                    break;
                }
                else if (b_xd->size()<3)    // at least the positional part must be given 
                {
                    reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                    break;
                }
            
                lock();
            
                // get the target
                Vector xd(b_xd->size());
                for (size_t i=0; i<xd.length(); i++)
                    xd[i]=b_xd->get(i).asDouble();
            
                // accounts for the starting DOF
                // if different from the actual one
                if (b_q!=NULL)
                {
                    size_t len=std::min((size_t)b_q->size(),(size_t)prt->chn->getDOF());
                    for (size_t i=0; i<len; i++)
                        (*prt->chn)(i).setAng(CTRL_DEG2RAD*b_q->get(i).asDouble());
                }
                else
                    getFeedback();  // otherwise get the current configuration
            
                // account for the pose 
                if (options.check(Vocab::decode(IKINSLV_VOCAB_OPT_POSE)))
                {
                    int pose=options.find(Vocab::decode(IKINSLV_VOCAB_OPT_POSE)).asVocab();
            
                    if (pose==IKINSLV_VOCAB_VAL_POSE_FULL)
                        slv->set_ctrlPose(IKINCTRL_POSE_FULL);
                    else if (pose==IKINSLV_VOCAB_VAL_POSE_XYZ)
                        slv->set_ctrlPose(IKINCTRL_POSE_XYZ);
                }
            
                // set things for the 3rd task
                for (unsigned int i=0; i<prt->chn->getDOF(); i++)
                    if (idx_3rdTask[i]!=0.0)
                        qd_3rdTask[i]=(*prt->chn)(i).getAng();
            
                // call the solver to converge
                double t0=Time::now();
                Vector q=solve(xd);
                double t1=Time::now();
            
                Vector x=prt->chn->EndEffPose(q);
            
                // change to degrees
                q=CTRL_RAD2DEG*q;
            
                // dump on screen
                if (verbosity)
                    printInfo("ask",xd,x,q,t1-t0);
            
                unlock();
            
                // prepare the complete joints configuration
                Vector _q(prt->chn->getN());
                for (unsigned int i=0; i<prt->chn->getN(); i++)
                    _q[i]=CTRL_RAD2DEG*prt->chn->getAng(i);
            
                // fill the reply accordingly
                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                addVectorOption(reply,IKINSLV_VOCAB_OPT_X,x);
                addVectorOption(reply,IKINSLV_VOCAB_OPT_Q,_q);
            
                break;
            }

            //-----------------
            case IKINSLV_VOCAB_CMD_SUSP:
            {
                suspend();
                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                break;
            }
            
            //-----------------
            case IKINSLV_VOCAB_CMD_RUN:
            {
                if (!isRunning())
                {
                    initPos();
                    start();
                }
                else
                    resume();
            
                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                break;
            }
                        
            //-----------------
            case IKINSLV_VOCAB_CMD_STATUS:
            {
                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                if (isSuspended())
                    reply.addString("suspended");
                else if (isRunning())
                    reply.addString("running");
                else
                    reply.addString("not_started");
                break; 
            }

            //-----------------
            case IKINSLV_VOCAB_CMD_CFG:
            {
                Property options(command.toString().c_str());
                printf("Configuring with options: %s\n",options.toString().c_str());
            
                if (open(options))
                    reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                else
                    reply.addVocab(IKINSLV_VOCAB_REP_NACK);
            
                break;
            }
            
            //-----------------
            case IKINSLV_VOCAB_CMD_HELP:
            {                    
                reply.addVocab(Vocab::encode("many"));
                reply.addString("***** commands");
                reply.addVocab(IKINSLV_VOCAB_CMD_GET);
                reply.addVocab(IKINSLV_VOCAB_CMD_SET);
                reply.addVocab(IKINSLV_VOCAB_CMD_ASK);
                reply.addVocab(IKINSLV_VOCAB_CMD_SUSP);
                reply.addVocab(IKINSLV_VOCAB_CMD_RUN);
                reply.addVocab(IKINSLV_VOCAB_CMD_STATUS);
                reply.addVocab(IKINSLV_VOCAB_CMD_CFG);
                reply.addVocab(IKINSLV_VOCAB_CMD_HELP);
                reply.addVocab(IKINSLV_VOCAB_CMD_QUIT);
                reply.addString("***** options");
                reply.addVocab(IKINSLV_VOCAB_OPT_MODE);
                reply.addVocab(IKINSLV_VOCAB_OPT_POSE);
                reply.addVocab(IKINSLV_VOCAB_OPT_DOF);
                reply.addVocab(IKINSLV_VOCAB_OPT_LIM);
                reply.addVocab(IKINSLV_VOCAB_OPT_VERB);
                reply.addVocab(IKINSLV_VOCAB_OPT_TOKEN);
                reply.addVocab(IKINSLV_VOCAB_OPT_REST_POS);
                reply.addVocab(IKINSLV_VOCAB_OPT_REST_WEIGHTS);
                reply.addVocab(IKINSLV_VOCAB_OPT_TIP_FRAME);
                reply.addVocab(IKINSLV_VOCAB_OPT_TASK2);
                reply.addVocab(IKINSLV_VOCAB_OPT_XD);
                reply.addVocab(IKINSLV_VOCAB_OPT_X);
                reply.addVocab(IKINSLV_VOCAB_OPT_Q);
                reply.addString("***** values");
                reply.addVocab(IKINSLV_VOCAB_VAL_POSE_FULL);
                reply.addVocab(IKINSLV_VOCAB_VAL_POSE_XYZ);
                reply.addVocab(IKINSLV_VOCAB_VAL_MODE_TRACK);
                reply.addVocab(IKINSLV_VOCAB_VAL_MODE_SINGLE);
                reply.addVocab(IKINSLV_VOCAB_VAL_ON);
                reply.addVocab(IKINSLV_VOCAB_VAL_OFF);
                break;
            }
            
            //-----------------
            case IKINSLV_VOCAB_CMD_QUIT:
            {
                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                close();
                break;
            }
            
            //-----------------
            default:
                reply.addVocab(IKINSLV_VOCAB_REP_NACK);
        }
    }
    else
        reply.addVocab(IKINSLV_VOCAB_REP_NACK);
}


/************************************************************************/
void CartesianSolver::send(const Vector &xd, const Vector &x, const Vector &q,
                           double *tok)
{       
    Bottle &b=outPort->prepare();
    b.clear();

    addVectorOption(b,IKINSLV_VOCAB_OPT_XD,xd);
    addVectorOption(b,IKINSLV_VOCAB_OPT_X,x);
    addVectorOption(b,IKINSLV_VOCAB_OPT_Q,q);

    if (tok!=NULL)
        addTokenOption(b,*tok);

    outPort->writeStrict();
}


/************************************************************************/
void CartesianSolver::printInfo(const string &typ, const Vector &xd,
                                const Vector &x, const Vector &q,
                                const double t)
{
    // ensure same lenght of vectors
    Vector _x=x.subVector(0,xd.length()-1);

    // compute error
    Vector e=xd-_x;

    printf("\n");
    printf("   Request type       = %s\n",typ.c_str());
    printf("  Target rxPose   [m] = %s\n",const_cast<Vector&>(xd).toString().c_str());
    printf("  Target txPose   [m] = %s\n",const_cast<Vector&>(_x).toString().c_str());
    printf("Target txJoints [deg] = %s\n",const_cast<Vector&>(q).toString().c_str());
    printf("  norm(rxPose-txPose) = pos [m]: %g\n",getNorm(e,"pos"));
    if (ctrlPose==IKINCTRL_POSE_FULL)
    printf("                        ang [*]: %g\n",getNorm(e,"ang"));
    printf("    computed in   [s] = %g\n",t);
    printf("\n");
}


/************************************************************************/
Vector &CartesianSolver::encodeDOF()
{
    dof.resize(prt->chn->getN());
    bool fullness=true;

    for (unsigned int i=0; i<prt->chn->getN(); i++)
        if (!(dof[i]=!(*prt->chn)[i].isBlocked()))
            fullness=false;

    fullDOF=fullness;
    return dof;
}


/************************************************************************/
bool CartesianSolver::decodeDOF(const Vector &_dof)
{
    size_t len=std::min((size_t)prt->chn->getN(),_dof.length());
    for (size_t i=0; i<len; i++)
    {
        if (_dof[i]>1.0)
            continue;
        else if (_dof[i]!=0.0)
            prt->chn->releaseLink(i);
        else
            prt->chn->blockLink(i);
    }

    prepareJointsRestTask();

    return true;
}


/************************************************************************/
bool CartesianSolver::handleJointsRestPosition(const Bottle *options, Bottle *reply)
{
    bool ret=false;

    if (options!=NULL)
    {            
        size_t len=std::min((size_t)options->size(),restJntPos.length());
        for (size_t i=0; i<len; i++)
        {
            double val=CTRL_DEG2RAD*options->get(i).asDouble();
            double min=(*prt->chn)[i].getMin();
            double max=(*prt->chn)[i].getMax();

            restJntPos[i]=std::min(std::max(val,min),max);
        }

        ret=true;
    }

    if (reply!=NULL)
    {
        Bottle &b=reply->addList();
        for (size_t i=0; i<restJntPos.length(); i++)
            b.addDouble(CTRL_RAD2DEG*restJntPos[i]);
    }

    return ret;
}


/************************************************************************/
bool CartesianSolver::handleJointsRestWeights(const Bottle *options, Bottle *reply)
{
    bool ret=false;

    if (options!=NULL)
    {            
        size_t len=std::min((size_t)options->size(),restWeights.length());
        for (size_t i=0; i<len; i++)
        {
            double val=options->get(i).asInt();
            restWeights[i]=std::max(val,0.0);
        }

        ret=true;
    }

    if (reply!=NULL)
    {
        Bottle &b=reply->addList();
        for (size_t i=0; i<restWeights.length(); i++)
            b.addDouble(restWeights[i]);
    }

    return ret;
}


/************************************************************************/
bool CartesianSolver::isNewDOF(const Vector &_dof)
{
    size_t len=std::min((size_t)prt->chn->getN(),_dof.length());
    for (size_t i=0; i<len; i++)
    {
        if (_dof[i]>1.0)
            continue;
        else if (_dof[i]!=dof[i])
            return true;
    }

    return false;
}


/************************************************************************/
bool CartesianSolver::open(Searchable &options)
{    
    if (configured)
    {
        printf("%s already configured\n",slvName.c_str());
        return true;
    }

    prt=getPartDesc(options);
    if (prt==NULL)
    {
        printf("Detected errors while processing parts description!\n");
        return false;
    }

    Property DHTable; prt->lmb->toLinksProperties(DHTable);
    printf("DH Table: %s\n",DHTable.toString().c_str());
    
    if (options.check("ping_robot_tmo"))
        ping_robot_tmo=options.find("ping_robot_tmo").asDouble();

    // open drivers
    int remainingJoints=prt->chn->getN();
    for (int i=0; i<prt->num; i++)
    {
        printf("Allocating device driver for %s ...\n",
               prt->prp[i].find("part").asString().c_str());

        PolyDriver *pDrv=(ping_robot_tmo>0.0)?
                         waitPart(prt->prp[i]):
                         new PolyDriver(prt->prp[i]);

        if (!pDrv->isValid())
        {
            delete pDrv;
            printf("Device driver not available!\n");            
            close();
            return false;
        }

        // create interfaces
        IControlLimits *pLim;
        IEncoders      *pEnc;
        int             joints;

        if (!pDrv->view(pLim) || !pDrv->view(pEnc))
        {    
            printf("Problems acquiring interfaces!\n");
            close();
            return false;
        }

        pEnc->getAxes(&joints);

        // this is for allocating vector
        // to read data from the interface
        if (joints>maxPartJoints)
            maxPartJoints=joints;

        // handle chain's bounds
        if (joints>remainingJoints)
            joints=remainingJoints;

        remainingJoints-=joints;

        int *rmpTmp=new int[joints];
        for (int j=0; j<joints; j++)
            rmpTmp[j]=prt->rvs[i]?(joints-j-1):j;

        drv.push_back(pDrv);
        lim.push_back(pLim);
        enc.push_back(pEnc);
        jnt.push_back(joints);
        rmp.push_back(rmpTmp);
    }

    // handle joints rest position and weights
    restJntPos.resize(prt->chn->getN(),0.0);
    restWeights.resize(prt->chn->getN(),0.0);
    handleJointsRestPosition(options.find("rest_pos").asList());
    handleJointsRestWeights(options.find("rest_weights").asList());
    prepareJointsRestTask();

    // dof here is not defined yet, so define it
    encodeDOF();

    // handle dof from options
    if (Bottle *v=options.find("dof").asList())
    {            
        Vector _dof(v->size());
        for (size_t i=0; i<_dof.length(); i++)
            _dof[i]=v->get(i).asInt();

        decodeDOF(_dof);
        encodeDOF();
    }

    // joints bounds alignment
    alignJointsBounds();    

    // parse configuration options
    setRate(period=options.check("period",Value(CARTSLV_DEFAULT_PER)).asInt());

    ctrlPose=IKINCTRL_POSE_FULL;
    if (options.check(Vocab::decode(IKINSLV_VOCAB_OPT_POSE)))
        if (options.find(Vocab::decode(IKINSLV_VOCAB_OPT_POSE)).asVocab()==IKINSLV_VOCAB_VAL_POSE_XYZ)
            ctrlPose=IKINCTRL_POSE_XYZ;

    bool mode=false;
    if (options.check(Vocab::decode(IKINSLV_VOCAB_OPT_MODE)))
        if (options.find(Vocab::decode(IKINSLV_VOCAB_OPT_MODE)).asVocab()==IKINSLV_VOCAB_VAL_MODE_TRACK)
            mode=true;

    if (options.check("verbosity"))
        if (options.find("verbosity").asVocab()==IKINSLV_VOCAB_VAL_ON)
            verbosity=true;

    double tol=options.check("tol",Value(CARTSLV_DEFAULT_TOL)).asDouble();
    int maxIter=options.check("maxIter",Value(200)).asInt();

    // instantiate the optimizer
    slv=new iKinIpOptMin(*prt->chn,ctrlPose,tol,maxIter);

    if (options.check("xyzTol"))
        slv->setTranslationalTol(options.find("xyzTol").asDouble());

    // instantiate solver callback object if required    
    if (options.check("interPoints"))
        if (options.find("interPoints").asVocab()==IKINSLV_VOCAB_VAL_ON)
            clb=new SolverCallback(this);

    // enable scaling
    slv->setUserScaling(true,100.0,100.0,100.0);

    // enforce linear inequalities constraints, if any
    if (prt->cns!=NULL)
    {
        slv->attachLIC(*prt->cns);
        
        double lower_bound_inf, upper_bound_inf;        
        slv->getBoundsInf(lower_bound_inf,upper_bound_inf);
        slv->getLIC().getLowerBoundInf()=2.0*lower_bound_inf;
        slv->getLIC().getUpperBoundInf()=2.0*upper_bound_inf;
        slv->getLIC().update(NULL);
    }

    // set up 2nd task
    xd_2ndTask.resize(3,0.0);
    w_2ndTask.resize(3,0.0);

    // define input port
    inPort=new InputPort(this);
    inPort->useCallback();
    
    // init input port data
    inPort->set_dof(dof);
    inPort->get_pose()=ctrlPose;
    inPort->get_contMode()=contModeOld=mode;

    // define output port
    outPort=new BufferedPort<Bottle>;    

    // count uncontrolled joints
    countUncontrolledJoints();

    // get starting position
    initPos();

    configured=true;

    start();

    // open ports as very last thing, so that
    // the solver is completely operative
    // when it becomes yarp-visible
    inPort->open(("/"+slvName+"/in").c_str());
    outPort->open(("/"+slvName+"/out").c_str());

    return true;
}


/************************************************************************/
bool CartesianSolver::changeDOF(const Vector &_dof)
{
    if (isNewDOF(_dof))
    {
        // dof stuff
        Vector curDOF=dof;
        decodeDOF(_dof);
        inPort->set_dof(encodeDOF());

        if (dof==curDOF)
            return false;

        // update LinIneqConstr if any
        if (prt->cns!=NULL)
            prt->cns->update(NULL);

        // count uncontrolled joints
        countUncontrolledJoints();

        // get starting position
        getFeedback();
        latchUncontrolledJoints(unctrlJointsOld);

        return true;
    }
    else
        return false;
}


/************************************************************************/
void CartesianSolver::prepareJointsRestTask()
{
    unsigned int nDOF=prt->chn->getDOF();
    int offs=0;
    
    qd_3rdTask.resize(nDOF);
    w_3rdTask.resize(nDOF);
    idx_3rdTask.resize(nDOF);

    for (unsigned int i=0; i<prt->chn->getN(); i++)
    {
        if (!(*prt->chn)[i].isBlocked())
        {            
            qd_3rdTask[offs]=restJntPos[i];
            w_3rdTask[offs]=(restWeights[i]!=0.0)?restWeights[i]:1.0;
            idx_3rdTask[offs]=(restWeights[i]!=0.0)?0.0:1.0;

            offs++;
        }
    }
}


/************************************************************************/
Vector CartesianSolver::solve(Vector &xd)
{
    return slv->solve(prt->chn->getAng(),xd,
                      slv->get2ndTaskChain().getN()>0?CARTSLV_WEIGHT_2ND_TASK:0.0,xd_2ndTask,w_2ndTask,
                      CARTSLV_WEIGHT_3RD_TASK,qd_3rdTask,w_3rdTask,
                      NULL,NULL,clb);
}


/************************************************************************/
void CartesianSolver::interrupt()
{
    interrupting=true;
}


/************************************************************************/
void CartesianSolver::close()
{
    if (closed)
        return;

    closing=true;

    if (isRunning())
        stop();

    if (inPort!=NULL)
    {
        inPort->interrupt();
        inPort->close();
        delete inPort;
        inPort=NULL;
    }

    if (outPort!=NULL)
    {
        outPort->interrupt();
        outPort->close();
        delete outPort;
        outPort=NULL;
    }

    delete slv;
    delete clb;
    slv=NULL;
    clb=NULL;

    for (size_t i=0; i<drv.size(); i++)
    {
        delete drv[i];
        drv[i]=NULL;
    }

    for (size_t i=0; i<rmp.size(); i++)
    {
        delete[] rmp[i];
        rmp[i]=NULL;
    }

    drv.clear();
    lim.clear();
    enc.clear();
    jnt.clear();
    rmp.clear();

    if (prt!=NULL)
    {
        delete prt->lmb;
        delete prt->cns;
        delete prt;
        prt=NULL;
    }

    printf("%s closed\n",slvName.c_str());
    closed=true;
}


/************************************************************************/
bool CartesianSolver::threadInit()
{
    if (!configured)
        printf("Error: %s not configured!\n",slvName.c_str());
    else
        printf("Starting %s at %d ms\n",slvName.c_str(),period);

    return configured;
}


/************************************************************************/
void CartesianSolver::afterStart(bool s)
{
    printf("%s %s\n",slvName.c_str(),s?"started successfully":"did not start!");
}


/************************************************************************/
void CartesianSolver::suspend()
{
    if (isRunning())
    {
        printf("%s suspended\n",slvName.c_str());
        RateThread::suspend();
    }
    else
        printf("%s is already suspended\n",slvName.c_str());
}


/************************************************************************/
void CartesianSolver::resume()
{
    if (isSuspended())
    {
        printf("%s resumed\n",slvName.c_str());
        initPos();
        RateThread::resume();
    }
    else
        printf("%s is already running\n",slvName.c_str());
}


/************************************************************************/
void CartesianSolver::run()
{
    lock();

    // init conditions
    bool doSolve=false;

    // handle changeDOF()
    changeDOF(inPort->get_dof());    

    // wake up sleeping threads
    postDOFHandling();

    // get the current configuration
    getFeedback();

    // acquire uncontrolled joints configuration
    Vector unctrlJoints;
    if (!fullDOF)
    {
        latchUncontrolledJoints(unctrlJoints);

        // upon setting the mode to continuous,
        // latch the old state so that we skip any
        // adjustment
        if (inPort->get_contMode() && !contModeOld)
            unctrlJointsOld=unctrlJoints;

        // detect movements of uncontrolled joints
        double distExtMoves=CTRL_RAD2DEG*norm(unctrlJoints-unctrlJointsOld);        

        // run the solver if movements of uncontrolled joints
        // are detected and we are in continuous mode
        doSolve|=inPort->get_contMode() && (distExtMoves>CARTSLV_UNCTRLEDJNTS_THRES);
        if (doSolve && verbosity)
            printf("%s: detected movements on uncontrolled joints (norm=%g>%g deg)\n",
                   slvName.c_str(),distExtMoves,CARTSLV_UNCTRLEDJNTS_THRES);
    }

    // run the solver if any input is received
    if (inPort->isNewDataEvent())
    {    
        // update solver condition
        doSolve=true;        
    }

    // solver part
    if (doSolve)
    {
        // point to the desired pose
        Vector xd=inPort->get_xd();
        if (inPort->get_tokenPtr()) // latch token
        {
            token=*inPort->get_tokenPtr();
            pToken=&token;
        }
        else
            pToken=NULL;

        // set things for the 3rd task
        for (unsigned int i=0; i<prt->chn->getDOF(); i++)
            if (idx_3rdTask[i]!=0.0)
                qd_3rdTask[i]=(*prt->chn)(i).getAng();

        // update optimizer's options
        slv->set_ctrlPose(ctrlPose=inPort->get_pose());

        // call the solver to converge
        double t0=Time::now();
        Vector q=solve(xd);
        double t1=Time::now();

        // q is the estimation of the real qd,
        // so that x is the actual achieved pose
        Vector x=prt->chn->EndEffPose(q);
        
        // change to degrees
        q=CTRL_RAD2DEG*q;

        // send data
        send(xd,x,q,pToken);

        // dump on screen
        if (verbosity)
            printInfo("go",xd,x,q,t1-t0);

        // save the values of uncontrolled joints
        if (!fullDOF)
            unctrlJointsOld=unctrlJoints; 
    }

    contModeOld=inPort->get_contMode();

    unlock();
}


/************************************************************************/
void CartesianSolver::threadRelease()
{    
    printf("Stopping %s ...\n",slvName.c_str());
}


/************************************************************************/
CartesianSolver::~CartesianSolver()
{
    close();

    rpcPort->interrupt();
    rpcPort->close();
    delete rpcPort;
    delete cmdProcessor;
}


/************************************************************************/
PartDescriptor *iCubArmCartesianSolver::getPartDesc(Searchable &options)
{
    type="right";
    string part_type=type;
    if (options.check("type"))
    {
        type=options.find("type").asString().c_str();
        part_type=type.substr(0,type.find("_"));
        if ((part_type!="left") && (part_type!="right"))
            type=part_type="right";
    }

    string robot=options.check("robot",Value("icub")).asString().c_str();
    Property optTorso("(device remote_controlboard)");
    Property optArm("(device remote_controlboard)");

    string partTorso  ="torso";
    string remoteTorso="/"+robot+"/"+partTorso;
    string localTorso ="/"+slvName+"/"+partTorso;
    optTorso.put("remote",remoteTorso.c_str());
    optTorso.put("local",localTorso.c_str());
    optTorso.put("robot",robot.c_str());
    optTorso.put("part",partTorso.c_str());

    string partArm  =part_type=="left"?"left_arm":"right_arm";
    string remoteArm="/"+robot+"/"+partArm;
    string localArm ="/"+slvName+"/"+partArm;
    optArm.put("remote",remoteArm.c_str());
    optArm.put("local",localArm.c_str());
    optArm.put("robot",robot.c_str());
    optArm.put("part",partArm.c_str());

    PartDescriptor *p=new PartDescriptor;
    p->lmb=new iCubArm(type);
    p->chn=p->lmb->asChain();
    p->cns=new iCubShoulderConstr(*static_cast<iCubArm*>(p->lmb));
    p->prp.push_back(optTorso);
    p->prp.push_back(optArm);
    p->rvs.push_back(true);     // torso
    p->rvs.push_back(false);    // arm
    p->num=2;                   // number of parts

    return p;
}


/************************************************************************/
bool iCubArmCartesianSolver::open(Searchable &options)
{
    // call father's open() method
    if (CartesianSolver::open(options))
    {
        // Identify the elbow xyz position to be used as 2nd task
        slv->specify2ndTaskEndEff(6);

        // try to keep elbow as low as possible
        xd_2ndTask[2]=-1.0;
        w_2ndTask[2]=1.0;
    }

    return configured;
}


/************************************************************************/
bool iCubArmCartesianSolver::decodeDOF(const Vector &_dof)
{
    // latch current status
    Vector newDOF=dof;

    // update desired status
    size_t len=std::min((size_t)prt->chn->getN(),_dof.length());
    for (size_t i=0; i<len; i++)
        newDOF[i]=_dof[i];

    // check whether shoulder's axes are treated properly
    if (!(((newDOF[3]!=0.0) && (newDOF[4]!=0)   && (newDOF[5]!=0)) || 
          ((newDOF[3]==0.0) && (newDOF[4]==0.0) && (newDOF[5]==0.0))))
    {        
        // restore previous condition:
        // they all shall be on or off
        newDOF[3]=newDOF[4]=newDOF[5]=dof[3];

        printf("%s: attempt to set one shoulder's joint differently from others\n",slvName.c_str());
    }

    return CartesianSolver::decodeDOF(newDOF);
}


/************************************************************************/
PartDescriptor *iCubLegCartesianSolver::getPartDesc(Searchable &options)
{
    type="right";
    string part_type=type;
    if (options.check("type"))
    {
        type=options.find("type").asString().c_str();
        part_type=type.substr(0,type.find("_"));
        if ((part_type!="left") && (part_type!="right"))
            type=part_type="right";
    }

    string robot=options.check("robot",Value("icub")).asString().c_str();
    Property optLeg("(device remote_controlboard)");

    string partLeg  =part_type=="left"?"left_leg":"right_leg";
    string remoteLeg="/"+robot+"/"+partLeg;
    string localLeg ="/"+slvName+"/"+partLeg;

    optLeg.put("remote",remoteLeg.c_str());
    optLeg.put("local",localLeg.c_str());
    optLeg.put("robot",robot.c_str());
    optLeg.put("part",partLeg.c_str());

    PartDescriptor *p=new PartDescriptor;
    p->lmb=new iCubLeg(type);
    p->chn=p->lmb->asChain();
    p->cns=NULL;
    p->prp.push_back(optLeg);   
    p->rvs.push_back(false);
    p->num=1;

    return p;
}



