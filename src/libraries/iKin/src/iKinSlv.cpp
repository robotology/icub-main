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

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

#include <stdio.h>

#include <iCub/iKin/iKinVocabs.h>
#include <iCub/iKin/iKinSlv.h>

#define CARTSLV_SHOULDER_MAXABDUCTION       (100.0*CTRL_DEG2RAD)
#define CARTSLV_DEFAULT_PER                 20      // [ms]
#define CARTSLV_DEFAULT_TOL                 1e-3
#define CARTSLV_DEFAULT_TMO                 1.0     // [s]
#define CARTSLV_WEIGHT_2ND_TASK             0.01
#define CARTSLV_WEIGHT_3RD_TASK             0.01
#define CARTSLV_UNCTRLEDJNTS_THRES          0.1     // [deg]

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


namespace iCub
{

namespace iKin
{

class iCubShoulderConstr : public iKinLinIneqConstr
{
protected:    
    double shou_m, shou_n;
    double elb_m,  elb_n;

    iKinChain *chain;

    virtual void clone(const iKinLinIneqConstr *obj)
    {
        iKinLinIneqConstr::clone(obj);

        const iCubShoulderConstr *ptr=static_cast<const iCubShoulderConstr*>(obj);

        shou_m=ptr->shou_m;
        shou_n=ptr->shou_n;
        elb_m=ptr->elb_m;
        elb_n=ptr->elb_n;
        
        chain=ptr->chain;
    }

    void appendMatrixRow(Matrix &dest, const Vector &row)
    {
        Matrix tmp;

        // if dest is already filled with something
        if (dest.rows())
        {   
            // exit if lengths do not match     
            if (row.length()!=dest.cols())
                return;

            tmp.resize(dest.rows()+1,dest.cols());

            // copy the content of dest in temp
            for (int i=0; i<dest.rows(); i++)
                for (int j=0; j<dest.cols(); j++)
                    tmp(i,j)=dest(i,j);

            // reassign dest
            dest=tmp;
        }
        else
            dest.resize(1,row.length());

        // append the last row
        for (int i=0; i<dest.cols(); i++)
            dest(dest.rows()-1,i)=row[i];
    }

    void appendVectorValue(Vector &dest, double val)
    {
        Vector tmp(dest.length()+1);

        for (int i=0; i<dest.length(); i++)
            tmp[i]=dest[i];

        dest=tmp;
        dest[dest.length()-1]=val;
    }

public:
    iCubShoulderConstr(iKinChain *_chain) : iKinLinIneqConstr()
    {      
        chain=_chain;

        double joint1_0, joint1_1;
        double joint2_0, joint2_1;
        joint1_0= 28.0*CTRL_DEG2RAD;
        joint1_1= 23.0*CTRL_DEG2RAD;
        joint2_0=-37.0*CTRL_DEG2RAD;
        joint2_1= 80.0*CTRL_DEG2RAD;
        shou_m=(joint1_1-joint1_0)/(joint2_1-joint2_0);
        shou_n=joint1_0-shou_m*joint2_0;

        double joint3_0, joint3_1;
        double joint4_0, joint4_1;
        joint3_0= 85.0*CTRL_DEG2RAD;
        joint3_1=105.0*CTRL_DEG2RAD;
        joint4_0= 90.0*CTRL_DEG2RAD;
        joint4_1= 40.0*CTRL_DEG2RAD;
        elb_m=(joint4_1-joint4_0)/(joint3_1-joint3_0);
        elb_n=joint4_0-elb_m*joint3_0;

        update(NULL);
    }

    virtual void update(void*)
    {
        // optimization won't use LinIneqConstr by default
        setActive(false);

        Matrix _C;
        Vector _lB;
        Vector _uB;

        Vector row(chain->getDOF());

        // if shoulder's axes are controlled, constraint them
        if (!(*chain)[3].isBlocked() && !(*chain)[3+1].isBlocked() &&
            !(*chain)[3+2].isBlocked())
        {
            // compute offset to shoulder's axes
            // given the blocked/release status of
            // previous link
            unsigned int offs=0;
            for (unsigned int i=0; i<3; i++)
                if (!(*chain)[i].isBlocked())
                    offs++;

            // constraints on the cables length
            row.zero();
            row[offs]=1.71; row[offs+1]=-1.71;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,-347.00*CTRL_DEG2RAD);
            appendVectorValue(_uB,upperBoundInf);

            row.zero();
            row[offs]=1.71; row[offs+1]=-1.71; row[offs+2]=-1.71;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,-366.57*CTRL_DEG2RAD);
            appendVectorValue(_uB,112.42*CTRL_DEG2RAD);

            row.zero();
            row[offs+1]=1.0; row[offs+2]=1.0;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,-66.600*CTRL_DEG2RAD);
            appendVectorValue(_uB,213.30*CTRL_DEG2RAD);

            // constraints to prevent arm from touching torso
            row.zero();
            row[offs+1]=1.0; row[offs+2]=-shou_m;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,shou_n);
            appendVectorValue(_uB,upperBoundInf);

            // constraints to limit shoulder abduction
            row.zero();
            row[offs+1]=1.0;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,lowerBoundInf);
            appendVectorValue(_uB,CARTSLV_SHOULDER_MAXABDUCTION);

            // optimization will use LinIneqConstr
            getC()=_C;
            getlB()=_lB;
            getuB()=_uB;
            setActive(true);
        }

        // if elbow and pronosupination axes are controlled, constraint them
        if (!(*chain)[3+3].isBlocked() && !(*chain)[3+3+1].isBlocked())
        {
            // compute offset to elbow's axis
            // given the blocked/release status of
            // previous link
            unsigned int offs=0;
            for (unsigned int i=0; i<(3+3); i++)
                if (!(*chain)[i].isBlocked())
                    offs++;

            // constraints to prevent the forearm from hitting the arm
            row.zero();
            row[offs]=-elb_m; row[offs+1]=1.0;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,lowerBoundInf);
            appendVectorValue(_uB,elb_n);

            row.zero();
            row[offs]=elb_m; row[offs+1]=1.0;
            appendMatrixRow(_C,row);
            appendVectorValue(_lB,-elb_n);
            appendVectorValue(_uB,upperBoundInf);

            // optimization will use LinIneqConstr
            getC()=_C;
            getlB()=_lB;
            getuB()=_uB;
            setActive(true);
        }
    }
};

}

}


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
    mutex.wait();
    dof=_dof;
    mutex.post();
}


/************************************************************************/
Vector InputPort::get_dof()
{
    mutex.wait();
    Vector _dof=dof;
    mutex.post();
    return _dof;
}


/************************************************************************/
Vector InputPort::get_xd()
{
    mutex.wait();
    Vector _xd=xd;
    mutex.post();
    return _xd;
}


/************************************************************************/
void InputPort::reset_xd(const Vector &_xd)
{
    mutex.wait();
    xd=_xd;
    mutex.post();
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
        int len=b->size();
        int l=maxLen<len ? maxLen : len;

        mutex.wait();
        for (int i=0; i<l; i++)
            xd[i]=b->get(i).asDouble();
        mutex.post();

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
        int len=b->size();        
        
        slv->lock();

        mutex.wait();
        dof.resize(len);
        for (int i=0; i<len; i++)
            dof[i]=b->get(i).asInt();
        mutex.post();

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
        contMode=true;
        return true;
    }
    else if (newMode==IKINSLV_VOCAB_VAL_MODE_SINGLE)
    {
        contMode=false;
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
            fprintf(stdout,"%s: got incomplete %s command\n",slv->slvName.c_str(),
                    Vocab::decode(IKINSLV_VOCAB_OPT_MODE).c_str());

    if (dofOptIn)
        if (!handleDOF(b.find(Vocab::decode(IKINSLV_VOCAB_OPT_DOF)).asList()))
            fprintf(stdout,"%s: expected %s data\n",slv->slvName.c_str(),
                    Vocab::decode(IKINSLV_VOCAB_OPT_DOF).c_str());

    if (poseOptIn)
        if (!handlePose(b.find(Vocab::decode(IKINSLV_VOCAB_OPT_POSE)).asVocab()))
            fprintf(stdout,"%s: got incomplete %s command\n",slv->slvName.c_str(),
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
            fprintf(stdout,"%s: expected %s data\n",slv->slvName.c_str(),
                    Vocab::decode(IKINSLV_VOCAB_OPT_XD).c_str());
    }
    else
        fprintf(stdout,"%s: missing %s data; it shall be present\n",
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
    closed=false;
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
bool CartesianSolver::waitPart(const Property &partOpt)
{
    string robotName=const_cast<Property&>(partOpt).find("robot").asString().c_str();
    string partName=const_cast<Property&>(partOpt).find("part").asString().c_str();
    string portName="/"+robotName+"/"+partName+"/state:o";
    double t0=Time::now();

    while ((Time::now()-t0)<ping_robot_tmo)
    {   
        fprintf(stdout,"%s: Checking if %s port is active ... ",
                slvName.c_str(),portName.c_str());
    
        bool ok=Network::exists(portName.c_str(),true);    

        fprintf(stdout,"%s\n",ok?"ok":"not yet");

        if (ok)
            return true;
        else
        {
            double t1=Time::now();
            while ((Time::now()-t1)<CARTSLV_DEFAULT_TMO)
                Time::delay(0.1);
        }
    }

    return false;
}


/************************************************************************/
void CartesianSolver::alignJointsBounds()
{
    double min, max;
    int cnt=0;

    fprintf(stdout,"%s: aligning joints bounds ...\n",slvName.c_str());

    for (int i=0; i<prt->num; i++)
    {
        fprintf(stdout,"part #%d: %s\n",i,prt->prp[i].find("part").asString().c_str());

        for (int j=0; j<jnt[i]; j++)
        {               
            lim[i]->getLimits(rmp[i][j],&min,&max);

            fprintf(stdout,"joint #%d: [%g, %g] deg\n",cnt,min,max);
        
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
            
                if (min>=curMin && max<=curMax)
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
    if (unctrlJointsNum)
    {
        joints.resize(unctrlJointsNum);
        unsigned int j=0;
        
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
        bool ok;

        if (wait)
        {
            while (!enc[i]->getEncoders(fbTmp.data()))
                Time::delay(0.00025*getRate());     // wait for 1/4 of thread period

            ok=true;
        }
        else
            ok=enc[i]->getEncoders(fbTmp.data());

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
            fprintf(stdout,"%s: timeout detected on part %s!\n",
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
    mutex.wait();
}


/************************************************************************/
void CartesianSolver::unlock()
{
    mutex.post();
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
            case IKINSLV_VOCAB_CMD_HELP:
            {                    
                string ack("commands={");
                ack+=Vocab::decode(IKINSLV_VOCAB_CMD_HELP);        ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_CMD_SUSP);        ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_CMD_RUN);         ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_CMD_CFG);         ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_CMD_GET);         ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_CMD_SET);         ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_CMD_ASK);         ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_CMD_QUIT);        ack+="};";
                ack+="options={";            
                ack+=Vocab::decode(IKINSLV_VOCAB_OPT_MODE);        ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_OPT_POSE);        ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_OPT_DOF);         ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_OPT_LIM);         ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_OPT_VERB);        ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_OPT_TOKEN);       ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_OPT_REST_POS);    ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_OPT_REST_WEIGHTS);ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_OPT_XD);          ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_OPT_X);           ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_OPT_Q);           ack+="};";
                ack+="values={";
                ack+=Vocab::decode(IKINSLV_VOCAB_VAL_POSE_FULL);   ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_VAL_POSE_XYZ);    ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_VAL_MODE_TRACK);  ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_VAL_MODE_SINGLE); ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_VAL_ON);          ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_VAL_OFF);         ack+="};";
                reply.addString(ack.c_str());
                break;
            }
    
            case IKINSLV_VOCAB_CMD_SUSP:
            {                    
                fprintf(stdout,"%s suspended\n",slvName.c_str());
                suspend();
                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                break;
            }
    
            case IKINSLV_VOCAB_CMD_RUN:
            {   
                if (!isRunning())
                {
                    initPos();
                    start();
                }
                else if (isSuspended())
                {    
                    fprintf(stdout,"%s resumed\n",slvName.c_str());
                    initPos();
                    resume();
                }
                else
                    fprintf(stdout,"%s is already running\n",slvName.c_str());

                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                break;
            }

            case IKINSLV_VOCAB_CMD_QUIT:
            {                    
                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                close();
                break;
            }

            case IKINSLV_VOCAB_CMD_GET:
            {
                if (command.size()>1)
                    switch (command.get(1).asVocab())
                    {
                        case IKINSLV_VOCAB_OPT_POSE:
                        {
                            reply.addVocab(IKINSLV_VOCAB_REP_ACK);

                            if (ctrlPose==IKINCTRL_POSE_FULL)
                                reply.addVocab(IKINSLV_VOCAB_VAL_POSE_FULL);
                            else
                                reply.addVocab(IKINSLV_VOCAB_VAL_POSE_XYZ);

                            break;
                        }

                        case IKINSLV_VOCAB_OPT_MODE:
                        {
                            reply.addVocab(IKINSLV_VOCAB_REP_ACK);

                            if (inPort->get_contMode())
                                reply.addVocab(IKINSLV_VOCAB_VAL_MODE_TRACK);
                            else
                                reply.addVocab(IKINSLV_VOCAB_VAL_MODE_SINGLE);

                            break;
                        }

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

                        case IKINSLV_VOCAB_OPT_VERB:
                        {
                            reply.addVocab(IKINSLV_VOCAB_REP_ACK);

                            if (verbosity)
                                reply.addVocab(IKINSLV_VOCAB_VAL_ON);
                            else
                                reply.addVocab(IKINSLV_VOCAB_VAL_OFF);
    
                            break;
                        }

                        case IKINSLV_VOCAB_OPT_DOF:
                        {
                            reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            Bottle &dofPart=reply.addList();
                            fillDOFInfo(dofPart);
                            break;
                        }

                        case IKINSLV_VOCAB_OPT_REST_POS:
                        {
                            reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            handleJointsRestPosition(NULL,&reply);
                            break;
                        }

                        case IKINSLV_VOCAB_OPT_REST_WEIGHTS:
                        {
                            reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            handleJointsRestWeights(NULL,&reply);
                            break;
                        }

                        default:
                            reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                    }
                else
                    reply.addVocab(IKINSLV_VOCAB_REP_NACK);

                break;
            }

            case IKINSLV_VOCAB_CMD_SET:
            {
                if (command.size()>2)
                    switch (command.get(1).asVocab())
                    {
                        case IKINSLV_VOCAB_OPT_POSE:
                        {
                            if (inPort->handlePose(command.get(2).asVocab()))
                                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            else
                                reply.addVocab(IKINSLV_VOCAB_REP_NACK);

                            break;
                        }

                        case IKINSLV_VOCAB_OPT_MODE:
                        {
                            if (inPort->handleMode(command.get(2).asVocab()))
                            {
                                initPos();
                                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            }
                            else
                                reply.addVocab(IKINSLV_VOCAB_REP_NACK);

                            break;
                        }

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

                        default:
                            reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                    }
                else
                    reply.addVocab(IKINSLV_VOCAB_REP_NACK);

                break;
            }

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
                for (int i=0; i<xd.length(); i++)
                    xd[i]=b_xd->get(i).asDouble();

                // accounts for the starting DOF
                // if different from the actual one
                if (b_q!=NULL)
                {
                    int len=b_q->size()<(int)prt->chn->getDOF()?b_q->size():prt->chn->getDOF();
                    for (int i=0; i<len; i++)
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
                    printInfo(xd,x,q,t1-t0);

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

            case IKINSLV_VOCAB_CMD_CFG:
            {
                Property options(command.toString().c_str());
                fprintf(stdout,"Configuring with options: %s\n",options.toString().c_str());

                if (open(options))
                    reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                else
                    reply.addVocab(IKINSLV_VOCAB_REP_NACK);

                break;
            }

            default:
                reply.addVocab(IKINSLV_VOCAB_REP_NACK);
        }
    }
    else
        reply.addVocab(IKINSLV_VOCAB_REP_NACK);
}


/************************************************************************/
void CartesianSolver::send(const Vector &xd, const Vector &x, const Vector &q, double *tok)
{       
    Bottle &b=outPort->prepare();
    b.clear();

    addVectorOption(b,IKINSLV_VOCAB_OPT_XD,xd);
    addVectorOption(b,IKINSLV_VOCAB_OPT_X,x);
    addVectorOption(b,IKINSLV_VOCAB_OPT_Q,q);

    if (tok)
        addTokenOption(b,*tok);

    outPort->writeStrict();
}


/************************************************************************/
void CartesianSolver::printInfo(const Vector &xd, const Vector &x, const Vector &q,
                                const double t)
{
    // compute error
    Vector e=xd-x;
    
    fprintf(stdout,"\n");
    fprintf(stdout,"  Target rxPose   [m] = %s\n",const_cast<Vector&>(xd).toString().c_str());
    fprintf(stdout,"  Target txPose   [m] = %s\n",const_cast<Vector&>(x).toString().c_str());
    fprintf(stdout,"Target txJoints [deg] = %s\n",const_cast<Vector&>(q).toString().c_str());
    fprintf(stdout,"  norm(rxPose-txPose) = pos [m]: %g\n",getNorm(e,"pos"));
    if (ctrlPose==IKINCTRL_POSE_FULL)
    fprintf(stdout,"                        ang [*]: %g\n",getNorm(e,"ang"));
    fprintf(stdout,"    computed in   [s] = %g\n",t);
    fprintf(stdout,"\n");
}


/************************************************************************/
Vector &CartesianSolver::encodeDOF()
{
    dof.resize(prt->chn->getN());
    fullDOF=true;

    for (unsigned int i=0; i<prt->chn->getN(); i++)
        if (!(dof[i]=!(*prt->chn)[i].isBlocked()))
            fullDOF=false;

    return dof;
}


/************************************************************************/
bool CartesianSolver::decodeDOF(const Vector &_dof)
{
    int l1=prt->chn->getN();
    int l2=_dof.length();
    int len=l1<l2 ? l1 : l2;

    for (int i=0; i<len; i++)
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

    if (options)
    {            
        int sz=options->size();
        int len=restJntPos.length();
        int l=sz>len?len:sz;

        for (int i=0; i<l; i++)
        {
            double val=CTRL_DEG2RAD*options->get(i).asDouble();
            double min=(*prt->chn)[i].getMin();
            double max=(*prt->chn)[i].getMax();

            restJntPos[i]=val<min?min:(val>max?max:val);
        }

        ret=true;
    }

    if (reply)
    {
        Bottle &b=reply->addList();
        for (int i=0; i<restJntPos.length(); i++)
            b.addDouble(CTRL_RAD2DEG*restJntPos[i]);
    }

    return ret;
}


/************************************************************************/
bool CartesianSolver::handleJointsRestWeights(const Bottle *options, Bottle *reply)
{
    bool ret=false;

    if (options)
    {            
        int sz=options->size();
        int len=restWeights.length();
        int l=sz>len?len:sz;

        for (int i=0; i<l; i++)
        {
            double val=options->get(i).asInt();
            restWeights[i]=val<0.0?0.0:val;
        }

        ret=true;
    }

    if (reply)
    {
        Bottle &b=reply->addList();
        for (int i=0; i<restWeights.length(); i++)
            b.addDouble(restWeights[i]);
    }

    return ret;
}


/************************************************************************/
bool CartesianSolver::isNewDOF(const Vector &_dof)
{
    int l1=prt->chn->getN();
    int l2=_dof.length();
    int len=l1<l2 ? l1 : l2;

    for (int i=0; i<len; i++)
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
        fprintf(stdout,"%s already configured\n",slvName.c_str());
        return true;
    }

    prt=getPartDesc(options);
    if (prt==NULL)
    {
        fprintf(stdout,"Detected errors while processing parts description!\n");
        return false;
    }    
    
    if (options.check("ping_robot_tmo"))
        ping_robot_tmo=options.find("ping_robot_tmo").asDouble();

    // open drivers
    int remainingJoints=prt->chn->getN();
    for (int i=0; i<prt->num; i++)
    {
        fprintf(stdout,"Allocating device driver for %s ...\n",
                prt->prp[i].find("part").asString().c_str());

        if (ping_robot_tmo>0.0)
            waitPart(prt->prp[i]);

        PolyDriver *pDrv=new PolyDriver(prt->prp[i]);

        if (!pDrv->isValid())
        {
            fprintf(stdout,"Device driver not available!\n");

            delete pDrv;
            close();

            return false;
        }

        // create interfaces
        IControlLimits *pLim;
        IEncoders      *pEnc;
        int             joints;

        if (!pDrv->view(pLim) || !pDrv->view(pEnc))
        {    
            fprintf(stdout,"Problems acquiring interfaces!\n");
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
            rmpTmp[j]=prt->rvs[i] ? joints-j-1 : j;

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

        for (int i=0; i<_dof.length(); i++)
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
    if (prt->cns)
    {
        slv->attachLIC(*prt->cns);
        
        double lower_bound_inf, upper_bound_inf;        
        slv->getBoundsInf(lower_bound_inf,upper_bound_inf);
        slv->getLIC().getLowerBoundInf()=2.0*lower_bound_inf;
        slv->getLIC().getUpperBoundInf()=2.0*upper_bound_inf;
        slv->getLIC().update(NULL);
    }

    // define input port
    inPort=new InputPort(this);
    inPort->useCallback();
    
    // init input port data
    inPort->set_dof(dof);
    inPort->get_pose()=ctrlPose;
    inPort->get_contMode()=mode;

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
        if (prt->cns)
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
    Vector dummy(1);

    // call the solver and start the convergence from the current point
    return slv->solve(prt->chn->getAng(),xd,
                      0.0,dummy,dummy,
                      CARTSLV_WEIGHT_3RD_TASK,qd_3rdTask,w_3rdTask,
                      NULL,NULL,clb);
}


/************************************************************************/
void CartesianSolver::close()
{
    if (closed)
        return;

    if (isRunning())
        stop();

    if (inPort!=NULL)
    {
        inPort->interrupt();
        inPort->close();
        delete inPort;
    }

    if (outPort!=NULL)
    {
        outPort->interrupt();
        outPort->close();
        delete outPort;
    }

    if (slv!=NULL)
        delete slv;

    if (clb!=NULL)
        delete clb;

    for (unsigned int i=0; i<drv.size(); i++)
        delete drv[i];

    for (unsigned int i=0; i<rmp.size(); i++)
        delete[] rmp[i];

    drv.clear();
    lim.clear();
    enc.clear();
    jnt.clear();
    rmp.clear();

    if (prt!=NULL)
    {
        delete prt->lmb;

        if (prt->cns!=NULL)
            delete prt->cns;

        delete prt;
    }

    fprintf(stdout,"%s closed\n",slvName.c_str());

    closed=true;
}


/************************************************************************/
bool CartesianSolver::threadInit()
{
    if (!configured)
        fprintf(stdout,"Error: %s not configured!\n",slvName.c_str());
    else
        fprintf(stdout,"Starting %s at %d ms\n",slvName.c_str(),period);

    return configured;
}


/************************************************************************/
void CartesianSolver::afterStart(bool s)
{
    fprintf(stdout,"%s %s\n",slvName.c_str(),
            s?"started successfully":"did not start!");
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
    if (!fullDOF)
    {
        Vector unctrlJoints;
        latchUncontrolledJoints(unctrlJoints);
    
        // detect movements of uncontrolled joints
        double distExtMoves=norm(CTRL_RAD2DEG*(unctrlJoints-unctrlJointsOld));
        unctrlJointsOld=unctrlJoints;
    
        // run the solver if movements of uncontrolled joints
        // are detected and mode==continuous
        doSolve|=inPort->get_contMode() && (distExtMoves>CARTSLV_UNCTRLEDJNTS_THRES);
        if (doSolve && verbosity)
            fprintf(stdout,"%s: detected movements on uncontrolled joints (norm=%g>%g deg)\n",
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
        if (inPort->get_tokenPtr()) // latch the token
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
            printInfo(xd,x,q,t1-t0);
    }

    unlock();
}


/************************************************************************/
void CartesianSolver::threadRelease()
{    
    fprintf(stdout,"Stopping %s ...\n",slvName.c_str());
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
    if (options.check("type"))
    {    
        type=options.find("type").asString().c_str();
        if (type!="left" && type!="right")
            type="right";
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

    string partArm  =type=="left" ? "left_arm" : "right_arm";
    string remoteArm="/"+robot+"/"+partArm;
    string localArm ="/"+slvName+"/"+partArm;
    optArm.put("remote",remoteArm.c_str());
    optArm.put("local",localArm.c_str());
    optArm.put("robot",robot.c_str());
    optArm.put("part",partArm.c_str());

    PartDescriptor *p=new PartDescriptor;
    p->lmb=new iCubArm(type);
    p->chn=p->lmb->asChain();
    p->cns=new iCubShoulderConstr(p->chn);
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
    }

    return configured;
}


/************************************************************************/
bool iCubArmCartesianSolver::decodeDOF(const Vector &_dof)
{
    // latch current status
    Vector newDOF=dof;

    int l1=prt->chn->getN();
    int l2=_dof.length();
    int len=l1<l2 ? l1 : l2;

    // update desired status
    for (int i=0; i<len; i++)
        newDOF[i]=_dof[i];

    // check whether shoulder's axes are treated properly
    if (!(newDOF[3]&&newDOF[4]&&newDOF[5] || !newDOF[3]&&!newDOF[4]&&!newDOF[5]))
    {        
        // restore previous condition:
        // they all shall be on or off
        newDOF[3]=newDOF[4]=newDOF[5]=dof[3];

        fprintf(stdout,"%s: attempt to set one shoulder's joint differently from others\n",
                slvName.c_str());
    }

    return CartesianSolver::decodeDOF(newDOF);
}


/************************************************************************/
Vector iCubArmCartesianSolver::solve(Vector &xd)
{
    // try to keep elbow height as low as possible
    Vector w_2nd(3); w_2nd=0.0; w_2nd[2]=1.0;
    Vector xdElb(3); xdElb=0.0; xdElb[2]=-1.0;

    // call the solver and start the convergence from the current point
    return slv->solve(prt->chn->getAng(),xd,
                      CARTSLV_WEIGHT_2ND_TASK,xdElb,w_2nd,
                      CARTSLV_WEIGHT_3RD_TASK,qd_3rdTask,w_3rdTask,
                      NULL,NULL,clb);
}


/************************************************************************/
PartDescriptor *iCubLegCartesianSolver::getPartDesc(Searchable &options)
{
    type="right";
    if (options.check("type"))
    {    
        type=options.find("type").asString().c_str();
        if (type!="left" && type!="right")
            type="right";
    }

    string robot=options.check("robot",Value("icub")).asString().c_str();
    Property optLeg("(device remote_controlboard)");

    string partLeg  =type=="left" ? "left_arm" : "right_arm";
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



