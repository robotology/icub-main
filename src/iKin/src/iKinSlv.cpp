
#include <ace/Auto_Event.h>
#include <yarp/os/Time.h>

#include <stdio.h>

#include <iCub/iKinVocabs.h>
#include <iCub/iKinSlv.h>

#define RES_EVENT(x)            (static_cast<ACE_Auto_Event*>(x))

#define SHOULDER_MAXABDUCTION   (100.0*(M_PI/180.0))
#define CARTSLV_DEFAULT_PER     20

namespace iKin
{
    class iCubShoulderConstr;
}

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace ctrl;
using namespace iKin;


class iKin::iCubShoulderConstr : public iKinLinIneqConstr
{
protected:    
    double joint1_0, joint1_1;
    double joint2_0, joint2_1;
    double m, n;

    unsigned int numAxes2Shou;

    iKinChain *chain;

    virtual void _allocate(const iKinLinIneqConstr *obj)
    {
        iKinLinIneqConstr::_allocate(obj);

        const iCubShoulderConstr *ptr=static_cast<const iCubShoulderConstr*>(obj);

        joint1_0=ptr->joint1_0;
        joint1_1=ptr->joint1_1;
        joint2_0=ptr->joint2_0;
        joint2_1=ptr->joint2_1;

        m=ptr->m;
        n=ptr->n;

        numAxes2Shou=ptr->numAxes2Shou;
        chain=ptr->chain;
    }

public:
    iCubShoulderConstr(iKinChain *_chain) : iKinLinIneqConstr()
    {      
        chain=_chain;

        // number of axes to reach shoulder's ones
        // from root reference  
        numAxes2Shou=3;

        joint1_0= 10.0*(M_PI/180.0);
        joint1_1= 15.0*(M_PI/180.0);
        joint2_0=-33.0*(M_PI/180.0);
        joint2_1= 60.0*(M_PI/180.0);
        m=(joint1_1-joint1_0)/(joint2_1-joint2_0);
        n=joint1_0-m*joint2_0;

        update(NULL);
    }

    virtual void update(void*)
    {
        // if any of shoulder's axes is blocked, skip
        if ((*chain)[numAxes2Shou].isBlocked()   ||
            (*chain)[numAxes2Shou+1].isBlocked() ||
            (*chain)[numAxes2Shou+2].isBlocked())
            setActive(false);   // optimization won't use LinIneqConstr
        else
        {
            // compute offset to shoulder's axes
            // given the blocked/release status of
            // previous link
            unsigned int offs=0;
            for (unsigned int i=0; i<numAxes2Shou; i++)
                if (!(*chain)[i].isBlocked())
                    offs++;

            // linear inequalities matrix
            C.resize(5,chain->getDOF()); C.zero();
            // constraints on the cables length
            C(0,offs)=1.71; C(0,offs+1)=-1.71;
            C(1,offs)=1.71; C(1,offs+1)=-1.71; C(1,offs+2)=-1.71;
                            C(2,offs+1)=1.0;   C(2,offs+2)=1.0;
            // constraints to prevent arm from touching torso
                            C(3,offs+1)=1.0;   C(3,offs+2)=-m;
            // constraints to limit shoulder abduction
                            C(4,offs+1)=1.0;
    
            // lower and upper bounds
            lB.resize(5); uB.resize(5);
            lB[0]=-347.00*(M_PI/180.0); uB[0]=upperBoundInf;
            lB[1]=-366.57*(M_PI/180.0); uB[1]=112.42*(M_PI/180.0);
            lB[2]=-66.600*(M_PI/180.0); uB[2]=213.30*(M_PI/180.0);
            lB[3]=n;                    uB[3]=upperBoundInf;
            lB[4]=lowerBoundInf;        uB[4]=SHOULDER_MAXABDUCTION;

            // optimization will use LinIneqConstr
            setActive(true);
        }
    }
};


/************************************************************************/
bool RpcProcessor::read(ConnectionReader &connection)
{
    Bottle cmd, reply;

    if (!cmd.read(connection))
        return false;

    if (slv->respond(cmd,reply))
        if (ConnectionWriter *writer=connection.getWriter())
            reply.write(*writer);

    return true;
}


/************************************************************************/
InputPort::InputPort(CartesianSolver *_slv)
{
    slv=_slv;

    maxLen=7;
    xd.resize(maxLen,0.0);
    xdOld=xd;

    dof.resize(1,0);

    pose=IKINCTRL_POSE_FULL;
    contMode=false;
    isNew=false;
    dofChanged=false;
    token=0.0;
    pToken=NULL;
}


/************************************************************************/
void InputPort::reset_xd(const Vector &_xd)
{
    xdOld=xd=_xd;
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
    if (b)
    {
        int len=b->size();
        int l=maxLen<len ? maxLen : len;

        for (int i=0; i<l; i++)
            xd[i]=b->get(i).asDouble();

        if (!(xd==xdOld) || dofChanged)
        {
            isNew=true;
            xdOld=xd;

            dofChanged=false;
        }
        else
            slv->send(xd,pToken);

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool InputPort::handleDOF(Bottle *b)
{
    if (b)
    {
        int len=b->size();        
        
        slv->lock();

        dof.resize(len);

        for (int i=0; i<len; i++)
            dof[i]=b->get(i).asInt();

        dofChanged=true;

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
    if (CartesianHelper::getTokenOption(b,&token))
        pToken=&token;
    else
        pToken=NULL;

    if (b.check(Vocab::decode(IKINSLV_VOCAB_OPT_XD)))
        if (!handleTarget(b.find(Vocab::decode(IKINSLV_VOCAB_OPT_XD)).asList()))
            fprintf(stdout,"expected %s data\n",Vocab::decode(IKINSLV_VOCAB_OPT_XD).c_str());

    if (b.check(Vocab::decode(IKINSLV_VOCAB_OPT_DOF)))
        if (!handleDOF(b.find(Vocab::decode(IKINSLV_VOCAB_OPT_DOF)).asList()))
            fprintf(stdout,"expected %s data\n",Vocab::decode(IKINSLV_VOCAB_OPT_DOF).c_str());

    if (b.check(Vocab::decode(IKINSLV_VOCAB_OPT_POSE)))
        if (!handlePose(b.find(Vocab::decode(IKINSLV_VOCAB_OPT_POSE)).asVocab()))
            fprintf(stdout,"got incomplete %s command\n",Vocab::decode(IKINSLV_VOCAB_OPT_POSE).c_str());

    if (b.check(Vocab::decode(IKINSLV_VOCAB_OPT_MODE)))
        if (!handleMode(b.find(Vocab::decode(IKINSLV_VOCAB_OPT_MODE)).asVocab()))
            fprintf(stdout,"got incomplete %s command\n",Vocab::decode(IKINSLV_VOCAB_OPT_MODE).c_str());
}


/************************************************************************/
void SolverCallback::exec(Vector xd, Vector q)
{
    slv->send(xd,slv->prt->chn->EndEffPose(),q,slv->pToken);
}


/************************************************************************/
CartesianSolver::CartesianSolver(const string &_slvName) : RateThread(CARTSLV_DEFAULT_PER)
{          
    // initialization
    slvName=_slvName;
    configured=false;
    closed=false;
    verbosity=false;
    maxPartJoints=0;
    unctrlJointsNum=0;

    prt=NULL;
    slv=NULL;
    clb=NULL;
    inPort=NULL;
    outPort=NULL;

    // dof event
    dofEvent=new ACE_Auto_Event;

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
void CartesianSolver::alignJointsBounds()
{
    double min, max;
    int cnt=0;

    fprintf(stdout,"Aligning joints bounds ...\n");

    for (int i=0; i<prt->num; i++)
    {
        fprintf(stdout,"part #%d: %s\n",i,prt->prp[i].find("part").asString().c_str());

        for (int j=0; j<jnt[i]; j++)
        {               
            lim[i]->getLimits(rmp[i][j],&min,&max);

            fprintf(stdout,"joint #%d: [%g, %g] deg\n",cnt,min,max);
        
            (*prt->chn)[cnt].setMin((M_PI/180.0)*min);
            (*prt->chn)[cnt].setMax((M_PI/180.0)*max);
        
            cnt++;
        }
    }
}


/************************************************************************/
bool CartesianSolver::setLimits(int axis, double min, double max)
{
    int cnt=0;

    for (int i=0; i<prt->num; i++)
        for (int j=0; j<jnt[i]; j++)
            if (cnt++==axis)
            {
                double curMin, curMax;

                lim[i]->getLimits(rmp[i][j],&curMin,&curMax);

                if (min>=curMin && max<=curMax)
                {
                    (*prt->chn)[axis].setMin((M_PI/180.0)*min);
                    (*prt->chn)[axis].setMax((M_PI/180.0)*max);

                    return true;
                }
                else
                    return false;
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
void CartesianSolver::getFeedback()
{
    Vector fbTmp(maxPartJoints);
    int chainCnt=0;

    for (int i=0; i<prt->num; i++)
        if (enc[i]->getEncoders(fbTmp.data()))
            for (int j=0; j<jnt[i]; j++)
            {
                double tmp=(M_PI/180.0)*fbTmp[rmp[i][j]];

                if ((*prt->chn)[chainCnt].isBlocked())
                    prt->chn->setBlockingValue(chainCnt,tmp);
                else
                    prt->chn->setAng(chainCnt,tmp);

                chainCnt++;
            }
        else
            chainCnt+=jnt[i];
}


/************************************************************************/
void CartesianSolver::initPos()
{
    getFeedback();    
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
    RES_EVENT(dofEvent)->reset();
    RES_EVENT(dofEvent)->wait();
}


/************************************************************************/
void CartesianSolver::postDOFHandling()
{
    RES_EVENT(dofEvent)->signal();
}


/************************************************************************/
void CartesianSolver::fillDOFInfo(Bottle &reply)
{
    for (unsigned int i=0; i<prt->chn->getN(); i++)
        reply.addInt(!(*prt->chn)[i].isBlocked());
}


/************************************************************************/
bool CartesianSolver::respond(const Bottle &command, Bottle &reply)
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
                ack+=Vocab::decode(IKINSLV_VOCAB_CMD_HELP);ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_CMD_SUSP);ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_CMD_RUN); ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_CMD_CFG); ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_CMD_GET); ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_CMD_SET); ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_CMD_QUIT);ack+="};";
                ack+="options={";            
                ack+=Vocab::decode(IKINSLV_VOCAB_OPT_MODE);ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_OPT_POSE);ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_OPT_DOF); ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_OPT_LIM); ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_OPT_VERB);ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_OPT_XD);  ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_OPT_X);   ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_OPT_Q);   ack+="};";
                ack+="values={";
                ack+=Vocab::decode(IKINSLV_VOCAB_VAL_POSE_FULL);  ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_VAL_POSE_XYZ);   ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_VAL_MODE_TRACK); ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_VAL_MODE_SINGLE);ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_VAL_ON);         ack+=", ";
                ack+=Vocab::decode(IKINSLV_VOCAB_VAL_OFF);        ack+="};";
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
                                    reply.addDouble((180.0/M_PI)*(*prt->chn)[axis].getMin());
                                    reply.addDouble((180.0/M_PI)*(*prt->chn)[axis].getMax());
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
                                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
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

                        default:
                            reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                    }
                else
                    reply.addVocab(IKINSLV_VOCAB_REP_NACK);

                break;
            }

            case IKINSLV_VOCAB_CMD_CFG:
            {
                Property options;

                if (command.size()>1)
                {    
                    options.fromString(command.get(1).asList()->toString());
                    fprintf(stdout,"Configuring with options: %s\n",options.toString().c_str());
                }
                else
                    fprintf(stdout,"Configuring with default options\n");

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

    return true;
}


/************************************************************************/
void CartesianSolver::send(const Vector &xd, const Vector &x, const Vector &q, double *tok)
{       
    Bottle &b=outPort->prepare();
    b.clear();
    solutionBottle.clear();

    addVectorOption(b,IKINSLV_VOCAB_OPT_XD,xd);
    addVectorOption(solutionBottle,IKINSLV_VOCAB_OPT_X,x);
    addVectorOption(solutionBottle,IKINSLV_VOCAB_OPT_Q,q);
    b.append(solutionBottle);

    if (tok)
        addTokenOption(b,*tok);

    outPort->write();
}


/************************************************************************/
void CartesianSolver::send(const Vector &xd, double *tok)
{
    Bottle &b=outPort->prepare();
    b.clear();

    addVectorOption(b,IKINSLV_VOCAB_OPT_XD,xd);
    b.append(solutionBottle);

    if (tok)
        addTokenOption(b,*tok);

    outPort->write();
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
        if (_dof[i]>1.0)
            continue;
        else if (_dof[i])
            prt->chn->releaseLink(i);
        else
            prt->chn->blockLink(i);

    return true;
}


/************************************************************************/
bool CartesianSolver::isNewDOF(const Vector &_dof)
{
    int l1=prt->chn->getN();
    int l2=_dof.length();
    int len=l1<l2 ? l1 : l2;

    for (int i=0; i<len; i++)
        if (_dof[i]>1.0)
            continue;
        else if (_dof[i]!=dof[i])
            return true;

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

    int remainingJoints=prt->chn->getN();

    // open drivers
    for (int i=0; i<prt->num; i++)
    {
        fprintf(stdout,"Allocating device driver for %s ...\n",
                prt->prp[i].find("part").asString().c_str());

        PolyDriver *pDrv=new PolyDriver(prt->prp[i]);

        if (!pDrv->isValid())
        {
            fprintf(stdout,"Device driver not available!\n");
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
    period=CARTSLV_DEFAULT_PER;
    if (options.check("period"))
        setRate(period=options.find("period").asInt());

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

    double tol=1e-3;
    if (options.check("tol"))
        tol=options.find("tol").asDouble();

    int maxIter=200;
    if (options.check("maxIter"))
        maxIter=options.find("maxIter").asInt();

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
    inPort->get_dof()=dof;
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
        inPort->get_dof()=encodeDOF();

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
Vector CartesianSolver::solve(Vector &xd)
{
    Vector dummy(1);

    return slv->solve(prt->chn->getAng(),xd,
                      0.0,dummy,dummy,
                      0.0,dummy,dummy,
                      NULL,NULL,clb);
}


/************************************************************************/
void CartesianSolver::close()
{
    if (closed)
        return;

    if (isRunning())
        stop();

    rpcPort->interrupt();
    rpcPort->close();
    delete rpcPort;
    delete cmdProcessor;

    delete RES_EVENT(dofEvent);

    if (inPort)
    {
        inPort->interrupt();
        inPort->close();
        delete inPort;
    }

    if (outPort)
    {
        outPort->interrupt();
        outPort->close();
        delete outPort;
    }

    if (slv)
        delete slv;

    if (clb)
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

    if (prt)
    {
        delete prt->lmb;

        if (prt->cns)
            delete prt->cns;

        delete prt;
    }

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
    // init conditions
    bool dofChanged=false;
    bool doSolve=false;

    // handle changeDOF() safely and trigger
    // dofChanged event (if any)
    lock();
    if (changeDOF(inPort->get_dof()))
        dofChanged=true;    
    unlock();

    // wake up sleeping threads
    postDOFHandling();

    // get current configuration
    getFeedback();

    // acquire uncontrolled joints configuration
    if (!fullDOF)
    {
        Vector unctrlJoints;
        latchUncontrolledJoints(unctrlJoints);
    
        // detect movements of uncontrolled links
        double distExtMoves=(180.0/M_PI)*norm(unctrlJoints-unctrlJointsOld);
        unctrlJointsOld=unctrlJoints;
    
        // run the solver if movements of uncontrolled links 
        // are detected and mode==continuous
        doSolve|=inPort->get_contMode() && distExtMoves>1.0;
        if (doSolve && verbosity)
            fprintf(stdout,"Detected movements on uncontrolled links\
                            (norm=%g deg) ...\n",distExtMoves);
    }

    // run the solver if any input is received or dof is changed in continuous mode
    if (inPort->isNewDataEvent() || dofChanged && inPort->get_contMode())
    {    
        // update optimizer's options
        slv->set_ctrlPose(ctrlPose=inPort->get_pose());

        // update solver condition
        doSolve=true;        
    }

    // solver part
    if (doSolve)
    {        
        // point to the desired pose
        Vector &xd=inPort->get_xd();
        if (inPort->get_tokenPtr()) // latch the token
        {
            token=*inPort->get_tokenPtr();
            pToken=&token;
        }
        else
            pToken=NULL;

        // call the solver to converge
        double t0=Time::now();
        Vector q=solve(xd);
        double t1=Time::now();

        // q is the estimation of the real qd,
        // so that x is the actual achieved pose
        Vector x=prt->chn->EndEffPose(q);
        
        // change to degrees
        q=(180.0/M_PI)*q;

        // send data
        send(xd,x,q,pToken);

        // dump on screen
        if (verbosity)
            printInfo(xd,x,q,t1-t0);
    }    
}


/************************************************************************/
void CartesianSolver::threadRelease()
{    
    fprintf(stdout,"Stopping %s\n",slvName.c_str());
}


/************************************************************************/
CartesianSolver::~CartesianSolver()
{
    close();
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

    string partArm  =type=="left" ? "left_arm" : "right_arm";
    string remoteArm="/"+robot+"/"+partArm;
    string localArm ="/"+slvName+"/"+partArm;
    optArm.put("remote",remoteArm.c_str());
    optArm.put("local",localArm.c_str());

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

        fprintf(stdout,"Warning on shoulder:\
                        attempt to set one joint differently from others\n");
    }

    return CartesianSolver::decodeDOF(newDOF);
}


/************************************************************************/
Vector iCubArmCartesianSolver::solve(Vector &xd)
{
    Vector dummyVector(1);

    // try to keep elbow height as low as possible
    double weight2ndTask=0.01;
    Vector xdElb(3); xdElb=0.0; xdElb[2]=-1.0;
    Vector w_2nd(3); w_2nd=0.0; w_2nd[2]=1.0;

    double weight3rdTask=0.0;
    Vector qd_3rd=dummyVector;
    Vector w_3rd=dummyVector;

    // minimize the torso displacement wrt rest position (0,0,0)
    if (!(*prt->chn)[0].isBlocked() ||
        !(*prt->chn)[1].isBlocked() ||
        !(*prt->chn)[2].isBlocked())
    {
        weight3rdTask=0.01;

        qd_3rd.resize(prt->chn->getDOF(),0.0);
        w_3rd=qd_3rd;

        int offs=0;
        for (int i=0; i<3; i++)
            if (!(*prt->chn)[i].isBlocked())
                w_3rd[offs++]=1.0;
    }

    // call the solver and start the convergence from the current point
    return slv->solve(prt->chn->getAng(),xd,
                      weight2ndTask,xdElb,w_2nd,
                      weight3rdTask,qd_3rd,w_3rd,
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

    PartDescriptor *p=new PartDescriptor;
    p->lmb=new iCubLeg(type);
    p->chn=p->lmb->asChain();
    p->cns=NULL;
    p->prp.push_back(optLeg);   
    p->rvs.push_back(false);
    p->num=1;

    return p;
}



