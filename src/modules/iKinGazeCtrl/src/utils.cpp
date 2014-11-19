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

#include <algorithm>

#include <iCub/utils.h>
#include <iCub/solver.h>

#define MUTEX_XD            0
#define MUTEX_QD            1
#define MUTEX_X             2
#define MUTEX_Q             3
#define MUTEX_TORSO         4
#define MUTEX_V             5
#define MUTEX_COUNTERV      6
#define MUTEX_FPFRAME       7


/************************************************************************/
xdPort::xdPort(const Vector &xd0, void *_slv)
{   
    xdDelayed=xd=xd0;
    isNewDelayed=isNew=false;
    locked=false;
    closing=false;
    rx=0;

    slv=_slv;
    if (slv!=NULL)
        start();
}


/************************************************************************/
xdPort::~xdPort()
{
    closing=true;
    syncEvent.signal();

    if (slv!=NULL)
        stop();
}


/************************************************************************/
void xdPort::onRead(Bottle &b)
{
    if (locked)
        return;

    mutex_0.lock();
    int n=std::min(b.size(),(int)xd.length());
    for (int i=0; i<n; i++)
        xd[i]=b.get(i).asDouble();

    isNew=true;
    rx++;

    mutex_0.unlock();
    syncEvent.signal();
}


/************************************************************************/
void xdPort::set_xd(const Vector &_xd)
{
    if (locked)
        return;

    mutex_0.lock();
    xd=_xd;
    isNew=true;
    rx++;
    mutex_0.unlock();
    syncEvent.signal();
}


/************************************************************************/
Vector xdPort::get_xd()
{
    mutex_0.lock();
    Vector _xd=xd;
    mutex_0.unlock();
    return _xd;
}


/************************************************************************/
Vector xdPort::get_xdDelayed()
{
    mutex_1.lock();
    Vector _xdDelayed=xdDelayed;
    mutex_1.unlock();
    return _xdDelayed;
}


/************************************************************************/
void xdPort::run()
{
    while (!isStopping() && !closing)
    {
        syncEvent.reset();
        syncEvent.wait();

        double timeDelay=0.0;
        double theta=static_cast<Solver*>(slv)->neckTargetRotAngle(xd);
        if (theta<NECKSOLVER_RESTORINGANGLE)
            timeDelay=NECKSOLVER_ACTIVATIONDELAY;

        Time::delay(timeDelay);

        mutex_1.lock();

        xdDelayed=xd;
        isNewDelayed=true;

        mutex_1.unlock();
    }
}


/************************************************************************/
exchangeData::exchangeData()
{
    isCtrlActive=false;
    canCtrlBeDisabled=true;
    saccadeUnderway=false;
    minAllowedVergence=0.0;

    robotName="";
    localStemName="";
    eyeTiltMin=-1e9;
    eyeTiltMax=1e9;
    head_version=1.0;
    tweakOverwrite=true;
    tweakFile="";
}


/************************************************************************/
void exchangeData::resize_v(const int sz, const double val)
{
    mutex[MUTEX_V].lock();
    v.resize(sz,val);
    mutex[MUTEX_V].unlock();
}


/************************************************************************/
void exchangeData::resize_counterv(const int sz, const double val)
{
    mutex[MUTEX_COUNTERV].lock();
    counterv.resize(sz,val);
    mutex[MUTEX_COUNTERV].unlock();
}


/************************************************************************/
void exchangeData::set_xd(const Vector &_xd)
{
    mutex[MUTEX_XD].lock();
    xd=_xd;
    mutex[MUTEX_XD].unlock();
}


/************************************************************************/
void exchangeData::set_qd(const Vector &_qd)
{
    mutex[MUTEX_QD].lock();
    qd=_qd;
    mutex[MUTEX_QD].unlock();
}


/************************************************************************/
void exchangeData::set_qd(const int i, const double val)
{
    mutex[MUTEX_QD].lock();
    qd[i]=val;
    mutex[MUTEX_QD].unlock();
}


/************************************************************************/
void exchangeData::set_x(const Vector &_x)
{
    mutex[MUTEX_X].lock();
    x=_x;
    mutex[MUTEX_X].unlock();
}


/************************************************************************/
void exchangeData::set_x(const Vector &_x, const double stamp)
{
    mutex[MUTEX_X].lock();
    x=_x;
    x_stamp=stamp;
    mutex[MUTEX_X].unlock();
}


/************************************************************************/
void exchangeData::set_q(const Vector &_q)
{
    mutex[MUTEX_Q].lock();
    q=_q;
    mutex[MUTEX_Q].unlock();
}


/************************************************************************/
void exchangeData::set_torso(const Vector &_torso)
{
    mutex[MUTEX_TORSO].lock();
    torso=_torso;
    mutex[MUTEX_TORSO].unlock();
}


/************************************************************************/
void exchangeData::set_v(const Vector &_v)
{
    mutex[MUTEX_V].lock();
    v=_v;
    mutex[MUTEX_V].unlock();
}


/************************************************************************/
void exchangeData::set_counterv(const Vector &_counterv)
{
    mutex[MUTEX_COUNTERV].lock();
    counterv=_counterv;
    mutex[MUTEX_COUNTERV].unlock();
}


/************************************************************************/
void exchangeData::set_fpFrame(const Matrix &_S)
{
    mutex[MUTEX_FPFRAME].lock();
    S=_S;
    mutex[MUTEX_FPFRAME].unlock();
}


/************************************************************************/
Vector exchangeData::get_xd()
{
    mutex[MUTEX_XD].lock();
    Vector _xd=xd;
    mutex[MUTEX_XD].unlock();

    return _xd;
}


/************************************************************************/
Vector exchangeData::get_qd()
{
    mutex[MUTEX_QD].lock();
    Vector _qd=qd;
    mutex[MUTEX_QD].unlock();

    return _qd;
}


/************************************************************************/
Vector exchangeData::get_x()
{
    mutex[MUTEX_X].lock();
    Vector _x=x;
    mutex[MUTEX_X].unlock();

    return _x;
}


/************************************************************************/
Vector exchangeData::get_x(double &stamp)
{
    mutex[MUTEX_X].lock();
    Vector _x=x;
    stamp=x_stamp;
    mutex[MUTEX_X].unlock();

    return _x;
}


/************************************************************************/
Vector exchangeData::get_q()
{
    mutex[MUTEX_Q].lock();
    Vector _q=q;
    mutex[MUTEX_Q].unlock();

    return _q;
}


/************************************************************************/
Vector exchangeData::get_torso()
{
    mutex[MUTEX_TORSO].lock();
    Vector _torso=torso;
    mutex[MUTEX_TORSO].unlock();

    return _torso;
}


/************************************************************************/
Vector exchangeData::get_v()
{
    mutex[MUTEX_V].lock();
    Vector _v=v;
    mutex[MUTEX_V].unlock();

    return _v;
}


/************************************************************************/
Vector exchangeData::get_counterv()
{
    mutex[MUTEX_COUNTERV].lock();
    Vector _counterv=counterv;
    mutex[MUTEX_COUNTERV].unlock();

    return _counterv;
}


/************************************************************************/
Matrix exchangeData::get_fpFrame()
{
    mutex[MUTEX_FPFRAME].lock();
    Matrix _S=S;
    mutex[MUTEX_FPFRAME].unlock();

    return _S;
}


/************************************************************************/
bool GazeComponent::getExtrinsicsMatrix(const string &type, Matrix &M)
{
    if (type=="left")
    {
        M=eyeL->asChain()->getHN();
        return true;
    }
    else if (type=="right")
    {
        M=eyeR->asChain()->getHN();
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool GazeComponent::setExtrinsicsMatrix(const string &type, const Matrix &M)
{
    if (type=="left")
    {
        eyeL->asChain()->setHN(M);
        return true;
    }
    else if (type=="right")
    {
        eyeR->asChain()->setHN(M);
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool getCamPrj(const ResourceFinder &rf, const string &type,
               Matrix **Prj, const bool verbose)
{
    ResourceFinder &_rf=const_cast<ResourceFinder&>(rf);
    *Prj=NULL;

    if (!_rf.isConfigured())
        return false;

    string message=_rf.findFile("from").c_str();
    if (!message.empty())
    {
        message+=": intrinsic parameters for "+type;
        Bottle &parType=_rf.findGroup(type.c_str());
        if (!parType.isNull())
        {
            if (parType.check("fx") && parType.check("fy") &&
                parType.check("cx") && parType.check("cy"))
            {
                double fx=parType.find("fx").asDouble();
                double fy=parType.find("fy").asDouble();
                double cx=parType.find("cx").asDouble();
                double cy=parType.find("cy").asDouble();

                if (verbose)
                {
                    yInfo("%s found:",message.c_str());
                    yInfo("fx = %g",fx);
                    yInfo("fy = %g",fy);
                    yInfo("cx = %g",cx);
                    yInfo("cy = %g",cy);
                }

                *Prj=new Matrix(eye(3,4));

                Matrix &K=**Prj;
                K(0,0)=fx; K(1,1)=fy;
                K(0,2)=cx; K(1,2)=cy;

                return true;
            }
        }
    }
    else
    {
        message=_rf.find("from").asString().c_str();
        message+=": intrinsic parameters for "+type;
    }

    if (verbose)
        yWarning("%s not found!",message.c_str());

    return false;
}


/************************************************************************/
bool getAlignHN(const ResourceFinder &rf, const string &type,
                iKinChain *chain, const bool verbose)
{
    ResourceFinder &_rf=const_cast<ResourceFinder&>(rf);
    if ((chain!=NULL) && _rf.isConfigured())
    {
        string message=_rf.findFile("from").c_str();
        if (!message.empty())
        {
            message+=": aligning matrix for "+type;
            Bottle &parType=_rf.findGroup(type.c_str());
            if (!parType.isNull())
            {
                if (Bottle *bH=parType.find("HN").asList())
                {
                    int i=0;
                    int j=0;

                    Matrix HN(4,4); HN=0.0;
                    for (int cnt=0; (cnt<bH->size()) && (cnt<HN.rows()*HN.cols()); cnt++)
                    {
                        HN(i,j)=bH->get(cnt).asDouble();
                        if (++j>=HN.cols())
                        {
                            i++;
                            j=0;
                        }
                    }

                    // enforce the homogeneous property
                    HN(3,0)=HN(3,1)=HN(3,2)=0.0;
                    HN(3,3)=1.0;

                    chain->setHN(HN);

                    if (verbose)
                    {
                        yInfo("%s found:",message.c_str());
                        yInfo("%s",HN.toString(3,3).c_str());
                    }

                    return true;
                }
            }
        }
        else
        {
            message=_rf.find("from").asString().c_str();
            message+=": aligning matrix for "+type;
        }

        if (verbose)
            yWarning("%s not found!",message.c_str());
    }

    return false;
}


/************************************************************************/
Matrix alignJointsBounds(iKinChain *chain, PolyDriver *drvTorso, PolyDriver *drvHead,
                         const double eyeTiltMin, const double eyeTiltMax)
{
    IEncoders      *encs;
    IControlLimits *lims;

    double min, max;
    int nJointsTorso=3;    

    if (drvTorso!=NULL)
    {
        drvTorso->view(encs);
        drvTorso->view(lims);        
        encs->getAxes(&nJointsTorso);

        for (int i=0; i<nJointsTorso; i++)
        {   
            if (lims->getLimits(i,&min,&max))
            {
                (*chain)[nJointsTorso-1-i].setMin(CTRL_DEG2RAD*min); // reversed order
                (*chain)[nJointsTorso-1-i].setMax(CTRL_DEG2RAD*max);
            }
            else
                yError("unable to retrieve limits for torso joint #%d",i);
        }
    }

    drvHead->view(encs);
    drvHead->view(lims);
    int nJointsHead;
    encs->getAxes(&nJointsHead);
    Matrix lim(nJointsHead,2);

    for (int i=0; i<nJointsHead; i++)
    {   
        if (lims->getLimits(i,&min,&max))
        {
            // limit eye's tilt due to eyelids
            if (i==3)
            {
                min=std::max(min,eyeTiltMin);
                max=std::min(max,eyeTiltMax);
            }

            lim(i,0)=CTRL_DEG2RAD*min;
            lim(i,1)=CTRL_DEG2RAD*max;

            // just one eye's got only 5 dofs
            if (i<nJointsHead-1)
            {
                (*chain)[nJointsTorso+i].setMin(lim(i,0));
                (*chain)[nJointsTorso+i].setMax(lim(i,1));
            }
        }
        else
            yError("unable to retrieve limits for head joint #%d",i);
    }

    return lim;
}


/************************************************************************/
void copyJointsBounds(iKinChain *ch1, iKinChain *ch2)
{
    unsigned int N1=ch1->getN();
    unsigned int N2=ch2->getN();
    unsigned int N =N1>N2 ? N2 : N1;

    for (unsigned int i=0; i<N; i++)
    {
        (*ch2)[i].setMin((*ch1)[i].getMin());
        (*ch2)[i].setMax((*ch1)[i].getMax());
    }
}


/************************************************************************/
void updateTorsoBlockedJoints(iKinChain *chain, const Vector &fbTorso)
{
    for (size_t i=0; i<fbTorso.length(); i++)
         chain->setBlockingValue(i,fbTorso[i]);
}


/************************************************************************/
void updateNeckBlockedJoints(iKinChain *chain, const Vector &fbNeck)
{
    for (int i=0; i<3; i++)
         chain->setBlockingValue(3+i,fbNeck[i]);
}


/************************************************************************/
bool getFeedback(Vector &fbTorso, Vector &fbHead, PolyDriver *drvTorso,
                 PolyDriver *drvHead, exchangeData *commData, double *timeStamp)
{
    IEncodersTimed *encs;

    int nJointsTorso=fbTorso.length();
    int nJointsHead=fbHead.length();

    Vector fb(std::max(nJointsTorso,nJointsHead));
    Vector stamps(nJointsTorso+nJointsHead,0.0);
    bool ret=true;
    
    if (drvTorso!=NULL)
    {
        drvTorso->view(encs);
        if (encs->getEncodersTimed(fb.data(),stamps.data()))
        {
            for (int i=0; i<nJointsTorso; i++)
                fbTorso[i]=CTRL_DEG2RAD*fb[nJointsTorso-1-i];    // reversed order
        }
        else
            ret=false;
    }
    else
        fbTorso=0.0;

    drvHead->view(encs);
    if (encs->getEncodersTimed(fb.data(),stamps.data()+nJointsTorso))
    {
        for (int i=0; i<nJointsHead; i++)
            fbHead[i]=CTRL_DEG2RAD*fb[i];
    }
    else
        ret=false;
    
    // impose vergence != 0.0
    if (commData!=NULL)
        if (fbHead[nJointsHead-1]<commData->get_minAllowedVergence())
            fbHead[nJointsHead-1]=commData->get_minAllowedVergence();
    
    // retrieve the highest encoders time stamp
    if (timeStamp!=NULL)
        *timeStamp=findMax(stamps);

    return ret;
}


