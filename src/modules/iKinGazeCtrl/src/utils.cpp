/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini, Alessandro Roncone
 * email:  ugo.pattacini@iit.it, alessandro.roncone@iit.it
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

#include <cmath>
#include <limits>
#include <algorithm>
#include <sstream>

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
#define MUTEX_IMU           8


/************************************************************************/
xdPort::xdPort(void *_slv) : slv(_slv)
{   
    isNewDelayed=isNew=false;
    locked=false;
    closing=false;
    rx=0;

    useCallback();
    start();
}


/************************************************************************/
void xdPort::init(const Vector &xd0)
{
    xdDelayed=xd=xd0;
}


/************************************************************************/
xdPort::~xdPort()
{
    closing=true;
    triggerNeck.signal();
    stop();
}


/************************************************************************/
void xdPort::onRead(Bottle &b)
{
    LockGuard lg(mutex_0);
    if (locked)
        return;
        
    int n=std::min(b.size(),(int)xd.length());
    for (int i=0; i<n; i++)
        xd[i]=b.get(i).asDouble();

    isNew=true;
    rx++;

    triggerNeck.signal();
}


/************************************************************************/
bool xdPort::set_xd(const Vector &_xd)
{
    LockGuard lg(mutex_0);
    if (locked)
        return false;

    xd=_xd;
    isNew=true;
    rx++;
    triggerNeck.signal();
    return true;
}


/************************************************************************/
Vector xdPort::get_xd()
{
    LockGuard lg(mutex_0);
    Vector _xd=xd;
    return _xd;
}


/************************************************************************/
Vector xdPort::get_xdDelayed()
{
    LockGuard lg(mutex_1);
    Vector _xdDelayed=xdDelayed;
    return _xdDelayed;
}


/************************************************************************/
void xdPort::run()
{
    while (!isStopping() && !closing)
    {
        triggerNeck.reset();
        triggerNeck.wait();

        double timeDelay=0.0;
        double theta=static_cast<Solver*>(slv)->neckTargetRotAngle(xd);
        if (theta<NECKSOLVER_RESTORINGANGLE)
            timeDelay=NECKSOLVER_ACTIVATIONDELAY;

        Time::delay(timeDelay);

        LockGuard lg(mutex_1);
        xdDelayed=xd;
        isNewDelayed=true;
    }
}


/************************************************************************/
ExchangeData::ExchangeData()
{
    imu.resize(12,0.0);
    port_xd=NULL;

    ctrlActive=false;
    trackingModeOn=false;
    saccadeUnderway=false;
    minAllowedVergence=0.0;    
    eyesBoundVer=-1.0;
    neckSolveCnt=0;

    saccadesInhibitionPeriod=SACCADES_INHIBITION_PERIOD;
    saccadesActivationAngle=SACCADES_ACTIVATION_ANGLE;

    eyeTiltLim.resize(2);
    eyeTiltLim[0]=-std::numeric_limits<double>::max();
    eyeTiltLim[1]=std::numeric_limits<double>::max();

    robotName="";
    localStemName="";
    head_version=1.0;
    tweakOverwrite=true;
    tweakFile="";
}


/************************************************************************/
void ExchangeData::resize_v(const int sz, const double val)
{
    LockGuard lg(mutex[MUTEX_V]);
    v.resize(sz,val);
}


/************************************************************************/
void ExchangeData::resize_counterv(const int sz, const double val)
{
    LockGuard lg(mutex[MUTEX_COUNTERV]);
    counterv.resize(sz,val);
}


/************************************************************************/
void ExchangeData::set_xd(const Vector &_xd)
{
    LockGuard lg(mutex[MUTEX_XD]);
    xd=_xd;
}


/************************************************************************/
void ExchangeData::set_qd(const Vector &_qd)
{
    LockGuard lg(mutex[MUTEX_QD]);
    qd=_qd;
}


/************************************************************************/
void ExchangeData::set_qd(const int i, const double val)
{
    LockGuard lg(mutex[MUTEX_QD]);
    qd[i]=val;
}


/************************************************************************/
void ExchangeData::set_x(const Vector &_x)
{
    LockGuard lg(mutex[MUTEX_X]);
    x=_x;
}


/************************************************************************/
void ExchangeData::set_x(const Vector &_x, const double stamp)
{
    LockGuard lg(mutex[MUTEX_X]);
    x=_x;
    x_stamp=stamp;
}


/************************************************************************/
void ExchangeData::set_q(const Vector &_q)
{
    LockGuard lg(mutex[MUTEX_Q]);
    q=_q;
}


/************************************************************************/
void ExchangeData::set_torso(const Vector &_torso)
{
    LockGuard lg(mutex[MUTEX_TORSO]);
    torso=_torso;
}


/************************************************************************/
void ExchangeData::set_v(const Vector &_v)
{
    LockGuard lg(mutex[MUTEX_V]);
    v=_v;
}


/************************************************************************/
void ExchangeData::set_counterv(const Vector &_counterv)
{
    LockGuard lg(mutex[MUTEX_COUNTERV]);
    counterv=_counterv;
}


/************************************************************************/
void ExchangeData::set_fpFrame(const Matrix &_S)
{
    LockGuard lg(mutex[MUTEX_FPFRAME]);
    S=_S;
}


/************************************************************************/
void ExchangeData::set_imu(const Vector &_imu)
{
    LockGuard lg(mutex[MUTEX_IMU]);
    imu=_imu;
}


/************************************************************************/
Vector ExchangeData::get_xd()
{
    LockGuard lg(mutex[MUTEX_XD]);
    Vector _xd=xd;
    return _xd;
}


/************************************************************************/
Vector ExchangeData::get_qd()
{
    LockGuard lg(mutex[MUTEX_QD]);
    Vector _qd=qd;
    return _qd;
}


/************************************************************************/
Vector ExchangeData::get_x()
{
    LockGuard lg(mutex[MUTEX_X]);
    Vector _x=x;
    return _x;
}


/************************************************************************/
Vector ExchangeData::get_x(double &stamp)
{
    LockGuard lg(mutex[MUTEX_X]);
    Vector _x=x;
    stamp=x_stamp;
    return _x;
}


/************************************************************************/
Vector ExchangeData::get_q()
{
    LockGuard lg(mutex[MUTEX_Q]);
    Vector _q=q;
    return _q;
}


/************************************************************************/
Vector ExchangeData::get_torso()
{
    LockGuard lg(mutex[MUTEX_TORSO]);
    Vector _torso=torso;
    return _torso;
}


/************************************************************************/
Vector ExchangeData::get_v()
{
    LockGuard lg(mutex[MUTEX_V]);
    Vector _v=v;
    return _v;
}


/************************************************************************/
Vector ExchangeData::get_counterv()
{
    LockGuard lg(mutex[MUTEX_COUNTERV]);
    Vector _counterv=counterv;
    return _counterv;
}


/************************************************************************/
Matrix ExchangeData::get_fpFrame()
{
    LockGuard lg(mutex[MUTEX_FPFRAME]);
    Matrix _S=S;
    return _S;
}


/************************************************************************/
Vector ExchangeData::get_imu()
{
    LockGuard lg(mutex[MUTEX_IMU]);
    Vector _imu=imu;
    return _imu;
}


/************************************************************************/
string ExchangeData::headVersion2String()
{
    ostringstream str;
    str<<"v"<<head_version;
    return str.str();
}


/************************************************************************/
IMUPort::IMUPort() : commData(NULL)
{
    useCallback();
}


/************************************************************************/
void IMUPort::setExchangeData(ExchangeData *commData)
{
    this->commData=commData;
}


/************************************************************************/
void IMUPort::onRead(Vector &imu)
{
    if (commData!=NULL)
        commData->set_imu(imu);
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
bool getCamParams(const ResourceFinder &rf, const string &type,
                  Matrix **Prj, int &w, int &h, const bool verbose)
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
        if (parType.check("w")  && parType.check("h") &&
            parType.check("fx") && parType.check("fy") &&
            parType.check("cx") && parType.check("cy"))
        {
            w=parType.find("w").asInt();
            h=parType.find("h").asInt();
            double fx=parType.find("fx").asDouble();
            double fy=parType.find("fy").asDouble();
            double cx=parType.find("cx").asDouble();
            double cy=parType.find("cy").asDouble();

            if (verbose)
            {
                yInfo("%s found:",message.c_str());
                yInfo("w  = %d",w);
                yInfo("h  = %d",h);
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
Matrix alignJointsBounds(iKinChain *chain, PolyDriver *drvTorso,
                         PolyDriver *drvHead, const Vector &eyeTiltLim)
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
                min=std::max(min,eyeTiltLim[0]);
                max=std::min(max,eyeTiltLim[1]);
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
                 PolyDriver *drvHead, ExchangeData *commData, double *timeStamp)
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
        fbHead[nJointsHead-1]=std::max(fbHead[nJointsHead-1],commData->minAllowedVergence);
    
    // retrieve the highest encoders time stamp
    if (timeStamp!=NULL)
        *timeStamp=findMax(stamps);

    return ret;
}


