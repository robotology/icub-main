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

#include <limits>
#include <cstdio>
#include <sstream>

#include <iCub/solver.h>
#include <iCub/controller.h>


/************************************************************************/
Controller::Controller(PolyDriver *_drvTorso, PolyDriver *_drvHead, ExchangeData *_commData,
                       const double _neckTime, const double _eyesTime, const double _min_abs_vel,
                       const unsigned int _period) :
                       RateThread(_period),       drvTorso(_drvTorso), drvHead(_drvHead),
                       commData(_commData),       neckTime(_neckTime), eyesTime(_eyesTime),
                       min_abs_vel(_min_abs_vel), period(_period),     Ts(_period/1000.0),
                       printAccTime(0.0)
{
    // Instantiate objects
    neck=new iCubHeadCenter(commData->head_version>1.0?"right_v2":"right");
    eyeL=new iCubEye(commData->head_version>1.0?"left_v2":"left");
    eyeR=new iCubEye(commData->head_version>1.0?"right_v2":"right");
    imu=new iCubInertialSensor(commData->head_version>1.0?"v2":"v1");

    // remove constraints on the links: logging purpose
    imu->setAllConstraints(false);

    // release links
    neck->releaseLink(0); eyeL->releaseLink(0); eyeR->releaseLink(0);
    neck->releaseLink(1); eyeL->releaseLink(1); eyeR->releaseLink(1);
    neck->releaseLink(2); eyeL->releaseLink(2); eyeR->releaseLink(2);

    // Get the chain objects
    chainNeck=neck->asChain();
    chainEyeL=eyeL->asChain();
    chainEyeR=eyeR->asChain();

    // add aligning matrices read from configuration file
    getAlignHN(commData->rf_cameras,"ALIGN_KIN_LEFT",eyeL->asChain());
    getAlignHN(commData->rf_cameras,"ALIGN_KIN_RIGHT",eyeR->asChain());

    // overwrite aligning matrices iff specified through tweak values
    if (commData->tweakOverwrite)
    {
        getAlignHN(commData->rf_tweak,"ALIGN_KIN_LEFT",eyeL->asChain());
        getAlignHN(commData->rf_tweak,"ALIGN_KIN_RIGHT",eyeR->asChain());
    }

    // read number of joints
    if (drvTorso!=NULL)
    {
        IEncoders *encTorso; drvTorso->view(encTorso);
        encTorso->getAxes(&nJointsTorso);
    }
    else
        nJointsTorso=3;
    
    IEncoders *encHead; drvHead->view(encHead);
    encHead->getAxes(&nJointsHead);

    drvHead->view(modHead);
    drvHead->view(posHead);
    drvHead->view(velHead);

    // if requested check if position control is available
    if (commData->neckPosCtrlOn)
    {
        commData->neckPosCtrlOn=drvHead->view(posNeck);
        yInfo("### neck control - requested POSITION mode: IPositionDirect [%s] => %s mode selected",
              commData->neckPosCtrlOn?"available":"not available",commData->neckPosCtrlOn?"POSITION":"VELOCITY");
    }
    else
        yInfo("### neck control - requested VELOCITY mode => VELOCITY mode selected");

    // joints bounds alignment
    lim=alignJointsBounds(chainNeck,drvTorso,drvHead,commData->eyeTiltLim);

    // read starting position
    fbTorso.resize(nJointsTorso,0.0);
    fbHead.resize(nJointsHead,0.0);

    // exclude acceleration constraints by fixing
    // thresholds at high values
    Vector a_robHead(nJointsHead,std::numeric_limits<double>::max());
    velHead->setRefAccelerations(a_robHead.data());

    copyJointsBounds(chainNeck,chainEyeL);
    copyJointsBounds(chainEyeL,chainEyeR);

    // find minimum allowed vergence
    startupMinVer=lim(nJointsHead-1,0);
    findMinimumAllowedVergence();

    // reinforce vergence min bound
    lim(nJointsHead-1,0)=commData->minAllowedVergence;
    getFeedback(fbTorso,fbHead,drvTorso,drvHead,commData);

    fbNeck=fbHead.subVector(0,2);
    fbEyes=fbHead.subVector(3,5);
    qdNeck.resize(3,0.0); qdEyes.resize(3,0.0);
    vNeck.resize(3,0.0);  vEyes.resize(3,0.0);
    v.resize(nJointsHead,0.0);

    // Set the task execution time
    setTeyes(eyesTime);
    setTneck(neckTime);

    mjCtrlNeck=new minJerkVelCtrlForIdealPlant(Ts,fbNeck.length());
    mjCtrlEyes=new minJerkVelCtrlForIdealPlant(Ts,fbEyes.length());
    IntState=new Integrator(Ts,fbHead,lim);
    IntPlan=new Integrator(Ts,fbNeck,lim.submatrix(0,2,0,1));
    IntStabilizer=new Integrator(Ts,zeros(vNeck.length()));

    neckJoints.resize(3);
    eyesJoints.resize(3);
    neckJoints[0]=0;
    neckJoints[1]=1;
    neckJoints[2]=2;
    eyesJoints[0]=3;
    eyesJoints[1]=4;
    eyesJoints[2]=5;

    qd=fbHead;
    q0=qd;
    qddeg=CTRL_RAD2DEG*qd;
    vdeg =CTRL_RAD2DEG*v;

    ctrlActiveRisingEdgeTime=0.0;
    saccadeStartTime=0.0;
    pathPerc=0.0;

    unplugCtrlEyes=false;
    ctrlInhibited=false;
    motionDone=true;
    reliableGyro=true;
    stabilizeGaze=false;
}


/************************************************************************/
void Controller::findMinimumAllowedVergence()
{
    iKinChain cl(*chainEyeL), cr(*chainEyeR);
    cl.setAng(zeros(cl.getDOF()));
    cr.setAng(zeros(cl.getDOF()));

    double minVer=startupMinVer;
    double maxVer=lim(nJointsHead-1,1);
    for (double ver=minVer; ver<maxVer; ver+=0.5*CTRL_DEG2RAD)
    {
        cl(cl.getDOF()-1).setAng(ver/2.0);
        cr(cr.getDOF()-1).setAng(-ver/2.0);

        Vector fp;        
        if (CartesianHelper::computeFixationPointData(cl,cr,fp))
        {
            // impose homogeneous coordinates
            fp.push_back(1.0);

            // if the component along eye's z-axis is positive
            // then this means that the fixation point is ok,
            // being in front of the robot
            Vector fpe=SE3inv(cl.getH())*fp;
            if (fpe[2]>0.0)
            {
                minVer=ver;
                break;
            }
        }
    }

    yInfo("### computed minimum allowed vergence = %g [deg]",minVer*CTRL_RAD2DEG);
    commData->minAllowedVergence=minVer;
}


/************************************************************************/
void Controller::minAllowedVergenceChanged()
{
    lim(nJointsHead-1,0)=commData->minAllowedVergence;
    IntState->setLim(lim);
}


/************************************************************************/
void Controller::notifyEvent(const string &event, const double checkPoint)
{
    if (port_event.getOutputCount()>0)
    {
        Bottle &ev=port_event.prepare();
        ev.clear();

        ev.addString(event.c_str());
        ev.addDouble(q_stamp);
        if (checkPoint>=0.0)
            ev.addDouble(checkPoint);

        txInfo_event.update(q_stamp);
        port_event.setEnvelope(txInfo_event);
        port_event.writeStrict();
    }
}


/************************************************************************/
void Controller::motionOngoingEventsHandling()
{
    if (motionOngoingEventsCurrent.size()!=0)
    {
        double curCheckPoint=*motionOngoingEventsCurrent.begin();
        if (pathPerc>=curCheckPoint)
        {            
            notifyEvent("motion-ongoing",curCheckPoint);
            motionOngoingEventsCurrent.erase(curCheckPoint);
        }
    }
}


/************************************************************************/
void Controller::motionOngoingEventsFlush()
{
    while (motionOngoingEventsCurrent.size()!=0)
    {
        double curCheckPoint=*motionOngoingEventsCurrent.begin();
        notifyEvent("motion-ongoing",curCheckPoint);
        motionOngoingEventsCurrent.erase(curCheckPoint);
    }
}


/************************************************************************/
void Controller::stopLimb(const bool execStopPosition)
{
    if (commData->neckPosCtrlOn)
    {
        if (execStopPosition)
            posHead->stop(neckJoints.size(),neckJoints.getFirst());

        // note: vel==0.0 is always achievable
        velHead->velocityMove(eyesJoints.size(),eyesJoints.getFirst(),
                              Vector(eyesJoints.size(),0.0).data());
    }
    else
        velHead->velocityMove(Vector(nJointsHead,0.0).data());

    if (commData->debugInfoEnabled && (port_debug.getOutputCount()>0))
    {
        Bottle info;
        int j=0;

        if (commData->neckPosCtrlOn)
        {
            if (execStopPosition)
            {
                for (size_t i=0; i<neckJoints.size(); i++)
                {
                    ostringstream ss;
                    ss<<"pos_"<<neckJoints[i];
                    info.addString(ss.str().c_str());
                    info.addString("stop");
                }
            }

            j=eyesJoints[0];
        }

        for (int i=j; i<nJointsHead; i++)
        {
            ostringstream ss;
            ss<<"vel_"<<i;
            info.addString(ss.str().c_str());
            info.addDouble(0.0);
        }

        port_debug.prepare()=info;
        txInfo_debug.update(q_stamp);
        port_debug.setEnvelope(txInfo_debug);
        port_debug.writeStrict();
    }

    commData->ctrlActive=false;
    motionDone=true;
}


/************************************************************************/
void Controller::stopControl()
{
    mutexRun.lock();
    stopControlHelper();
    mutexRun.unlock();
}


/************************************************************************/
void Controller::stopControlHelper()
{
    mutexCtrl.lock();

    if (commData->ctrlActive)
    {
        q_stamp=Time::now();
        ctrlInhibited=true;
        stopLimb();
        notifyEvent("motion-done");
    }

    mutexCtrl.unlock();
}


/************************************************************************/
void Controller::printIter(Vector &xd, Vector &fp, Vector &qd, Vector &q,
                           Vector &v, double printTime)
{
    if ((printAccTime+=Ts)>=printTime)
    {
        printAccTime=0.0;

        printf("norm(e)           = %g\n",norm(xd-fp));
        printf("Target fix. point = %s\n",xd.toString().c_str());
        printf("Actual fix. point = %s\n",fp.toString().c_str());
        printf("Target Joints     = %s\n",qd.toString().c_str());
        printf("Actual Joints     = %s\n",q.toString().c_str());
        printf("Velocity          = %s\n",v.toString().c_str());
        printf("\n");
    }
}


/************************************************************************/
bool Controller::threadInit()
{
    port_x.open((commData->localStemName+"/x:o").c_str());
    port_q.open((commData->localStemName+"/q:o").c_str());
    port_event.open((commData->localStemName+"/events:o").c_str());

    if (commData->debugInfoEnabled)
        port_debug.open((commData->localStemName+"/dbg:o").c_str());        

    yInfo("Starting Controller at %d ms",period);
    q_stamp=Time::now();

    return true;
}


/************************************************************************/
void Controller::afterStart(bool s)
{
    if (s)
        yInfo("Controller started successfully");
    else
        yError("Controller did not start!");
}


/************************************************************************/
Vector Controller::computedxFP(const Matrix &H, const Vector &v,
                               const Vector &w, const Vector &x_FP)
{
    Matrix H_=H;
    Vector w_=w;
    w_.push_back(1.0);
    
    H_(0,3)=x_FP[0]-H_(0,3);
    H_(1,3)=x_FP[1]-H_(1,3);
    H_(2,3)=x_FP[2]-H_(2,3);
    Vector dx_FP_pos=v+(w_[0]*cross(H_,0,H_,3)+
                        w_[1]*cross(H_,1,H_,3)+
                        w_[2]*cross(H_,2,H_,3));    

    H_(0,3)=H_(1,3)=H_(2,3)=0.0;
    Vector dx_FP_rot=H_*w_;
    dx_FP_rot.pop_back();

    return cat(dx_FP_pos,dx_FP_rot);
}


/************************************************************************/
Vector Controller::computeNeckVelFromdxFP(const Vector &fp, const Vector &dfp)
{
    // convert fp from root to the neck reference frame
    Vector fpR=fp;
    fpR.push_back(1.0);
    Vector fpE=SE3inv(chainNeck->getH())*fpR;

    // compute the Jacobian of the head joints alone 
    // (by adding the new fixation point beforehand)
    Matrix HN=eye(4);
    HN(0,3)=fpE[0];
    HN(1,3)=fpE[1];
    HN(2,3)=fpE[2];

    mutexChain.lock();
    chainNeck->setHN(HN);
    Matrix JN=chainNeck->GeoJacobian();
    chainNeck->setHN(eye(4,4));
    mutexChain.unlock();

    // take only the last three rows of the Jacobian
    // belonging to the head joints
    Matrix JNp=JN.submatrix(3,5,3,5);

    return pinv(JNp)*dfp.subVector(3,5);
}


/************************************************************************/
Vector Controller::computeEyesVelFromdxFP(const Vector &dfp)
{
    Matrix eyesJ; Vector tmp;
    if ((fbEyes[2]>CTRL_DEG2RAD*GAZECTRL_CRITICVER_STABILIZATION) &&
        CartesianHelper::computeFixationPointData(*chainEyeL,*chainEyeR,tmp,eyesJ))
        return pinv(eyesJ)*dfp.subVector(0,2);
    else
        return zeros(vEyes.length());
}


/************************************************************************/
void Controller::doSaccade(const Vector &ang, const Vector &vel)
{
    LockGuard guard(mutexCtrl);
    if (ctrlInhibited)
        return;

    setJointsCtrlMode();

    // enforce joints bounds
    Vector ang_(3);
    ang_[0]=CTRL_RAD2DEG*sat(ang[0],lim(eyesJoints[0],0),lim(eyesJoints[0],1));
    ang_[1]=CTRL_RAD2DEG*sat(ang[1],lim(eyesJoints[1],0),lim(eyesJoints[1],1));
    ang_[2]=CTRL_RAD2DEG*sat(ang[2],lim(eyesJoints[2],0),lim(eyesJoints[2],1));

    posHead->setRefSpeeds(eyesJoints.size(),eyesJoints.getFirst(),vel.data());
    posHead->positionMove(eyesJoints.size(),eyesJoints.getFirst(),ang_.data());

    if (commData->debugInfoEnabled && (port_debug.getOutputCount()>0))
    {
        Bottle info;
        for (size_t i=0; i<ang_.length(); i++)
        {
            ostringstream ss;
            ss<<"pos_"<<eyesJoints[i];
            info.addString(ss.str().c_str());
            info.addDouble(ang_[i]);
        }

        port_debug.prepare()=info;
        txInfo_debug.update(q_stamp);
        port_debug.setEnvelope(txInfo_debug);
        port_debug.writeStrict();
    }

    saccadeStartTime=Time::now();
    commData->saccadeUnderway=true;
    unplugCtrlEyes=true;

    notifyEvent("saccade-onset");
}


/************************************************************************/
void Controller::resetCtrlEyes()
{
    mutexCtrl.lock();
    mjCtrlEyes->reset(zeros(fbEyes.length()));
    unplugCtrlEyes=false;
    mutexCtrl.unlock();
}


/************************************************************************/
bool Controller::areJointsHealthyAndSet()
{
    VectorOf<int> modes(nJointsHead);
    modHead->getControlModes(modes.getFirst());

    jointsToSet.clear();
    for (size_t i=0; i<modes.size(); i++)
    {
        if ((modes[i]==VOCAB_CM_HW_FAULT) || (modes[i]==VOCAB_CM_IDLE))
            return false;
        else if (i<(size_t)eyesJoints[0])
        {
            if (commData->neckPosCtrlOn)
            {
                if (modes[i]!=VOCAB_CM_POSITION_DIRECT)
                    jointsToSet.push_back(i);
            }
            else if (modes[i]!=VOCAB_CM_VELOCITY)
                jointsToSet.push_back(i);
        }
        else if (modes[i]!=VOCAB_CM_MIXED)
            jointsToSet.push_back(i);
    }

    return true;
}


/************************************************************************/
void Controller::setJointsCtrlMode()
{
    if (jointsToSet.size()==0)
        return;

    VectorOf<int> modes;
    for (size_t i=0; i<jointsToSet.size(); i++)
    {
        if (jointsToSet[i]<eyesJoints[0])
        {
            if (commData->neckPosCtrlOn)
                modes.push_back(VOCAB_CM_POSITION_DIRECT);
            else
                modes.push_back(VOCAB_CM_VELOCITY);
        }
        else
            modes.push_back(VOCAB_CM_MIXED);
    }

    modHead->setControlModes(jointsToSet.size(),jointsToSet.getFirst(),
                             modes.getFirst());
}


/************************************************************************/
bool Controller::setGazeStabilization(const bool f)
{
    if (commData->stabilizationOn)
    {
        if (stabilizeGaze!=f)
        {
            LockGuard guard(mutexRun);
            if (f)
            {
                if (!commData->ctrlActive)
                {
                    IntPlan->reset(fbNeck); 
                    IntStabilizer->reset(zeros(vNeck.length()));
                }
                notifyEvent("stabilization-on");
            }
            else
            {
                LockGuard guardCtrl(mutexCtrl);
                q_stamp=Time::now();
                stopLimb();
                notifyEvent("stabilization-off");
            }

            stabilizeGaze=f;
        }

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool Controller::getGazeStabilization() const
{
    return stabilizeGaze;
}


/************************************************************************/
void Controller::run()
{
    LockGuard guard(mutexRun);
    
    mutexCtrl.lock();
    bool jointsHealthy=areJointsHealthyAndSet();
    mutexCtrl.unlock();

    if (!jointsHealthy)
    {
        stopControlHelper();
        commData->port_xd->get_new()=false;
    }

    string event="none";

    // verify if any saccade is still underway
    mutexCtrl.lock();
    if (commData->saccadeUnderway && (Time::now()-saccadeStartTime>=Ts))
    {
        bool done;
        posHead->checkMotionDone(eyesJoints.size(),eyesJoints.getFirst(),&done);
        commData->saccadeUnderway=!done;

        if (!commData->saccadeUnderway)
            notifyEvent("saccade-done");
    }
    mutexCtrl.unlock();
    
    // get data
    double x_stamp;
    Vector xd=commData->get_xd();
    Vector x=commData->get_x(x_stamp);
    qd=commData->get_qd();

    // read feedbacks
    q_stamp=Time::now();
    if (!getFeedback(fbTorso,fbHead,drvTorso,drvHead,commData,&q_stamp))
    {
        yWarning("Communication timeout detected!");
        notifyEvent("comm-timeout");
        suspend();
        return;
    }

    // update pose information
    {
        mutexChain.lock();
        for (int i=0; i<nJointsTorso; i++)
        {
            chainNeck->setAng(i,fbTorso[i]);
            chainEyeL->setAng(i,fbTorso[i]);
            chainEyeR->setAng(i,fbTorso[i]);
        }
        for (int i=0; i<3; i++)
        {
            chainNeck->setAng(nJointsTorso+i,fbHead[i]);
            chainEyeL->setAng(nJointsTorso+i,fbHead[i]);
            chainEyeR->setAng(nJointsTorso+i,fbHead[i]);
        }

        chainEyeL->setAng(nJointsTorso+3,fbHead[3]);               
        chainEyeL->setAng(nJointsTorso+4,fbHead[4]+fbHead[5]/2.0);
        chainEyeR->setAng(nJointsTorso+3,fbHead[3]);
        chainEyeR->setAng(nJointsTorso+4,fbHead[4]-fbHead[5]/2.0);

        txInfo_pose.update(q_stamp);
        mutexChain.unlock();
    }

    IntState->reset(fbHead);

    fbNeck=fbHead.subVector(0,2);
    fbEyes=fbHead.subVector(3,5);

    double errNeck=norm(qd.subVector(0,2)-fbNeck);
    double errEyes=norm(qd.subVector(3,qd.length()-1)-fbEyes);
    bool swOffCond=(Time::now()-ctrlActiveRisingEdgeTime<GAZECTRL_SWOFFCOND_DISABLETIME) ? false :
                   (!commData->saccadeUnderway &&
                   (errNeck<GAZECTRL_MOTIONDONE_NECK_QTHRES*CTRL_DEG2RAD) &&
                   (errEyes<GAZECTRL_MOTIONDONE_EYES_QTHRES*CTRL_DEG2RAD));

    // verify control switching conditions
    if (commData->ctrlActive)
    {
        // manage new target while controller is active
        if (commData->port_xd->get_new())
        {
            reliableGyro=true;

            event="motion-onset";

            mutexData.lock();
            motionOngoingEventsCurrent=motionOngoingEvents;
            mutexData.unlock();

            commData->port_xd->get_new()=false;
        }
        // switch-off condition
        else if (swOffCond)
        {
            if (commData->trackingModeOn)
            {
                if (!motionDone)
                    event="motion-done"; 
                motionDone=true;
            }
            else
            {
                event="motion-done";
                mutexCtrl.lock(); 
                stopLimb(false);
                mutexCtrl.unlock();
            }
        }        
    }
    else if (jointsHealthy)
    {
        // inhibition is cleared upon new target arrival
        if (ctrlInhibited)
            ctrlInhibited=!commData->port_xd->get_new();

        // switch-on condition
        commData->ctrlActive=commData->port_xd->get_new() || (!ctrlInhibited && (commData->neckSolveCnt>0));

        // reset controllers
        if (commData->ctrlActive)
        {
            ctrlActiveRisingEdgeTime=Time::now();
            commData->port_xd->get_new()=false;
            commData->neckSolveCnt=0;

            if (stabilizeGaze)
            {
                mjCtrlNeck->reset(vNeck);
                mjCtrlEyes->reset(vEyes);
            }
            else
            {
                mjCtrlNeck->reset(zeros(fbNeck.length())); 
                mjCtrlEyes->reset(zeros(fbEyes.length()));
                IntStabilizer->reset(zeros(vNeck.length()));
                IntPlan->reset(fbNeck);
            }
            
            reliableGyro=true;

            event="motion-onset";

            mutexData.lock();
            motionOngoingEventsCurrent=motionOngoingEvents;
            mutexData.unlock();
        }
    }

    mutexCtrl.lock();
    if (event=="motion-onset")
    {
        setJointsCtrlMode();
        jointsToSet.clear();
        motionDone=false;
        q0=fbHead;
    }
    mutexCtrl.unlock();

    if (commData->trackingModeOn || stabilizeGaze)
    {
        mutexCtrl.lock();
        setJointsCtrlMode();
        mutexCtrl.unlock();
    }

    qdNeck=qd.subVector(0,2);
    qdEyes=qd.subVector(3,5);

    // compute current point [%] in the path
    double dist=norm(qd-q0);
    pathPerc=(dist>IKIN_ALMOST_ZERO)?norm(fbHead-q0)/dist:1.0;
    pathPerc=sat(pathPerc,0.0,1.0);
    
    if (commData->ctrlActive)
    {
        // control
        vNeck=mjCtrlNeck->computeCmd(neckTime,qdNeck-fbNeck);
        
        if (unplugCtrlEyes)
        {
            if (Time::now()-saccadeStartTime>=Ts)
                vEyes=commData->get_counterv();
        }
        else
            vEyes=mjCtrlEyes->computeCmd(eyesTime,qdEyes-fbEyes)+commData->get_counterv();

        // stabilization
        if (commData->stabilizationOn)
        {
            Vector gyro=CTRL_DEG2RAD*commData->get_imu().subVector(6,8);
            Vector dx=computedxFP(imu->getH(cat(fbTorso,fbNeck)),zeros(fbNeck.length()),gyro,x);
            Vector imuNeck=computeNeckVelFromdxFP(x,dx);

            if (reliableGyro)
            {
                vNeck=commData->stabilizationGain*IntStabilizer->integrate(vNeck-imuNeck);

                // only if the speed is low and we are close to the target
                if ((norm(vNeck)<commData->gyro_noise_threshold) && (pathPerc>0.9))
                    reliableGyro=false;
            }
            // hysteresis
            else if ((norm(imuNeck)>1.5*commData->gyro_noise_threshold) || (pathPerc<0.9))
            {
                IntStabilizer->reset(zeros(vNeck.length()));
                reliableGyro=true;
            }
        }

        IntPlan->integrate(vNeck);
    }
    else if (stabilizeGaze)
    {
        Vector gyro=CTRL_DEG2RAD*commData->get_imu().subVector(6,8);
        Vector dx=computedxFP(imu->getH(cat(fbTorso,fbNeck)),zeros(fbNeck.length()),gyro,x);
        Vector imuNeck=computeNeckVelFromdxFP(x,dx);

        vNeck=commData->stabilizationGain*IntStabilizer->integrate(-1.0*imuNeck);
        vEyes=computeEyesVelFromdxFP(dx);

        IntPlan->integrate(vNeck);
    }
    else 
    {
        vNeck=0.0;
        vEyes=0.0;
    }

    v.setSubvector(0,vNeck);
    v.setSubvector(3,vEyes);

    // apply bang-bang just in case to compensate
    // for unachievable low velocities
    for (size_t i=0; i<v.length(); i++)
        if ((v[i]!=0.0) && (v[i]>-min_abs_vel) && (v[i]<min_abs_vel))
            v[i]=yarp::math::sign(qd[i]-fbHead[i])*min_abs_vel;

    // convert to degrees
    mutexData.lock();
    qddeg=CTRL_RAD2DEG*qd;
    qdeg =CTRL_RAD2DEG*fbHead;
    vdeg =CTRL_RAD2DEG*v;
    mutexData.unlock();

    // send commands to the robot
    if (commData->ctrlActive || stabilizeGaze)
    {
        mutexCtrl.lock();

        Vector posdeg;
        if (commData->neckPosCtrlOn)
        {
            posdeg=(CTRL_RAD2DEG)*IntPlan->get();
            posNeck->setPositions(neckJoints.size(),neckJoints.getFirst(),posdeg.data());
            velHead->velocityMove(eyesJoints.size(),eyesJoints.getFirst(),vdeg.subVector(3,5).data());
        }
        else
            velHead->velocityMove(vdeg.data());

        if (commData->debugInfoEnabled && (port_debug.getOutputCount()>0))
        {
            Bottle info;
            int j=0;

            if (commData->neckPosCtrlOn)
            {
                for (size_t i=0; i<posdeg.length(); i++)
                {
                    ostringstream ss;
                    ss<<"pos_"<<i;
                    info.addString(ss.str().c_str());
                    info.addDouble(posdeg[i]);
                }

                j=eyesJoints[0];
            }

            for (size_t i=j; i<vdeg.length(); i++)
            {
                ostringstream ss;
                ss<<"vel_"<<i;
                info.addString(ss.str().c_str());
                info.addDouble(vdeg[i]);
            }

            port_debug.prepare()=info;
            txInfo_debug.update(q_stamp);
            port_debug.setEnvelope(txInfo_debug);
            port_debug.writeStrict();
        }

        mutexCtrl.unlock();
    }

    // print info
    if (commData->verbose)
        printIter(xd,x,qddeg,qdeg,vdeg,1.0); 

    // send x,q through YARP ports
    Vector q(nJointsTorso+nJointsHead);
    int j;
    for (j=0; j<nJointsTorso; j++)
        q[j]=CTRL_RAD2DEG*fbTorso[j];
    for (; (size_t)j<q.length(); j++)
        q[j]=qdeg[j-nJointsTorso];

    txInfo_x.update(x_stamp);
    if (port_x.getOutputCount()>0)
    {
        port_x.prepare()=x;
        port_x.setEnvelope(txInfo_x);
        port_x.write();
    }

    txInfo_q.update(q_stamp);
    if (port_q.getOutputCount()>0)
    {
        port_q.prepare()=q;
        port_q.setEnvelope(txInfo_q);
        port_q.write();
    }

    if (event=="motion-onset")
        notifyEvent(event);

    motionOngoingEventsHandling();

    if (event=="motion-done")
    {
        motionOngoingEventsFlush();
        notifyEvent(event);
    }

    // update joints angles
    fbHead=IntState->integrate(v);
    commData->set_q(fbHead);
    commData->set_torso(fbTorso);
    commData->set_v(v);
}


/************************************************************************/
void Controller::threadRelease()
{
    stopLimb();
    notifyEvent("closing");

    port_x.interrupt();
    port_x.close();

    port_q.interrupt();
    port_q.close();

    port_event.interrupt();
    port_event.close();

    if (commData->debugInfoEnabled)
    {
        port_debug.interrupt();
        port_debug.close();
    }

    delete neck;
    delete eyeL;
    delete eyeR;
    delete imu;
    delete mjCtrlNeck;
    delete mjCtrlEyes;
    delete IntState;
    delete IntPlan;
    delete IntStabilizer;
}


/************************************************************************/
void Controller::suspend()
{
    mutexCtrl.lock();
    RateThread::suspend();    
    stopLimb();
    commData->saccadeUnderway=false;
    yInfo("Controller has been suspended!");
    notifyEvent("suspended");
    mutexCtrl.unlock();
}


/************************************************************************/
void Controller::resume()
{
    getFeedback(fbTorso,fbHead,drvTorso,drvHead,commData);
    fbNeck=fbHead.subVector(0,2);
    fbEyes=fbHead.subVector(3,5);
    
    RateThread::resume();
    yInfo("Controller has been resumed!");
    notifyEvent("resumed");
}


/************************************************************************/
double Controller::getTneck() const
{
    return neckTime;
}


/************************************************************************/
double Controller::getTeyes() const
{
    return eyesTime;
}


/************************************************************************/
void Controller::setTneck(const double execTime)
{
    double lowerThresNeck=eyesTime+0.2;
    if (execTime<lowerThresNeck)
    {        
        yWarning("neck execution time is under the lower bound!");
        yWarning("a new neck execution time of %g s is chosen",lowerThresNeck);
        neckTime=lowerThresNeck;
    }
    else
        neckTime=execTime;
}


/************************************************************************/
void Controller::setTeyes(const double execTime)
{
    double lowerThresEyes=10.0*Ts;
    if (execTime<lowerThresEyes)
    {        
        yWarning("eyes execution time is under the lower bound!");
        yWarning("a new eyes execution time of %g s is chosen",lowerThresEyes);
        eyesTime=lowerThresEyes;
    }
    else
        eyesTime=execTime;
}


/************************************************************************/
bool Controller::isMotionDone()
{
    LockGuard guard(mutexRun);
    return motionDone;
}


/************************************************************************/
void Controller::setTrackingMode(const bool f)
{
    commData->trackingModeOn=f;
    if (commData->trackingModeOn)
        commData->port_xd->set_xd(commData->get_x());
}


/************************************************************************/
bool Controller::getTrackingMode() const 
{
    return commData->trackingModeOn;
}


/************************************************************************/
bool Controller::getDesired(Vector &des)
{
    mutexData.lock();
    des=qddeg;
    mutexData.unlock();
    return true;
}


/************************************************************************/
bool Controller::getVelocity(Vector &vel)
{
    mutexData.lock();
    vel=vdeg;
    mutexData.unlock();
    return true;
}


/************************************************************************/
bool Controller::getPose(const string &poseSel, Vector &x, Stamp &stamp)
{
    if (poseSel=="left")
    {
        mutexChain.lock();
        x=chainEyeL->EndEffPose();
        stamp=txInfo_pose;
        mutexChain.unlock();
        return true;
    }
    else if (poseSel=="right")
    {
        mutexChain.lock();
        x=chainEyeR->EndEffPose();
        stamp=txInfo_pose;
        mutexChain.unlock();
        return true;
    }
    else if (poseSel=="head")
    {
        mutexChain.lock();
        x=chainNeck->EndEffPose();
        stamp=txInfo_pose;
        mutexChain.unlock();
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool Controller::registerMotionOngoingEvent(const double checkPoint)
{
    if ((checkPoint>=0.0) && (checkPoint<=1.0))
    {
        mutexData.lock();
        motionOngoingEvents.insert(checkPoint);
        mutexData.unlock();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool Controller::unregisterMotionOngoingEvent(const double checkPoint)
{
    bool ret=false;
    if ((checkPoint>=0.0) && (checkPoint<=1.0))
    {
        mutexData.lock();
        multiset<double>::iterator itr=motionOngoingEvents.find(checkPoint);
        if (itr!=motionOngoingEvents.end())
        {
            motionOngoingEvents.erase(itr);
            ret=true;
        }
        mutexData.unlock();
    }

    return ret;
}


/************************************************************************/
Bottle Controller::listMotionOngoingEvents()
{
    Bottle events;

    mutexData.lock();
    for (multiset<double>::iterator itr=motionOngoingEvents.begin(); itr!=motionOngoingEvents.end(); itr++)
        events.addDouble(*itr);
    mutexData.unlock();

    return events;
}


