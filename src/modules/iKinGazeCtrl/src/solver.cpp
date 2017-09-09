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
#include <algorithm>

#include <yarp/math/SVD.h>
#include <iCub/solver.h>


/************************************************************************/
EyePinvRefGen::EyePinvRefGen(PolyDriver *_drvTorso, PolyDriver *_drvHead,
                             ExchangeData *_commData, Controller *_ctrl,
                             const Vector &_counterRotGain, const unsigned int _period) :
                             RateThread(_period), drvTorso(_drvTorso), drvHead(_drvHead),
                             commData(_commData), ctrl(_ctrl),         period(_period),
                             Ts(_period/1000.0),  counterRotGain(_counterRotGain)
{
    // Instantiate objects
    neck=new iCubHeadCenter("right_"+commData->headVersion2String());
    eyeL=new iCubEye("left_"+commData->headVersion2String());
    eyeR=new iCubEye("right_"+commData->headVersion2String());
    imu=new iCubInertialSensor(commData->headVersion2String());

    // remove constraints on the links: logging purpose
    imu->setAllConstraints(false);

    // block neck dofs
    eyeL->blockLink(3,0.0); eyeR->blockLink(3,0.0);
    eyeL->blockLink(4,0.0); eyeR->blockLink(4,0.0);
    eyeL->blockLink(5,0.0); eyeR->blockLink(5,0.0);

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

    if (commData->tweakOverwrite)
    {
        getAlignHN(commData->rf_tweak,"ALIGN_KIN_LEFT",eyeL->asChain());
        getAlignHN(commData->rf_tweak,"ALIGN_KIN_RIGHT",eyeR->asChain());
    }

    // get the length of the half of the eyes baseline
    eyesHalfBaseline=0.5*norm(eyeL->EndEffPose().subVector(0,2)-eyeR->EndEffPose().subVector(0,2));
    
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

    // joints bounds alignment
    lim=alignJointsBounds(chainNeck,drvTorso,drvHead,commData->eyeTiltLim);
    copyJointsBounds(chainNeck,chainEyeL);
    copyJointsBounds(chainEyeL,chainEyeR);

    // just eye part is required
    lim=lim.submatrix(3,5,0,1);

    // reinforce vergence min bound
    lim(2,0)=commData->minAllowedVergence;

    // read starting position
    fbTorso.resize(nJointsTorso,0.0);
    fbHead.resize(nJointsHead,0.0);
    getFeedback(fbTorso,fbHead,drvTorso,drvHead,commData);

    // save original eyes tilt and pan bounds
    orig_eye_tilt_min=(*chainEyeL)[nJointsTorso+3].getMin();
    orig_eye_tilt_max=(*chainEyeL)[nJointsTorso+3].getMax();
    orig_eye_pan_min=(*chainEyeL)[nJointsTorso+4].getMin();
    orig_eye_pan_max=(*chainEyeL)[nJointsTorso+4].getMax();
    orig_lim=lim;

    // Instantiate integrator
    qd.resize(3);
    qd[0]=fbHead[3];
    qd[1]=fbHead[4];
    qd[2]=fbHead[5];
    I=new Integrator(Ts,qd,lim);

    fp.resize(3,0.0);
    eyesJ.resize(3,3);
    eyesJ.zero();

    genOn=false;
    saccadeUnderWayOld=false;
}


/************************************************************************/
EyePinvRefGen::~EyePinvRefGen()
{
    delete neck;
    delete eyeL;
    delete eyeR;
    delete imu;
    delete I;
}


/************************************************************************/
void EyePinvRefGen::minAllowedVergenceChanged()
{
    LockGuard lg(mutex);
    lim(2,0)=commData->minAllowedVergence;
    I->setLim(lim);
    orig_lim=lim;
}


/************************************************************************/
bool EyePinvRefGen::bindEyes(const double ver)
{
    LockGuard lg(mutex);
    if (ver>=0.0)
    {
        double ver_rad=std::max(CTRL_DEG2RAD*ver,commData->minAllowedVergence);
        commData->eyesBoundVer=CTRL_RAD2DEG*ver_rad;

        // block pan of the left eye
        (*chainEyeL)[nJointsTorso+4].setMin(ver_rad/2.0);
        (*chainEyeL)[nJointsTorso+4].setMax(ver_rad/2.0);

        // block pan of the right eye
        (*chainEyeR)[nJointsTorso+4].setMin(-ver_rad/2.0);
        (*chainEyeR)[nJointsTorso+4].setMax(-ver_rad/2.0);

        // change the limits of the integrator
        lim(1,0)=lim(1,1)=0.0;
        lim(2,0)=lim(2,1)=ver_rad;
        I->setLim(lim);

        qd[0]=fbHead[3];
        qd[1]=0.0;
        qd[2]=ver_rad;
        I->reset(qd);

        yInfo("eyes constrained at vergence %g deg",commData->eyesBoundVer);
        ctrl->look(fp);
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool EyePinvRefGen::clearEyes()
{
    LockGuard lg(mutex);
    if (commData->eyesBoundVer>=0.0)
    {        
        commData->eyesBoundVer=-1.0;

        // reinstate pan bound of the left eye
        (*chainEyeL)[nJointsTorso+4].setMin(orig_eye_pan_min);
        (*chainEyeL)[nJointsTorso+4].setMax(orig_eye_pan_max);

        // reinstate pan bound of the right eye
        (*chainEyeR)[nJointsTorso+4].setMin(orig_eye_pan_min);
        (*chainEyeR)[nJointsTorso+4].setMax(orig_eye_pan_max);

        // reinstate the limits of the integrator
        lim=orig_lim;
        I->setLim(lim);

        yInfo("eyes cleared");
        return true;
    }
    else
        return false;
}


/************************************************************************/
void EyePinvRefGen::manageBindEyes(const double ver)
{
    if (!bindEyes(ver))
        clearEyes();
}


/************************************************************************/
Vector EyePinvRefGen::getCounterRotGain()
{
    LockGuard lg(mutex);
    return counterRotGain;
}


/************************************************************************/
void EyePinvRefGen::setCounterRotGain(const Vector &gain)
{
    LockGuard lg(mutex);
    size_t len=std::min(counterRotGain.length(),gain.length());
    for (size_t i=0; i<len; i++)
        counterRotGain[i]=gain[i];
    yInfo("counter-rotation gains set to (%s)",counterRotGain.toString(3,3).c_str());
}


/************************************************************************/
Vector EyePinvRefGen::getEyesCounterVelocity(const Matrix &eyesJ, const Vector &fp)
{
    // ********** implement VOR
    Matrix H=imu->getH(cat(fbTorso,fbHead.subVector(0,2)));

    H(0,3)=fp[0]-H(0,3);
    H(1,3)=fp[1]-H(1,3);
    H(2,3)=fp[2]-H(2,3);

    // gyro rate [rad/s]
    Vector gyro=CTRL_DEG2RAD*commData->get_imu().subVector(6,8);

    // filter out the noise on the gyro readouts
    if (norm(gyro)<commData->gyro_noise_threshold)
        gyro=0.0;

    Vector vor_fprelv=(gyro[0]*cross(H,0,H,3)+
                       gyro[1]*cross(H,1,H,3)+
                       gyro[2]*cross(H,2,H,3));

    // ********** implement OCR
    H=chainNeck->getH();
    Matrix HN=eye(4,4);
    HN(0,3)=fp[0]-H(0,3);
    HN(1,3)=fp[1]-H(1,3);
    HN(2,3)=fp[2]-H(2,3);

    chainNeck->setHN(HN);
    Vector ocr_fprelv=chainNeck->GeoJacobian()*commData->get_v().subVector(0,2);
    ocr_fprelv=ocr_fprelv.subVector(0,2);
    chainNeck->setHN(eye(4,4));

    // ********** blend the contributions
    return -1.0*(pinv(eyesJ)*(counterRotGain[0]*vor_fprelv+counterRotGain[1]*ocr_fprelv));
}


/************************************************************************/
bool EyePinvRefGen::threadInit()
{
    yInfo("Starting Pseudoinverse Reference Generator at %d ms",period);

    saccadesRxTargets=0;
    saccadesClock=Time::now();

    return true;
}


/************************************************************************/
void EyePinvRefGen::threadRelease()
{
}


/************************************************************************/
void EyePinvRefGen::afterStart(bool s)
{
    if (s)
        yInfo("Pseudoinverse Reference Generator started successfully");
    else
        yError("Pseudoinverse Reference Generator did not start!");
}


/************************************************************************/
void EyePinvRefGen::run()
{
    if (genOn)
    {
        LockGuard lg(mutex);
        double timeStamp;

        // read encoders
        getFeedback(fbTorso,fbHead,drvTorso,drvHead,commData,&timeStamp);
        updateTorsoBlockedJoints(chainNeck,fbTorso);
        updateTorsoBlockedJoints(chainEyeL,fbTorso);
        updateTorsoBlockedJoints(chainEyeR,fbTorso);

        // get current target
        Vector xd=commData->port_xd->get_xd();

        // update neck chain
        chainNeck->setAng(nJointsTorso+0,fbHead[0]);
        chainNeck->setAng(nJointsTorso+1,fbHead[1]);
        chainNeck->setAng(nJointsTorso+2,fbHead[2]);

        // ask for saccades (if possible)
        if (commData->saccadesOn && (saccadesRxTargets!=commData->port_xd->get_rx()) &&
            !commData->saccadeUnderway && (Time::now()-saccadesClock>commData->saccadesInhibitionPeriod))
        {
            Vector fph=xd; fph.push_back(1.0);
            fph=SE3inv(chainNeck->getH())*fph; fph[3]=0.0;
            double rot=CTRL_RAD2DEG*acos(fph[2]/norm(fph)); fph[3]=1.0;

            // estimate geometrically the target tilt and pan of the eyes
            Vector ang(3,0.0);
            ang[0]=-atan2(fph[1],fabs(fph[2]));
            ang[1]=atan2(fph[0],fph[2]);

            // enforce joints bounds
            ang[0]=sat(ang[0],lim(0,0),lim(0,1));
            ang[1]=sat(ang[1],lim(1,0),lim(1,1));

            // favor the smooth-pursuit in case saccades are small
            if (rot>commData->saccadesActivationAngle)
            {
                // init vergence
                ang[2]=fbHead[5];

                // get rid of eyes tilt
                Vector axis(4);
                axis[0]=1.0; axis[1]=0.0; axis[2]=0.0; axis[3]=-ang[0];
                fph=axis2dcm(axis)*fph;

                // go on iff the point is in front of us
                if (fph[2]>0.0)
                {
                    double L,R;

                    // estimate geometrically the target vergence Vg=L-R                    
                    if (fph[0]>=eyesHalfBaseline)
                    {
                        L=M_PI/2.0-atan2(fph[2],fph[0]+eyesHalfBaseline);
                        R=M_PI/2.0-atan2(fph[2],fph[0]-eyesHalfBaseline);
                    }
                    else if (fph[0]>-eyesHalfBaseline)
                    {
                        L=M_PI/2.0-atan2(fph[2],fph[0]+eyesHalfBaseline);
                        R=-(M_PI/2.0-atan2(fph[2],eyesHalfBaseline-fph[0]));
                    }
                    else
                    {
                        L=-(M_PI/2.0-atan2(fph[2],-fph[0]-eyesHalfBaseline));
                        R=-(M_PI/2.0-atan2(fph[2],eyesHalfBaseline-fph[0]));
                    }

                    ang[2]=L-R;
                }

                // enforce joints bounds
                ang[2]=sat(ang[2],lim(2,0),lim(2,1));

                commData->set_qd(3,ang[0]);
                commData->set_qd(4,ang[1]);
                commData->set_qd(5,ang[2]);

                Vector vel(3,SACCADES_VEL);
                ctrl->doSaccade(ang,vel);
                saccadesClock=Time::now();
            }
        }

        // update eyes chains for convergence purpose
        updateNeckBlockedJoints(chainEyeL,fbHead);         updateNeckBlockedJoints(chainEyeR,fbHead);
        chainEyeL->setAng(nJointsTorso+3,qd[0]);           chainEyeR->setAng(nJointsTorso+3,qd[0]);
        chainEyeL->setAng(nJointsTorso+4,qd[1]+qd[2]/2.0); chainEyeR->setAng(nJointsTorso+4,qd[1]-qd[2]/2.0);

        // converge on target
        if (CartesianHelper::computeFixationPointData(*chainEyeL,*chainEyeR,fp,eyesJ))
        {
            Vector v=EYEPINVREFGEN_GAIN*(pinv(eyesJ)*(xd-fp));

            // update eyes chains in actual configuration for velocity compensation
            chainEyeL->setAng(nJointsTorso+3,fbHead[3]);               chainEyeR->setAng(nJointsTorso+3,fbHead[3]);
            chainEyeL->setAng(nJointsTorso+4,fbHead[4]+fbHead[5]/2.0); chainEyeR->setAng(nJointsTorso+4,fbHead[4]-fbHead[5]/2.0);

            // compensate neck rotation at eyes level
            if ((commData->eyesBoundVer>=0.0) || !CartesianHelper::computeFixationPointData(*chainEyeL,*chainEyeR,fp,eyesJ))
                commData->set_counterv(zeros(qd.length()));
            else
                commData->set_counterv(getEyesCounterVelocity(eyesJ,fp));
            
            // reset eyes controller and integral upon saccades transition on=>off
            if (saccadeUnderWayOld && !commData->saccadeUnderway)
            {
                ctrl->resetCtrlEyes();

                qd[0]=fbHead[3];
                qd[1]=fbHead[4];
                qd[2]=fbHead[5];
                I->reset(qd);
            }

            // update reference
            qd=I->integrate(v+commData->get_counterv());
        }
        else
            commData->set_counterv(zeros(qd.length()));

        // set a new target position
        commData->set_xd(xd);
        commData->set_x(fp,timeStamp);
        commData->set_fpFrame(chainNeck->getH());
        if (!commData->saccadeUnderway)
        {
            commData->set_qd(3,qd[0]);
            commData->set_qd(4,qd[1]);
            commData->set_qd(5,qd[2]);
        }

        // latch the saccades status
        saccadeUnderWayOld=commData->saccadeUnderway;
        saccadesRxTargets=commData->port_xd->get_rx();
    }
}


/************************************************************************/
void EyePinvRefGen::suspend()
{    
    RateThread::suspend();
    yInfo("Pseudoinverse Reference Generator has been suspended!");
}


/************************************************************************/
void EyePinvRefGen::resume()
{    
    RateThread::resume();
    yInfo("Pseudoinverse Reference Generator has been resumed!");
}


/************************************************************************/
Solver::Solver(PolyDriver *_drvTorso, PolyDriver *_drvHead, ExchangeData *_commData,
               EyePinvRefGen *_eyesRefGen, Localizer *_loc, Controller *_ctrl,
               const unsigned int _period) :
               RateThread(_period), drvTorso(_drvTorso),     drvHead(_drvHead),
               commData(_commData), eyesRefGen(_eyesRefGen), loc(_loc),
               ctrl(_ctrl),         period(_period),         Ts(_period/1000.0)
{
    // Instantiate objects
    neck=new iCubHeadCenter("right_"+commData->headVersion2String());
    eyeL=new iCubEye("left_"+commData->headVersion2String());
    eyeR=new iCubEye("right_"+commData->headVersion2String());
    imu=new iCubInertialSensor(commData->headVersion2String());
    torsoVel=new AWLinEstimator(16,0.5);    

    // remove constraints on the links: logging purpose
    imu->setAllConstraints(false);

    // block neck dofs
    eyeL->blockLink(3,0.0); eyeR->blockLink(3,0.0);
    eyeL->blockLink(4,0.0); eyeR->blockLink(4,0.0);
    eyeL->blockLink(5,0.0); eyeR->blockLink(5,0.0);

    // Get the chain objects
    chainNeck=neck->asChain();
    chainEyeL=eyeL->asChain();        
    chainEyeR=eyeR->asChain();

    invNeck=new GazeIpOptMin(*chainNeck,1e-3,1e-3,20);

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

    // joints bounds alignment
    alignJointsBounds(chainNeck,drvTorso,drvHead,commData->eyeTiltLim);

    // read starting position
    fbTorso.resize(nJointsTorso,0.0);
    fbHead.resize(nJointsHead,0.0);
    getFeedback(fbTorso,fbHead,drvTorso,drvHead,commData);

    copyJointsBounds(chainNeck,chainEyeL);
    copyJointsBounds(chainEyeL,chainEyeR);

    // store neck pitch/roll/yaw bounds
    neckPitchMin=(*chainNeck)[3].getMin();
    neckPitchMax=(*chainNeck)[3].getMax();
    neckRollMin =(*chainNeck)[4].getMin();
    neckRollMax =(*chainNeck)[4].getMax();
    neckYawMin  =(*chainNeck)[5].getMin();
    neckYawMax  =(*chainNeck)[5].getMax();

    neckPos.resize(3);
    gazePos.resize(3);
    updateAngles();

    updateTorsoBlockedJoints(chainNeck,fbTorso);
    chainNeck->setAng(3,fbHead[0]);
    chainNeck->setAng(4,fbHead[1]);
    chainNeck->setAng(5,fbHead[2]);

    updateTorsoBlockedJoints(chainEyeL,fbTorso);
    updateTorsoBlockedJoints(chainEyeR,fbTorso);
    updateNeckBlockedJoints(chainEyeL,fbHead);
    updateNeckBlockedJoints(chainEyeR,fbHead);

    Vector eyePos(2);
    eyePos[0]=gazePos[0];
    eyePos[1]=gazePos[1]+gazePos[2]/2.0;
    chainEyeL->setAng(eyePos);
    eyePos[1]=gazePos[1]-gazePos[2]/2.0;
    chainEyeR->setAng(eyePos);

    neckAngleUserTolerance=0.0;
}


/************************************************************************/
Solver::~Solver()
{
    delete neck;
    delete eyeL;
    delete eyeR;
    delete imu;
    delete torsoVel;
    delete invNeck;
}


/************************************************************************/
void Solver::bindNeckPitch(const double min_deg, const double max_deg)
{
    LockGuard lg(mutex);
    double min_rad=sat(CTRL_DEG2RAD*min_deg,neckPitchMin,neckPitchMax);
    double max_rad=sat(CTRL_DEG2RAD*max_deg,neckPitchMin,neckPitchMax);

    (*chainNeck)(0).setMin(min_rad);
    (*chainNeck)(0).setMax(max_rad);    

    yInfo("neck pitch constrained in [%g,%g] deg",min_deg,max_deg);
}


/************************************************************************/
void Solver::bindNeckRoll(const double min_deg, const double max_deg)
{
    LockGuard lg(mutex);
    double min_rad=sat(CTRL_DEG2RAD*min_deg,neckRollMin,neckRollMax);
    double max_rad=sat(CTRL_DEG2RAD*max_deg,neckRollMin,neckRollMax);

    (*chainNeck)(1).setMin(min_rad);
    (*chainNeck)(1).setMax(max_rad);

    yInfo("neck roll constrained in [%g,%g] deg",min_deg,max_deg);
}


/************************************************************************/
void Solver::bindNeckYaw(const double min_deg, const double max_deg)
{
    LockGuard lg(mutex);
    double min_rad=sat(CTRL_DEG2RAD*min_deg,neckYawMin,neckYawMax);
    double max_rad=sat(CTRL_DEG2RAD*max_deg,neckYawMin,neckYawMax);

    (*chainNeck)(2).setMin(min_rad);
    (*chainNeck)(2).setMax(max_rad);

    yInfo("neck yaw constrained in [%g,%g] deg",min_deg,max_deg);
}


/************************************************************************/
void Solver::getCurNeckPitchRange(double &min_deg, double &max_deg)
{
    LockGuard lg(mutex);
    min_deg=CTRL_RAD2DEG*(*chainNeck)(0).getMin();
    max_deg=CTRL_RAD2DEG*(*chainNeck)(0).getMax();
}


/************************************************************************/
void Solver::getCurNeckRollRange(double &min_deg, double &max_deg)
{
    LockGuard lg(mutex);
    min_deg=CTRL_RAD2DEG*(*chainNeck)(1).getMin();
    max_deg=CTRL_RAD2DEG*(*chainNeck)(1).getMax();
}


/************************************************************************/
void Solver::getCurNeckYawRange(double &min_deg, double &max_deg)
{
    LockGuard lg(mutex);
    min_deg=CTRL_RAD2DEG*(*chainNeck)(2).getMin();
    max_deg=CTRL_RAD2DEG*(*chainNeck)(2).getMax();
}


/************************************************************************/
void Solver::clearNeckPitch()
{
    LockGuard lg(mutex);
    (*chainNeck)(0).setMin(neckPitchMin);
    (*chainNeck)(0).setMax(neckPitchMax);

    yInfo("neck pitch cleared");
}


/************************************************************************/
void Solver::clearNeckRoll()
{
    LockGuard lg(mutex);
    (*chainNeck)(1).setMin(neckRollMin);
    (*chainNeck)(1).setMax(neckRollMax);

    yInfo("neck roll cleared");
}


/************************************************************************/
void Solver::clearNeckYaw()
{
    LockGuard lg(mutex);
    (*chainNeck)(2).setMin(neckYawMin);
    (*chainNeck)(2).setMax(neckYawMax);

    yInfo("neck yaw cleared");
}


/************************************************************************/
double Solver::getNeckAngleUserTolerance() const
{
    return neckAngleUserTolerance;
}


/************************************************************************/
void Solver::setNeckAngleUserTolerance(const double angle)
{
    neckAngleUserTolerance=fabs(angle);
}


/************************************************************************/
void Solver::updateAngles()
{
    neckPos[0]=fbHead[0];
    neckPos[1]=fbHead[1];
    neckPos[2]=fbHead[2];
    gazePos[0]=fbHead[3];
    gazePos[1]=fbHead[4];
    gazePos[2]=fbHead[5];
}


/************************************************************************/
double Solver::neckTargetRotAngle(const Vector &xd)
{
    Matrix H=commData->get_fpFrame();
    Vector xdh=xd; xdh.push_back(1.0);
    xdh=SE3inv(H)*xdh; xdh[3]=0.0;

    return (CTRL_RAD2DEG*acos(xdh[2]/norm(xdh)));
}


/************************************************************************/
Vector Solver::computeTargetUserTolerance(const Vector &xd)
{
    Matrix H=commData->get_fpFrame();
    Vector z(3,0.0); z[2]=1.0;
    Vector xdh=xd; xdh.push_back(1.0);
    xdh=SE3inv(H)*xdh;

    Vector xdh3=xdh; xdh3.pop_back();
    Vector rot=cross(xdh3,z);
    double r=norm(rot);
    if (r<IKIN_ALMOST_ZERO)
        return xd;

    rot=rot/r;
    rot.push_back(neckAngleUserTolerance*CTRL_DEG2RAD);
    rot=H*(axis2dcm(rot)*xdh);
    rot.pop_back();

    return rot;
}


/************************************************************************/
bool Solver::threadInit()
{
    // Initialization
    Vector fp;
    CartesianHelper::computeFixationPointData(*chainEyeL,*chainEyeR,fp);    

    // init commData structure
    commData->port_xd->init(fp);
    commData->set_xd(fp);
    commData->set_qd(fbHead);
    commData->set_x(fp);
    commData->set_q(fbHead);
    commData->set_torso(fbTorso);
    commData->resize_v(fbHead.length(),0.0);
    commData->resize_counterv(3,0.0);
    commData->set_fpFrame(chainNeck->getH());    

    // use eyes pseudoinverse reference generator
    eyesRefGen->enable();

    yInfo("Starting Solver at %d ms",period);
    return true;
}


/************************************************************************/
void Solver::threadRelease()
{
    eyesRefGen->disable();
}


/************************************************************************/
void Solver::afterStart(bool s)
{
    if (s)
        yInfo("Solver started successfully");
    else
        yError("Solver did not start!");
}


/************************************************************************/
void Solver::run()
{
    typedef enum { ctrl_off, ctrl_wait, ctrl_on } cstate;
    static cstate state_=ctrl_off;

    LockGuard lg(mutex);

    // get the current target
    Vector xd=commData->port_xd->get_xdDelayed();

    // update the target straightaway 
    commData->set_xd(xd);

    // read encoders
    getFeedback(fbTorso,fbHead,drvTorso,drvHead,commData);
    updateTorsoBlockedJoints(chainNeck,fbTorso);
    updateTorsoBlockedJoints(chainEyeL,fbTorso);
    updateTorsoBlockedJoints(chainEyeR,fbTorso);

    torsoVel->feedData(AWPolyElement(fbTorso,Time::now()));
    bool torsoChanged=(norm(torsoVel->estimate())>NECKSOLVER_ACTIVATIONANGLE_JOINTS*CTRL_DEG2RAD);

    // update kinematics
    updateAngles();
    updateNeckBlockedJoints(chainEyeL,fbHead);
    updateNeckBlockedJoints(chainEyeR,fbHead);
    chainNeck->setAng(neckPos);

    // hereafter accumulate solving conditions: the order does matter

    // 1) compute the angular distance
    double theta=neckTargetRotAngle(xd);
    bool doSolve=(theta>NECKSOLVER_ACTIVATIONANGLE);

    // 2) skip if controller is active and no torso motion is detected
    doSolve&=!(commData->ctrlActive && !torsoChanged);

    // 3) skip if controller is inactive and we are not in tracking mode
    doSolve&=commData->ctrlActive || commData->trackingModeOn;

    // 4) solve straightaway if we are in tracking mode and torso motion is detected
    doSolve|=commData->trackingModeOn && torsoChanged;

    // 5) solve straightaway if the target has changed
    doSolve|=commData->port_xd->get_newDelayed();

    // 6) skip if the angle to target is lower than the user tolerance
    doSolve&=(theta>neckAngleUserTolerance);

    // clear triggers
    commData->port_xd->get_newDelayed()=false;

    // call the solver for neck
    if (doSolve)
    {
        Vector gDir(3,0.0); gDir[2]=-1.0;
        if (commData->stabilizationOn)
        {
            Vector acc=-1.0*commData->get_imu().subVector(3,5);
            Matrix H=imu->getH(cat(fbTorso,fbHead.subVector(0,2)));
            gDir=H.submatrix(0,2,0,2)*acc;
        }

        Vector xdUserTol=computeTargetUserTolerance(xd);
        neckPos=invNeck->solve(neckPos,xdUserTol,gDir);

        // update neck pitch,roll,yaw        
        commData->set_qd(0,neckPos[0]);
        commData->set_qd(1,neckPos[1]);
        commData->set_qd(2,neckPos[2]);
        commData->neckSolveCnt++;

        state_=ctrl_wait;
    }

    if (state_==ctrl_off)
    {
        // keep neck targets equal to current angles
        // to avoid glitches in the control,
        // especially during stabilization
        commData->set_qd(0,neckPos[0]);
        commData->set_qd(1,neckPos[1]);
        commData->set_qd(2,neckPos[2]);
    }
    else if (state_==ctrl_wait)
    {
        if (commData->ctrlActive)
            state_=ctrl_on;
    }
    else if (state_==ctrl_on)
    {
        if (!commData->ctrlActive)
            state_=ctrl_off;
    }
}


/************************************************************************/
void Solver::suspend()
{
    commData->port_xd->lock();
    RateThread::suspend();
    yInfo("Solver has been suspended!");
}


/************************************************************************/
void Solver::resume()
{
    LockGuard lg(mutex);

    // read encoders
    getFeedback(fbTorso,fbHead,drvTorso,drvHead,commData);
    updateTorsoBlockedJoints(chainNeck,fbTorso);
    updateTorsoBlockedJoints(chainEyeL,fbTorso);
    updateTorsoBlockedJoints(chainEyeR,fbTorso);        

    // update kinematics
    updateAngles();
    updateNeckBlockedJoints(chainEyeL,fbHead);
    updateNeckBlockedJoints(chainEyeR,fbHead);
    chainNeck->setAng(neckPos);
    chainEyeL->setAng(nJointsTorso+3,gazePos[0]);                chainEyeR->setAng(nJointsTorso+3,gazePos[0]);
    chainEyeL->setAng(nJointsTorso+4,gazePos[1]+gazePos[2]/2.0); chainEyeR->setAng(nJointsTorso+4,gazePos[1]-gazePos[2]/2.0);

    torsoVel->reset();       
    commData->port_xd->unlock();

    RateThread::resume();
    yInfo("Solver has been resumed!");
}



