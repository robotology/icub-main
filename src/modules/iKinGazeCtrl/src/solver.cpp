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

#include <gsl/gsl_math.h>

#include <yarp/math/SVD.h>
#include <iCub/solver.h>


/************************************************************************/
EyePinvRefGen::EyePinvRefGen(PolyDriver *_drvTorso, PolyDriver *_drvHead,
                             exchangeData *_commData, Controller *_ctrl,
                             const bool _saccadesOn, const Vector &_counterRotGain,
                             const unsigned int _period) :
                             RateThread(_period),     drvTorso(_drvTorso), drvHead(_drvHead),
                             commData(_commData),     ctrl(_ctrl),         eyesBoundVer(-1.0),
                             saccadesOn(_saccadesOn), period(_period),     Ts(_period/1000.0),
                             counterRotGain(_counterRotGain)
{
    // Instantiate objects
    neck=new iCubHeadCenter(commData->head_version>1.0?"right_v2":"right");
    eyeL=new iCubEye(commData->head_version>1.0?"left_v2":"left");
    eyeR=new iCubEye(commData->head_version>1.0?"right_v2":"right");
    imu=new iCubInertialSensor(commData->head_version>1.0?"v2":"v1");

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
    lim=alignJointsBounds(chainNeck,drvTorso,drvHead,commData->eyeTiltMin,commData->eyeTiltMax);
    copyJointsBounds(chainNeck,chainEyeL);
    copyJointsBounds(chainEyeL,chainEyeR);

    // just eye part is required
    lim=lim.submatrix(3,5,0,1);

    // reinforce vergence min bound
    lim(2,0)=commData->get_minAllowedVergence();

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

    gyro.resize(12,0.0);
    fp.resize(3,0.0);
    eyesJ.resize(3,3);
    eyesJ.zero();

    genOn=false;
    saccadeUnderWayOld=false;
    saccadesInhibitionPeriod=SACCADES_INHIBITION_PERIOD;
    saccadesActivationAngle=SACCADES_ACTIVATION_ANGLE;
    port_xd=NULL;
}


/************************************************************************/
void EyePinvRefGen::minAllowedVergenceChanged()
{
    lim(2,0)=commData->get_minAllowedVergence();    
    I->setLim(lim);
    orig_lim=lim;
}


/************************************************************************/
bool EyePinvRefGen::getGyro(Vector &data)
{
    if (port_inertial.getInputCount()>0)
    {
        mutex.lock();
        data=gyro;
        mutex.unlock();
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool EyePinvRefGen::bindEyes(const double ver)
{
    if (ver>=0.0)
    {
        double ver_rad=std::max(CTRL_DEG2RAD*ver,commData->get_minAllowedVergence());
        eyesBoundVer=CTRL_RAD2DEG*ver_rad;

        // block tilt and pan of the left eye
        (*chainEyeL)[nJointsTorso+3].setMin(0.0);          (*chainEyeL)[nJointsTorso+3].setMax(0.0);
        (*chainEyeL)[nJointsTorso+4].setMin(ver_rad/2.0);  (*chainEyeL)[nJointsTorso+4].setMax(ver_rad/2.0);

        // block tilt and pan of the right eye
        (*chainEyeR)[nJointsTorso+3].setMin(0.0);          (*chainEyeR)[nJointsTorso+3].setMax(0.0);
        (*chainEyeR)[nJointsTorso+4].setMin(-ver_rad/2.0); (*chainEyeR)[nJointsTorso+4].setMax(-ver_rad/2.0);

        // change the limits of the integrator
        lim(0,0)=lim(0,1)=0.0;
        lim(1,0)=lim(1,1)=0.0;
        lim(2,0)=lim(2,1)=ver_rad;
        I->setLim(lim);

        port_xd->set_xd(fp);
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool EyePinvRefGen::clearEyes()
{
    if (eyesBoundVer>=0.0)
    {
        eyesBoundVer=-1.0;

        // reinstate tilt and pan bound of the left eye
        (*chainEyeL)[nJointsTorso+3].setMin(orig_eye_tilt_min); (*chainEyeL)[nJointsTorso+3].setMax(orig_eye_tilt_max);
        (*chainEyeL)[nJointsTorso+4].setMin(orig_eye_pan_min);  (*chainEyeL)[nJointsTorso+4].setMax(orig_eye_pan_max);

        // reinstate tilt and pan bound of the right eye
        (*chainEyeR)[nJointsTorso+3].setMin(orig_eye_tilt_min); (*chainEyeR)[nJointsTorso+3].setMax(orig_eye_tilt_max);
        (*chainEyeR)[nJointsTorso+4].setMin(orig_eye_pan_min);  (*chainEyeR)[nJointsTorso+4].setMax(orig_eye_pan_max);

        // reinstate the limits of the integrator
        lim=orig_lim;
        I->setLim(lim);
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
void EyePinvRefGen::setCounterRotGain(const Vector &gain)
{
    size_t len=std::min(counterRotGain.length(),gain.length());
    for (size_t i=0; i<len; i++)
        counterRotGain[i]=gain[i];
}


/************************************************************************/
Vector EyePinvRefGen::getEyesCounterVelocity(const Matrix &eyesJ, const Vector &fp)
{
    // ********** implement VOR
    Vector q(imu->getDOF());
    q[0]=fbTorso[0];
    q[1]=fbTorso[1];
    q[2]=fbTorso[2];
    q[3]=fbHead[0];
    q[4]=fbHead[1];
    q[5]=fbHead[2];
    Matrix H=imu->getH(q);

    H(0,3)=fp[0]-H(0,3);
    H(1,3)=fp[1]-H(1,3);
    H(2,3)=fp[2]-H(2,3);

    // gyro rate [deg/s]
    mutex.lock();
    double gyrX=gyro[6];
    double gyrY=gyro[7];
    double gyrZ=gyro[8];
    mutex.unlock();

    // filter out the noise on the gyro readouts
    Vector vor_fprelv;
    if ((fabs(gyrX)<GYRO_BIAS_STABILITY) && (fabs(gyrY)<GYRO_BIAS_STABILITY) &&
        (fabs(gyrZ)<GYRO_BIAS_STABILITY))
        vor_fprelv.resize(eyesJ.rows(),0.0);    // pinv(eyesJ) => use rows
    else
        vor_fprelv=CTRL_DEG2RAD*(gyrX*cross(H,0,H,3)+gyrY*cross(H,1,H,3)+gyrZ*cross(H,2,H,3));

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
    string robotPortInertial=("/"+commData->robotName+"/inertial");
    port_inertial.open((commData->localStemName+"/inertial:i").c_str());
    if (!Network::connect(robotPortInertial.c_str(),port_inertial.getName().c_str()))
    {
        counterRotGain[0]=0.0; counterRotGain[1]=1.0;
        printf("Unable to connect to %s => (vor,ocr) gains = (%s)\n",
               robotPortInertial.c_str(),counterRotGain.toString(3,3).c_str());
    }

    printf("Starting Pseudoinverse Reference Generator at %d ms\n",period);

    saccadesRxTargets=0;
    saccadesClock=Time::now();

    return true;
}


/************************************************************************/
void EyePinvRefGen::afterStart(bool s)
{
    s?printf("Pseudoinverse Reference Generator started successfully\n"):
      printf("Pseudoinverse Reference Generator did not start\n");
}


/************************************************************************/
void EyePinvRefGen::run()
{
    if (genOn)
    {
        double timeStamp;

        // read encoders
        getFeedback(fbTorso,fbHead,drvTorso,drvHead,commData,&timeStamp);
        updateTorsoBlockedJoints(chainNeck,fbTorso);
        updateTorsoBlockedJoints(chainEyeL,fbTorso);
        updateTorsoBlockedJoints(chainEyeR,fbTorso);

        // read gyro data
        if (Vector *_gyro=port_inertial.read(false))
        {
            mutex.lock();
            gyro=*_gyro;
            mutex.unlock();
        }

        // get current target
        Vector xd=port_xd->get_xd();

        // update neck chain
        chainNeck->setAng(nJointsTorso+0,fbHead[0]);
        chainNeck->setAng(nJointsTorso+1,fbHead[1]);
        chainNeck->setAng(nJointsTorso+2,fbHead[2]);

        // ask for saccades (if possible)
        if (saccadesOn && (saccadesRxTargets!=port_xd->get_rx()) && !commData->get_isSaccadeUnderway() &&
            (Time::now()-saccadesClock>saccadesInhibitionPeriod))
        {
            Vector fph=xd; fph.push_back(1.0);
            fph=SE3inv(chainNeck->getH())*fph; fph[3]=0.0;
            double rot=CTRL_RAD2DEG*acos(fph[2]/norm(fph)); fph[3]=1.0;

            // estimate geometrically the target tilt and pan of the eyes
            Vector ang(3,0.0);
            ang[0]=-atan2(fph[1],fabs(fph[2]));
            ang[1]=atan2(fph[0],fph[2]);

            // enforce joints bounds
            ang[0]=std::min(std::max(lim(0,0),ang[0]),lim(0,1));
            ang[1]=std::min(std::max(lim(1,0),ang[1]),lim(1,1));

            // favor the smooth-pursuit in case saccades are small
            if (rot>saccadesActivationAngle)
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
                ang[2]=std::min(std::max(lim(2,0),ang[2]),lim(2,1));

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
            if ((eyesBoundVer>=0.0) || !CartesianHelper::computeFixationPointData(*chainEyeL,*chainEyeR,fp,eyesJ))
                commData->set_counterv(zeros(3));
            else
                commData->set_counterv(getEyesCounterVelocity(eyesJ,fp));
            
            // reset eyes controller and integral upon saccades transition on=>off
            if (saccadeUnderWayOld && !commData->get_isSaccadeUnderway())
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
            commData->set_counterv(zeros(3));

        // set a new target position
        commData->set_xd(xd);
        commData->set_x(fp,timeStamp);
        commData->set_fpFrame(chainNeck->getH());
        if (!commData->get_isSaccadeUnderway())
        {
            commData->set_qd(3,qd[0]);
            commData->set_qd(4,qd[1]);
            commData->set_qd(5,qd[2]);
        }

        // latch the saccades status
        saccadeUnderWayOld=commData->get_isSaccadeUnderway();
        saccadesRxTargets=port_xd->get_rx();
    }
}


/************************************************************************/
void EyePinvRefGen::threadRelease()
{
    port_inertial.interrupt();
    port_inertial.close();

    delete neck;
    delete eyeL;
    delete eyeR;
    delete imu;
    delete I;
}


/************************************************************************/
void EyePinvRefGen::suspend()
{    
    RateThread::suspend();
    printf("Pseudoinverse Reference Generator has been suspended!\n");
}


/************************************************************************/
void EyePinvRefGen::resume()
{    
    RateThread::resume();
    printf("Pseudoinverse Reference Generator has been resumed!\n");
}


/************************************************************************/
Solver::Solver(PolyDriver *_drvTorso, PolyDriver *_drvHead, exchangeData *_commData,
               EyePinvRefGen *_eyesRefGen, Localizer *_loc, Controller *_ctrl,
               const unsigned int _period) :
               RateThread(_period), drvTorso(_drvTorso),     drvHead(_drvHead),
               commData(_commData), eyesRefGen(_eyesRefGen), loc(_loc),
               ctrl(_ctrl),         period(_period),         Ts(_period/1000.0)
{
    // Instantiate objects
    neck=new iCubHeadCenter(commData->head_version>1.0?"right_v2":"right");
    eyeL=new iCubEye(commData->head_version>1.0?"left_v2":"left");
    eyeR=new iCubEye(commData->head_version>1.0?"right_v2":"right");
    imu=new iCubInertialSensor(commData->head_version>1.0?"v2":"v1");

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
    alignJointsBounds(chainNeck,drvTorso,drvHead,commData->eyeTiltMin,commData->eyeTiltMax);

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

    gDefaultDir.resize(4,0.0);
    gDefaultDir[2]=gDefaultDir[3]=1.0;

    fbTorsoOld=fbTorso;
    fbHeadOld=fbHead;

    solveRequest=false;
    neckAngleUserTolerance=0.0;
}


/************************************************************************/
void Solver::bindNeckPitch(const double min_deg, const double max_deg)
{
    double min_rad=sat(CTRL_DEG2RAD*min_deg,neckPitchMin,neckPitchMax);
    double max_rad=sat(CTRL_DEG2RAD*max_deg,neckPitchMin,neckPitchMax);
    double cur_rad=(*chainNeck)(0).getAng();

    solveRequest=(cur_rad<min_rad) || (cur_rad>max_rad);

    mutex.lock();
    (*chainNeck)(0).setMin(min_rad);
    (*chainNeck)(0).setMax(max_rad);    
    mutex.unlock();

    printf("\nneck pitch constrained in [%g,%g] deg\n\n",min_deg,max_deg);
}


/************************************************************************/
void Solver::bindNeckRoll(const double min_deg, const double max_deg)
{
    double min_rad=sat(CTRL_DEG2RAD*min_deg,neckRollMin,neckRollMax);
    double max_rad=sat(CTRL_DEG2RAD*max_deg,neckRollMin,neckRollMax);
    double cur_rad=(*chainNeck)(1).getAng();

    solveRequest=(cur_rad<min_rad) || (cur_rad>max_rad);

    mutex.lock();
    (*chainNeck)(1).setMin(min_rad);
    (*chainNeck)(1).setMax(max_rad);
    mutex.unlock();

    printf("\nneck roll constrained in [%g,%g] deg\n\n",min_deg,max_deg);
}


/************************************************************************/
void Solver::bindNeckYaw(const double min_deg, const double max_deg)
{
    double min_rad=sat(CTRL_DEG2RAD*min_deg,neckYawMin,neckYawMax);
    double max_rad=sat(CTRL_DEG2RAD*max_deg,neckYawMin,neckYawMax);
    double cur_rad=(*chainNeck)(2).getAng();

    solveRequest=(cur_rad<min_rad) || (cur_rad>max_rad);

    mutex.lock();
    (*chainNeck)(2).setMin(min_rad);
    (*chainNeck)(2).setMax(max_rad);
    mutex.unlock();

    printf("\nneck yaw constrained in [%g,%g] deg\n\n",min_deg,max_deg);
}


/************************************************************************/
void Solver::getCurNeckPitchRange(double &min_deg, double &max_deg)
{
    mutex.lock();
    min_deg=CTRL_RAD2DEG*(*chainNeck)(0).getMin();
    max_deg=CTRL_RAD2DEG*(*chainNeck)(0).getMax();
    mutex.unlock();
}


/************************************************************************/
void Solver::getCurNeckRollRange(double &min_deg, double &max_deg)
{
    mutex.lock();
    min_deg=CTRL_RAD2DEG*(*chainNeck)(1).getMin();
    max_deg=CTRL_RAD2DEG*(*chainNeck)(1).getMax();
    mutex.unlock();
}


/************************************************************************/
void Solver::getCurNeckYawRange(double &min_deg, double &max_deg)
{
    mutex.lock();
    min_deg=CTRL_RAD2DEG*(*chainNeck)(2).getMin();
    max_deg=CTRL_RAD2DEG*(*chainNeck)(2).getMax();
    mutex.unlock();
}


/************************************************************************/
void Solver::clearNeckPitch()
{
    mutex.lock();
    (*chainNeck)(0).setMin(neckPitchMin);
    (*chainNeck)(0).setMax(neckPitchMax);
    mutex.unlock();

    printf("\nneck pitch cleared\n\n");
}


/************************************************************************/
void Solver::clearNeckRoll()
{
    mutex.lock();
    (*chainNeck)(1).setMin(neckRollMin);
    (*chainNeck)(1).setMax(neckRollMax);
    mutex.unlock();

    printf("\nneck roll cleared\n\n");
}


/************************************************************************/
void Solver::clearNeckYaw()
{
    mutex.lock();
    (*chainNeck)(2).setMin(neckYawMin);
    (*chainNeck)(2).setMax(neckYawMax);
    mutex.unlock();

    printf("\nneck yaw cleared\n\n");
}


/************************************************************************/
double Solver::getNeckAngleUserTolerance()
{
    return neckAngleUserTolerance;
}


/************************************************************************/
void Solver::setNeckAngleUserTolerance(const double angle)
{
    double fangle=fabs(angle);
    solveRequest=fangle<neckAngleUserTolerance;
    neckAngleUserTolerance=fangle;
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
Vector Solver::getGravityDirection(const Vector &gyro)
{
    double roll =CTRL_DEG2RAD*gyro[0];
    double pitch=CTRL_DEG2RAD*gyro[1];

    // compute rotational matrix to
    // account for roll and pitch
    Vector x(4), y(4);
    x[0]=1.0;    y[0]=0.0;
    x[1]=0.0;    y[1]=1.0;
    x[2]=0.0;    y[2]=0.0;
    x[3]=roll;   y[3]=pitch;   
    Matrix R=axis2dcm(y)*axis2dcm(x);

    Vector q(imu->getDOF());
    q[0]=fbTorso[0];
    q[1]=fbTorso[1];
    q[2]=fbTorso[2];
    q[3]=fbHead[0];
    q[4]=fbHead[1];
    q[5]=fbHead[2];
    Matrix H=imu->getH(q)*R.transposed();

    // gravity is aligned along the z-axis
    Vector gDir=H.getCol(2);
    gDir[3]=1.0;    // impose homogeneous coordinates

    return gDir;
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
    // Instantiate optimizer
    invNeck=new GazeIpOptMin(*chainNeck,1e-3,20);

    // Initialization
    Vector fp(3);
    Matrix J(3,3);
    CartesianHelper::computeFixationPointData(*chainEyeL,*chainEyeR,fp,J);

    // init commData structure
    commData->set_xd(fp);
    commData->set_qd(fbHead);
    commData->set_x(fp);
    commData->set_q(fbHead);
    commData->set_torso(fbTorso);
    commData->resize_v(fbHead.length(),0.0);
    commData->resize_counterv(3,0.0);
    commData->set_fpFrame(chainNeck->getH());

    port_xd=new xdPort(fp,this);
    port_xd->useCallback();
    port_xd->open((commData->localStemName+"/xd:i").c_str());

    loc->set_xdport(port_xd);
    eyesRefGen->set_xdport(port_xd);
    ctrl->set_xdport(port_xd);

    // use eyes pseudoinverse reference generator
    eyesRefGen->enable();

    printf("Starting Solver at %d ms\n",period);
    return true;
}


/************************************************************************/
void Solver::afterStart(bool s)
{
    s?printf("Solver started successfully\n"):
      printf("Solver did not start\n");
}


/************************************************************************/
void Solver::run()
{
    mutex.lock();

    // get the current target
    Vector xd=port_xd->get_xdDelayed();

    // update the target straightaway 
    commData->set_xd(xd);

    bool torsoChanged=false;

    // read encoders
    getFeedback(fbTorso,fbHead,drvTorso,drvHead,commData);
    updateTorsoBlockedJoints(chainNeck,fbTorso);
    updateTorsoBlockedJoints(chainEyeL,fbTorso);
    updateTorsoBlockedJoints(chainEyeR,fbTorso);

    torsoChanged=norm(fbTorso-fbTorsoOld)>NECKSOLVER_ACTIVATIONANGLE_JOINTS*CTRL_DEG2RAD;
    bool headChanged=norm(fbHead-fbHeadOld)>NECKSOLVER_ACTIVATIONANGLE_JOINTS*CTRL_DEG2RAD;

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
    doSolve&=!(commData->get_isCtrlActive() && !torsoChanged);

    // 3) skip if controller is inactive and we are not in tracking mode
    doSolve&=!(!commData->get_isCtrlActive() && commData->get_canCtrlBeDisabled());

    // 4) skip if controller is inactive, we are in tracking mode and nothing has changed
    // note that the De Morgan's law has been applied hereafter
    doSolve&=commData->get_isCtrlActive() || commData->get_canCtrlBeDisabled() ||
             torsoChanged || headChanged;

    // 5) solve straightaway if we are in tracking mode and a solve request is raised
    doSolve|=!commData->get_canCtrlBeDisabled() && solveRequest;

    // 6) solve straightaway if the target has changed
    doSolve|=port_xd->get_newDelayed();

    // 7) skip if the angle to target is lower than the user tolerance
    doSolve&=theta>neckAngleUserTolerance;

    // clear triggers
    port_xd->get_newDelayed()=false;
    solveRequest=false;

    // call the solver for neck
    if (doSolve)
    {
        Vector  gyro;
        Vector  gDir;
        Vector *pgDir;

        if (eyesRefGen->getGyro(gyro))
        {
            gDir=getGravityDirection(gyro);
            pgDir=&gDir;
        }
        else
            pgDir=&gDefaultDir;

        Vector xdUserTol=computeTargetUserTolerance(xd);
        neckPos=invNeck->solve(neckPos,xdUserTol,*pgDir);

        // update neck pitch,roll,yaw
        commData->set_qd(0,neckPos[0]);
        commData->set_qd(1,neckPos[1]);
        commData->set_qd(2,neckPos[2]);
    }

    // latch quantities
    fbTorsoOld=fbTorso;
    fbHeadOld=fbHead;

    mutex.unlock();
}


/************************************************************************/
void Solver::threadRelease()
{
    eyesRefGen->disable();

    port_xd->interrupt();
    port_xd->close();

    delete port_xd;
    delete invNeck;
    delete neck;
    delete eyeL;
    delete eyeR;
    delete imu;
}


/************************************************************************/
void Solver::suspend()
{
    port_xd->lock();    
    RateThread::suspend();
    printf("Solver has been suspended!\n");
}


/************************************************************************/
void Solver::resume()
{
    mutex.lock();

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

    // compute fixation point
    Vector fp(3);
    Matrix J(3,3);
    CartesianHelper::computeFixationPointData(*chainEyeL,*chainEyeR,fp,J);

    // update latched quantities
    fbTorsoOld=fbTorso;
    fbHeadOld=fbHead;    
    
    mutex.unlock();

    port_xd->unlock();
    RateThread::resume();
    printf("Solver has been resumed!\n");
}



