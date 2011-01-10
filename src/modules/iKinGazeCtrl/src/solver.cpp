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

#include <yarp/math/SVD.h>
#include <iCub/solver.h>


/************************************************************************/
EyePinvRefGen::EyePinvRefGen(PolyDriver *_drvTorso, PolyDriver *_drvHead,
                             exchangeData *_commData, const string &_robotName,
                             const string &_localName, const string &_configFile,
                             const double _eyeTiltMin, const double _eyeTiltMax,
                             const bool _VOR, unsigned int _period) :
                             RateThread(_period),     drvTorso(_drvTorso),   drvHead(_drvHead),
                             commData(_commData),     robotName(_robotName), localName(_localName),
                             configFile(_configFile), period(_period),       eyeTiltMin(_eyeTiltMin),
                             eyeTiltMax(_eyeTiltMax), Ts(_period/1000.0)
{
    Robotable=(drvHead!=NULL);
    VOR=Robotable&&_VOR;

    // Instantiate objects
    neck=new iCubHeadCenter();
    eyeL=new iCubEye("left");
    eyeR=new iCubEye("right");

    // remove constraints on the links: logging purpose
    inertialSensor.setAllConstraints(false);

    // block neck dofs
    eyeL->blockLink(3,0.0); eyeR->blockLink(3,0.0);
    eyeL->blockLink(4,0.0); eyeR->blockLink(4,0.0);
    eyeL->blockLink(5,0.0); eyeR->blockLink(5,0.0);

    // Get the chain objects
    chainNeck=neck->asChain();
    chainEyeL=eyeL->asChain();
    chainEyeR=eyeR->asChain();

    // add aligning links read from configuration file
    if (getAlignLinks(configFile,"ALIGN_KIN_LEFT",&alignLnkLeft1,&alignLnkLeft2))
    {
        *chainEyeL<<*alignLnkLeft1<<*alignLnkLeft2;
        chainEyeL->blockLink(chainEyeL->getN()-1,0.0);
        chainEyeL->blockLink(chainEyeL->getN()-2,0.0);
    }
    else
        alignLnkLeft1=alignLnkLeft2=NULL;

    if (getAlignLinks(configFile,"ALIGN_KIN_RIGHT",&alignLnkRight1,&alignLnkRight2))
    {
        *chainEyeR<<*alignLnkRight1<<*alignLnkRight2;
        chainEyeR->blockLink(chainEyeR->getN()-1,0.0);
        chainEyeR->blockLink(chainEyeR->getN()-2,0.0);
    }
    else
        alignLnkRight1=alignLnkRight2=NULL;

    Matrix lim;

    if (Robotable)
    {
        // create interfaces
        bool ok=true;

        if (drvTorso!=NULL)
            ok&=drvTorso->view(encTorso);
        else
            encTorso=NULL;

        ok&=drvHead->view(encHead);

        if (!ok)
            fprintf(stdout,"Problems acquiring interfaces!\n");

        // read number of joints
        if (encTorso!=NULL)
            encTorso->getAxes(&nJointsTorso);
        else
            nJointsTorso=3;

        encHead->getAxes(&nJointsHead);

        // joints bounds alignment
        lim=alignJointsBounds(chainNeck,drvTorso,drvHead,eyeTiltMin,eyeTiltMax);
        copyJointsBounds(chainNeck,chainEyeL);
        copyJointsBounds(chainEyeL,chainEyeR);

        // reinforce vergence min bound
        lim(nJointsHead-1,0)=MINALLOWED_VERGENCE*CTRL_DEG2RAD;

        // just eye part is required
        lim=lim.submatrix(3,5,0,1);

        // read starting position
        fbTorso.resize(nJointsTorso,0.0);
        fbHead.resize(nJointsHead,0.0);
        getFeedback(fbTorso,fbHead,encTorso,encHead);        
    }
    else
    {
        nJointsTorso=3;
        nJointsHead =6;

        // create bounds matrix for integrators
        // just for eye part
        lim.resize(3,2);

        // tilt
        lim(0,0)=(*chainEyeL)[nJointsTorso+3].getMin();
        lim(0,1)=(*chainEyeL)[nJointsTorso+3].getMax();

        // version
        lim(1,0)=(*chainEyeL)[nJointsTorso+4].getMin();
        lim(1,1)=(*chainEyeL)[nJointsTorso+4].getMax();

        // vergence
        lim(2,0)=MINALLOWED_VERGENCE*CTRL_DEG2RAD;
        lim(2,1)=lim(1,1);

        fbTorso.resize(nJointsTorso,0.0);
        fbHead.resize(nJointsHead,0.0);

        // impose starting vergence != 0.0
        fbHead[5]=MINALLOWED_VERGENCE*CTRL_DEG2RAD;
    }

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
    port_xd=NULL;
}


/************************************************************************/
bool EyePinvRefGen::getGyro(Vector &data)
{
    if (port_inertial!=NULL)
    {
        data=gyro;
        return true;
    }
    else
        return false;
}


/************************************************************************/
Vector EyePinvRefGen::getVelocityDueToNeckRotation(const Matrix &eyesJ, const Vector &fp)
{
    Vector fprelv;

    if (VOR)
    {
        // implement VOR
        Vector q(inertialSensor.getDOF());
        q[0]=fbTorso[0];
        q[1]=fbTorso[1];
        q[2]=fbTorso[2];
        q[3]=fbHead[0];
        q[4]=fbHead[1];
        q[5]=fbHead[2];
        Matrix H=inertialSensor.getH(q);

        H(0,3)=fp[0]-H(0,3);
        H(1,3)=fp[1]-H(1,3);
        H(2,3)=fp[2]-H(2,3);

        // gyro rate [rad/s]
        double &gyrX=gyro[6];
        double &gyrY=gyro[7];
        double &gyrZ=gyro[8];

        fprelv=gyrX*cross(H,0,H,3)+gyrY*cross(H,1,H,3)+gyrZ*cross(H,2,H,3);
    }
    else
    {
        // implement OCR
        Matrix H0=chainNeck->getH(2,true);
        Matrix H1=chainNeck->getH(3,true);
        Matrix H2=chainNeck->getH(4,true);
    
        for (int i=0; i<3; i++)
        {
            H0(i,3)=fp[i]-H0(i,3);
            H1(i,3)=fp[i]-H1(i,3);
            H2(i,3)=fp[i]-H2(i,3);
        }
    
        fprelv=commData->get_v()[0]*cross(H0,2,H0,3)+
               commData->get_v()[1]*cross(H1,2,H1,3)+
               commData->get_v()[2]*cross(H2,2,H2,3);
    }

    return pinv(eyesJ)*fprelv;
}


/************************************************************************/
bool EyePinvRefGen::threadInit()
{
    if (Robotable)
    {
        port_inertial=new BufferedPort<Vector>;
        string portName=localName+"/inertial:i";
        port_inertial->open(portName.c_str());

        if (!Network::connect(("/"+robotName+"/inertial").c_str(),portName.c_str()))
        {
            delete port_inertial;
            port_inertial=NULL;
            VOR=false;
        }
    }
    else
        port_inertial=NULL;

    fprintf(stdout,"Starting Pseudoinverse Reference Generator at %d ms\n",period);

    return true;
}


/************************************************************************/
void EyePinvRefGen::afterStart(bool s)
{
    if (s)
        fprintf(stdout,"Pseudoinverse Reference Generator started successfully\n");
    else
        fprintf(stdout,"Pseudoinverse Reference Generator did not start\n");
}


/************************************************************************/
void EyePinvRefGen::run()
{
    if (genOn)
    {        
        if (Robotable)
        {
            // read encoders
            getFeedback(fbTorso,fbHead,encTorso,encHead);
            updateTorsoBlockedJoints(chainEyeL,fbTorso);
            updateTorsoBlockedJoints(chainEyeR,fbTorso);
        }
        else
            fbHead=commData->get_q();

        // read gyro data
        if (port_inertial!=NULL)
            if (Vector *_gyro=port_inertial->read(false))
                gyro=*_gyro;

        // get current target
        Vector &xd=port_xd->get_xd();

        // beware of too small vergence
        if (qd[2]<MINALLOWED_VERGENCE*CTRL_DEG2RAD)
            qd[2]=MINALLOWED_VERGENCE*CTRL_DEG2RAD;

        // update neck chain
        chainNeck->setAng(nJointsTorso+0,fbHead[0]);
        chainNeck->setAng(nJointsTorso+1,fbHead[1]);
        chainNeck->setAng(nJointsTorso+2,fbHead[2]);

        // update eyes chains for convergence purpose
        updateNeckBlockedJoints(chainEyeL,fbHead);         updateNeckBlockedJoints(chainEyeR,fbHead);
        chainEyeL->setAng(nJointsTorso+3,qd[0]);           chainEyeR->setAng(nJointsTorso+3,qd[0]);
        chainEyeL->setAng(nJointsTorso+4,qd[1]+qd[2]/2.0); chainEyeR->setAng(nJointsTorso+4,qd[1]-qd[2]/2.0);
        
        if (computeFixationPointData(*chainEyeL,*chainEyeR,fp,eyesJ))
        {
            // converge
            Vector v=EYEPINVREFGEN_GAIN*pinv(eyesJ)*(xd-fp);

            // update eyes chains in actual configuration for velocity compensation
            chainEyeL->setAng(nJointsTorso+3,fbHead[3]);               chainEyeR->setAng(nJointsTorso+3,fbHead[3]);
            chainEyeL->setAng(nJointsTorso+4,fbHead[4]+fbHead[5]/2.0); chainEyeR->setAng(nJointsTorso+4,fbHead[4]-fbHead[5]/2.0);

            // compensate neck rotation at eyes level
            if (computeFixationPointData(*chainEyeL,*chainEyeR,fp,eyesJ))
                commData->get_compv()=getVelocityDueToNeckRotation(eyesJ,fp);
            else
                commData->get_compv()=0.0;

            // update reference
            qd=I->integrate(v-commData->get_compv());
        }
        else
            commData->get_compv()=0.0;

        // set a new target position
        commData->get_xd()=xd;
        commData->get_x()=fp;
        commData->get_qd()[3]=qd[0];
        commData->get_qd()[4]=qd[1];
        commData->get_qd()[5]=qd[2];
        commData->get_fpFrame()=chainNeck->getH();
    }
}


/************************************************************************/
void EyePinvRefGen::threadRelease()
{
    if (port_inertial!=NULL)
    {
        port_inertial->interrupt();
        port_inertial->close();
        delete port_inertial;
    }

    delete neck;
    delete eyeL;
    delete eyeR;
    delete I;

    if (alignLnkLeft1!=NULL)
        delete alignLnkLeft1;

    if (alignLnkLeft2!=NULL)
        delete alignLnkLeft2;

    if (alignLnkRight1!=NULL)
        delete alignLnkRight1;

    if (alignLnkRight2!=NULL)
        delete alignLnkRight2;
}


/************************************************************************/
void EyePinvRefGen::suspend()
{
    fprintf(stdout,"\nPseudoinverse Reference Generator has been suspended!\n\n");
    RateThread::suspend();
}


/************************************************************************/
void EyePinvRefGen::resume()
{
    fprintf(stdout,"\nPseudoinverse Reference Generator has been resumed!\n\n");
    RateThread::resume();
}


/************************************************************************/
void EyePinvRefGen::stopControl()
{
    port_xd->set_xd(fp);
}


/************************************************************************/
Solver::Solver(PolyDriver *_drvTorso, PolyDriver *_drvHead, exchangeData *_commData,
               EyePinvRefGen *_eyesRefGen, Localizer *_loc, Controller *_ctrl,
               const string &_localName, const string &_configFile, const double _eyeTiltMin,
               const double _eyeTiltMax, unsigned int _period) :
               RateThread(_period),     drvTorso(_drvTorso),     drvHead(_drvHead),
               commData(_commData),     eyesRefGen(_eyesRefGen), loc(_loc),
               ctrl(_ctrl),             localName(_localName),   configFile(_configFile),
               eyeTiltMin(_eyeTiltMin), eyeTiltMax(_eyeTiltMax), period(_period),
               Ts(_period/1000.0)
{
    Robotable=(drvHead!=NULL);

    // Instantiate objects
    neck=new iCubHeadCenter();
    eyeL=new iCubEye("left");
    eyeR=new iCubEye("right");

    // remove constraints on the links: logging purpose
    inertialSensor.setAllConstraints(false);

    // block neck dofs
    eyeL->blockLink(3,0.0); eyeR->blockLink(3,0.0);
    eyeL->blockLink(4,0.0); eyeR->blockLink(4,0.0);
    eyeL->blockLink(5,0.0); eyeR->blockLink(5,0.0);

    // Get the chain objects
    chainNeck=neck->asChain();
    chainEyeL=eyeL->asChain();        
    chainEyeR=eyeR->asChain();    

    // add aligning links read from configuration file
    if (getAlignLinks(configFile,"ALIGN_KIN_LEFT",&alignLnkLeft1,&alignLnkLeft2))
    {
        *chainEyeL<<*alignLnkLeft1<<*alignLnkLeft2;
        chainEyeL->blockLink(chainEyeL->getN()-1,0.0);
        chainEyeL->blockLink(chainEyeL->getN()-2,0.0);
    }
    else
        alignLnkLeft1=alignLnkLeft2=NULL;

    if (getAlignLinks(configFile,"ALIGN_KIN_RIGHT",&alignLnkRight1,&alignLnkRight2))
    {
        *chainEyeR<<*alignLnkRight1<<*alignLnkRight2;
        chainEyeR->blockLink(chainEyeR->getN()-1,0.0);
        chainEyeR->blockLink(chainEyeR->getN()-2,0.0);
    }
    else
        alignLnkRight1=alignLnkRight2=NULL;

    if (Robotable)
    {
        // create interfaces
        bool ok=true;

        if (drvTorso!=NULL)
            ok&=drvTorso->view(encTorso);
        else
            encTorso=NULL;

        ok&=drvHead->view(encHead);

        if (!ok)
            fprintf(stdout,"Problems acquiring interfaces!\n");

        if (encTorso!=NULL)
            encTorso->getAxes(&nJointsTorso);
        else
            nJointsTorso=3;

        encHead->getAxes(&nJointsHead);

        // joints bounds alignment
        alignJointsBounds(chainNeck,drvTorso,drvHead,eyeTiltMin,eyeTiltMax);
        copyJointsBounds(chainNeck,chainEyeL);
        copyJointsBounds(chainEyeL,chainEyeR);

        // read starting position
        fbTorso.resize(nJointsTorso,0.0);
        fbHead.resize(nJointsHead,0.0);
        getFeedback(fbTorso,fbHead,encTorso,encHead);
    }
    else
    {
        nJointsTorso=3;
        nJointsHead =6;

        fbTorso.resize(nJointsTorso,0.0);
        fbHead.resize(nJointsHead,0.0);

        // impose starting vergence != 0.0
        fbHead[5]=MINALLOWED_VERGENCE*CTRL_DEG2RAD;
    }

    // store neck pitch/yaw bounds
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
}


/************************************************************************/
void Solver::bindNeckPitch(const double min_deg, const double max_deg)
{
    double min_rad=CTRL_DEG2RAD*min_deg;
    double max_rad=CTRL_DEG2RAD*max_deg;

    min_rad=(min_rad<neckPitchMin)?neckPitchMin:(min_rad>neckPitchMax?neckPitchMax:min_rad);
    max_rad=(max_rad<neckPitchMin)?neckPitchMin:(max_rad>neckPitchMax?neckPitchMax:max_rad);

    (*chainNeck)(0).setMin(min_rad);
    (*chainNeck)(0).setMax(max_rad);

    fprintf(stdout,"\nneck pitch constrained in [%g,%g] deg\n\n",min_deg,max_deg);
}


/************************************************************************/
void Solver::bindNeckRoll(const double min_deg, const double max_deg)
{
    double min_rad=CTRL_DEG2RAD*min_deg;
    double max_rad=CTRL_DEG2RAD*max_deg;

    min_rad=(min_rad<neckRollMin)?neckRollMin:(min_rad>neckRollMax?neckRollMax:min_rad);
    max_rad=(max_rad<neckRollMin)?neckRollMin:(max_rad>neckRollMax?neckRollMax:max_rad);

    (*chainNeck)(1).setMin(min_rad);
    (*chainNeck)(1).setMax(max_rad);

    fprintf(stdout,"\nneck roll constrained in [%g,%g] deg\n\n",min_deg,max_deg);
}


/************************************************************************/
void Solver::bindNeckYaw(const double min_deg, const double max_deg)
{
    double min_rad=CTRL_DEG2RAD*min_deg;
    double max_rad=CTRL_DEG2RAD*max_deg;

    min_rad=(min_rad<neckYawMin)?neckYawMin:(min_rad>neckYawMax?neckYawMax:min_rad);
    max_rad=(max_rad<neckYawMin)?neckYawMin:(max_rad>neckYawMax?neckYawMax:max_rad);

    (*chainNeck)(2).setMin(min_rad);
    (*chainNeck)(2).setMax(max_rad);

    fprintf(stdout,"\nneck yaw constrained in [%g,%g] deg\n\n",min_deg,max_deg);
}


/************************************************************************/
void Solver::getCurNeckPitchRange(double &min_deg, double &max_deg) const
{
    min_deg=CTRL_RAD2DEG*(*chainNeck)(0).getMin();
    max_deg=CTRL_RAD2DEG*(*chainNeck)(0).getMax();
}


/************************************************************************/
void Solver::getCurNeckRollRange(double &min_deg, double &max_deg) const
{
    min_deg=CTRL_RAD2DEG*(*chainNeck)(1).getMin();
    max_deg=CTRL_RAD2DEG*(*chainNeck)(1).getMax();
}


/************************************************************************/
void Solver::getCurNeckYawRange(double &min_deg, double &max_deg) const
{
    min_deg=CTRL_RAD2DEG*(*chainNeck)(2).getMin();
    max_deg=CTRL_RAD2DEG*(*chainNeck)(2).getMax();
}


/************************************************************************/
void Solver::clearNeckPitch()
{
    (*chainNeck)(0).setMin(neckPitchMin);
    (*chainNeck)(0).setMax(neckPitchMax);

    fprintf(stdout,"\nneck pitch cleared\n\n");
}


/************************************************************************/
void Solver::clearNeckRoll()
{
    (*chainNeck)(1).setMin(neckRollMin);
    (*chainNeck)(1).setMax(neckRollMax);

    fprintf(stdout,"\nneck roll cleared\n\n");
}


/************************************************************************/
void Solver::clearNeckYaw()
{
    (*chainNeck)(2).setMin(neckYawMin);
    (*chainNeck)(2).setMax(neckYawMax);

    fprintf(stdout,"\nneck yaw cleared\n\n");
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
    Vector x(4); Vector y(4);
    x[0]=1.0;    y[0]=0.0;
    x[1]=0.0;    y[1]=1.0;
    x[2]=0.0;    y[2]=0.0;
    x[3]=roll;   y[3]=pitch;   
    Matrix R=axis2dcm(y)*axis2dcm(x);

    Vector q(inertialSensor.getDOF());
    q[0]=fbTorso[0];
    q[1]=fbTorso[1];
    q[2]=fbTorso[2];
    q[3]=fbHead[0];
    q[4]=fbHead[1];
    q[5]=fbHead[2];
    Matrix H=inertialSensor.getH(q)*R.transposed();

    // gravity is aligned along the z-axis
    Vector gDir=H.getCol(2);
    gDir[3]=1.0;    // impose homogeneous coordinates

    return gDir;
}


/************************************************************************/
Vector Solver::neckTargetRotAngles(const Vector &xd)
{    
    Matrix H=chainNeck->getH();

    for (int i=0; i<3; i++)
        H(i,3)-=xd[i];

    H(3,3)=0.0;

    // projection on the transverse and sagittal planes
    double x=dot(H,0,H,3);
    double y=dot(H,1,H,3);
    double z=dot(H,2,H,3);

    Vector res(2);
    res[0]=atan2(x,-z);
    res[1]=atan2(y,-z);

    if (res[0]<0.0)
        res[0]=-res[0];

    if (res[1]<0.0)
        res[1]=-res[1];

    return res;
}


/************************************************************************/
bool Solver::threadInit()
{
    // Instantiate optimizer
    invNeck=new GazeIpOptMin(*chainNeck,1e-3,20);

    // Initialization
    Vector fp(3);
    Matrix J(3,3);
    computeFixationPointData(*chainEyeL,*chainEyeR,fp,J);

    // init commData structure
    commData->get_xd()=fp;
    commData->get_qd()=fbHead;
    commData->get_x()=fp;
    commData->get_q()=fbHead;
    commData->get_torso()=fbTorso;
    commData->get_v().resize(fbHead.length(),0.0);
    commData->get_compv().resize(3,0.0);
    commData->get_fpFrame()=chainNeck->getH();

    port_xd=new xdPort(fp,this);
    port_xd->useCallback();
    port_xd->open((localName+"/xd:i").c_str());

    loc->set_xdport(port_xd);
    eyesRefGen->set_xdport(port_xd);
    ctrl->set_xdport(port_xd);

    // use eyes pseudoinverse reference generator
    eyesRefGen->enable();

    fprintf(stdout,"Starting Solver at %d ms\n",period);

    return true;
}


/************************************************************************/
void Solver::afterStart(bool s)
{
    if (s)
        fprintf(stdout,"Solver started successfully\n");
    else
        fprintf(stdout,"Solver did not start\n");
}


/************************************************************************/
void Solver::run()
{
    // get the current target
    Vector &xd=port_xd->get_xdDelayed();

    // update the target straightaway 
    commData->get_xd()=xd;

    bool torsoChanged=false;

    if (Robotable)
    {
        // read encoders
        getFeedback(fbTorso,fbHead,encTorso,encHead);
        updateTorsoBlockedJoints(chainNeck,fbTorso);
        updateTorsoBlockedJoints(chainEyeL,fbTorso);
        updateTorsoBlockedJoints(chainEyeR,fbTorso);

        torsoChanged=norm(fbTorso-fbTorsoOld)>NECKSOLVER_ACTIVATIONANGLE_JOINTS*CTRL_DEG2RAD;        
    }
    else
        fbHead=commData->get_q();

    bool headChanged=norm(fbHead-fbHeadOld)>NECKSOLVER_ACTIVATIONANGLE_JOINTS*CTRL_DEG2RAD;

    // update kinematics
    updateAngles();
    updateNeckBlockedJoints(chainEyeL,fbHead);
    updateNeckBlockedJoints(chainEyeR,fbHead);
    chainNeck->setAng(neckPos);

    // hereafter accumulate solving conditions: the order does matter

    // 1) compute the distance in the transverse and sagittal planes
    Vector theta=neckTargetRotAngles(xd);
    bool doSolve=(theta[0]>NECKSOLVER_ACTIVATIONANGLE_TRA*CTRL_DEG2RAD) ||
                 (theta[1]>NECKSOLVER_ACTIVATIONANGLE_SAG*CTRL_DEG2RAD);

    // 2) skip if controller is active and no torso motion is detected
    doSolve&=!(commData->get_isCtrlActive() && !torsoChanged);

    // 3) skip if controller is inactive and we are not in tracking mode
    doSolve&=!(!commData->get_isCtrlActive() && commData->get_canCtrlBeDisabled());

    // 4) skip if controller is inactive, we are in tracking mode and nothing has changed
    // note that the De Morgan's law has been applied hereafter
    doSolve&=commData->get_isCtrlActive() || commData->get_canCtrlBeDisabled() ||
             torsoChanged || headChanged;

    // 5) solve straightaway if the target has changed
    doSolve|=port_xd->get_newDelayed();

    // clear trigger
    port_xd->get_newDelayed()=false;

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

        neckPos=invNeck->solve(neckPos,xd,*pgDir);

        // update neck pitch,roll,yaw        
        commData->get_qd()[0]=neckPos[0];
        commData->get_qd()[1]=neckPos[1];
        commData->get_qd()[2]=neckPos[2];
    }

    // latch quantities
    fbTorsoOld=fbTorso;
    fbHeadOld=fbHead;
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

    if (alignLnkLeft1!=NULL)
        delete alignLnkLeft1;

    if (alignLnkLeft2!=NULL)
        delete alignLnkLeft2;

    if (alignLnkRight1!=NULL)
        delete alignLnkRight1;

    if (alignLnkRight2!=NULL)
        delete alignLnkRight2;
}


/************************************************************************/
void Solver::suspend()
{
    fprintf(stdout,"\nSolver has been suspended!\n\n");
    RateThread::suspend();
}


/************************************************************************/
void Solver::resume()
{
    if (Robotable)
    {
        // read encoders
        getFeedback(fbTorso,fbHead,encTorso,encHead);

        updateTorsoBlockedJoints(chainNeck,fbTorso);
        updateTorsoBlockedJoints(chainEyeL,fbTorso);
        updateTorsoBlockedJoints(chainEyeR,fbTorso);        
    }
    else
        fbHead=commData->get_q();    

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
    computeFixationPointData(*chainEyeL,*chainEyeR,fp,J);

    // update input port
    port_xd->set_xd(fp);

    // update latched quantities
    fbTorsoOld=fbTorso;
    fbHeadOld=fbHead;

    fprintf(stdout,"\nSolver has been resumed!\n\n");

    RateThread::resume();
}



