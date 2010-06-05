
#include <yarp/math/SVD.h>
#include <iCub/solver.hpp>


/************************************************************************/
EyePinvRefGen::EyePinvRefGen(PolyDriver *_drvTorso, PolyDriver *_drvHead,
                             exchangeData *_commData, const string &_robotName,
                             const string &_localName, const string &_inertialName,
                             const string &_configFile, const double _eyeTiltMin,
                             const double _eyeTiltMax, unsigned int _period) :
                             RateThread(_period),         drvTorso(_drvTorso),     drvHead(_drvHead),
                             commData(_commData),         robotName(_robotName),   localName(_localName),
                             inertialName(_inertialName), configFile(_configFile), period(_period),
                             eyeTiltMin(_eyeTiltMin),     eyeTiltMax(_eyeTiltMax), Ts(_period/1000.0)
{
    Robotable=drvTorso&&drvHead;

    // Instantiate objects
    neck=new iCubHeadCenter();
    eyeL=new iCubEye("left");
    eyeR=new iCubEye("right");

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
        bool ok;
        ok =drvTorso->view(limTorso);
        ok&=drvTorso->view(encTorso);
        ok&=drvHead->view(limHead);
        ok&=drvHead->view(encHead);

        if (!ok)
            cout << "Problems acquiring interfaces!" << endl;

        // read number of joints
        encTorso->getAxes(&nJointsTorso);
        encHead->getAxes(&nJointsHead);

        // joints bounds alignment
        lim=alignJointsBounds(chainNeck,limTorso,limHead,
                              eyeTiltMin,eyeTiltMax);
        copyJointsBounds(chainNeck,chainEyeL);
        copyJointsBounds(chainEyeL,chainEyeR);

        // reinforce vergence min bound
        lim(nJointsHead-1,0)=0.0;

        // just eye part is required
        lim=lim.submatrix(3,5,0,1);

        // read starting position
        fbTorso.resize(nJointsTorso);
        fbHead.resize(nJointsHead);
        getFeedback(fbTorso,fbHead,encTorso,encHead);
    }
    else
    {
        nJointsTorso=3;
        nJointsHead =6;
        
        fbTorso.resize(nJointsTorso,0.0);
        fbHead.resize(nJointsHead,0.0);

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
        lim(2,0)=0.0;
        lim(2,1)=lim(1,1);
    }

    // Instantiate integrator
    qd.resize(3);
    qd[0]=fbHead[3];
    qd[1]=fbHead[4];
    qd[2]=fbHead[5];
    I=new Integrator(Ts,qd,lim);

    fp.resize(3,0.0);
    eyesJ.resize(3,3);
    eyesJ.zero();
    gyro.resize(12,0.0);

    genOn=false;
    port_xd=NULL;
}


/************************************************************************/
bool EyePinvRefGen::threadInit()
{
    if (Robotable)
    {
        port_inertial=new BufferedPort<Vector>;
        string n=localName+inertialName+":i";
        port_inertial->open(n.c_str());

        if (!Network::connect(("/"+robotName+"/"+inertialName).c_str(),n.c_str(),"udp"))
        {
            delete port_inertial;
            port_inertial=NULL;                
        }
    }
    else
        port_inertial=NULL;

    cout << "Starting Pseudoinverse Reference Generator at " << period << " ms" << endl;

    return true;
}


/************************************************************************/
void EyePinvRefGen::afterStart(bool s)
{
    if (s)
        cout << "Pseudoinverse Reference Generator started successfully" << endl;
    else
        cout << "Pseudoinverse Reference Generator did not start" << endl;
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
        if (port_inertial)
            if (Vector *_gyro=port_inertial->read(false))
                gyro=*_gyro;

        // get current target
        Vector &xd=port_xd->get_xd();

        // beware of too small vergence
        if (qd[2]<0.5*CTRL_DEG2RAD)
            qd[2]=0.5*CTRL_DEG2RAD;

        // update neck chain
        chainNeck->setAng(nJointsTorso+0,fbHead[0]);
        chainNeck->setAng(nJointsTorso+2,fbHead[2]);

        // update eyes chains for convergence purpose
        updateNeckBlockedJoints(chainEyeL,fbHead);         updateNeckBlockedJoints(chainEyeR,fbHead);
        chainEyeL->setAng(nJointsTorso+3,qd[0]);           chainEyeR->setAng(nJointsTorso+3,qd[0]);
        chainEyeL->setAng(nJointsTorso+4,qd[1]+qd[2]/2.0); chainEyeR->setAng(nJointsTorso+4,qd[1]-qd[2]/2.0);
        
        if (!computeFixationPointData(*chainEyeL,*chainEyeR,fp,eyesJ))
        {
            // converge
            Vector v=EYEPINVREFGEN_GAIN*pinv(eyesJ)*(xd-fp);

            // update eyes chains in actual configuration for velocity compensation
            chainEyeL->setAng(nJointsTorso+3,fbHead[3]);               chainEyeR->setAng(nJointsTorso+3,fbHead[3]);
            chainEyeL->setAng(nJointsTorso+4,fbHead[4]+fbHead[5]/2.0); chainEyeR->setAng(nJointsTorso+4,fbHead[4]-fbHead[5]/2.0);

            computeFixationPointData(*chainEyeL,*chainEyeR,fp,eyesJ);

            // implement OCR
            Matrix h0=chainNeck->getH(2,true);
            Matrix h2=chainNeck->getH(4,true);

            for (unsigned int i=0; i<3; i++)
            {
                h0(i,3)=fp[i]-h0(i,3);
                h2(i,3)=fp[i]-h2(i,3);
            }

            h0(3,3)=h2(3,3)=0.0;

            Vector fprelv=commData->get_v()[0]*cross(h0,2,h0,3)+
                          commData->get_v()[2]*cross(h2,2,h2,3);

            commData->get_compv()=pinv(eyesJ)*fprelv;

            // this is the place where introduce some data fusion
            // on compv with gyro readouts (VOR)

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
    if (port_inertial)
    {
        port_inertial->interrupt();
        port_inertial->close();
        delete port_inertial;
    }

    delete neck;
    delete eyeL;
    delete eyeR;
    delete I;

    if (alignLnkLeft1)
        delete alignLnkLeft1;

    if (alignLnkLeft2)
        delete alignLnkLeft2;

    if (alignLnkRight1)
        delete alignLnkRight1;

    if (alignLnkRight2)
        delete alignLnkRight2;
}


/************************************************************************/
void EyePinvRefGen::suspend()
{
    cout << endl;
    cout << "Pseudoinverse Reference Generator has been suspended!" << endl;
    cout << endl;

    RateThread::suspend();
}


/************************************************************************/
void EyePinvRefGen::resume()
{
    cout << endl;
    cout << "Pseudoinverse Reference Generator has been resumed!" << endl;
    cout << endl;

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
    Robotable=drvTorso&&drvHead;

    // Instantiate objects
    neck=new iCubHeadCenter();
    eyeL=new iCubEye("left");
    eyeR=new iCubEye("right");
    neckCallbackObj=new neckCallback(commData);
    eyesCallbackObj=new eyesCallback(commData);

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
        bool ok;
        ok =drvTorso->view(limTorso);
        ok&=drvTorso->view(encTorso);
        ok&=drvHead->view(limHead);
        ok&=drvHead->view(encHead);

        if (!ok)
            cout << "Problems acquiring interfaces!" << endl;

        encTorso->getAxes(&nJointsTorso);
        encHead->getAxes(&nJointsHead);

        // joints bounds alignment
        alignJointsBounds(chainNeck,limTorso,limHead,
                          eyeTiltMin,eyeTiltMax);
        copyJointsBounds(chainNeck,chainEyeL);
        copyJointsBounds(chainEyeL,chainEyeR);

        // read starting position
        fbTorso.resize(nJointsTorso);
        fbHead.resize(nJointsHead);        
        getFeedback(fbTorso,fbHead,encTorso,encHead);        
    }
    else
    {
        nJointsTorso=3;
        nJointsHead =6;

        fbTorso.resize(nJointsTorso,0.0);
        fbHead.resize(nJointsHead,0.0);
    }

    // store neck pitch/yaw bounds
    neckPitchMin=(*chainNeck)[3].getMin();
    neckPitchMax=(*chainNeck)[3].getMax();
    neckYawMin  =(*chainNeck)[5].getMin();
    neckYawMax  =(*chainNeck)[5].getMax();

    // neck roll disabled
    fbHead[1]=0.0;

    neckPos.resize(2);
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

    alignNeckCnt=0;

    // latch torso joints
    fbTorsoOld.push_front(fbTorso);
}


/************************************************************************/
void Solver::blockNeckPitch(const double val_deg)
{
    double val_rad=CTRL_DEG2RAD*val_deg;
    val_rad=val_rad<neckPitchMin?neckPitchMin:(val_rad>neckPitchMax?neckPitchMax:val_rad);

    (*chainNeck)[3].setMin(val_rad);
    (*chainNeck)[3].setMax(val_rad);

    cout << endl;
    cout << "neck pitch blocked at " << val_deg << " [deg]" << endl;
    cout << endl;
}


/************************************************************************/
void Solver::blockNeckYaw(const double val_deg)
{
    double val_rad=CTRL_DEG2RAD*val_deg;
    val_rad=val_rad<neckYawMin?neckYawMin:(val_rad>neckYawMax?neckYawMax:val_rad);

    (*chainNeck)[5].setMin(val_rad);
    (*chainNeck)[5].setMax(val_rad);

    cout << endl;
    cout << "neck yaw blocked at " << val_deg << " [deg]" << endl;
    cout << endl;
}


/************************************************************************/
void Solver::clearNeckPitch()
{
    (*chainNeck)[3].setMin(neckPitchMin);
    (*chainNeck)[3].setMax(neckPitchMax);

    cout << endl;
    cout << "neck pitch cleared" << endl;
    cout << endl;
}


/************************************************************************/
void Solver::clearNeckYaw()
{
    (*chainNeck)[5].setMin(neckYawMin);
    (*chainNeck)[5].setMax(neckYawMax);

    cout << endl;
    cout << "neck yaw cleared" << endl;
    cout << endl;
}


/************************************************************************/
void Solver::updateAngles()
{
    neckPos[0]=fbHead[0];
    neckPos[1]=fbHead[2];
    gazePos[0]=fbHead[3];
    gazePos[1]=fbHead[4];
    gazePos[2]=fbHead[5];
}


/************************************************************************/
Vector Solver::neckTargetRotAngles(const Vector &xd)
{    
    Matrix H=chainNeck->getH();

    for (unsigned int i=0; i<3; i++)
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
    // Instantiate optimizers
    iKinChain dummyChain;
    invNeck=new GazeIpOptMin("neck",*chainNeck,dummyChain,1e-3,20);
    invEyes=new GazeIpOptMin("eyes",*chainEyeL,*chainEyeR,1e-3,20);

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
    commData->get_v().resize(5,0.0);
    commData->get_compv().resize(3,0.0);
    commData->get_fpFrame()=chainNeck->getH();

    xdOld=fp;

    port_xd=new xdPort(fp);
    port_xd->useCallback();
    string n=localName+"/xd:i";
    port_xd->open(n.c_str());

    loc->set_xdport(port_xd);
    eyesRefGen->set_xdport(port_xd);
    ctrl->set_xdport(port_xd);

    // use eyes pseudoinverse reference generator
    eyesRefGen->enable();

    cout << "Starting Solver at " << period << " ms" << endl;

    return true;
}


/************************************************************************/
void Solver::afterStart(bool s)
{
    if (s)
        cout << "Solver started successfully" << endl;
    else
        cout << "Solver did not start" << endl;
}


/************************************************************************/
void Solver::run()
{
    // get the current target
    Vector &xd=port_xd->get_xd();

    bool movedTorso=false;

    if (Robotable)
    {
        // read encoders
        getFeedback(fbTorso,fbHead,encTorso,encHead);
        updateTorsoBlockedJoints(chainNeck,fbTorso);
        updateTorsoBlockedJoints(chainEyeL,fbTorso);
        updateTorsoBlockedJoints(chainEyeR,fbTorso);

        if (fbTorsoOld.size()>NECKSOLVER_MOVEDTORSOQUEUSIZE)
        {
            movedTorso=norm(fbTorso-fbTorsoOld.back())>1.0*CTRL_DEG2RAD;
            fbTorsoOld.pop_back();
        }

        // latch torso joints
        fbTorsoOld.push_front(fbTorso);
    }
    else
        fbHead=commData->get_q();

    // update kinematics
    updateAngles();
    updateNeckBlockedJoints(chainEyeL,fbHead);
    updateNeckBlockedJoints(chainEyeR,fbHead);
    chainNeck->setAng(neckPos);

    Vector theta=neckTargetRotAngles(xd);

    if (!(xd==xdOld) || movedTorso)
    {
        // call the solver for neck (only if necessary)
        if (movedTorso ||
            theta[0]>NECKSOLVER_ACTIVATIONANGLE_TRA*CTRL_DEG2RAD ||
            theta[1]>NECKSOLVER_ACTIVATIONANGLE_SAG*CTRL_DEG2RAD)
        {
            //invNeck->solve(neckPos,xd,NULL,NULL,neckCallbackObj);
            neckPos=invNeck->solve(neckPos,xd);

            // update neck pitch,yaw
            commData->get_xd()=xd;
            commData->get_qd()[0]=neckPos[0];
            commData->get_qd()[2]=neckPos[1];
        }

        xdOld=xd;
    }

    if (theta[0]>1.0*CTRL_DEG2RAD && theta[0]<NECKSOLVER_ACTIVATIONANGLE_TRA*CTRL_DEG2RAD ||
        theta[1]>1.0*CTRL_DEG2RAD && theta[1]<NECKSOLVER_ACTIVATIONANGLE_SAG*CTRL_DEG2RAD)
        alignNeckCnt++;

    // re-align neck after a timeout
    if (alignNeckCnt>2.0/Ts)
    {
        //invNeck->solve(neckPos,xd,NULL,NULL,neckCallbackObj);
        neckPos=invNeck->solve(neckPos,xd);

        // update neck pitch,yaw
        commData->get_xd()=xd;
        commData->get_qd()[0]=neckPos[0];
        commData->get_qd()[2]=neckPos[1];

        alignNeckCnt=0;
    }
}


/************************************************************************/
void Solver::threadRelease()
{
    eyesRefGen->disable();

    port_xd->interrupt();
    port_xd->close();

    delete port_xd;
    delete invNeck;
    delete invEyes;
    delete neckCallbackObj;
    delete eyesCallbackObj;
    delete neck;
    delete eyeL;
    delete eyeR;

    if (alignLnkLeft1)
        delete alignLnkLeft1;

    if (alignLnkLeft2)
        delete alignLnkLeft2;

    if (alignLnkRight1)
        delete alignLnkRight1;

    if (alignLnkRight2)
        delete alignLnkRight2;
}


/************************************************************************/
void Solver::suspend()
{
    cout << endl;
    cout << "Solver has been suspended!" << endl;
    cout << endl;

    RateThread::suspend();
}


/************************************************************************/
void Solver::resume()
{
    if (Robotable)
    {
        // read encoders
        getFeedback(fbTorso,fbHead,encTorso,encHead);

        // neck roll disabled
        fbHead[1]=0.0;

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

    // clear the buffer
    fbTorsoOld.clear();
    // latch torso joints
    fbTorsoOld.push_front(fbTorso);

    cout << endl;
    cout << "Solver has been resumed!" << endl;
    cout << endl;

    RateThread::resume();
}



