
#include "solver.h"


/************************************************************************/
EyePinvRefGen::EyePinvRefGen(PolyDriver *_drvTorso, PolyDriver *_drvHead,
                             exchangeData *_commData, const string &_robotName,
                             const string &_localName, const string &_inertialName,
                             const string &_configFile, unsigned int _period) :
                             RateThread(_period),         drvTorso(_drvTorso),   drvHead(_drvHead),
                             commData(_commData),         robotName(_robotName), localName(_localName),
                             inertialName(_inertialName), configFile(_configFile),
                             period(_period),             Ts(_period/1000.0)
{
    Robotable=drvTorso&&drvHead;

    // Instantiate objects
    neck=new iCubEye("left");
    eyeL=new iCubEye("left");
    eyeR=new iCubEye("right");

    // block neck roll and eyes dofs
    neck->blockLink(4,0.0);
    neck->blockLink(6,0.0);
    neck->blockLink(7,0.0);

    // block neck dofs
    eyeL->blockLink(3,0.0); eyeR->blockLink(3,0.0);
    eyeL->blockLink(4,0.0); eyeR->blockLink(4,0.0);
    eyeL->blockLink(5,0.0); eyeR->blockLink(5,0.0);

    // Get the chain object attached to the arm
    chainNeck=neck->asChain();
    chainEyeL=eyeL->asChain();
    chainEyeR=eyeR->asChain();

    // add aligning links read from configuration file
    if (getAlignLinks(configFile,"LEFT",&alignLnkLeft1,&alignLnkLeft2))
    {
        *chainEyeL<<*alignLnkLeft1<<*alignLnkLeft2;
        chainEyeL->blockLink(chainEyeL->getN()-1,0.0);
        chainEyeL->blockLink(chainEyeL->getN()-2,0.0);
    }
    if (getAlignLinks(configFile,"RIGHT",&alignLnkRight1,&alignLnkRight2))
    {
        *chainEyeR<<*alignLnkRight1<<*alignLnkRight2;
        chainEyeR->blockLink(chainEyeR->getN()-1,0.0);
        chainEyeR->blockLink(chainEyeR->getN()-2,0.0);
    }

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
        lim=alignJointsBounds(chainEyeL,limTorso,limHead);
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
        
        fbTorso.resize(nJointsTorso); fbTorso=0.0;
        fbHead.resize(nJointsHead);   fbHead=0.0;

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

    fp.resize(3);      fp=0.0;
    eyesJ.resize(3,3); eyesJ=0.0;
    gyro.resize(12);   gyro=0.0;

    genOn=false;
}


/************************************************************************/
bool EyePinvRefGen::threadInit()
{
    if (Robotable)
    {
        port_inertial=new BufferedPort<Vector>();
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
        if (qd[2]<0.5*(M_PI/180.0))
            qd[2]=0.5*(M_PI/180.0);

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

            // implement VOC
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
        commData->get_fpFrame()=chainEyeL->getH()+chainEyeR->getH();
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
Solver::Solver(PolyDriver *_drvTorso, PolyDriver *_drvHead, exchangeData *_commData,
               EyePinvRefGen *_eyesRefGen, Localizer *_loc, const string &_localName,
               const string &_configFile, unsigned int _period) :
               RateThread(_period),     drvTorso(_drvTorso),     drvHead(_drvHead),
               commData(_commData),     eyesRefGen(_eyesRefGen), loc(_loc),
               localName(_localName),   configFile(_configFile), period(_period),
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

    // Get the chain object attached to the arm
    chainNeck=neck->asChain();
    chainEyeL=eyeL->asChain();        
    chainEyeR=eyeR->asChain();

    // add aligning links read from configuration file
    if (getAlignLinks(configFile,"LEFT",&alignLnkLeft1,&alignLnkLeft2))
    {
        *chainEyeL<<*alignLnkLeft1<<*alignLnkLeft2;
        chainEyeL->blockLink(chainEyeL->getN()-1,0.0);
        chainEyeL->blockLink(chainEyeL->getN()-2,0.0);
    }
    if (getAlignLinks(configFile,"RIGHT",&alignLnkRight1,&alignLnkRight2))
    {
        *chainEyeR<<*alignLnkRight1<<*alignLnkRight2;
        chainEyeR->blockLink(chainEyeR->getN()-1,0.0);
        chainEyeR->blockLink(chainEyeR->getN()-2,0.0);
    }

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
        alignJointsBounds(chainNeck,limTorso,limHead);
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

        fbTorso.resize(nJointsTorso); fbTorso=0.0;
        fbHead.resize(nJointsHead);   fbHead=0.0;
    }

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
    fbTorso_old=fbTorso;
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
double Solver::neckTargetRotAngle(const Vector &xd)
{
    Matrix H=chainNeck->getH();

    for (unsigned int i=0; i<3; i++)
        H(i,3)-=xd[i];

    H(3,3)=0.0;

    // distance of target from the head
    double d1=norm(H,3);

    if (d1)
    {
        // distance of target from the straight-ahead line
        double d2=norm(cross(H,2,H,3));

        return asin(d2/d1);
    }
    else
        return 0.0;
}


/************************************************************************/
void Solver::setStart()
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

    // latch torso joints
    fbTorso_old=fbTorso;
}


/************************************************************************/
bool Solver::threadInit()
{
    // Instantiate the optimizers
    iKinChain dummyChain;
    invNeck=new GazeIpOptMin("neck",*chainNeck,dummyChain,1e-3,20);
    invEyes=new GazeIpOptMin("eyes",*chainEyeL,*chainEyeR,1e-3,20);

    // Initialization
    Vector fp(3);
    Matrix J(3,3);
    if (computeFixationPointData(*chainEyeL,*chainEyeR,fp,J) || fbHead[5]<1.0*(M_PI/180.0))
    {
        // standard value for initial vergence~0.0
        fbHead[5]=10.0*(M_PI/180.0);
        updateAngles();

        Vector eyePos=chainEyeL->getAng();
        eyePos[1]=fbHead[4]+fbHead[5]/2.0;
        chainEyeL->setAng(eyePos);
        eyePos[1]=fbHead[4]-fbHead[5]/2.0;
        chainEyeR->setAng(eyePos);

        computeFixationPointData(*chainEyeL,*chainEyeR,fp,J);
    }

    // init commData structure
    commData->get_xd()=fp;
    commData->get_qd()=fbHead;
    commData->get_x()=fp;
    commData->get_q()=fbHead;
    commData->get_v().resize(5);
    commData->get_v()=0.0;
    commData->get_compv().resize(3);
    commData->get_compv()=0.0;
    commData->get_fpFrame()=chainEyeL->getH()+chainEyeR->getH();

    xdOld=fp;

    port_xd=new xdPort(fp);
    port_xd->useCallback();
    string n=localName+"/xd:i";
    port_xd->open(n.c_str());

    loc->set_xdport(port_xd);
    eyesRefGen->set_xdport(port_xd);

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
    Vector xd=port_xd->get_xd();

    bool movedTorso=false;

    if (Robotable)
    {
        // read encoders
        getFeedback(fbTorso,fbHead,encTorso,encHead);
        updateTorsoBlockedJoints(chainNeck,fbTorso);
        updateTorsoBlockedJoints(chainEyeL,fbTorso);
        updateTorsoBlockedJoints(chainEyeR,fbTorso);

        movedTorso=norm(fbTorso-fbTorso_old)>1.0*(M_PI/180.0);
    }
    else
        fbHead=commData->get_q();

    // update kinematics
    updateAngles();
    updateNeckBlockedJoints(chainEyeL,fbHead);
    updateNeckBlockedJoints(chainEyeR,fbHead);
    chainNeck->setAng(neckPos);

    double theta=neckTargetRotAngle(xd);

    if (norm(xd-xdOld)>1e-6 || movedTorso)
    {
        // call the solver for neck (only if necessary)        
        if (theta>NECKSOLVER_ACTIVATIONANGLE*(M_PI/180.0) || movedTorso)
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

    if (theta>1.0*(M_PI/180.0) && theta<NECKSOLVER_ACTIVATIONANGLE*(M_PI/180.0))
        alignNeckCnt++;

    // re-align neck after a timeout
    if (alignNeckCnt>3.0/Ts)
    {
        //invNeck->solve(neckPos,xd,NULL,NULL,neckCallbackObj);
        neckPos=invNeck->solve(neckPos,xd);

        // update neck pitch,yaw
        commData->get_xd()=xd;
        commData->get_qd()[0]=neckPos[0];
        commData->get_qd()[2]=neckPos[1];

        alignNeckCnt=0;
    }

    // latch torso joints
    fbTorso_old=fbTorso;
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


