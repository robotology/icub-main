/* 
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <algorithm>

#include <gsl/gsl_math.h>

#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <yarp/math/NormRand.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/outliersDetection.h>

#include "module.h"

using namespace std;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace iCub::optimization;


/************************************************************************/
void DisparityProcessor::onRead(ImageOf<PixelMono> &imgIn)
{
    module->onRead(imgIn);
}

/************************************************************************/
DisparityProcessor::DisparityProcessor(CalibModule *module)
{
    this->module=module;
}


/************************************************************************/
bool CalibModule::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}


/************************************************************************/
bool CalibModule::createTargets(const Vector &c, const Vector &size)
{
    if ((c.length()<3) || (size.length()<2))
        return false;
    
    double a=std::max(fabs(size[0]),0.04);
    double b=std::max(fabs(size[1]),0.02);

    Vector ax1(4,0.0);
    ax1[2]=1.0;
    ax1[3]=90.0*CTRL_DEG2RAD;

    Matrix H=axis2dcm(ax1);
    H(0,3)=c[0];
    H(1,3)=c[1];
    H(2,3)=c[2];
    H(3,3)=1.0;

    targets.clear();
    for (int i=0; i<2; i++)
    {
        for (double theta=0.0; theta<=2.0*M_PI; theta+=(2.0*M_PI)/50.0)
        {
            Vector x(4);
            x[0]=a*cos(theta);
            x[1]=b*sin(theta);
            x[2]=0.0;
            x[3]=1.0;
            x=H*x;
            x.pop_back();
            targets.push_back(x);
            printf("created point #%d=(%s)\n",(int)targets.size(),x.toString(3,3).c_str());
        }

        Vector ax2(4,0.0);
        ax2[1]=1.0;
        ax2[3]=90.0*CTRL_DEG2RAD;
        H=axis2dcm(ax2)*axis2dcm(ax1);
        H(0,3)=c[0];
        H(1,3)=c[1];
        H(2,3)=c[2];
        H(3,3)=1.0;
    }

    curExplorationCenter=c;
    return true;
}


/************************************************************************/
bool CalibModule::isTypeValid(const string &type)
{
    return ((type=="se3")    || (type=="se3+scale") ||
            (type=="affine") || (type=="lssvm"));
}


/************************************************************************/
Calibrator *CalibModule::factory(const string &type)
{
    Calibrator *calibrator=NULL;
    if ((type=="se3") || (type=="se3+scale") || (type=="affine"))
        calibrator=new MatrixCalibrator(type);
    else if (type=="lssvm")
        calibrator=new LSSVMCalibrator(type);

    calibrated=false;
    return calibrator;
}


/************************************************************************/
bool CalibModule::factory(Value &v)
{
    if (!v.check("arm") || !v.check("type")) 
        return false;

    string arm=v.find("arm").asString().c_str();
    string type=v.find("type").asString().c_str();
    if ((arm!="left") && (arm!="right"))
        return false;

    if (isTypeValid(type))
    {
        Property info;
        info.fromString(v.toString());

        Calibrator *c=factory(type);
        if (c->fromProperty(info))
        {            
            c->toProperty(info); 
            printf("loaded %s calibrator: %s\n",arm.c_str(),info.toString().c_str());
            ((arm=="left")?expertsL:expertsR)<<*c;
            return true;
        }
    }

    return false;
}


/************************************************************************/
cv::Rect CalibModule::extractFingerTip(ImageOf<PixelMono> &imgIn, ImageOf<PixelBgr> &imgOut,
                                       const Vector &c, Vector &px)
{
    cv::Mat imgInMat((IplImage*)imgIn.getIplImage());

    // produce a colored image
    imgOut.resize(imgIn);
    cv::Mat imgOutMat((IplImage*)imgOut.getIplImage());
    
    // proceed iff the center is within the image plane
    if ((c[0]<10.0) || (c[0]>imgIn.width()-10) ||
        (c[1]<10.0) || (c[1]>imgIn.height()-10))
    {
        cv::cvtColor(imgInMat,imgOutMat,CV_GRAY2BGR);
        return cv::Rect();
    }

    // saturate the top-left and bottom-right corners
    int roi_side2=roi_side>>1;
    cv::Point ct((int)c[0],(int)c[1]);
    cv::Point tl((int)(c[0]-roi_side2),(int)(c[1]-roi_side2));
    cv::Point br((int)(c[0]+roi_side2),(int)(c[1]+roi_side2));
    tl.x=std::max(1,tl.x); tl.x=std::min(tl.x,imgIn.width()-1);
    tl.y=std::max(1,tl.y); tl.y=std::min(tl.y,imgIn.height()-1);
    br.x=std::max(1,br.x); br.x=std::min(br.x,imgIn.width()-1);
    br.y=std::max(1,br.y); br.y=std::min(br.y,imgIn.height()-1);
    cv::Rect rect(tl,br);

    // run Otsu algorithm to segment out the finger    
    cv::Mat imgInMatRoi(imgInMat,rect);
    cv::threshold(imgInMatRoi,imgInMatRoi,0,255,cv::THRESH_BINARY|cv::THRESH_OTSU);
    cv::cvtColor(imgInMat,imgOutMat,CV_GRAY2BGR);

    px.resize(2,0.0);
    bool ok=false;
    for (int y=tl.y; y<br.y; y++)
    {
        for (int x=tl.x; x<br.x; x++)
        {
            if (imgIn(x,y)>0)
            {
                // predict the center of the finger a bit shifted
                x+=3;    y+=5;
                px[0]=x; px[1]=y;
                cv::circle(imgOutMat,cv::Point(x,y),5,cv::Scalar(0,0,255),-1);
                ok=true;
                break;
            }
        }

        if (ok)
            break;
    }

    cv::circle(imgOutMat,ct,5,cv::Scalar(0,255,0),-1);
    cv::rectangle(imgOutMat,tl,br,cv::Scalar(255,255,255),4);
    return rect;
}


/************************************************************************/
double CalibModule::getMinVer() const
{
    Bottle info;
    igaze->getInfo(info);
    return info.find("min_allowed_vergence").asDouble();
}


/************************************************************************/
bool CalibModule::getGazeParams(const string &eye, const string &type, Matrix &M)
{
    if (((eye!="left") && (eye!="right")) ||
        ((type!="intrinsics") && (type!="extrinsics")))
        return false;

    Bottle info;
    igaze->getInfo(info);
    if (Bottle *pB=info.find(("camera_"+type+"_"+eye).c_str()).asList())
    {        
        M.resize((type=="intrinsics")?3:4,4);

        int cnt=0;
        for (int r=0; r<M.rows(); r++)
            for (int c=0; c<M.cols(); c++)
                M(r,c)=pB->get(cnt++).asDouble();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool CalibModule::pushExtrinsics(const string &eye, const Matrix &H)
{
    if ((eye!="left") && (eye!="right"))
        return false;

    Bottle options;
    Bottle &ext=options.addList();
    ext.addString(eye=="left"?"camera_extrinsics_left":"camera_extrinsics_right");
    Bottle &val=ext.addList();
    for (int r=0; r<H.rows(); r++)
        for (int c=0; c<H.cols(); c++)
            val.addDouble(H(r,c));

    return igaze->tweakSet(options);
}


/************************************************************************/
bool CalibModule::getDepth(const Vector &px, Vector &x, Vector &pxr)
{
    Bottle cmd,reply;
    cmd.addInt((int)px[0]);
    cmd.addInt((int)px[1]);
    depthRpcPort.write(cmd,reply);

    if (reply.size()<5)
        return false;

    x.resize(3);
    pxr.resize(2);
    x[0]=reply.get(0).asDouble();
    x[1]=reply.get(1).asDouble();
    x[2]=reply.get(2).asDouble();
    pxr[0]=reply.get(3).asInt();
    pxr[1]=reply.get(4).asInt();

    return (norm(x)>0.0);
}


/************************************************************************/
bool CalibModule::getDepthAveraged(const Vector &px, Vector &x, Vector &pxr,
                                   const int maxSamples)
{        
    x.resize(3,0.0);
    pxr.resize(2,0.0);

    Vector depth,pixel;
    double cnt=0.0;
    for (int i=0; i<maxSamples; i++)
    {
        if (getDepth(px,depth,pixel))
        {
            x+=depth;
            pxr+=pixel;
            cnt+=1.0;
        }
    }
    
    if (norm(x)==0.0)
        return false;

    x/=cnt;
    pxr/=cnt;
    return true;
}


/************************************************************************/
void CalibModule::openHand(IPositionControl *ipos)
{
    Vector poss(9,0.0);
    Vector vels(9,0.0);

    poss[0]=20.0; vels[0]=50.0;
    poss[1]=90.0; vels[1]=50.0;
    poss[2]=0.0;  vels[2]=50.0;
    poss[3]=0.0;  vels[3]=50.0;
    poss[4]=0.0;  vels[4]=50.0;
    poss[5]=0.0;  vels[5]=50.0;
    poss[6]=0.0;  vels[6]=50.0;
    poss[7]=0.0;  vels[7]=50.0;
    poss[8]=0.0;  vels[8]=100.0;

    printf("opening hand\n");
    int i0=nEncs-poss.length();
    for (int i=i0; i<nEncs; i++)
    {
        ipos->setRefAcceleration(i,1e9);
        ipos->setRefSpeed(i,vels[i-i0]);
        ipos->positionMove(i,poss[i-i0]);
    }
}


/************************************************************************/
void CalibModule::postureHelper(const Vector &gaze_ang, const Matrix &targetL,
                                const Matrix &targetR)
{
    IPositionControl  *ipos;
    ICartesianControl *icart;
    int ctxtL,ctxtR;
    Vector dof;

    printf("looking at target\n");
    igaze->lookAtAbsAngles(gaze_ang);

    if (useArmL)
    {
        drvArmL.view(ipos);
        openHand(ipos);

        drvCartL.view(icart);
        icart->storeContext(&ctxtL);
        icart->getDOF(dof);
        dof=1.0;
        icart->setDOF(dof,dof);
        icart->setLimits(0,0.0,0.0);
        icart->setLimits(1,0.0,0.0);
        icart->setLimits(2,0.0,0.0);
        icart->setTrajTime(1.0);

        printf("reaching for left target\n");
        icart->goToPoseSync(targetL.getCol(3),dcm2axis(targetL));
    }

    if (useArmR)
    {
        drvArmR.view(ipos);
        openHand(ipos);

        drvCartR.view(icart);
        icart->storeContext(&ctxtR);
        icart->getDOF(dof);
        dof=1.0;
        dof[0]=dof[1]=dof[2]=0.0;
        icart->setDOF(dof,dof);
        icart->setTrackingMode(true);
        icart->setTrajTime(1.0);

        printf("reaching for right target\n");
        icart->goToPoseSync(targetR.getCol(3),dcm2axis(targetR));

        printf("waiting for right arm... ");
        icart->waitMotionDone();
        printf("done\n");
    }

    if (useArmL)
    {
        drvCartL.view(icart);

        printf("waiting for left arm... ");
        icart->waitMotionDone();
        printf("done\n");

        icart->restoreContext(ctxtL);
        icart->deleteContext(ctxtL);
    }

    if (useArmR)
    {
        drvCartR.view(icart);
        icart->restoreContext(ctxtR);
        icart->deleteContext(ctxtR);
    }
}


/************************************************************************/
bool CalibModule::posture(const string &type)
{
    Vector gaze_ang(3,0.0);
    Matrix targetL=zeros(4,4);
    Matrix targetR=zeros(4,4);
    targetL(3,3)=targetR(3,3)=1.0;

    if (type=="home")
    {
        targetL(0,0)=targetR(0,0)=-1.0;
        targetL(2,1)=targetR(2,1)=-1.0;
        targetL(1,2)=targetR(1,2)=-1.0;

        targetL(0,3)=targetR(0,3)=-0.25;
        targetL(2,3)=targetR(2,3)=0.0;

        targetL(1,3)=-0.15;
        targetR(1,3)=0.15;
    }
    else if (type=="look_hands")
    {
        gaze_ang[1]=-35.0;
        gaze_ang[2]=block_eyes;

        targetL(2,0)=1.0;
        targetL(1,1)=-1.0;
        targetL(0,2)=1.0;
        targetL(0,3)=-0.25;
        targetL(1,3)=-0.04;
        targetL(2,3)=0.12;

        targetR(2,0)=1.0;
        targetR(1,1)=1.0;
        targetR(0,2)=-1.0;
        targetR(0,3)=-0.32;
        targetR(1,3)=0.04;
        targetR(2,3)=0.12;
    }
    else
        return false;

    postureHelper(gaze_ang,targetL,targetR);
    return true;
}


/************************************************************************/
bool CalibModule::calibrateDepth()
{
    if (depthRpcPort.getOutputCount()>0)
    {
        posture("look_hands");
        igaze->stopControl();
        Time::delay(1.0);

        Bottle cmd,reply;
        cmd.addString("calibrate");
        depthRpcPort.write(cmd,reply);
        if (reply.get(0).asString()=="ACK")
        {
            cmd.clear();
            cmd.addString("save");
            depthRpcPort.write(cmd,reply);
            if (reply.get(0).asString()=="ACK")
                return true;
        }
    }

    return false;
}


/************************************************************************/
void CalibModule::prepareRobot()
{
    // drive the hand in pointing pose
    Vector poss(9,0.0);
    Vector vels(9,0.0);

    poss[0]=40.0;  vels[0]=50.0;
    poss[1]=30.0;  vels[1]=50.0;
    poss[2]=20.0;  vels[2]=50.0;
    poss[3]=90.0;  vels[3]=50.0;
    poss[4]=00.0;  vels[4]=50.0;
    poss[5]=00.0;  vels[5]=50.0;
    poss[6]=70.0;  vels[6]=50.0;
    poss[7]=100.0; vels[7]=50.0;
    poss[8]=200.0; vels[8]=100.0;

    printf("configuring hand... ");
    int i0=nEncs-poss.length();
    for (int i=i0; i<nEncs; i++)
    {
        iposs->setRefAcceleration(i,1e9);
        iposs->setRefSpeed(i,vels[i-i0]);
        iposs->positionMove(i,poss[i-i0]);
    }

    bool done=false;
    while (!done)
    {
        Time::delay(1.0);
        iposs->checkMotionDone(i0+4,&done);
    }    
    printf("done\n");

    Vector encs(nEncs);
    iencs->getEncoders(encs.data());
    poss=encs.subVector(i0,nEncs-1);

    Vector joints;
    iCubFinger finger=this->finger;
    finger.getChainJoints(poss,joints);
    Vector xf=finger.getH(CTRL_DEG2RAD*joints).getCol(3);

    iarm->storeContext(&context_arm);
    
    Vector dof;
    iarm->getDOF(dof);
    dof=1.0;
    dof[1]=0.0;
    iarm->setDOF(dof,dof);

    double min, max;
    iarm->getLimits(0,&min,&max);
    iarm->setLimits(0,0.0,0.5*max);

    iarm->setInTargetTol(0.01);
    iarm->setTrajTime(1.0);
    iarm->attachTipFrame(xf,Vector(4,0.0));

    igaze->storeContext(&context_gaze);
    igaze->getNeckPitchRange(&min,&max);
    igaze->bindNeckPitch(0.9*min,max);
    igaze->blockNeckRoll(0.0);
    igaze->setNeckTrajTime(0.8);
}


/************************************************************************/
int CalibModule::removeOutliers()
{
    // calibrate using the current setting
    double error;
    calibrator->calibrate(error);

    deque<Vector> p_depth, p_kin;
    calibrator->getPoints(p_depth,p_kin);

    // compute the prediction errors
    Vector e(p_depth.size());
    for (size_t i=0; i<p_depth.size(); i++)
    {
        Vector x;
        calibrator->retrieve(p_depth[i],x);
        e[i]=norm(p_kin[i]-x);
    }

    // perform outliers removal
    ModifiedThompsonTau detector;
    set<size_t> idx=detector.detect(e,Property("(recursive)"));

    // feed the calibrator with inliers only
    if (!idx.empty())
    {
        calibrator->clearPoints();
        for (size_t i=0; i<p_depth.size(); i++)
            if (idx.find(i)==idx.end())
                calibrator->addPoints(p_depth[i],p_kin[i]);
    }

    return idx.size();
}


/************************************************************************/
void CalibModule::doMotorExploration()
{
    mutex.lock();

    // skip if in idle
    if (motorExplorationState==motorExplorationStateIdle)
    {
        mutex.unlock();
        return;
    }

    // end the movement
    if (motorExplorationAsyncStop ||
        ((motorExplorationState==motorExplorationStateTrigger) && targetsConsumed.empty()))
    {
        iarm->stopControl();
        iarm->restoreContext(context_arm);
        iarm->deleteContext(context_arm);

        igaze->stopControl();
        igaze->restoreContext(context_gaze);
        igaze->deleteContext(context_gaze);

        motorExplorationAsyncStop=enabled=false;
        motorExplorationState=motorExplorationStateIdle;

        mutex.unlock();
        return;
    }

    // generate new target
    if (motorExplorationState==motorExplorationStateTrigger)
    {
        // set up target (position part)
        Vector xd=targetsConsumed.front();
        targetsConsumed.pop_front();

        // set up target (orientation part)
        Matrix Hd=zeros(4,4);
        Hd(0,0)=-1.0;
        Hd(1,1)=(arm=="left"?-1.0:1.0);
        Hd(2,2)=(arm=="left"?1.0:-1.0);
        Hd(3,3)=1.0;

        Vector rot_y(4,0.0);
        rot_y[1]=1.0;
        rot_y[3]=(curExplorationCenter[2]>0.2?90.0:45.0)*CTRL_DEG2RAD;

        Vector od=dcm2axis(axis2dcm(rot_y)*Hd);
        printf("moving to xd=(%s); od=(%s); #%d remaining points\n",
               xd.toString(3,3).c_str(),od.toString(3,3).c_str(),(int)targetsConsumed.size());

        iarm->goToPoseSync(xd,od);
        igaze->lookAtFixationPoint(xd);

        motorExplorationState=motorExplorationStateReach;
    }
    // check if target has been attained
    else if (motorExplorationState==motorExplorationStateReach)
    {
        bool done=false;
        iarm->checkMotionDone(&done);
        if (done)
        {
            igaze->stopControl();
            mutex.unlock();
            Time::delay(exploration_wait);
            mutex.lock();
            motorExplorationState=motorExplorationStateLog;
        }
    }

    mutex.unlock();
}


/************************************************************************/
void CalibModule::doTouch(const Vector &xd)
{
    prepareRobot();

    // set up target (orientation part)
    Matrix Hd=zeros(4,4);
    Hd(0,0)=-1.0;
    Hd(1,1)=(arm=="left"?-1.0:1.0);
    Hd(2,2)=(arm=="left"?1.0:-1.0);
    Hd(3,3)=1.0;

    Vector od=dcm2axis(Hd);
    igaze->setTrackingMode(true);
    igaze->lookAtFixationPoint(xd);

    Vector x=xd; x[2]+=0.05;
    printf("moving to xd=(%s); od=(%s)\n",x.toString(3,3).c_str(),od.toString(3,3).c_str());
    iarm->goToPoseSync(x,od);
    iarm->waitMotionDone();

    printf("moving to xd=(%s); od=(%s)\n",xd.toString(3,3).c_str(),od.toString(3,3).c_str());
    iarm->setInTargetTol(0.001);
    iarm->goToPoseSync(xd,od);
    iarm->waitMotionDone(0.1,5.0);

    Time::delay(2.0);

    printf("moving to xd=(%s); od=(%s)\n",x.toString(3,3).c_str(),od.toString(3,3).c_str());
    iarm->goToPoseSync(x,od);
    iarm->waitMotionDone();

    iarm->setInTargetTol(0.01);

    x[0]=-0.35;
    x[1]=(arm=="left"?-0.2:0.2);
    x[2]=0.1;
    printf("moving to xd=(%s); od=(%s)\n",x.toString(3,3).c_str(),od.toString(3,3).c_str());
    iarm->setLimits(0,0.0,0.0);
    iarm->setLimits(2,0.0,0.0);
    iarm->goToPoseSync(x,od);
    iarm->waitMotionDone();

    iarm->restoreContext(context_arm);
    iarm->deleteContext(context_arm);

    igaze->stopControl();
    igaze->restoreContext(context_gaze);
    igaze->deleteContext(context_gaze);
}


/************************************************************************/
void CalibModule::doTest()
{
    printf("running test #%d\n",test);
    Rand::init();

    Vector x_rad(6);
    x_rad[0]=0.0;
    x_rad[1]=0.0;
    x_rad[2]=0.015;
    x_rad[3]=CTRL_DEG2RAD*0.0;
    x_rad[4]=CTRL_DEG2RAD*5.0;
    x_rad[5]=CTRL_DEG2RAD*2.3;
    
    Vector x_deg=x_rad.subVector(0,2);
    x_deg=cat(x_deg,CTRL_RAD2DEG*x_rad.subVector(3,5));
    Matrix H=computeH(x_rad);
    Matrix invH=SE3inv(H);

    bool ok=false;
    switch (test)
    {
        //*******************
        case 0:
        {
            printf("#0a \"check solver's robustness against seg-fault\"\n");
            printf("#0b \"check correctness of first-order derivative\"\n");

            Vector p2d(2); p2d[0]=160.0; p2d[1]=120.0;
            Vector p3d(4); p3d[0]=-0.5;  p3d[1]=0.0; p3d[2]=0.34; p3d[3]=1.0;
            aligner.addPoints(p2d,p3d);
            Matrix T; double error;
            aligner.calibrate(T,error,ALIGN_IPOPT_MAX_ITER,5,"first-order");
            ok=true;

            break;
        }

        //*******************
        case 1:
        {
            printf("#1a \"check solver's functionality\"\n");

            // generate synthetic data
            Matrix Prj=aligner.getProjection();
            for (int i=0; i<100; i++)
            {
                Vector p3d(4);
                p3d[0]=Rand::scalar(-0.5,0.5);
                p3d[1]=Rand::scalar(-0.5,0.5);
                p3d[2]=Rand::scalar(0.1,1.0);
                p3d[3]=1.0;

                Vector p2d=Prj*invH*p3d;
                p2d=p2d/p2d[2];
                p2d.pop_back();
                p2d+=NormRand::vector(2,0.0,5.0);   // add up some noise

                aligner.addPoints(p2d,p3d);
            }   

            Matrix H1; double error;
            ok=aligner.calibrate(H1,error);
            Vector x1=cat(H1.getCol(3).subVector(0,2),
                          CTRL_RAD2DEG*dcm2rpy(H1));

            printf("x=(%s)\n",x_deg.toString(5,5).c_str());
            printf("H\n%s\n\n",H.toString(5,5).c_str());
            printf("solution_x=(%s)\n",x1.toString(5,5).c_str());
            printf("solution_H\n%s\n\n",H1.toString(5,5).c_str());
            printf("error=%g\n",error);

            break;
        }

        //*******************
        case 2:
        {
            printf("#2a \"check exploration space\"\n");
            ok=setExplorationSpaceDelta(0.0,0.0,0.0,0.0,0.0);

            break;
        }

        //*******************
        case 3:
        {
            printf("#3a \"check saving data\"\n");

            // generate synthetic data
            for (int i=0; i<10; i++)
            {
                Vector p(4);
                p[0]=Rand::scalar(-0.5,0.5);
                p[1]=Rand::scalar(-0.5,0.5);
                p[2]=Rand::scalar(0.1,1.0);
                p[3]=1.0;

                calibrator->addPoints(p,H*p);
            }

            arm="left";
            experts=&expertsL;
            Property ret=calibrate(false);
            pushCalibrator();
            printf("H\n%s\n\n",H.toString(5,5).c_str());
            printf("calibration output\n%s\n",ret.toString().c_str());
            ok=save();

            printf("#3b \"check retrieving data\"\n");

            Vector in(4);
            in[0]=1.0;
            in[1]=-1.0;
            in[2]=2.0;
            in[3]=1.0;

            Vector out=H*in;
            Vector pred;
            ok&=experts->retrieve(in,pred);
            printf("in=(%s); out=H*in=(%s); pred=(%s)\n",
                   in.subVector(0,2).toString(3,3).c_str(),
                   out.subVector(0,2).toString(3,3).c_str(),
                   pred.toString(3,3).c_str());

            printf("#3c \"check logging data\"\n");
            ok&=log("experts");
        }
    }

    printf("test #%d %s!\n",test,ok?"passed":"failed/unknown");
}


/************************************************************************/
CalibModule::CalibModule() : depthInPort(this) { }


/************************************************************************/
bool CalibModule::configure(ResourceFinder &rf)
{
    this->rf=&rf;
    string robot=rf.check("robot",Value("icub")).asString().c_str();
    string name=rf.check("name",Value("depth2kin")).asString().c_str();
    string type=rf.check("type",Value("se3+scale")).asString().c_str();
    test=rf.check("test",Value(-1)).asInt();    
    max_dist=fabs(rf.check("max_dist",Value(0.25)).asDouble());
    roi_side=abs(rf.check("roi_side",Value(100)).asInt());
    block_eyes=fabs(rf.check("block_eyes",Value(5.0)).asDouble());
    exploration_wait=fabs(rf.check("exploration_wait",Value(0.5)).asDouble());

    motorExplorationAsyncStop=false;
    motorExplorationState=motorExplorationStateIdle;

    enabled=false;
    isSaved=true;
    closing=false;
    exp_depth2kin=true;
    exp_aligneyes=false;
    touchWithExperts=true;

    if (!isTypeValid(type))
    {
        printf("Unknown method %s!\n",type.c_str());
        return false;
    }
    
    load();
    calibrator=factory(type);

    Vector min(6),max(6);
    min[0]=-0.005;             max[0]=0.005;
    min[1]=-0.005;             max[1]=0.005;
    min[2]=-0.01;              max[2]=0.01;
    min[3]=-CTRL_DEG2RAD*15.0; max[3]=CTRL_DEG2RAD*15.0;    // roll
    min[4]=-CTRL_DEG2RAD*15.0; max[4]=CTRL_DEG2RAD*15.0;    // pitch
    min[5]=-CTRL_DEG2RAD*15.0; max[5]=CTRL_DEG2RAD*15.0;    // yaw
    aligner.setBounds(min,max);
    aligner.setInitialGuess(eye(4,4));

    if (test>=0)
    {
        Matrix K=eye(3,4);
        K(0,0)=257.34; K(1,1)=257.34;
        K(0,2)=160.0;  K(1,2)=120.0; 

        aligner.setProjection(K);
        return true;
    }

    // open drivers
    Property optionArmL("(device remote_controlboard)");
    optionArmL.put("remote",("/"+robot+"/left_arm").c_str());
    optionArmL.put("local",("/"+name+"/joint/left").c_str());
    if (!drvArmL.open(optionArmL))
        printf("Position left_arm controller not available!\n");

    Property optionArmR("(device remote_controlboard)");
    optionArmR.put("remote",("/"+robot+"/right_arm").c_str());
    optionArmR.put("local",("/"+name+"/joint/right").c_str());
    if (!drvArmR.open(optionArmR))
        printf("Position right_arm controller not available!\n");

    Property optionCartL("(device cartesiancontrollerclient)");
    optionCartL.put("remote",("/"+robot+"/cartesianController/left_arm").c_str());
    optionCartL.put("local",("/"+name+"/cartesian/left").c_str());
    if (!drvCartL.open(optionCartL))
        printf("Cartesian left_arm controller not available!\n");

    Property optionCartR("(device cartesiancontrollerclient)");
    optionCartR.put("remote",("/"+robot+"/cartesianController/right_arm").c_str());
    optionCartR.put("local",("/"+name+"/cartesian/right").c_str());
    if (!drvCartR.open(optionCartR))
        printf("Cartesian right_arm controller not available!\n");

    Property optionGaze("(device gazecontrollerclient)");
    optionGaze.put("remote","/iKinGazeCtrl");
    optionGaze.put("local",("/"+name+"/gaze").c_str());
    if (!drvGaze.open(optionGaze))
        printf("Gaze controller not available!\n");
    
    // set up some global vars
    useArmL=(drvArmL.isValid()==drvCartL.isValid());
    useArmR=(drvArmR.isValid()==drvCartR.isValid());
    selectArmEnabled=(useArmL && useArmR);

    // quitting condition
    if (!drvGaze.isValid() || (!useArmL && !useArmR))
    {
        printf("Something wrong occured while configuring drivers... quitting!\n");
        terminate();
        return false;
    }

    // set up initial arm and experts
    arm=(useArmL?"left":"right");
    experts=&(arm=="left"?expertsL:expertsR);

    // open devices views
    IControlLimits *ilim;
    (arm=="left")?drvArmL.view(iencs):drvArmR.view(iencs);
    (arm=="left")?drvArmL.view(iposs):drvArmR.view(iposs);
    (arm=="left")?drvArmL.view(ilim):drvArmR.view(ilim);
    (arm=="left")?drvCartL.view(iarm):drvCartR.view(iarm);    
    drvGaze.view(igaze);
    iencs->getAxes(&nEncs);

    Matrix Prj;
    if (!getGazeParams("left","intrinsics",Prj))
    {
        printf("Intrinsic parameters for left camera not available!\n");
        terminate();
        return false;
    }
    aligner.setProjection(Prj);

    finger=iCubFinger(arm+"_index");
    deque<IControlLimits*> lim;
    lim.push_back(ilim);
    finger.alignJointsBounds(lim);

    double minVer=getMinVer();
    if (block_eyes<minVer)
    {
        block_eyes=minVer;
        printf("*** warning: blockEyes saturated at minimum allowed vergence angle %g\n",block_eyes);
    }

    touchInPort.open(("/"+name+"/touch:i").c_str());
    depthInPort.open(("/"+name+"/depth:i").c_str());
    depthOutPort.open(("/"+name+"/depth:o").c_str());
    depthRpcPort.open(("/"+name+"/depth:rpc").c_str());
    rpcPort.open(("/"+name+"/rpc").c_str());
    depthInPort.useCallback();
    attach(rpcPort);

    setExplorationSpaceDelta(0.0,0.0,0.0,0.0,0.0);

    // request high resolution scheduling
    Time::turboBoost();

    return true;
}


/************************************************************************/
void CalibModule::onRead(ImageOf<PixelMono> &imgIn)
{
    mutex.lock();

    Vector kinPoint,o;
    iarm->getPose(kinPoint,o);

    Vector c,tipl(2,0.0),tipr(2,0.0);
    igaze->get2DPixel(0,kinPoint,c);

    ImageOf<PixelBgr> imgOut;
    cv::Rect rect=extractFingerTip(imgIn,imgOut,c,tipl);
    
    bool holdImg=false;
    if (motorExplorationState==motorExplorationStateLog)
    {
        cv::Scalar color=cv::Scalar(0,0,255);
        string tag="discarded";

        if (enabled && (depthRpcPort.getOutputCount()>0))
        {
            Vector depthPoint;
            if (getDepthAveraged(tipl,depthPoint,tipr))
            {
                // consistency check
                double dist=norm(depthPoint-kinPoint);
                if (dist<max_dist)
                {
                    color=cv::Scalar(0,255,0);
                    tag="matched";

                    if (exp_depth2kin)
                    {
                        calibrator->addPoints(depthPoint,kinPoint);
                        printf("collecting calibration points: p_depth=(%s); p_kin=(%s);\n",
                               depthPoint.toString(3,3).c_str(),kinPoint.toString(3,3).c_str());

                        tag="acquired";
                    }

                    if (exp_aligneyes)
                    {
                        Vector kinPoint4=kinPoint;
                        kinPoint4.push_back(1.0);

                        Vector xe,oe,kinPoint_e;
                        Matrix He;

                        igaze->getLeftEyePose(xe,oe);
                        He=axis2dcm(oe);
                        xe.push_back(1.0);
                        He.setCol(3,xe);
                        kinPoint_e=SE3inv(He)*kinPoint4;

                        aligner.addPoints(tipl,kinPoint_e);
                        printf("collecting points for aligning eye: tip=(%s); p_kin=(%s);\n",
                               tipl.toString(3,3).c_str(),kinPoint_e.toString(3,3).c_str());

                        tag="acquired";
                    }
                }
                else
                    printf("discarding calibration points: p_depth=(%s); p_kin=(%s); (|p_depth-p_kin|=%g)>%g\n",
                           depthPoint.toString(3,3).c_str(),kinPoint.toString(3,3).c_str(),dist,max_dist);
            }
            else
                printf("unavailable depth; discarding...\n");
        }

        cv::Mat img((IplImage*)imgOut.getIplImage());
        cv::putText(img,tag.c_str(),cv::Point(rect.x+5,rect.y+15),CV_FONT_HERSHEY_SIMPLEX,0.5,color);

        motorExplorationState=motorExplorationStateTrigger;
        holdImg=true;
    }
    
    if (depthOutPort.getOutputCount()>0)
    {
        depthOutPort.prepare()=imgOut;
        depthOutPort.write();
        if (holdImg)
            Time::delay(0.5);
    }

    mutex.unlock();
}


/************************************************************************/
int CalibModule::getNumExperts()
{
    mutex.lock();
    int num=(int)experts->size();
    mutex.unlock();

    return num;
}


/************************************************************************/
bool CalibModule::clearExperts()
{
    mutex.lock();
    experts->clear();
    mutex.unlock();

    return true;
}


/************************************************************************/
bool CalibModule::load()
{
    string fileName=rf->findFile("calibrationFile").c_str();
    if (fileName.empty())
    {
        printf("calibration file not found\n");
        return false;
    }

    Property data; data.fromConfigFile(fileName.c_str()); 
    Bottle b; b.read(data);

    mutex.lock();
    printf("loading experts from file: %s\n",fileName.c_str());
    for (int i=0; i<b.size(); i++)
        factory(b.get(i));
    mutex.unlock();

    return true;
}


/************************************************************************/
bool CalibModule::save()
{
    mutex.lock();
    if (!isSaved)
    {
        ofstream fout;
        string contextPath=rf->getHomeContextPath().c_str();
        string fileName=rf->find("calibrationFile").asString().c_str();
        fileName=contextPath+"/"+fileName;
        
        printf("saving experts into file: %s\n",fileName.c_str());
        fout.open(fileName.c_str());
        
        if (fout.is_open())
        {        
            for (size_t i=0; i<expertsL.size(); i++)
            {
                Property info;
                expertsL[i]->toProperty(info);
                info.put("arm","left");
                ostringstream entry;
                entry<<"expert_left_"<<i;
                fout<<entry.str()<<" "<<info.toString().c_str()<<endl;
            }

            for (size_t i=0; i<expertsR.size(); i++)
            {
                Property info;
                expertsR[i]->toProperty(info);
                info.put("arm","right");
                ostringstream entry;
                entry<<"expert_right_"<<i;
                fout<<entry.str()<<" "<<info.toString().c_str()<<endl;
            }        

            fout.close();
            isSaved=true;
        }
    }

    mutex.unlock();
    return isSaved;
}


/************************************************************************/
bool CalibModule::log(const string &type)
{
    if ((type!="experts") && (type!="calibrator"))
        return false;

    if ((type=="calibrator") && !calibrated)
        return false;

    mutex.lock();
    deque<Vector> p_depth, p_kin;
    calibrator->getPoints(p_depth,p_kin);

    ofstream fout;
    string contextPath=rf->getHomeContextPath().c_str();
    string fileName=contextPath+"/points_"+arm+"_"+type+".log";

    printf("logging data into file: %s\n",fileName.c_str());
    fout.open(fileName.c_str());

    bool ret=false;
    bool useExperts=(type=="experts");
    if (fout.is_open())
    {
        size_t i;
        for (i=0; i<p_depth.size(); i++)
        {
            Vector x;
            if (!(useExperts?experts->retrieve(p_depth[i],x):
                             calibrator->retrieve(p_depth[i],x)))
                break;

            fout<<p_depth[i].toString(3,3).c_str();
            fout<<" ";
            fout<<p_kin[i].toString(3,3).c_str();
            fout<<" ";
            fout<<x.toString(3,3).c_str();
            fout<<" ";
            fout<<norm(p_kin[i]-x);
            fout<<endl;
        }

        fout.close();
        ret=(i>=p_depth.size());
    }

    mutex.unlock();
    return ret;
}


/************************************************************************/
bool CalibModule::explore()
{
    prepareRobot();
    targetsConsumed=targets;
    motorExplorationState=motorExplorationStateTrigger;
    enabled=true;

    return true;
}


/************************************************************************/
bool CalibModule::stop()
{
    mutex.lock();
    printf("received stop command => stopping exploration...\n");
    motorExplorationAsyncStop=true;
    mutex.unlock();
    return true;
}


/************************************************************************/
bool CalibModule::setMaxDist(const double max_dist)
{
    this->max_dist=fabs(max_dist);
    return true;
}


/************************************************************************/
double CalibModule::getMaxDist()
{
    return max_dist;
}


/************************************************************************/
bool CalibModule::setRoi(const int side)
{
    this->roi_side=abs(side);
    return true;
}


/************************************************************************/
int CalibModule::getRoi()
{
    return roi_side;
}


/************************************************************************/
bool CalibModule::setBlockEyes(const double block_eyes)
{
    if (block_eyes>=getMinVer())
    {
        this->block_eyes=block_eyes;
        return true;
    }
    else
        return false;
}


/************************************************************************/
double CalibModule::getBlockEyes()
{
    return block_eyes;
}


/************************************************************************/
bool CalibModule::blockEyes()
{
    return igaze->blockEyes(block_eyes);
}


/************************************************************************/
bool CalibModule::clearEyes()
{
    return igaze->clearEyes();
}


/************************************************************************/
bool CalibModule::setArm(const string &arm)
{
    if (!selectArmEnabled || ((arm!="left") && (arm!="right")))
        return false;

    this->arm=arm;
    experts=&(arm=="left"?expertsL:expertsR);
    (arm=="left")?drvCartL.view(iarm):drvCartR.view(iarm);
    (arm=="left")?drvArmL.view(iencs):drvArmR.view(iencs);
    (arm=="left")?drvArmL.view(iposs):drvArmR.view(iposs);
    finger=iCubFinger(arm+"_index");
    return true;
}


/************************************************************************/
string CalibModule::getArm()
{
    return arm;
}


/************************************************************************/
bool CalibModule::setCalibrationType(const string &type, const string &extrapolation)
{
    if (isTypeValid(type))
    {
        delete calibrator;
        calibrator=factory(type);
        (extrapolation=="auto")?calibrator->setExtrapolation(type!="lssvm"):
                                calibrator->setExtrapolation(extrapolation=="true");

        return true; 
    }
    else
        return false;
}


/************************************************************************/
string CalibModule::getCalibrationType()
{
    return calibrator->getType();
}


/************************************************************************/
Property CalibModule::calibrate(const bool rm_outliers)
{
    mutex.lock();
    double error;
    Matrix H;
    Property reply;

    if (exp_depth2kin)
    {
        if (rm_outliers)
            reply.put("outliers_removed",removeOutliers());

        calibrator->calibrate(error);
        reply.put("calibrator",error);
        calibrated=true;
    }

    if (exp_aligneyes)
    {
        aligner.calibrate(H,error);
        reply.put("aligner",error);

        Matrix HL,HR;
        if (getGazeParams("left","extrinsics",HL) && getGazeParams("right","extrinsics",HR))
        {
            Bottle cmd,reply;
            cmd.addString("getH");
            depthRpcPort.write(cmd,reply);
            Matrix HRL; reply.write(HRL);
            Matrix HLR=SE3inv(HRL);

            Vector x,o;
            igaze->getLeftEyePose(x,o);
            Matrix TL=axis2dcm(o);
            TL(0,3)=x[0];
            TL(1,3)=x[1];
            TL(2,3)=x[2];

            igaze->getRightEyePose(x,o);
            Matrix TR=axis2dcm(o);
            TR(0,3)=x[0];
            TR(1,3)=x[1];
            TR(2,3)=x[2];

            HL=HL*H;
            HR=SE3inv(TR)*(TL*HL*HLR);

            pushExtrinsics("left",HL);
            pushExtrinsics("right",HR);
        }
    }

    mutex.unlock();
    return reply;
}


/************************************************************************/
bool CalibModule::pushCalibrator()
{
    mutex.lock();
    bool ret=false;
    if (calibrated)
    {
        (*experts)<<*calibrator;
        isSaved=false;
        ret=true;
    }
    mutex.unlock();
    return ret;
}


/************************************************************************/
bool CalibModule::setTouchWithExperts(const string &sw)
{
    if ((sw!="on") && (sw!="off"))
        return false;

    touchWithExperts=(sw=="on");
    return true;
}


/************************************************************************/
string CalibModule::getTouchWithExperts()
{
    return (touchWithExperts?"on":"off");
}


/************************************************************************/
bool CalibModule::touch(const int u, const int v)
{
    Vector px(2);
    px[0]=u;
    px[1]=v;

    printf("received touch request for pixel=(%d %d); => ",u,v);

    Vector in,pxr;
    if (getDepthAveraged(px,in,pxr))
    {
        printf("p_depth=(%s); => ",in.toString(3,3).c_str());

        Vector out;
        if (touchWithExperts)
        {
            printf("(apply correction) => ");
            experts->retrieve(in,out);
        }
        else
            out=in;

        printf("p_kin=(%s);\n",out.toString(3,3).c_str());
        doTouch(out);

        return true;
    }
    else
    {
        printf("unavailable depth; discarding...\n");
        return false;
    }
}


/************************************************************************/
PointReq CalibModule::getPoint(const string &arm, const double x, const double y,
                               const double z)
{
    Vector in(3);
    in[0]=x;
    in[1]=y;
    in[2]=z;

    LocallyWeightedExperts *experts=&(arm=="left"?expertsL:expertsR);

    Vector out;
    PointReq reply("fail",x,y,z);
    if (experts->retrieve(in,out))
    {
        reply.result="ok";
        reply.x=out[0];
        reply.y=out[1];
        reply.z=out[2];
    }

    return reply;
}


/************************************************************************/
bool CalibModule::setExperiment(const string &exp, const string &v)
{
    if ((v!="on") && (v!="off"))
        return false;

    if (exp=="depth2kin") 
    {
        exp_depth2kin=(v=="on");
        return true;
    }
    else if (exp=="aligneyes")
    {
        exp_aligneyes=(v=="on");
        return true;
    }
    else
        return false;
}


/************************************************************************/
string CalibModule::getExperiment(const string &exp)
{
    if (exp=="depth2kin")
        return (exp_depth2kin?"on":"off");
    else if (exp=="aligneyes")
        return (exp_aligneyes?"on":"off");
    else
        return string("");
}


/************************************************************************/
Vector CalibModule::getExtrinsics(const string &eye)
{
    Matrix H;
    Vector reply(6,0.0);
    if (getGazeParams(eye,"extrinsics",H))
        reply=cat(H.getCol(3).subVector(0,2),
                  CTRL_RAD2DEG*dcm2rpy(H));

    return reply;
}


/************************************************************************/
bool CalibModule::resetExtrinsics(const string &eye)
{
    return pushExtrinsics(eye,yarp::math::eye(4,4));
}


/************************************************************************/
bool CalibModule::setExplorationWait(const double wait)
{
    exploration_wait=fabs(wait);
    return true;
}


/************************************************************************/
double CalibModule::getExplorationWait()
{
    return exploration_wait;
}


/************************************************************************/
bool CalibModule::setExplorationSpace(const double cx, const double cy,
                                      const double cz, const double a,
                                      const double b)
{
    Vector c(3),size(2);
    c[0]=cx;
    c[1]=cy;
    c[2]=cz;
    size[0]=a;
    size[1]=b;
    
    return createTargets(c,size);
}


/************************************************************************/
bool CalibModule::setExplorationSpaceDelta(const double dcx, const double dcy,
                                           const double dcz, const double da,
                                           const double db)
{
    Vector dc(3),dsize(2);
    dc[0]=dcx;
    dc[1]=dcy;
    dc[2]=dcz;
    dsize[0]=da;
    dsize[1]=db;
    
    Vector c(3),size(2);
    c[0]=-0.4; c[1]=0.0; c[2]=0.05;
    size[0]=0.10; size[1]=0.05;
    return createTargets(c+dc,size+dsize);
}


/************************************************************************/
Property CalibModule::getExplorationData()
{
    mutex.lock();
    Property reply;

    reply.put("status",(motorExplorationState==motorExplorationStateIdle?"idle":"ongoing"));
    reply.put("total_points",(int)targets.size());
    reply.put("remaining_points",(int)targetsConsumed.size());
    reply.put("calibrator_points",(int)calibrator->getNumPoints());
    reply.put("aligner_points",(int)aligner.getNumPoints());

    mutex.unlock();
    return reply;
}


/************************************************************************/
bool CalibModule::clearExplorationData()
{
    mutex.lock();
    if (exp_depth2kin)
        calibrator->clearPoints(); 

    if (exp_aligneyes)
        aligner.clearPoints();

    mutex.unlock();
    return true;
}


/************************************************************************/
bool CalibModule::quit()
{
    return closing=true;
}


/************************************************************************/
double CalibModule::getPeriod()
{
    return 0.03;
}


/************************************************************************/
bool CalibModule::updateModule()
{
    if (test>=0)
    {
        doTest();
        return false;
    }

    doMotorExploration();

    // handle touch request
    if (Bottle *touchData=touchInPort.read(false))
    {
        if (touchData->size()>=2)
            touch(touchData->get(0).asInt(),touchData->get(1).asInt());
    }

    return !closing;
}


/************************************************************************/
void CalibModule::terminate()
{
    if (!touchInPort.isClosed())
    {
        touchInPort.close();        
        depthInPort.close();
        depthOutPort.close();
        depthRpcPort.close();
        rpcPort.close();
    }

    if (drvArmL.isValid())
        drvArmL.close();

    if (drvArmR.isValid())
        drvArmR.close();

    if (drvCartL.isValid())
        drvCartL.close();

    if (drvCartR.isValid())
        drvCartR.close();

    if (drvGaze.isValid())
        drvGaze.close();
}


/************************************************************************/
bool CalibModule::close()
{
    save();
    terminate();
    delete calibrator;
    return true;
}


