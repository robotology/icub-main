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
#include <iCub/localizer.h>
#include <iCub/solver.h>

#include <stdio.h>


/************************************************************************/
Localizer::Localizer(exchangeData *_commData, const string &_localName,
                     const string &_configFile, const bool _headV2,
                     const unsigned int _period) :
                     RateThread(_period), localName(_localName),
                     commData(_commData), configFile(_configFile),
                     headV2(_headV2),     period(_period)
{
    iCubHeadCenter eyeC(headV2?"right_v2":"right");
    eyeL=new iCubEye(headV2?"left_v2":"left");
    eyeR=new iCubEye(headV2?"right_v2":"right");

    // remove constraints on the links
    // we use the chains for logging purpose
    eyeL->setAllConstraints(false);
    eyeR->setAllConstraints(false);

    // release links
    eyeL->releaseLink(0); eyeC.releaseLink(0); eyeR->releaseLink(0);
    eyeL->releaseLink(1); eyeC.releaseLink(1); eyeR->releaseLink(1);
    eyeL->releaseLink(2); eyeC.releaseLink(2); eyeR->releaseLink(2);

    // add aligning links read from configuration file
    if (getAlignLinks(configFile,"ALIGN_KIN_LEFT",&alignLnkLeft1,&alignLnkLeft2))
    {
        iKinChain &chain=*(eyeL->asChain());

        chain<<*alignLnkLeft1<<*alignLnkLeft2;
        chain.blockLink(chain.getN()-1,0.0);
        chain.blockLink(chain.getN()-2,0.0);
    }
    else
        alignLnkLeft1=alignLnkLeft2=NULL;

    if (getAlignLinks(configFile,"ALIGN_KIN_RIGHT",&alignLnkRight1,&alignLnkRight2))
    {
        iKinChain &chain=*(eyeR->asChain());

        chain<<*alignLnkRight1<<*alignLnkRight2;
        chain.blockLink(chain.getN()-1,0.0);
        chain.blockLink(chain.getN()-2,0.0);
    }
    else
        alignLnkRight1=alignLnkRight2=NULL;

    // get the absolute reference frame of the head
    Vector q(eyeC.getDOF()); q=0.0;
    eyeCAbsFrame=eyeC.getH(q);
    // ... and its inverse
    invEyeCAbsFrame=SE3inv(eyeCAbsFrame);

    // get camera projection matrix from the configFile
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_LEFT",&PrjL))
    {
        Matrix &Prj=*PrjL;
        cxl=Prj(0,2);
        cyl=Prj(1,2);

        invPrjL=new Matrix(pinv(Prj.transposed()).transposed());
    }
    else
        PrjL=invPrjL=NULL;

    // get camera projection matrix from the configFile
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_RIGHT",&PrjR))
    {
        Matrix &Prj=*PrjR;
        cxr=Prj(0,2);
        cyr=Prj(1,2);

        invPrjR=new Matrix(pinv(Prj.transposed()).transposed());
    }
    else
        PrjR=invPrjR=NULL;

    Vector Kp(1), Ki(1), Kd(1);
    Vector Wp(1), Wi(1), Wd(1);
    Vector N(1),  Tt(1);
    Matrix satLim(1,2);

    Kp=0.001;
    Ki=0.001;
    Kd=0.0;

    Wp=Wi=Wd=1.0;

    N=10.0;
    Tt=1.0;

    satLim(0,0)=0.05;
    satLim(0,1)=10.0;

    pid=new parallelPID(0.05,Kp,Ki,Kd,Wp,Wi,Wd,N,Tt,satLim);

    Vector z0(1); z0[0]=0.5;
    pid->reset(z0);
    dominantEye="left";

    port_xd=NULL;
}


/************************************************************************/
bool Localizer::threadInit()
{ 
    port_mono.open((localName+"/mono:i").c_str());
    port_stereo.open((localName+"/stereo:i").c_str());
    port_anglesIn.open((localName+"/angles:i").c_str());
    port_anglesOut.open((localName+"/angles:o").c_str());

    fprintf(stdout,"Starting Localizer at %d ms\n",period);

    return true;
}


/************************************************************************/
void Localizer::afterStart(bool s)
{
    if (s)
        fprintf(stdout,"Localizer started successfully\n");
    else
        fprintf(stdout,"Localizer did not start\n");
}


/************************************************************************/
void Localizer::getPidOptions(Bottle &options)
{
    pid->getOptions(options);

    Bottle &bDominantEye=options.addList();
    bDominantEye.addString("dominantEye");
    bDominantEye.addString(dominantEye.c_str());
}


/************************************************************************/
void Localizer::setPidOptions(const Bottle &options)
{
    pid->setOptions(options);

    Bottle &opt=const_cast<Bottle&>(options);
    if (opt.check("dominantEye"))
    {
        string domEye=opt.find("dominantEye").asString().c_str();
        if ((domEye=="left") || (domEye=="right"))
            dominantEye=domEye;
    }
}


/************************************************************************/
Vector Localizer::getCurAbsAngles()
{
    Vector fp=commData->get_x();
    fp.push_back(1.0);  // impose homogeneous coordinates

    // get fp wrt head-centered system
    Vector fph=invEyeCAbsFrame*fp;

    Vector q=commData->get_q();
    Vector ang(3);

    ang[0]=atan2(fph[0],fph[2]);
    ang[1]=-atan2(fph[1],fph[2]);
    ang[2]=q[5];

    return ang;
}


/************************************************************************/
Vector Localizer::getFixationPoint(const string &type, const Vector &ang)
{
    double azi=ang[0];
    double ele=ang[1];
    double ver=ang[2];

    Vector q(8);
    if (type=="rel")
    {
        Vector torso=commData->get_torso();
        Vector head=commData->get_q();

        q[0]=torso[0];
        q[1]=torso[1];
        q[2]=torso[2];
        q[3]=head[0];
        q[4]=head[1];
        q[5]=head[2];
        q[6]=head[3];
        q[7]=head[4];

        ver+=head[5];
    }
    else
        q=0.0;
    
    // impose vergence != 0.0
    if (ver<MINALLOWED_VERGENCE*CTRL_DEG2RAD)
        ver=MINALLOWED_VERGENCE*CTRL_DEG2RAD;

    q[7]+=ver/2.0;
    eyeL->setAng(q);

    q[7]-=ver;
    eyeR->setAng(q);

    Vector fp(4);
    fp[3]=1.0;  // impose homogeneous coordinates

    // compute new fp due to changed vergence
    computeFixationPointOnly(*(eyeL->asChain()),*(eyeR->asChain()),fp);

    // compute rotational matrix to
    // account for elevation and azimuth
    Vector x(4); Vector y(4);
    x[0]=1.0;    y[0]=0.0;
    x[1]=0.0;    y[1]=1.0;
    x[2]=0.0;    y[2]=0.0;
    x[3]=ele;    y[3]=azi;   
    Matrix R=axis2dcm(y)*axis2dcm(x);

    Vector fph, xd;
    if (type=="rel")
    {
        Matrix frame=commData->get_fpFrame();
        fph=SE3inv(frame)*fp;       // get fp wrt relative head-centered frame
        xd=frame*(R*fph);           // apply rotation and retrieve fp wrt root frame
    }
    else
    {
        fph=invEyeCAbsFrame*fp;     // get fp wrt absolute head-centered frame
        xd=eyeCAbsFrame*(R*fph);    // apply rotation and retrieve fp wrt root frame
    }

    Vector xo(3);
    xo[0]=xd[0];
    xo[1]=xd[1];
    xo[2]=xd[2];

    return xo;
}


/************************************************************************/
bool Localizer::projectPoint(const string &type, const double u, const double v,
                             const double z, Vector &fp)
{
    bool isLeft=(type=="left");

    Matrix  *invPrj=(isLeft?invPrjL:invPrjR);
    iCubEye *eye=(isLeft?eyeL:eyeR);

    if (invPrj)
    {
        Vector torso=commData->get_torso();
        Vector head=commData->get_q();

        Vector q(8);
        q[0]=torso[0];
        q[1]=torso[1];
        q[2]=torso[2];
        q[3]=head[0];
        q[4]=head[1];
        q[5]=head[2];
        q[6]=head[3];

        if (isLeft)
            q[7]=head[4]+head[5]/2.0;
        else
            q[7]=head[4]-head[5]/2.0;

        Vector x(3);
        x[0]=z*u;
        x[1]=z*v;
        x[2]=z;

        // find the 3D position from the 2D projection,
        // knowing the distance z from the camera
        Vector xe=*invPrj*x;
        xe[3]=1.0;  // impose homogeneous coordinates                

        // update position wrt the root frame
        Vector xo=eye->getH(q)*xe;

        fp.resize(3);
        fp[0]=xo[0];
        fp[1]=xo[1];
        fp[2]=xo[2];

        return true;
    }
    else
    {
        fprintf(stdout,"Unspecified projection matrix for %s camera!\n",type.c_str());
        return false;
    }
}


/************************************************************************/
void Localizer::handleMonocularInput()
{
    if (Bottle *mono=port_mono.read(false))
    {
        if (mono->size()>=4)
        {
            string type=mono->get(0).asString().c_str();
            double u=mono->get(1).asDouble();
            double v=mono->get(2).asDouble();
            double z=mono->get(3).asDouble();

            Vector fp;
            if (projectPoint(type,u,v,z,fp))
            {
                if (port_xd!=NULL)
                    port_xd->set_xd(fp);
                else
                    fprintf(stdout,"Internal error occured!\n");
            }
        }
        else
            fprintf(stdout,"Got wrong mono information!\n");
    }
}


/************************************************************************/
void Localizer::handleStereoInput()
{
    if (Bottle *stereo=port_stereo.read(false))
    {
        if ((PrjL!=NULL) || (PrjR!=NULL))
        {
            if (stereo->size()>=4)
            {
                double ul=stereo->get(0).asDouble();
                double vl=stereo->get(1).asDouble();
                double ur=stereo->get(2).asDouble();
                double vr=stereo->get(3).asDouble();

                Vector ref(1), fb(1), fp;
                double u, v;

                ref=0.0;
                if (dominantEye=="left")
                {
                    u=ul;
                    v=vl;
                    fb=cxr-ur;
                    // by inverting the sign of the error (e=ref-fb=-fb)
                    // we can keep gains always positive
                }
                else
                {                    
                    u=ur;
                    v=vr;
                    fb=ul-cxl;
                }
                
                Vector z=pid->compute(ref,fb);

                if (projectPoint(dominantEye,u,v,z[0],fp))
                {
                    if (port_xd!=NULL)
                        port_xd->set_xd(fp);
                    else
                        fprintf(stdout,"Internal error occured!\n");
                }
            }
            else
                fprintf(stdout,"Got wrong stereo information!\n");
        }
        else
            fprintf(stdout,"Unspecified projection matrix!\n");
    }
}


/************************************************************************/
void Localizer::handleAnglesInput()
{
    if (Bottle *angles=port_anglesIn.read(false))
    {
        if (angles->size()>=4)
        {
            Vector ang(3);
        
            string type=angles->get(0).asString().c_str();
            ang[0]=CTRL_DEG2RAD*angles->get(1).asDouble();
            ang[1]=CTRL_DEG2RAD*angles->get(2).asDouble();
            ang[2]=CTRL_DEG2RAD*angles->get(3).asDouble();
        
            Vector xd=getFixationPoint(type,ang);
        
            if (port_xd!=NULL)
                port_xd->set_xd(xd);
            else
                fprintf(stdout,"Internal error occured!\n");
        }
        else
            fprintf(stdout,"Got wrong angles information!\n");
    }
}


/************************************************************************/
void Localizer::handleAnglesOutput()
{
    if (port_anglesOut.getOutputCount()>0)
    {
        port_anglesOut.prepare()=CTRL_RAD2DEG*getCurAbsAngles();
        port_anglesOut.write();
    }
}


/************************************************************************/
void Localizer::run()
{
    handleMonocularInput();
    handleStereoInput();
    handleAnglesInput();
    handleAnglesOutput();
}


/************************************************************************/
void Localizer::threadRelease()
{
    port_mono.interrupt();
    port_stereo.interrupt();
    port_anglesIn.interrupt();
    port_anglesOut.interrupt();

    port_mono.close();
    port_stereo.close();
    port_anglesIn.close();
    port_anglesOut.close();

    delete eyeL;
    delete eyeR;
    delete pid;

    if (PrjL!=NULL)
    {
        delete PrjL;
        delete invPrjL;
    }

    if (PrjR!=NULL)
    {
        delete PrjR;
        delete invPrjR;
    }

    if (alignLnkLeft1!=NULL)
        delete alignLnkLeft1;

    if (alignLnkLeft2!=NULL)
        delete alignLnkLeft2;

    if (alignLnkRight1!=NULL)
        delete alignLnkRight1;

    if (alignLnkRight2!=NULL)
        delete alignLnkRight2;
}



