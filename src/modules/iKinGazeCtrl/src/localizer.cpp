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
#include <iCub/localizer.h>
#include <iCub/solver.h>


/************************************************************************/
Localizer::Localizer(ExchangeData *_commData, const unsigned int _period) :
                     RateThread(_period), commData(_commData), period(_period)
{
    iCubHeadCenter eyeC("right_"+commData->headVersion2String());
    eyeL=new iCubEye("left_"+commData->headVersion2String());
    eyeR=new iCubEye("right_"+commData->headVersion2String());

    // remove constraints on the links
    // we use the chains for logging purpose
    eyeL->setAllConstraints(false);
    eyeR->setAllConstraints(false);

    // release links
    eyeL->releaseLink(0); eyeC.releaseLink(0); eyeR->releaseLink(0);
    eyeL->releaseLink(1); eyeC.releaseLink(1); eyeR->releaseLink(1);
    eyeL->releaseLink(2); eyeC.releaseLink(2); eyeR->releaseLink(2);

    // add aligning matrices read from configuration file
    getAlignHN(commData->rf_cameras,"ALIGN_KIN_LEFT",eyeL->asChain(),true);
    getAlignHN(commData->rf_cameras,"ALIGN_KIN_RIGHT",eyeR->asChain(),true);

    // overwrite aligning matrices iff specified through tweak values
    if (commData->tweakOverwrite)
    {
        getAlignHN(commData->rf_tweak,"ALIGN_KIN_LEFT",eyeL->asChain(),true);
        getAlignHN(commData->rf_tweak,"ALIGN_KIN_RIGHT",eyeR->asChain(),true);
    }

    // get the absolute reference frame of the head
    Vector q(eyeC.getDOF(),0.0);
    eyeCAbsFrame=eyeC.getH(q);
    // ... and its inverse
    invEyeCAbsFrame=SE3inv(eyeCAbsFrame);

    // get the length of the half of the eyes baseline
    eyesHalfBaseline=0.5*norm(eyeL->EndEffPose().subVector(0,2)-eyeR->EndEffPose().subVector(0,2));

    bool ret;

    // get camera projection matrix
    ret=getCamParams(commData->rf_cameras,"CAMERA_CALIBRATION_LEFT",&PrjL,widthL,heightL,true);
    if (commData->tweakOverwrite)
    {
        Matrix *Prj;
        if (getCamParams(commData->rf_tweak,"CAMERA_CALIBRATION_LEFT",&Prj,widthL,heightL,true))
        {
            delete PrjL;
            PrjL=Prj;
        }
    }

    if (ret)
    {
        cxl=(*PrjL)(0,2);
        cyl=(*PrjL)(1,2);
        invPrjL=new Matrix(pinv(PrjL->transposed()).transposed());
    }
    else
        PrjL=invPrjL=NULL;

    // get camera projection matrix
    ret=getCamParams(commData->rf_cameras,"CAMERA_CALIBRATION_RIGHT",&PrjR,widthR,heightR,true);
    if (commData->tweakOverwrite)
    {
        Matrix *Prj;
        if (getCamParams(commData->rf_tweak,"CAMERA_CALIBRATION_RIGHT",&Prj,widthR,heightR,true))
        {
            delete PrjR;
            PrjR=Prj;
        }
    }

    if (ret)
    {
        cxr=(*PrjR)(0,2);
        cyr=(*PrjR)(1,2);
        invPrjR=new Matrix(pinv(PrjR->transposed()).transposed());
    }
    else
        PrjR=invPrjR=NULL;

    Vector Kp(1,0.001), Ki(1,0.001), Kd(1,0.0);
    Vector Wp(1,1.0),   Wi(1,1.0),   Wd(1,1.0);
    Vector N(1,10.0),   Tt(1,1.0);
    Matrix satLim(1,2);

    satLim(0,0)=0.05;
    satLim(0,1)=10.0;

    pid=new parallelPID(0.05,Kp,Ki,Kd,Wp,Wi,Wd,N,Tt,satLim);

    Vector z0(1,0.5);
    pid->reset(z0);
    dominantEye="left";
}


/************************************************************************/
Localizer::~Localizer()
{
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
}


/************************************************************************/
bool Localizer::threadInit()
{ 
    port_mono.open(commData->localStemName+"/mono:i");
    port_stereo.open(commData->localStemName+"/stereo:i");
    port_anglesIn.open(commData->localStemName+"/angles:i");
    port_anglesOut.open(commData->localStemName+"/angles:o");

    yInfo("Starting Localizer at %d ms",period);
    return true;
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
}


/************************************************************************/
void Localizer::afterStart(bool s)
{
    if (s)
        yInfo("Localizer started successfully");
    else
        yError("Localizer did not start!");
}


/************************************************************************/
void Localizer::getPidOptions(Bottle &options)
{
    LockGuard lg(mutex);
    pid->getOptions(options);
    Bottle &bDominantEye=options.addList();
    bDominantEye.addString("dominantEye");
    bDominantEye.addString(dominantEye);
}


/************************************************************************/
void Localizer::setPidOptions(const Bottle &options)
{
    LockGuard lg(mutex);
    pid->setOptions(options);
    if (options.check("dominantEye"))
    {
        string domEye=options.find("dominantEye").asString();
        if ((domEye=="left") || (domEye=="right"))
            dominantEye=domEye;
    }
}


/************************************************************************/
Vector Localizer::getAbsAngles(const Vector &x)
{
    Vector fp=x;
    fp.push_back(1.0);  // impose homogeneous coordinates

    // get fp wrt head-centered system
    Vector fph=invEyeCAbsFrame*fp;
    fph.pop_back();

    Vector ang(3);
    ang[0]=atan2(fph[0],fph[2]);
    ang[1]=-atan2(fph[1],fabs(fph[2]));
    ang[2]=2.0*atan2(eyesHalfBaseline,norm(fph));

    return ang;
}


/************************************************************************/
Vector Localizer::get3DPoint(const string &type, const Vector &ang)
{
    LockGuard lg(mutex);
    double azi=ang[0];
    double ele=ang[1];
    double ver=ang[2];

    Vector q(8,0.0);
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
    
    // impose vergence != 0.0
    ver=std::max(ver,commData->minAllowedVergence);    

    q[7]+=ver/2.0;
    eyeL->setAng(q);

    q[7]-=ver;
    eyeR->setAng(q);

    // compute new fp due to changed vergence
    Vector fp;
    CartesianHelper::computeFixationPointData(*(eyeL->asChain()),*(eyeR->asChain()),fp);
    fp.push_back(1.0);  // impose homogeneous coordinates    

    // compute rotational matrix to
    // account for elevation and azimuth
    Vector x(4), y(4);
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

    xd.pop_back();
    return xd;
}


/************************************************************************/
bool Localizer::projectPoint(const string &type, const Vector &x, Vector &px)
{
    LockGuard lg(mutex);
    if (x.length()<3)
    {
        yError("Not enough values given for the point!");
        return false;
    }

    bool isLeft=(type=="left");

    Matrix  *Prj=(isLeft?PrjL:PrjR);
    iCubEye *eye=(isLeft?eyeL:eyeR);

    if (Prj!=NULL)
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
        q[7]=head[4]+head[5]/(isLeft?2.0:-2.0);
        
        Vector xo=x;
        // impose homogeneous coordinates
        if (xo.length()<4)
            xo.push_back(1.0);
        else
        {
            xo=xo.subVector(0,3); 
            xo[3]=1.0;
        }

        // find position wrt the camera frame
        Vector xe=SE3inv(eye->getH(q))*xo;

        // find the 2D projection
        px=*Prj*xe;
        px=px/px[2];
        px.pop_back();
        return true;
    }
    else
    {
        yError("Unspecified projection matrix for %s camera!",type.c_str());
        return false;
    }
}


/************************************************************************/
bool Localizer::projectPoint(const string &type, const double u, const double v,
                             const double z, Vector &x)
{
    LockGuard lg(mutex);
    bool isLeft=(type=="left");

    Matrix  *invPrj=(isLeft?invPrjL:invPrjR);
    iCubEye *eye=(isLeft?eyeL:eyeR);

    if (invPrj!=NULL)
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
        q[7]=head[4]+head[5]/(isLeft?2.0:-2.0);

        Vector p(3);
        p[0]=z*u;
        p[1]=z*v;
        p[2]=z;

        // find the 3D position from the 2D projection,
        // knowing the coordinate z in the camera frame
        Vector xe=*invPrj*p;
        xe[3]=1.0;  // impose homogeneous coordinates

        // find position wrt the root frame        
        x=eye->getH(q)*xe;
        x.pop_back();
        return true;
    }
    else
    {
        yError("Unspecified projection matrix for %s camera!",type.c_str());
        return false;
    }
}


/************************************************************************/
bool Localizer::projectPoint(const string &type, const double u, const double v,
                             const Vector &plane, Vector &x)
{
    if (plane.length()<4)
    {
        yError("Not enough values given for the projection plane!");
        return false;
    }

    bool isLeft=(type=="left");
    iCubEye *eye=(isLeft?eyeL:eyeR);

    if (projectPoint(type,u,v,1.0,x))
    {
        LockGuard lg(mutex);

        // pick up a point belonging to the plane
        Vector p0(3,0.0);
        if (plane[0]!=0.0)
            p0[0]=-plane[3]/plane[0];
        else if (plane[1]!=0.0)
            p0[1]=-plane[3]/plane[1];
        else if (plane[2]!=0.0)
            p0[2]=-plane[3]/plane[2];
        else
        {
            yError("Error while specifying projection plane!");
            return false;
        }

        // take a vector orthogonal to the plane
        Vector n(3);
        n[0]=plane[0];
        n[1]=plane[1];
        n[2]=plane[2];

        // compute the projection
        Vector e=eye->EndEffPose().subVector(0,2);
        Vector v=x-e;
        x=e+(dot(p0-e,n)/dot(v,n))*v;

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool Localizer::triangulatePoint(const Vector &pxl, const Vector &pxr, Vector &x)
{
    LockGuard lg(mutex);
    if ((pxl.length()<2) || (pxr.length()<2))
    {
        yError("Not enough values given for the pixels!");
        return false;
    }

    if (PrjL && PrjR)
    {
        Vector torso=commData->get_torso();
        Vector head=commData->get_q();

        Vector qL(8);
        qL[0]=torso[0];
        qL[1]=torso[1];
        qL[2]=torso[2];
        qL[3]=head[0];
        qL[4]=head[1];
        qL[5]=head[2];
        qL[6]=head[3];
        qL[7]=head[4]+head[5]/2.0;

        Vector qR=qL;
        qR[7]-=head[5];
        
        Matrix HL=SE3inv(eyeL->getH(qL));
        Matrix HR=SE3inv(eyeR->getH(qR));

        Matrix tmp=zeros(3,4); tmp(2,2)=1.0;
        tmp(0,2)=pxl[0]; tmp(1,2)=pxl[1];
        Matrix AL=(*PrjL-tmp)*HL;

        tmp(0,2)=pxr[0]; tmp(1,2)=pxr[1];
        Matrix AR=(*PrjR-tmp)*HR;

        Matrix A(4,3);
        Vector b(4);
        for (int i=0; i<2; i++)
        {
            b[i]=-AL(i,3);
            b[i+2]=-AR(i,3);

            for (int j=0; j<3; j++)
            {
                A(i,j)=AL(i,j);
                A(i+2,j)=AR(i,j);
            }
        }

        // solve the least-squares problem
        x=pinv(A)*b;

        return true;
    }
    else
    {
        yError("Unspecified projection matrix for at least one camera!");
        return false;
    }
}


/************************************************************************/
double Localizer::getDistFromVergence(const double ver)
{
    double tg=tan(CTRL_DEG2RAD*ver/2.0);
    return eyesHalfBaseline*sqrt(1.0+1.0/(tg*tg));
}


/************************************************************************/
void Localizer::handleMonocularInput()
{
    if (Bottle *mono=port_mono.read(false))
    {
        if (mono->size()>=4)
        {
            string type=mono->get(0).asString();
            double u=mono->get(1).asDouble();
            double v=mono->get(2).asDouble();
            double z;

            bool ok=false;
            if (mono->get(3).isDouble())
            {
                z=mono->get(3).asDouble();
                ok=true;
            }
            else if ((mono->get(3).asString()=="ver") && (mono->size()>=5))
            {
                double ver=mono->get(4).asDouble();
                z=getDistFromVergence(ver);
                ok=true;
            }

            if (ok)
            {
                Vector fp;
                if (projectPoint(type,u,v,z,fp))
                    commData->port_xd->set_xd(fp);
                return;
            }
        }

        yError("Got wrong monocular information!");
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
                
                mutex.lock();
                Vector z=pid->compute(ref,fb);
                mutex.unlock();

                if (projectPoint(dominantEye,u,v,z[0],fp))
                    commData->port_xd->set_xd(fp);
            }
            else
                yError("Got wrong stereo information!");
        }
        else
            yError("Unspecified projection matrix!");
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
        
            string type=angles->get(0).asString();
            ang[0]=CTRL_DEG2RAD*angles->get(1).asDouble();
            ang[1]=CTRL_DEG2RAD*angles->get(2).asDouble();
            ang[2]=CTRL_DEG2RAD*angles->get(3).asDouble();

            Vector xd=get3DPoint(type,ang);
            commData->port_xd->set_xd(xd);
        }
        else
            yError("Got wrong angles information!");
    }
}


/************************************************************************/
void Localizer::handleAnglesOutput()
{
    double x_stamp;
    Vector x=commData->get_x(x_stamp);
    txInfo_ang.update(x_stamp);

    if (port_anglesOut.getOutputCount()>0)
    {
        port_anglesOut.prepare()=CTRL_RAD2DEG*getAbsAngles(x);
        port_anglesOut.setEnvelope(txInfo_ang);
        port_anglesOut.write();
    }
}


/************************************************************************/
bool Localizer::getIntrinsicsMatrix(const string &type, Matrix &M,
                                    int &w, int &h)
{
    if (type=="left")
    {
        if (PrjL!=NULL)
        {
            M=*PrjL;
            w=widthL;
            h=heightL;
            return true;
        }
        else
            return false;
    }
    else if (type=="right")
    {
        if (PrjR!=NULL)
        {
            M=*PrjR;
            w=widthR;
            h=heightR;
            return true;
        }
        else
            return false;
    }
    else
        return false;
}


/************************************************************************/
bool Localizer::setIntrinsicsMatrix(const string &type, const Matrix &M,
                                    const int w, const int h)
{
    if (type=="left")
    {
        if (PrjL!=NULL)
        {
            *PrjL=M;
            *invPrjL=pinv(M.transposed()).transposed();
        }
        else
        {
            PrjL=new Matrix(M);
            invPrjL=new Matrix(pinv(M.transposed()).transposed());
        }

        widthL=w;
        heightL=h;

        return true;
    }
    else if (type=="right")
    {
        if (PrjR!=NULL)
        {
            *PrjR=M;
            *invPrjR=pinv(M.transposed()).transposed();
        }
        else
        {
            PrjR=new Matrix(M);
            invPrjR=new Matrix(pinv(M.transposed()).transposed());
        }

        widthR=w;
        heightR=h;

        return true;
    }
    else
        return false;
}


/************************************************************************/
void Localizer::run()
{
    handleMonocularInput();
    handleStereoInput();
    handleAnglesInput();
    handleAnglesOutput();
}



