
#include <yarp/math/SVD.h>

#include <iostream>
#include <iomanip>

#include <iCub/localizer.hpp>


/************************************************************************/
Localizer::Localizer(exchangeData *_commData, const string &_localName,
                     const string &_configFile, unsigned int _period) :
                     RateThread(_period), localName(_localName),
                     commData(_commData), configFile(_configFile),
                     period(_period),     Ts(_period/1000.0)
{
    eyeL=new iCubEye("left");
    eyeR=new iCubEye("right");

    // remove constraints on the links
    eyeL->setAllConstraints(false);
    eyeR->setAllConstraints(false);

    // release links
    eyeL->releaseLink(0); eyeR->releaseLink(0);
    eyeL->releaseLink(1); eyeR->releaseLink(1);
    eyeL->releaseLink(2); eyeR->releaseLink(2);

    // get the absolute reference frame of the cyclopic eye
    Vector q(eyeC.getDOF()); q=0.0;
    eyeCAbsFrame=eyeC.getH(q);
    // ... and its inverse
    invEyeCAbsFrame=SE3inv(eyeCAbsFrame);

    double cxl,cyl,cxr,cyr;

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

    if (PrjL && PrjR)
    {
        cx=(cxl+cxr)/2.0;
        cy=(cyl+cyr)/2.0;
    }
    else if (PrjL)
    {
        cx=cxl;
        cy=cyl;
    }
    else if (PrjR)
    {
        cx=cxr;
        cy=cyr;
    }
    else // default values if no configFile is given
    {
        cx=160;
        cy=120;
    }

    Vector Kp(3), Ki(3), Kd(3);
    Vector Wp(3), Wi(3), Wd(3);
    Vector N(3),  Tt(3);
    Matrix satLim(3,2);

    Kp[0]=KP_PAN;
    Kp[1]=KP_TILT;
    Kp[2]=KP_VERG;
    
    Ki[0]=KI_PAN;
    Ki[1]=KI_TILT;
    Ki[2]=KI_VERG;

    Kd=0.0;

    Wp=Wi=1.0;
    Wd=0.0;

    N=10.0;
    Tt=1.0;

    satLim(0,0)=satLim(1,0)=satLim(2,0)=LIM_LOW;
    satLim(0,1)=satLim(1,1)=satLim(2,1)=LIM_HIGH;

    pid=new parallelPID(Ts,Kp,Ki,Kd,Wp,Wi,Wd,N,Tt,satLim);

    port_xd=NULL;
}


/************************************************************************/
bool Localizer::threadInit()
{ 
    port_mono=new BufferedPort<Bottle>;
    string n1=localName+"/mono:i";
    port_mono->open(n1.c_str());

    port_stereo=new BufferedPort<Bottle>;
    string n2=localName+"/stereo:i";
    port_stereo->open(n2.c_str());

    port_anglesIn=new BufferedPort<Bottle>;
    string n3=localName+"/angles:i";
    port_anglesIn->open(n3.c_str());

    port_anglesOut=new BufferedPort<Vector>;
    string n4=localName+"/angles:o";
    port_anglesOut->open(n4.c_str());

    cout << "Starting Localizer at " << period << " ms" << endl;

    return true;
}


/************************************************************************/
void Localizer::afterStart(bool s)
{
    if (s)
        cout << "Localizer started successfully" << endl;
    else
        cout << "Localizer did not start" << endl;
}


/************************************************************************/
void Localizer::handleMonocularInput()
{
    if (Bottle *mono=port_mono->read(false))
        if (mono->size()>=4)
        {
            string type=mono->get(0).asString().c_str();
            double u=mono->get(1).asDouble();
            double v=mono->get(2).asDouble();
            double z=mono->get(3).asDouble();

            bool isLeft=(type=="left");

            Matrix  *invPrj=(isLeft?invPrjL:invPrjR);
            iCubEye *eye=(isLeft?eyeL:eyeR);

            if (invPrj)
            {
                Vector &torso=commData->get_torso();
                Vector &head=commData->get_q();

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
                // knowing the guessed distance z from the camera
                Vector Xe=*invPrj*x;
                Xe[3]=1.0;  // impose homogeneous coordinates                

                // update position wrt the root frame
                Vector Xo=eye->getH(q)*Xe;

                if (port_xd)
                {
                    Vector xd(3);
                    xd[0]=Xo[0];
                    xd[1]=Xo[1];
                    xd[2]=Xo[2];

                    port_xd->set_xd(xd);
                }
                else
                    cerr << "Internal error occured!" << endl;
            }
            else
                cerr << "Unspecified projection matrix for " << type << " camera!" << endl;
        }
        else
            cerr << "Got wrong mono information!" << endl;
}


/************************************************************************/
void Localizer::handleStereoInput()
{
    if (Bottle *stereo=port_stereo->read(false))
        if (stereo->size()>=4)
        {
            double ul=stereo->get(0).asDouble();
            double vl=stereo->get(1).asDouble();
            double ur=stereo->get(2).asDouble();
            double vr=stereo->get(3).asDouble();
            double um=(ul+ur)/2.0;
            double vm=(vl+vr)/2.0;

            // consider a reference frame attached to the current
            // fixation point aligned with the normalized sum
            // of the left eye axis and right eye axis
            Vector ref(3), fb(3);

            // along x
            ref[0]=cx; fb[0]=um;

            // along y
            ref[1]=cy; fb[1]=vm;

            // along z
            double el=um-ul;
            double er=um-ur;
            ref[2]=0.0; fb[2]=(fabs(el)+fabs(er))/2.0;
            if (el<0.0 || er>0.0)
                fb[2]=-fb[2];   // go towards increasing direction of z

            // predict next position relative to the
            // fixation point frame
            Vector u=pid->compute(ref,fb);
            Vector dx(4);
            dx[0]=-u[0];
            dx[1]=-u[1];
            dx[2]=-u[2];
            dx[3]=1.0;  // impose homogeneous coordinates

            // get the head-centered frame
            // do not use reference since we're gonna modify it
            // and we don't want to keep the changes
            Matrix fpFrame=commData->get_fpFrame();

            // set-up translational part
            for (unsigned int i=0; i<3; i++)
                fpFrame(i,3)=commData->get_x()[i];

            // update position wrt the root frame
            Vector xdO=fpFrame*dx;

            if (port_xd)
            {
                Vector xd(3);
                xd[0]=xdO[0];
                xd[1]=xdO[1];
                xd[2]=xdO[2];

                port_xd->set_xd(xd);
            }
            else
                cerr << "Internal error occured!" << endl;
        }
        else
            cerr << "Got wrong stereo information!" << endl;
}


/************************************************************************/
void Localizer::handleAnglesInput()
{
    if (Bottle *angles=port_anglesIn->read(false))
        if (angles->size()>=4)
        {
            string type=angles->get(0).asString().c_str();
            double azi=CTRL_DEG2RAD*angles->get(1).asDouble();
            double ele=CTRL_DEG2RAD*angles->get(2).asDouble();
            double ver=CTRL_DEG2RAD*angles->get(3).asDouble();

            bool isAbs=(type=="abs");

            Vector &torso=commData->get_torso();
            Vector &head=commData->get_q();

            Vector q(8);
            q[0]=torso[0];
            q[1]=torso[1];
            q[2]=torso[2];
            q[3]=head[0];
            q[4]=head[1];
            q[5]=head[2];
            q[6]=head[3];

            if (isAbs)
                q[7]=head[4]+ver/2.0;
            else
                q[7]=head[4]+(head[5]+ver)/2.0;
            eyeL->setAng(q);

            if (isAbs)
                q[7]=head[4]-ver/2.0;
            else
                q[7]=head[4]-(head[5]+ver)/2.0;
            eyeR->setAng(q);

            Vector fp(4);
            fp[3]=1.0;  // impose homogeneous coordinates

            // compute new fp due to changed vergence
            if (computeFixationPointOnly(*(eyeL->asChain()),*(eyeR->asChain()),fp) || head[5]<1.0*CTRL_DEG2RAD)
            {
                // keep the old fp if some errors occur
                fp[0]=commData->get_x()[0];
                fp[1]=commData->get_x()[1];
                fp[2]=commData->get_x()[2];

                cerr << "Vergence error occured!" << endl;
            }

            // compute rotational matrix
            Vector x(4);
            x[0]=1.0;
            x[1]=0.0;
            x[2]=0.0;
            x[3]=ele;

            Vector y(4);
            y[0]=0.0;
            y[1]=1.0;
            y[2]=0.0;
            y[3]=azi;
            
            Matrix R=axis2dcm(y)*axis2dcm(x);
            Vector fpe;

            // get the head-centered frame
            Matrix &frame=(isAbs?eyeCAbsFrame:commData->get_fpFrame());

            // get fp wrt head-centered frame
            if (isAbs)
                fpe=invEyeCAbsFrame*fp;
            else
                fpe=SE3inv(frame)*fp;

            // remove the 4th component to compute the norm
            fpe[3]=0.0;

            // get the rotated z-axis, properly scaled
            Vector z=norm(fpe)*R.getCol(2);
            z[3]=1.0;  // impose homogeneous coordinates

            // get fp wrt root frame
            Vector xdO=frame*z;

            if (port_xd)
            {
                Vector xd(3);
                xd[0]=xdO[0];
                xd[1]=xdO[1];
                xd[2]=xdO[2];

                port_xd->set_xd(xd);
            }
            else
                cerr << "Internal error occured!" << endl;
        }
        else
            cerr << "Got wrong angles information!" << endl;
}


/************************************************************************/
void Localizer::handleAnglesOutput()
{
    Vector fp(4);
    fp[0]=commData->get_x()[0];
    fp[1]=commData->get_x()[1];
    fp[2]=commData->get_x()[2];
    fp[3]=1.0;  // impose homogeneous coordinates

    // get fp wrt head-centered system
    Vector fpe=invEyeCAbsFrame*fp;

    Vector &angles=port_anglesOut->prepare();
    angles.resize(3);

    angles[0]=CTRL_RAD2DEG*atan2(fpe[0],fpe[2]);
    angles[1]=-CTRL_RAD2DEG*atan2(fpe[1],fpe[2]);
    angles[2]=CTRL_RAD2DEG*commData->get_q()[5];

    port_anglesOut->write();
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
    port_mono->interrupt();
    port_stereo->interrupt();
    port_anglesIn->interrupt();
    port_anglesOut->interrupt();

    port_mono->close();
    port_stereo->close();
    port_anglesIn->close();
    port_anglesOut->close();

    delete port_mono;
    delete port_stereo;
    delete port_anglesIn;
    delete port_anglesOut;

    if (PrjL)
    {
        delete PrjL;
        delete invPrjL;
    }

    if (PrjR)
    {
        delete PrjR;
        delete invPrjR;
    }

    delete eyeL;
    delete eyeR;

    delete pid;
}



