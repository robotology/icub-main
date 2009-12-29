
#include <iostream>
#include <iomanip>

#include "localizer.h"


/************************************************************************/
Localizer::Localizer(exchangeData *_commData, const string &_localName,
                     int _cx, int _cy, unsigned int _period) :
                     RateThread(_period), localName(_localName),
                     commData(_commData), cx(_cx), cy(_cy),
                     period(_period),     Ts(_period/1000.0)
{
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

    dx.resize(4);
}


/************************************************************************/
bool Localizer::threadInit()
{
    port_xd=NULL;

    port_pixel=new BufferedPort<Bottle>();
    string n=localName+"/pixel:i";
    port_pixel->open(n.c_str());

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
void Localizer::run()
{
    // get image feedback
    if (Bottle *pixel=port_pixel->read(false))
        if (pixel->size()>=4)
        {
            double ul=pixel->get(0).asDouble();
            double vl=pixel->get(1).asDouble();
            double ur=pixel->get(2).asDouble();
            double vr=pixel->get(3).asDouble();
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
            dx[0]=-u[0];
            dx[1]=-u[1];
            dx[2]=-u[2];
            dx[3]=1.0;  // homogeneous coordinates

            // normalize reference frame axes
            Matrix fpFrame=commData->get_fpFrame();
            for (unsigned int j=0; j<3; j++)
            {   
                double d=norm(fpFrame,j); 
                for (unsigned int i=0; i<3; i++)
                    fpFrame(i,j)/=d;
            }

            // set-up translational part
            fpFrame(3,3)=1.0;
            for (unsigned int i=0; i<3; i++)
                fpFrame(i,3)=commData->get_x()[i];

            // update position wrt root frame
            Vector xdO=fpFrame*dx;

            if (port_xd)
            {
                Vector xd(3);
                xd[0]=xdO[0];
                xd[1]=xdO[1];
                xd[2]=xdO[2];
                port_xd->set_xd(xd);
            }
        }
        else
            cerr << "Got wrong pixel information!" << endl;
}


/************************************************************************/
void Localizer::threadRelease()
{
    port_pixel->interrupt();
    port_pixel->close();
    delete port_pixel;

    delete pid;
}



