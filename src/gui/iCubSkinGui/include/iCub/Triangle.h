// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iCub/TouchSensor.h>

#ifndef __ALE_TRIANGLE_H__
#define __ALE_TRIANGLE_H__

class Triangle : public TouchSensor
{
public:
    Triangle(double cx,double cy,double th,double gain=1.0,int layoutNum=0,int lrMirror=0)
    {
        const double DEG2RAD=M_PI/180.0;

        const double CST=cos(DEG2RAD*th);
        const double SNT=sin(DEG2RAD*th);

        const double H=sin(DEG2RAD*60.0);
        const double L=2.0*H/9.0;
     
        dGain=gain;
		ilrMirror=lrMirror;
		ilayoutNum=layoutNum;
        nVerts=3;
        m_RadiusOrig=2.0;

        dX[5]=L*cos(DEG2RAD*30.0); dX[4]=0.5-dX[5]; dX[2]=0.5+dX[5]; dX[1]=1.0-dX[5];
        dX[6]=0.25; dX[3]=0.5; dX[0]=0.75;
        dX[7]=0.25+dX[5]; dX[11]=0.75-dX[5];
        dX[8]=dX[7]; dX[10]=dX[11];
        dX[9]=0.5;

        dY[5]=L*sin(DEG2RAD*30.0); dY[4]=dY[5]; dY[2]=dY[5]; dY[1]=dY[5];
        dY[6]=0.5*H-L; dY[3]=L; dY[0]=dY[6];
        dY[7]=0.5*H-L*sin(DEG2RAD*30.0); dY[11]=dY[7];
        dY[8]=0.5*H+L*sin(DEG2RAD*30.0); dY[10]=dY[8];
        dY[9]=H-L;

        for (int i=0; i<12; ++i)
        {
            double x=30.0*dX[i]-15.0;
            double y=30.0*dY[i]-10.0*H;

			if (lrMirror==1) x=-x;
            dX[i]=cx+CST*x-SNT*y;
            dY[i]=cy+SNT*x+CST*y;
        }

		dXv[0]=-15.0;
        dYv[0]=-8.66;

        dXv[1]=+15.0;
        dYv[1]=-8.66;

        dXv[2]=0;
        dYv[2]=+17.32;

		for (int i=0; i<nVerts; ++i)
        {
            double x=dXv[i];
            double y=dYv[i];
			if (lrMirror==1) x=-x;

            dXv[i]=cx+CST*x-SNT*y;
            dYv[i]=cy+SNT*x+CST*y;
        }

        // in static definition
        //dXmin=dYmin= HUGE; 
        //dXmax=dYmax=-HUGE;

        for (int i=0; i<nVerts; ++i)
        {
            if (dXv[i]<dXmin) dXmin=dXv[i];
            if (dXv[i]>dXmax) dXmax=dXv[i];
            if (dYv[i]<dYmin) dYmin=dYv[i];
            if (dYv[i]>dYmax) dYmax=dYv[i];
        }

        dXc=cx;
        dYc=cy;
    }
};

#endif
