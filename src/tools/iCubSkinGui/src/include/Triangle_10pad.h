// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Marco Maggiali marco.maggiali@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "include/TouchSensor.h"

#ifndef _TRIANGLE_10PAD_H__
#define _TRIANGLE_10PAD_H__

class Triangle_10pad : public TouchSensor
{
public:
    Triangle_10pad(double cx,double cy,double th,double gain=1.0,int layoutNum=0,int lrMirror=0)
    {
        const double DEG2RAD=M_PI/180.0;
		th=th;
        const double CST=cos(DEG2RAD*(th+0));
        const double SNT=sin(DEG2RAD*(th+0));

        const double H=sin(DEG2RAD*60.0);
        const double L=2.0*H/9.0;
        const double scale=1/40.0;
        dGain=gain;
        ilrMirror=lrMirror;
        ilayoutNum=layoutNum;
        nVerts=3;
        nTaxels=12;
        m_RadiusOrig=2.2;

        dX[8]=-128.62; dY[8]=222.83;
        dX[10]= 0; dY[10]=296.55;
        dX[ 9]= 0; dY[ 9]=445.54;
        dX[ 11]= 128.62; dY[ 11]=222.83;
        dX[ 0]= 257.2; dY[ 0]=0;
        dX[ 6]= -256.3; dY[ 6]=-147.64;
        dX[ 1]=  385.83; dY[1]=-222.83;
        dX[ 2]=128.62; dY[ 2]=-222.83;
        dX[ 3]=0.0; dY[ 3]=0.0;
        dX[ 4]=-128.62; dY[ 4]=-222.83;
        dX[ 5]=-385.83; dY[ 5]=-222.83;
        dX[ 7]=-257.2; dY[ 7]=0.0;        


        
        for (int i=0; i<nTaxels; ++i)
        {
            dX[i]*=scale;
            dY[i]*=scale;
        }

		dXv[0]=-55.0/4;
        dYv[0]=-32.5/4;

        dXv[1]=+55.0/4;
        dYv[1]=-32.5/4;

        dXv[2]=0.0;
        dYv[2]=63.0/4;

        for (int i=0; i<nTaxels; ++i)
        {
            double x=dX[i];
            double y=dY[i];
			if (lrMirror==1) x=-x;

            dX[i]=cx+CST*x-SNT*y;
            dY[i]=cy+SNT*x+CST*y;
        }

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
	void draw(unsigned char *image)
    {
        for (int i=0; i<nVerts; ++i)
        {
            drawLine(image,xv[i],yv[i],xv[(i+1)%nVerts],yv[(i+1)%nVerts]);
        }
        
        for (int i=0; i<nTaxels; ++i)
        {
			if ((i==6) || (i==10)) 
			{
				drawCircle(image,x[i],y[i],m_Radius/2);
			}
			else  
            drawCircle(image,x[i],y[i],m_Radius);
        }
    }
};

#endif
