// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Marco Maggiali marco.maggiali@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "include/TouchSensor.h"

#ifndef __ALE_PALMR_H__
#define __ALE_PALMR_H__

class PalmR : public TouchSensor
{
public:

    PalmR(double cx,double cy,double th,double gain=1.0,int layoutNum=0,int lrMirror=0)
    {
        const double DEG2RAD=M_PI/180.0;

        const double CST=cos(DEG2RAD*th);
        const double SNT=sin(DEG2RAD*th);

        const double H=sin(DEG2RAD*60.0);
        const double L=2.0*H/9.0;
     
        dGain=gain;
        ilrMirror=lrMirror;
        ilayoutNum=layoutNum;
        nVerts=4;
        nTaxels=49;
        m_RadiusOrig=4.0;

    min_tax=1;
    max_tax=48;
		
		for (int i=0; i<nTaxels; ++i)
        {
			dX[0]=15; //not used
			dY[0]=15; //not used 
		}
		dX[0]=1.5; //not used
		dY[0]=6.5; //not used 


        dX[35]=1.5;  dY[35]=6.5;
        dX[34]=6.5;  dY[34]=6;
        dX[36]=11.5; dY[36]=6;
        dX[40]=16.5; dY[40]=6;
		dX[42]=21.5; dY[42]=6;
        dX[31]=6.5;  dY[31]=1;
		dX[30]=11.5; dY[30]=1;
        dX[41]=16.5; dY[41]=1;
        dX[46]=21.5; dY[46]=1;
		dX[47]=9.5;  dY[47]=-2; //thermal_pad
		dX[28]=14.5; dY[28]=-3.5;
		dX[25]=21.5; dY[25]=-4;


		dX[13]=27;   dY[13]=6;
        dX[15]=32;   dY[15]=6;
        dX[17]=37;   dY[17]=6;
        dX[19]=42;   dY[19]=5.5;
        dX[21]=47;   dY[21]=4.5;
        dX[23]=51.7; dY[23]=4; //thermal_pad
		dX[17]=37;   dY[17]=6;
		dX[7]=27;    dY[7]=1;
        dX[3]=32;    dY[3]=1;
		dX[9]=37;    dY[9]=1;
		dX[11]=42;   dY[11]=0;
		dX[5]=27;    dY[5]=-3.5;
		dX[1]=32;    dY[1]=-3.5;


		dX[37]=37.5;    dY[37]=-4.5;
		dX[43]=42;    dY[43]=-5.5;
		dX[44]=46.5;    dY[44]=-8;
		dX[32]=27;    dY[32]=-9;
		dX[29]=32;    dY[29]=-9;
		dX[33]=37;    dY[33]=-9;
		dX[39]=42;    dY[39]=-10.5;
		dX[22]=28;    dY[22]=-14.5;
		dX[24]=33;    dY[24]=-14.5;
		dX[27]=38;    dY[27]=-14;
		dX[38]=43;    dY[38]=-16;
		dX[45]=47;    dY[45]=-13;
		dX[26]=47.5;    dY[26]=-18;
		dX[48]=43.5;    dY[48]=-20; //thermal_pad

		dX[20]=28;    dY[20]=-19.5;
		dX[14]=33;    dY[14]=-19.5;
		dX[2]=38;    dY[2]=-19.5;
		dX[18]=28;    dY[18]=-24.5;
		dX[10]=33;    dY[10]=-24.5;
		dX[6]=38;    dY[6]=-24.5;
		dX[4]=43;    dY[4]=-26;
		dX[12]=35;    dY[12]=-29;
		dX[8]=40;    dY[8]=-29.5;
		dX[16]=37;    dY[16]=-32.5;


        for (int i=0; i<nTaxels; ++i)
        {
            double x=4.0*dX[i]-2.0;
            double y=4.0*dY[i]-2.0;

            if (lrMirror==1) x=-x;
            dX[i]=cx+CST*x-SNT*y;
            dY[i]=cy+SNT*x+CST*y;
        }

        dXv[0]=-25;
        dYv[0]=-150;
        dXv[1]=+250;
        dYv[1]=-150;
        dXv[2]=+250;
        dYv[2]=+50;
        dXv[3]=-25;
        dYv[3]=+50;
        
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
            //drawLine(image,xv[i],yv[i],xv[(i+1)%nVerts],yv[(i+1)%nVerts]);
        }
        
        for (int i=0; i<nTaxels; ++i)
        {
			if ((i==16) || (i==48) || (i==47) || (i==23) ) 
			{
				drawCircle(image,x[i],y[i],m_Radius/2);
			}
			else  
            drawCircle(image,x[i],y[i],m_Radius);
        }
    }
};

#endif
