// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Marco Maggiali marco.maggiali@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "include/TouchSensor.h"

#ifndef __ALE_PALML_H__
#define __ALE_PALML_H__

class PalmL : public TouchSensor
{
public:
    PalmL(double cx,double cy,double th,double gain=1.0,int layoutNum=0,int lrMirror=0)
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
        nTaxels=48;
        m_RadiusOrig=1.8;

		for (int i=0; i<nTaxels; ++i)
        {
			dX[0]=15; //not used
			dY[0]=15; //not used 
		}



        dX[29]=1.5;  dY[29]=6.5;
        dX[28]=6.5;  dY[28]=6;
        dX[30]=11.5; dY[30]=6;
        dX[31]=16.5; dY[31]=6;
		dX[33]=21.5; dY[33]=6;
        dX[27]=6.5;  dY[27]=1;
		dX[26]=11.5; dY[26]=1;
        dX[32]=16.5; dY[32]=1;
        dX[34]=21.5; dY[34]=1;
		dX[35]=9.5;  dY[35]=-2; //thermal_pad
		dX[25]=14.5; dY[25]=-3.5;
		dX[24]=21.5; dY[24]=-4;


		dX[6]=27;   dY[6]=6;
        dX[7]=32;   dY[7]=6;
        dX[8]=37;   dY[8]=6;
        dX[9]=42;   dY[9]=5.5;
        dX[10]=47;   dY[10]=4.5;
        dX[11]=51.7; dY[11]=4; //thermal_pad
		dX[3]=27;    dY[3]=1;
        dX[1]=32;    dY[1]=1;
		dX[4]=37;    dY[4]=1;
		dX[5]=42;   dY[5]=0;
		dX[2]=27;    dY[2]=-3.5;
		dX[0]=32;    dY[0]=-3.5;


		dX[17]=37.5;    dY[17]=-4.5;
		dX[20]=42;     dY[20]=-5.5;
		dX[21]=46.5;    dY[21]=-8;
		dX[15]=27;    dY[15]=-9;
		dX[14]=32;    dY[14]=-9;
		dX[16]=37;    dY[16]=-9;
		dX[19]=42;    dY[19]=-10.5;
		
		dX[13]=38;    dY[13]=-14;
		dX[18]=43;    dY[18]=-16;
		dX[22]=47;    dY[22]=-13;
		dX[12]=47.5;    dY[12]=-18;
		dX[23]=43.5;    dY[23]=-20; //thermal_pad

		dX[45]=28;    dY[45]=-19.5;
		dX[42]=33;    dY[42]=-19.5;
		dX[36]=38;    dY[36]=-19.5;
		dX[44]=28;    dY[44]=-24.5;
		dX[40]=33;    dY[40]=-24.5;
		dX[38]=38;    dY[38]=-24.5;
		dX[37]=43;    dY[37]=-26;
		dX[41]=35;    dY[41]=-29;
		dX[39]=40;    dY[39]=-29.5;
		dX[43]=37;    dY[43]=-32.5; //thermal pad
		dX[47]=33;    dY[47]=-14.5;
		dX[46]=28;    dY[46]=-14.5;

        for (int i=0; i<nTaxels; ++i)
		{
            double x=1.2*dX[i]-0.0;
            double y=1.2*dY[i]-0.0;

            if (lrMirror==1) x=-x;
            dX[i]=cx+CST*x-SNT*y;
            dY[i]=cy+SNT*x+CST*y;
        }

        dXv[0]=-25;
        dYv[0]=-50;
        dXv[1]=+50;
        dYv[1]=-50;
        dXv[2]=+50;
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
			if ((i==43) || (i==23) || (i==11) || (i==35) ) 
			{
				drawCircle(image,x[i],y[i],m_Radius/2);
			}
			else  
            drawCircle(image,x[i],y[i],m_Radius);
        }
    }
};

#endif
