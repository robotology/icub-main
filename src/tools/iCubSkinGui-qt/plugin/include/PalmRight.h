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
        nTaxels=48;
        m_RadiusOrig=1.8;

  
		
		for (int i=0; i<nTaxels; ++i)
        {
			dX[0]=15; //not used
			dY[0]=15; //not used 
		}



        dX[27]=1.5;  dY[27]=6.5;
        dX[26]=6.5;  dY[26]=6;
        dX[25]=11.5; dY[25]=6;
        dX[24]=16.5; dY[24]=6;
		dX[31]=21.5; dY[31]=6;
        dX[29]=6.5;  dY[29]=1;
		dX[28]=11.5; dY[28]=1;
        dX[32]=16.5; dY[32]=1;
        dX[33]=21.5; dY[33]=1;
		dX[35]=9.5;  dY[35]=-2; //thermal_pad
		dX[30]=14.5; dY[30]=-3.5;
		dX[34]=21.5; dY[34]=-4;


		dX[6]=27;   dY[6]=6;
        dX[3]=32;   dY[3]=6;
        dX[2]=37;   dY[2]=6;
        dX[1]=42;   dY[1]=5.5;
        dX[0]=47;   dY[0]=4.5;
        dX[11]=51.7; dY[11]=4; //thermal_pad
		dX[7]=27;    dY[7]=1;
        dX[8]=32;    dY[8]=1;
		dX[4]=37;    dY[4]=1;
		dX[5]=42;   dY[5]=0;
		dX[9]=27;    dY[9]=-3.5;
		dX[10]=32;    dY[10]=-3.5;


		dX[16]=37.5;    dY[16]=-4.5;
		dX[15]=42;     dY[15]=-5.5;
		dX[14]=46.5;    dY[14]=-8;
		dX[20]=27;    dY[20]=-9;
		dX[21]=32;    dY[21]=-9;
		dX[17]=37;    dY[17]=-9;
		dX[19]=42;    dY[19]=-10.5;	
		dX[22]=38;    dY[22]=-14;
		dX[18]=43;    dY[18]=-16;
		dX[13]=47;    dY[13]=-13;
		dX[12]=47.5;    dY[12]=-18;
		dX[23]=43.5;    dY[23]=-20; //thermal_pad
		
		dX[46]=33;    dY[46]=-14.5;
		dX[47]=28;    dY[47]=-14.5;
		dX[36]=28;    dY[36]=-19.5;
		dX[42]=33;    dY[42]=-19.5;
		dX[45]=38;    dY[45]=-19.5;
		dX[37]=28;    dY[37]=-24.5;
		dX[38]=33;    dY[38]=-24.5;
		dX[41]=38;    dY[41]=-24.5;
		dX[44]=43;    dY[44]=-26;
		dX[39]=35;    dY[39]=-29;
		dX[40]=40;    dY[40]=-29.5;
		dX[43]=37;    dY[43]=-32.5; //thermal pad
		

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
