// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Marco Maggiali marco.maggiali@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "include/TouchSensor.h"

#ifndef __CER_SH_PDL_H__
#define __CER_SH_PDL_H__

class CER_SH_PDL : public TouchSensor
{
public:

    CER_SH_PDL(double cx,double cy,double th,double gain=1.0,int layoutNum=0,int lrMirror=0)
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
        nTaxels=29;
        m_RadiusOrig=1.8;

  
		
		for (int i=0; i<nTaxels; ++i)
        {
			dX[i]=15; //not used
			dY[i]=15; //not used 
		}



        dX[0]=13; dY[0]=3;
		dX[1]=7; dY[1]=9;
		dX[2]=7; dY[2]=3;
		dX[3]=19; dY[3]=-3;
		dX[4]=1; dY[4]=9;
		dX[5]=13; dY[5]=-3;
		dX[6]=19; dY[6]=-9;
		dX[7]=7; dY[7]=-9;
		dX[8]=13; dY[8]=-9;
		dX[9]=1; dY[9]=-9;
		dX[10]=7; dY[10]=-3;
		dX[11]=7.15; dY[11]=-6.1;
		dX[12]=1; dY[12]=-3;
		dX[13]=-11; dY[13]=-9;
		dX[14]=-17; dY[14]=-9;
		dX[15]=-5; dY[15]=-9;
		dX[16]=-17; dY[16]=-3;
		dX[17]=-11; dY[17]=-3;
		dX[18]=-17; dY[18]=3;
		dX[19]=-11; dY[19]=3;
		dX[20]=-5; dY[20]=-3;
		dX[21]=-5; dY[21]=3;
		dX[22]=1; dY[22]=3;
		dX[23]=-2; dY[23]=1.5;
		dX[24]=-17; dY[24]=9;
		dX[25]=-14; dY[25]=14;
		dX[26]=-11; dY[26]=9;
		dX[27]=-8; dY[27]=14;
		dX[28]=-5; dY[28]=9;
	//	dX[35]=-5.83; dY[35]=11.9;

		
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
