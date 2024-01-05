// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Marco Maggiali marco.maggiali@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "include/TouchSensor.h"

#ifndef __ALE_FPALM_H__
#define __ALE_FPALM_H__

class fakePalm : public TouchSensor
{
public:
    fakePalm(double cx,double cy,double th,double gain=1.0,int layoutNum=0,int lrMirror=0)
    {
        const double DEG2RAD=M_PI/180.0;

        const double CST=cos(DEG2RAD*th);
        const double SNT=sin(DEG2RAD*th);

        const double H=sin(DEG2RAD*60.0);
        const double L=2.0*H/9.0;
     
        dGain=gain;
        dGain=0.0; // for now to stick the dots to be always OFF
        ilrMirror=lrMirror;
        ilayoutNum=layoutNum;
        nVerts=4;
        nTaxels=1;
        m_RadiusOrig=0.1;

		for (int i=0; i<nTaxels; ++i)
        {
			dX[0]=15; //not used
			dY[0]=15; //not used 
			dX[i] = 0.0; dY[i] = 0.0;
		}



       

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
