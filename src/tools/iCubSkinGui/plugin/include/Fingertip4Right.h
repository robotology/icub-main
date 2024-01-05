// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Marco Maggiali marco.maggiali@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "include/TouchSensor.h"

#ifndef __ALE_FINGERTIP4_R_H__
#define __ALE_FINGERTIP4_R_H__

class Fingertip4R : public TouchSensor
{
public:
    Fingertip4R(double cx,double cy,double th,double gain=1.0,int layoutNum=0,int lrMirror=0)
    {
        dGain=gain;
        ilrMirror=lrMirror;
        ilayoutNum=layoutNum;
        nVerts=7;
        nTaxels=12;

        dY[4]=-25.0;  dX[4]= -20.0; 
        dY[5]=-25.0;  dX[5]=   0.0; 
        dY[6]=-25.0;  dX[6]=  20.0; 
        dY[8]=-39.0;  dX[8]=  35.0; 
        dY[7]=-39.0;  dX[7]= -35.0; 
        dY[9]=-53.0;  dX[9]=  20.0; 
        dY[10]=-53.0;  dX[10]=   0.0; 
        dY[11]=-53.0;  dX[11]= -20.0; 
        dY[3]= -2.0;  dX[3]= -22.0; 
        dY[2]= 10.0;  dX[2]= -5.0; 
        dY[1]= 28.0; dX[1]=  4.0;
        dY[0]= 30.0; dX[0]=  -32.0;     
        
		lrMirror=0; //forcing mirror

        dXv[0]=53.0; dYv[0]= 0.0;
        dXv[1]=53.0; dYv[1]=45.0;
        dXv[2]=dX[3]+10.0; dYv[2]=dY[3]+10.0;
        dXv[3]=0.0; dYv[3]=dY[5]+12.0;
        dXv[4]=-dXv[2]; dYv[4]=dYv[2];
        dXv[5]=-dXv[1]; dYv[5]=dYv[1];
        dXv[6]=-dXv[0]; dYv[6]=dYv[0];

        const double scale=2.7/15.3;
        for (int i=0; i<nTaxels; ++i)
        {
            dX[i]*=scale;
            dY[i]*=scale;
        }
        for (int i=0; i<nVerts; ++i)
        {
            dXv[i]*=scale;
            dYv[i]*=scale;
        }

        m_RadiusOrig=1.5;

        const double DEG2RAD=M_PI/180.0;
        const double CST=cos(DEG2RAD*th);
        const double SNT=sin(DEG2RAD*th);

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

            dXv[i]=0;
            dYv[i]=0;
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
