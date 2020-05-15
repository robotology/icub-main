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

        dGain=gain;
        ilrMirror=lrMirror;
        ilayoutNum=layoutNum;
        nVerts=7;
        nTaxels=36;
        m_RadiusOrig=1.8;

        for (int i = 0; i<nTaxels; ++i) dX[0] = dY[0] = 0.0;

        int n = 0;

        dX[n]=37.2; dY[n++]=16.7;  //   0
        dX[n]=31.2; dY[n++]=22.7;  //   1
        dX[n]=31.2; dY[n++]=16.7;  //   2
        dX[n]=43.2; dY[n++]=10.7;  //   3
        dX[n]=25.2; dY[n++]=22.7;  //   4
        dX[n]=37.2; dY[n++]=10.7;  //   5
        dX[n]=43.2; dY[n++]= 4.7;  //   6
        dX[n]=31.2; dY[n++]= 4.7;  //   7
        dX[n]=37.2; dY[n++]= 4.7;  //   8
        dX[n]=25.2; dY[n++]= 4.7;  //   9
        dX[n]=31.2; dY[n++]=10.7;  //  10
        connected[n++] = false;    //  11
        dX[n]=25.2; dY[n++]=10.7;  //  12
        dX[n]=13.2; dY[n++]= 4.7;  //  13
        dX[n]= 7.2; dY[n++]= 4.7;  //  14
        dX[n]=19.2; dY[n++]= 4.7;  //  15
        dX[n]= 7.2; dY[n++]=10.7;  //  16
        dX[n]=13.2; dY[n++]=10.7;  //  17
        dX[n]= 7.2; dY[n++]=16.7;  //  18
        dX[n]=13.2; dY[n++]=16.7;  //  19
        dX[n]=19.2; dY[n++]=10.7;  //  20
        dX[n]=19.2; dY[n++]=16.7;  //  21
        dX[n]=25.2; dY[n++]=16.7;  //  22
        connected[n++] = false;    //  23
        dX[n]= 7.2; dY[n++]=22.7;  //  24
        dX[n]=10.2; dY[n++]=27.7;  //  25
        dX[n]=13.2; dY[n++]=22.7;  //  26
        dX[n]=16.2; dY[n++]=27.7;  //  27
        dX[n]=19.2; dY[n++]=22.7;  //  28
        connected[n++] = false;    //  29
        connected[n++] = false;    //  30
        connected[n++] = false;    //  31
        connected[n++] = false;    //  32
        connected[n++] = false;    //  33
        connected[n++] = false;    //  34
        connected[n++] = false;    //  35

        for (int i=0; i<nTaxels; ++i)
        {
            double x=dX[i]-0.0;
            double y=dY[i]-0.0;

            if (lrMirror) x=-x;

            dX[i]=cx+CST*x-SNT*y;
            dY[i]=cy+SNT*x+CST*y;
        }

        dXv[0]=2;   dYv[0]=0;
        dXv[1]=2;   dYv[1]=28;
        dXv[2]=7;   dYv[2]=33;
        dXv[3]=22;  dYv[3]=33;
        dXv[4]=38;  dYv[4]=24;
        dXv[5]=49;  dYv[5]=12;
        dXv[6]=49;  dYv[6]=0;

        for (int i=0; i<nVerts; ++i)
        {
            double x=dXv[i];
            double y=dYv[i];
            if (lrMirror) x=-x;
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
        for (int i = 0; i<nVerts; ++i)
        {
            drawLine(image, xv[i], yv[i], xv[(i + 1) % nVerts], yv[(i + 1) % nVerts]);
        }

        for (int i = 0; i <= 28; ++i) if (connected[i])
        {
            drawCircle(image, x[i], y[i], m_Radius);
        }
    }
};

class CER_SH_PDR : public TouchSensor
{
public:

    CER_SH_PDR(double cx, double cy, double th, double gain = 1.0, int layoutNum = 0, int lrMirror = 0)
    {
        const double DEG2RAD = M_PI / 180.0;

        const double CST = cos(DEG2RAD*th);
        const double SNT = sin(DEG2RAD*th);

        dGain = gain;
        ilrMirror = lrMirror;
        ilayoutNum = layoutNum;
        nVerts = 7;
        nTaxels = 36;
        m_RadiusOrig = 1.8;

        for (int i = 0; i<nTaxels; ++i) dX[0] = dY[0] = 0.0;

        int n = 0;

        dX[n]=11.2; dY[n++]=16.7;  //   0
        dX[n]=17.2; dY[n++]=22.7;  //   1
        dX[n]=17.2; dY[n++]=16.7;  //   2
        dX[n]= 5.2; dY[n++]=10.7;  //   3
        dX[n]=23.2; dY[n++]=22.7;  //   4
        dX[n]=11.2; dY[n++]=10.7;  //   5
        dX[n]= 5.2; dY[n++]= 4.7;  //   6
        dX[n]=17.2; dY[n++]= 4.7;  //   7
        dX[n]=11.2; dY[n++]= 4.7;  //   8
        dX[n]=23.2; dY[n++]= 4.7;  //   9
        dX[n]=17.2; dY[n++]=10.7;  //  10
        connected[n++] = false;    //  11
        dX[n]=23.2; dY[n++]=10.7;  //  12
        dX[n]=35.2; dY[n++]= 4.7;  //  13
        dX[n]=41.2; dY[n++]= 4.7;  //  14
        dX[n]=29.2; dY[n++]= 4.7;  //  15
        dX[n]=41.2; dY[n++]=10.7;  //  16
        dX[n]=35.2; dY[n++]=10.7;  //  17
        dX[n]=41.2; dY[n++]=16.7;  //  18
        dX[n]=35.2; dY[n++]=16.7;  //  19
        dX[n]=29.2; dY[n++]=10.7;  //  20
        dX[n]=29.2; dY[n++]=16.7;  //  21
        dX[n]=23.2; dY[n++]=16.7;  //  22
        connected[n++] = false;    //  23
        dX[n]=41.2; dY[n++]=22.7;  //  24
        dX[n]=38.2; dY[n++]=27.7;  //  25
        dX[n]=35.2; dY[n++]=22.7;  //  26
        dX[n]=32.2; dY[n++]=27.7;  //  27
        dX[n]=29.2; dY[n++]=22.7;  //  28
        connected[n++] = false;    //  29
        connected[n++] = false;    //  30
        connected[n++] = false;    //  31
        connected[n++] = false;    //  32
        connected[n++] = false;    //  33
        connected[n++] = false;    //  34
        connected[n++] = false;    //  35

        for (int i = 0; i<nTaxels; ++i)
        {
            double x = dX[i];
            double y = dY[i];

            if (lrMirror) x = -x;

            dX[i] = cx + CST*x - SNT*y;
            dY[i] = cy + SNT*x + CST*y;
        }

        dXv[0] = 49-2;   dYv[0] = 0;
        dXv[1] = 49-2;   dYv[1] = 28;
        dXv[2] = 49-7;   dYv[2] = 33;
        dXv[3] = 49-22;  dYv[3] = 33;
        dXv[4] = 49-38;  dYv[4] = 24;
        dXv[5] = 49-49;  dYv[5] = 12;
        dXv[6] = 49-49;  dYv[6] = 0;

        for (int i = 0; i<nVerts; ++i)
        {
            double x = dXv[i];
            double y = dYv[i];
            if (lrMirror) x = -x;
            dXv[i] = cx + CST*x - SNT*y;
            dYv[i] = cy + SNT*x + CST*y;
        }

        // in static definition
        //dXmin=dYmin= HUGE;
        //dXmax=dYmax=-HUGE;

        for (int i = 0; i<nVerts; ++i)
        {
            if (dXv[i]<dXmin) dXmin = dXv[i];
            if (dXv[i]>dXmax) dXmax = dXv[i];
            if (dYv[i]<dYmin) dYmin = dYv[i];
            if (dYv[i]>dYmax) dYmax = dYv[i];
        }

        dXc = cx;
        dYc = cy;
    }
    void draw(unsigned char *image)
    {
        for (int i = 0; i<nVerts; ++i)
        {
            drawLine(image, xv[i], yv[i], xv[(i + 1) % nVerts], yv[(i + 1) % nVerts]);
        }

        for (int i = 0; i<=28; ++i) if (connected[i])
        {
            drawCircle(image, x[i], y[i], m_Radius);
        }
    }
};

class CER_SH_PP : public TouchSensor
{
public:

    CER_SH_PP(double cx, double cy, double th, double gain = 1.0, int layoutNum = 0, int lrMirror = 0)
    {
        const double DEG2RAD = M_PI / 180.0;

        const double CST = cos(DEG2RAD*th);
        const double SNT = sin(DEG2RAD*th);

        dGain = gain;
        ilrMirror = lrMirror;
        ilayoutNum = layoutNum;
        nVerts = 4;
        nTaxels = 24;
        m_RadiusOrig = 1.8;

        for (int i = 0; i<nTaxels; ++i) dX[0] = dY[0] = 0.0;

        int n = 0;

        dX[n] = 10; dY[n++] =  3.5;  //  48
        dX[n] = 16; dY[n++] =  3.5;  //  49
        dX[n] =  4; dY[n++] =  3.5;  //  50
        dX[n] =  4; dY[n++] =  9.5;  //  51
        dX[n] =  4; dY[n++] = 21.5;  //  52
        dX[n] =  4; dY[n++] = 15.5;  //  53
        dX[n] = 10; dY[n++] =  9.5;  //  54
        dX[n] = 10; dY[n++] = 15.5;  //  55
        dX[n] = 10; dY[n++] = 21.5;  //  56
        dX[n] = 16; dY[n++] = 15.5;  //  57
        dX[n] = 16; dY[n++] =  9.5;  //  58
        connected[n++] = false;      //  59
        dX[n] = 16; dY[n++] = 33.5;  //  60
        dX[n] = 16; dY[n++] = 39.5;  //  61
        dX[n] = 10; dY[n++] = 33.5;  //  62
        dX[n] = 16; dY[n++] = 27.5;  //  63
        dX[n] = 10; dY[n++] = 39.5;  //  64
        dX[n] = 16; dY[n++] = 21.5;  //  65
        dX[n] = 10; dY[n++] = 27.5;  //  66
        dX[n] =  4; dY[n++] = 27.5;  //  67
        dX[n] =  4; dY[n++] = 33.5;  //  68
        dX[n] =  4; dY[n++] = 39.5;  //  69
        connected[n++] = false;      //  70
        connected[n++] = false;      //  71

        for (int i = 0; i<nTaxels; ++i)
        {
            double x = dX[i];
            double y = dY[i];

            if (lrMirror) x = -x;

            dX[i] = cx + CST*x - SNT*y;
            dY[i] = cy + SNT*x + CST*y;
        }

        dXv[0] = -1;   dYv[0] = -2;
        dXv[1] = -1;   dYv[1] = 45;
        dXv[2] = 21;  dYv[2] = 45;
        dXv[3] = 21;  dYv[3] = -2;

        for (int i = 0; i<nVerts; ++i)
        {
            double x = dXv[i];
            double y = dYv[i];
            if (lrMirror) x = -x;
            dXv[i] = cx + CST*x - SNT*y;
            dYv[i] = cy + SNT*x + CST*y;
        }

        // in static definition
        //dXmin=dYmin= HUGE;
        //dXmax=dYmax=-HUGE;

        for (int i = 0; i<nVerts; ++i)
        {
            if (dXv[i]<dXmin) dXmin = dXv[i];
            if (dXv[i]>dXmax) dXmax = dXv[i];
            if (dYv[i]<dYmin) dYmin = dYv[i];
            if (dYv[i]>dYmax) dYmax = dYv[i];
        }

        dXc = cx;
        dYc = cy;
    }
    void draw(unsigned char *image)
    {
        for (int i = 0; i<nVerts; ++i)
        {
            drawLine(image, xv[i], yv[i], xv[(i + 1) % nVerts], yv[(i + 1) % nVerts]);
        }

        for (int i = 0; i <= 21; ++i) if (connected[i])
        {
            drawCircle(image, x[i], y[i], m_Radius);
        }
    }
};

class CER_SH_TD : public TouchSensor
{
public:

    CER_SH_TD(double cx, double cy, double th, double gain = 1.0, int layoutNum = 0, int lrMirror = 0)
    {
        const double DEG2RAD = M_PI / 180.0;

        const double CST = cos(DEG2RAD*th);
        const double SNT = sin(DEG2RAD*th);

        dGain = gain;
        ilrMirror = lrMirror;
        ilayoutNum = layoutNum;
        nVerts = 6;
        nTaxels = 24;
        m_RadiusOrig = 1.8;

        for (int i = 0; i < nTaxels; ++i) dX[0] = dY[0] = 0.0;

        int n = 0;

        dX[n] = 30.0; dY[n++] = 10.5;  //  96
        dX[n] = 30.0; dY[n++] =  4.5;  //  97
        dX[n] = 18.0; dY[n++] =  4.5;  //  98
        dX[n] =  6.1; dY[n++] =  4.5;  //  99
        dX[n] = 12.0; dY[n++] =  4.5;  // 100
        dX[n] = 24.0; dY[n++] =  4.5;  // 101
        dX[n] = 12.0; dY[n++] = 16.4;  // 102
        dX[n] = 18.0; dY[n++] = 10.5;  // 103
        dX[n] = 18.0; dY[n++] = 16.5;  // 104
        dX[n] = 24.0; dY[n++] = 16.5;  // 105
        dX[n] = 24.0; dY[n++] = 10.5;  // 106
        connected[n++] = false;        // 107
        dX[n] = 12.0; dY[n++] = 10.5;  // 108
        dX[n] =  6.0; dY[n++] = 10.5;  // 109
        connected[n++] = false;        // 110
        connected[n++] = false;        // 111
        connected[n++] = false;        // 112
        connected[n++] = false;        // 113
        connected[n++] = false;        // 114
        connected[n++] = false;        // 115
        connected[n++] = false;        // 116
        connected[n++] = false;        // 117
        connected[n++] = false;        // 118
        connected[n++] = false;        // 119

        for (int i = 0; i < nTaxels; ++i)
        {
            double x = dX[i];
            double y = dY[i];

            if (lrMirror) x = -x;

            dX[i] = cx + CST*x - SNT*y;
            dY[i] = cy + SNT*x + CST*y;
        }

        dXv[0] = 1;   dYv[0] = 0;
        dXv[1] = 1;   dYv[1] = 16;
        dXv[2] = 7;   dYv[2] = 22;
        dXv[3] = 29;  dYv[3] = 22;
        dXv[4] = 35;  dYv[4] = 16;
        dXv[5] = 35;  dYv[5] = 0;

        for (int i = 0; i < nVerts; ++i)
        {
            double x = dXv[i];
            double y = dYv[i];
            if (lrMirror) x = -x;
            dXv[i] = cx + CST*x - SNT*y;
            dYv[i] = cy + SNT*x + CST*y;
        }

        // in static definition
        //dXmin=dYmin= HUGE;
        //dXmax=dYmax=-HUGE;

        for (int i = 0; i < nVerts; ++i)
        {
            if (dXv[i] < dXmin) dXmin = dXv[i];
            if (dXv[i] > dXmax) dXmax = dXv[i];
            if (dYv[i] < dYmin) dYmin = dYv[i];
            if (dYv[i] > dYmax) dYmax = dYv[i];
        }

        dXc = cx;
        dYc = cy;
    }
    void draw(unsigned char *image)
    {
        for (int i = 0; i < nVerts; ++i)
        {
            drawLine(image, xv[i], yv[i], xv[(i + 1) % nVerts], yv[(i + 1) % nVerts]);
        }

        for (int i = 0; i <= 13; ++i) if (connected[i])
        {
            drawCircle(image, x[i], y[i], m_Radius);
        }
    }
};

class CER_SH_TP : public TouchSensor
{
public:

    CER_SH_TP(double cx, double cy, double th, double gain = 1.0, int layoutNum = 0, int lrMirror = 0)
    {
        const double DEG2RAD = M_PI / 180.0;

        const double CST = cos(DEG2RAD*th);
        const double SNT = sin(DEG2RAD*th);

        dGain = gain;
        ilrMirror = lrMirror;
        ilayoutNum = layoutNum;
        nVerts = 4;
        nTaxels = 12;
        m_RadiusOrig = 1.8;

        for (int i = 0; i < nTaxels; ++i) dX[0] = dY[0] = 0.0;

        int n = 0;

        dX[n] =  3.0; dY[n++] = 17;   // 144
        dX[n] =  3.0; dY[n++] =  5;   // 145
        dX[n] =  3.0; dY[n++] = 11;   // 146
        dX[n] = 10.0; dY[n++] =  5;   // 147
        dX[n] = 10.0; dY[n++] = 29;   // 148
        dX[n] =  3.0; dY[n++] = 23;   // 149
        dX[n] =  3.0; dY[n++] = 29;   // 150
        dX[n] = 10.0; dY[n++] = 23;   // 151
        dX[n] = 10.0; dY[n++] = 17;   // 152
        dX[n] = 10.0; dY[n++] = 11;   // 153
        connected[n++] = false;       // 154
        connected[n++] = false;       // 155


        for (int i = 0; i < nTaxels; ++i)
        {
            double x = dX[i];
            double y = dY[i];

            if (lrMirror) x = -x;

            dX[i] = cx + CST*x - SNT*y;
            dY[i] = cy + SNT*x + CST*y;
        }

        dXv[0] = -2;  dYv[0] = 0;
        dXv[1] = -2;  dYv[1] = 34;
        dXv[2] = 15;  dYv[2] = 34;
        dXv[3] = 15;  dYv[3] = 0;

        for (int i = 0; i < nVerts; ++i)
        {
            double x = dXv[i];
            double y = dYv[i];
            if (lrMirror) x = -x;
            dXv[i] = cx + CST*x - SNT*y;
            dYv[i] = cy + SNT*x + CST*y;
        }

        // in static definition
        //dXmin=dYmin= HUGE;
        //dXmax=dYmax=-HUGE;

        for (int i = 0; i < nVerts; ++i)
        {
            if (dXv[i] < dXmin) dXmin = dXv[i];
            if (dXv[i] > dXmax) dXmax = dXv[i];
            if (dYv[i] < dYmin) dYmin = dYv[i];
            if (dYv[i] > dYmax) dYmax = dYv[i];
        }

        dXc = cx;
        dYc = cy;
    }
    void draw(unsigned char *image)
    {
        for (int i = 0; i < nVerts; ++i)
        {
            drawLine(image, xv[i], yv[i], xv[(i + 1) % nVerts], yv[(i + 1) % nVerts]);
        }

        for (int i = 0; i <= 9; ++i) if (connected[i])
        {
            drawCircle(image, x[i], y[i], m_Radius);
        }
    }
};


#endif
