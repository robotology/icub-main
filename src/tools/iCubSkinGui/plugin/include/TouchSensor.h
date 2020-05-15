// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <gsl/gsl_math.h>
#include <memory.h>

#include <stdio.h>

#ifndef __ALE_TOUCHSENSOR_H__
#define __ALE_TOUCHSENSOR_H__

#define MAX_TAXELS 192

class TouchSensor
{
protected:

    bool calibrated_skin;

    TouchSensor()
    {
        for (int n = 0; n < MAX_TAXELS; ++n)
        {
            connected[n] = true;
        }
    }

public:

    void setColor(unsigned char r, unsigned char g, unsigned char b)
    {
        R_MAX = r;
        G_MAX = g;
        B_MAX = b;
    }

    void setCalibrationFlag (bool use_calibrated_skin)
    {
        calibrated_skin=use_calibrated_skin;
    }

    void resize(int width,int height,int margin)
    {
        if (3*margin>=width || 3*margin>=height) margin=0;

        double scaleX=double(width -2*margin)/(dXmax-dXmin);
        double scaleY=double(height-2*margin)/(dYmax-dYmin);
        double scale=scaleX<scaleY?scaleX:scaleY;

        m_Radius=m_RadiusOrig*scale;

        m_maxRangeLight=int(m_Radius);

        double dXmid=0.5*(dXmin+dXmax);
        double dYmid=0.5*(dYmin+dYmax);

        int w2=width /2;
        int h2=height/2;

        for (int i=0; i<nTaxels; ++i)
        {
            x[i]=w2+int(scale*(dX[i]-dXmid));
            y[i]=h2+int(scale*(dY[i]-dYmid));
        }

        for (int i=0; i<nVerts; ++i)
        {
            xv[i]=w2+int(scale*(dXv[i]-dXmid));
            yv[i]=h2+int(scale*(dYv[i]-dYmid));
        }

        double sigma=0.5*5.55*scale;
        int maxRange=int(2.5*sigma);

        if (maxRange!=m_maxRange)
        {
            m_maxRange=maxRange;

            delete [] Exponential;
            Exponential=new double[maxRange];

            double k=-0.5/(sigma*sigma);
            for (int x=0; x<maxRange; ++x)
            {
                Exponential[x]=exp(k*double(x*x));
            }
        }

        xMin=w2+int(scale*(dXc-dXmid-15.0))-maxRange;
        xMax=w2+int(scale*(dXc-dXmid+15.0))+maxRange;
        yMin=h2+int(scale*(dYc-dYmid-15.0))-maxRange;
        yMax=h2+int(scale*(dYc-dYmid+15.0))+maxRange;

        if (xMin<0)      xMin=0;
        if (xMax>width)  xMax=width;
        if (yMin<0)      yMin=0;
        if (yMax>height) yMax=height;

        m_Width=width;
        m_Height=height;
    }

    virtual ~TouchSensor()
    {
        if (Exponential)
        {
            delete [] Exponential;
            Exponential=0;
        }
    }

    int Abs(int x)
    {
        return x>=0?x:-x;
    }

    int get_nTaxels ()
    {
        return nTaxels;
    }

    void eval_light(unsigned char *image)
    {
        int act;
        int dx,dy;
        int Y0,Y1;
        int dya,dyb,dxa,dxb;

        switch (ilayoutNum)
        {
            case 0:
                for (int i=0; i<nTaxels; ++i)  remapped_activation[i]=activation[i];
                break;
            case 1:
                for (int i=0; i<nTaxels; ++i)  remapped_activation[nTaxels-1-i]=activation[i];
                break;
            default:
                for (int i=0; i<nTaxels; ++i)  remapped_activation[i]=activation[i];
                printf("WARN: unkwnown layout number.\n");
                break;
        }

        int maxRange2=m_maxRangeLight*m_maxRangeLight;

        for (int i=0; i<nTaxels; ++i) if (connected[i] && remapped_activation[i]>0.0)
        {
            act=int(dGain*remapped_activation[i]);
            Y0=(m_Height-y[i]-1)*m_Width+x[i];

            dya=(y[i]>=m_maxRangeLight)?-m_maxRangeLight:-y[i];
            dyb=(y[i]+m_maxRangeLight<m_Height)?m_maxRangeLight:m_Height-y[i]-1;

            dxa=(x[i]>=m_maxRangeLight)?-m_maxRangeLight:-x[i];
            dxb=(x[i]+m_maxRangeLight<m_Width)?m_maxRangeLight:m_Width-x[i]-1;

            for (dy=dya; dy<=dyb; ++dy)
            {
                Y1=Y0-dy*m_Width;

                for (dx=dxa; dx<=dxb; ++dx)
                {
                    if (dx*dx+dy*dy<=maxRange2)
                    {
                        image[(dx+Y1)*3]=act<255?act:255;
                    }
                }
            }
        }
    }

    void eval(unsigned char *image)
    {
        int act;
        int dx,dy;
        int Y0,Y1;
        int index;
        double k0,k1;
        int dya,dyb,dxa,dxb;

        switch (ilayoutNum)
        {
            case 0:
                for (int i=0; i<nTaxels; ++i)  remapped_activation[i]=activation[i];
                break;
            case 1:
                for (int i=0; i<nTaxels; ++i)  remapped_activation[nTaxels-1-i]=activation[i];
                break;
            default:
                for (int i=0; i<nTaxels; ++i)  remapped_activation[i]=activation[i];
                printf("WARN: unkwnown layout number.\n");
                break;
        }

        for (int i=0; i<nTaxels; ++i) if (connected[i] && remapped_activation[i]>0.0)
        {
            k0=dGain*remapped_activation[i];
            Y0=(m_Height-y[i]-1)*m_Width+x[i];

            dya=(y[i]>=m_maxRange)?-m_maxRange:-y[i];
            dyb=(y[i]+m_maxRange<m_Height)?m_maxRange:m_Height-y[i]-1;

            dxa=(x[i]>=m_maxRange)?-m_maxRange:-x[i];
            dxb=(x[i]+m_maxRange<m_Width)?m_maxRange:m_Width-x[i]-1;

            for (dy=dya; dy<=dyb; ++dy)
            {
                k1=k0*Exponential[Abs(dy)];
                Y1=Y0-dy*m_Width;

                for (dx=dxa; dx<=dxb; ++dx)
                {
                    index=(dx+Y1)*3;

                    if (image[index]<R_MAX || image[index+1]<G_MAX || image[index+2]<B_MAX)
                    {
                        act=int(k1*Exponential[Abs(dx)]);

                        int actR=image[index  ]+(act*R_MAX)/255;
                        int actG=image[index+1]+(act*G_MAX)/255;
                        int actB=image[index+2]+(act*B_MAX)/255;

                        image[index  ]=actR<R_MAX?actR:R_MAX;
                        image[index+1]=actG<G_MAX?actG:G_MAX;
                        image[index+2]=actB<B_MAX?actB:B_MAX;
                    }
                }
            }
        }
    }

    void setActivationFirst7(unsigned char* data)
    {
        for (int i=0; i<7; ++i)
        {
            activation[i]=data[i+1]<=244?double(244-data[i+1]):0.0;
        }
    }
    void setActivationLast5(unsigned char* data)
    {
        for (int i=1; i<=5; ++i)
        {
            activation[i+6]=data[i]<=244?double(244-data[i]):0.0;
        }
    }

    void setActivationFromPortData(double val, int id)
    {
        if (calibrated_skin)
        {
            if (val>244.0)
            {
                activation[id]=244.0;
            }
            else if (val<0.0)
            {
                activation[id]=0.0;
            }
            else
            {
                activation[id]=val;
            }
        }
        else
        {
            activation[id]=val<=244?double(244-val):0.0;
        }
    }

    virtual void draw(unsigned char *image)
    {
        for (int i=0; i<nVerts; ++i)
        {
            drawLine(image,xv[i],yv[i],xv[(i+1)%nVerts],yv[(i+1)%nVerts]);
        }

        for (int i=0; i<nTaxels; ++i)
        {
            drawCircle(image,x[i],y[i],m_Radius);
        }
    }

protected:
    void dither(int x,int y,unsigned char *image)
    {
        static const unsigned char R1=0x80,G1=0x50,B1=0x00;
        static const unsigned char R2=3*R1/4,G2=3*G1/4,B2=3*B1/4;
        static const unsigned char R4=3*R2/4,G4=3*G2/4,B4=3*B2/4;

        //y=m_Width-y-1;
        //int bytePos=(x+(y-1)*m_Width)*3;
        int bytePos=(x+(m_Height-y-2)*m_Width)*3;

        if (image[bytePos-3]<R4) image[bytePos-3]=R4;
        if (image[bytePos-2]<G4) image[bytePos-2]=G4;
        //if (image[bytePos-1]<B4) image[bytePos-1]=B4;

        if (image[bytePos  ]<R2) image[bytePos  ]=R2;
        if (image[bytePos+1]<G2) image[bytePos+1]=G2;
        //if (image[bytePos+2]<B2) image[bytePos+2]=B2;

        if (image[bytePos+3]<R4) image[bytePos+3]=R4;
        if (image[bytePos+4]<G4) image[bytePos+4]=G4;
        //if (image[bytePos+5]<B4) image[bytePos+5]=B4;

        bytePos+=m_Width*3;

        if (image[bytePos-3]<R2) image[bytePos-3]=R2;
        if (image[bytePos-2]<G2) image[bytePos-2]=G2;
        //if (image[bytePos-1]<B2) image[bytePos-1]=B2;

        if (image[bytePos  ]<R1) image[bytePos  ]=R1;
        if (image[bytePos+1]<G1) image[bytePos+1]=G1;
        //if (image[bytePos+2]<B1) image[bytePos+2]=B1;

        if (image[bytePos+3]<R2) image[bytePos+3]=R2;
        if (image[bytePos+4]<G2) image[bytePos+4]=G2;
        //if (image[bytePos+5]<B2) image[bytePos+5]=B2;

        bytePos+=m_Width*3;

        if (image[bytePos-3]<R4) image[bytePos-3]=R4;
        if (image[bytePos-2]<G4) image[bytePos-2]=G4;
        //if (image[bytePos-1]<B4) image[bytePos-1]=B4;

        if (image[bytePos  ]<R2) image[bytePos  ]=R2;
        if (image[bytePos+1]<G2) image[bytePos+1]=G2;
        //if (image[bytePos+2]<B2) image[bytePos+2]=B2;

        if (image[bytePos+3]<R4) image[bytePos+3]=R4;
        if (image[bytePos+4]<G4) image[bytePos+4]=G4;
        //if (image[bytePos+5]<B4) image[bytePos+5]=B4;
    }

    void drawLine(unsigned char *image,int x0,int y0,int x1,int y1)
    {
        if (x1==x0 && y1==y0) return;
        double Vx=double(x1-x0);
        double Vy=double(y1-y0);
        double dt=1.0/sqrt(Vx*Vx+Vy*Vy);

        for (double t=0.0; t<=1.0; t+=dt)
        {
            dither(x0+int(t*Vx),y0+int(t*Vy),image);
        }
    }

    void drawCircle(unsigned char *image,int cx,int cy,double radius)
    {
        double dt=1.0/(2*M_PI*radius);

        int dx,dy;

        double cs,sn;

        for (double t=0.0; t<=M_PI_4; t+=dt)
        {
            cs=cos(t);
            sn=sin(t);

            dx=int(radius*cs);
            dy=int(radius*sn);

            dither(cx+dx,cy+dy,image);
            dither(cx+dx,cy-dy,image);
            dither(cx-dx,cy-dy,image);
            dither(cx-dx,cy+dy,image);

            dither(cx+dy,cy+dx,image);
            dither(cx+dy,cy-dx,image);
            dither(cx-dy,cy-dx,image);
            dither(cx-dy,cy+dx,image);
        }
    }

    // original
    double dX[MAX_TAXELS],dY[MAX_TAXELS];
    static double dXmin,dXmax,dYmin,dYmax;
    double dXv[8],dYv[8];
    double dXc,dYc;
    double dGain;
    int    ilayoutNum;
    int    ilrMirror;

    double m_Radius,m_RadiusOrig;
    double activation[MAX_TAXELS];
    double remapped_activation[MAX_TAXELS];
    bool connected[MAX_TAXELS];

    unsigned char R_MAX, G_MAX, B_MAX;

    int m_maxRangeLight;
    static int m_maxRange;
    static double *Exponential;

    // scaled
    int x[MAX_TAXELS],y[MAX_TAXELS];
    int xv[8],yv[8];

    int nVerts;
    int nTaxels;

    int xMin,xMax,yMin,yMax;

    int m_Width,m_Height;

    public:
    int min_tax;
    int max_tax;
};

#endif
