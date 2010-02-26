// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <gsl/gsl_math.h>
#include <memory.h>

#include <stdio.h>

#ifndef __ALE_TOUCHSENSOR_H__
#define __ALE_TOUCHSENSOR_H__

class TouchSensor
{
public:
    void resize(int width,int height,int margin)
    {
        if (3*margin>=width || 3*margin>=height) margin=0;

        double scaleX=double(width -2*margin)/(dXmax-dXmin);
        double scaleY=double(height-2*margin)/(dYmax-dYmin);
        double scale=scaleX<scaleY?scaleX:scaleY;

        m_Radius=m_RadiusOrig*scale;

        double dXmid=0.5*(dXmin+dXmax);
        double dYmid=0.5*(dYmin+dYmax);

        int w2=width /2;
        int h2=height/2;

        for (int i=0; i<12; ++i)
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
        int maxRange=int(3.0*sigma);
        
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
    
    void eval(double *image)
    {
        int dx,dy;
        int Y0,Y1;
        double k0,k1;
        int dya,dyb,dxa,dxb;

        for (int i=0; i<12; ++i) if (activation[i]>0.0)
        {
            k0=dGain*activation[i];
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
                    image[dx+Y1]+=k1*Exponential[Abs(dx)];
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

    void draw(unsigned char *image)
    {
        for (int i=0; i<nVerts; ++i)
        {
            drawLine(image,xv[i],yv[i],xv[(i+1)%nVerts],yv[(i+1)%nVerts]);
        }
        
        for (int i=0; i<12; ++i)
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
        if (image[bytePos-1]<B4) image[bytePos-1]=B4;

        if (image[bytePos  ]<R2) image[bytePos  ]=R2;
        if (image[bytePos+1]<G2) image[bytePos+1]=G2;
        if (image[bytePos+2]<B2) image[bytePos+2]=B2;

        if (image[bytePos+3]<R4) image[bytePos+3]=R4;
        if (image[bytePos+4]<G4) image[bytePos+4]=G4;
        if (image[bytePos+5]<B4) image[bytePos+5]=B4;

        bytePos+=m_Width*3;

        if (image[bytePos-3]<R2) image[bytePos-3]=R2;
        if (image[bytePos-2]<G2) image[bytePos-2]=G2;
        if (image[bytePos-1]<B2) image[bytePos-1]=B2;

        if (image[bytePos  ]<R1) image[bytePos  ]=R1;
        if (image[bytePos+1]<G1) image[bytePos+1]=G1;
        if (image[bytePos+2]<B1) image[bytePos+2]=B1;

        if (image[bytePos+3]<R2) image[bytePos+3]=R2;
        if (image[bytePos+4]<G2) image[bytePos+4]=G2;
        if (image[bytePos+5]<B2) image[bytePos+5]=B2;

        bytePos+=m_Width*3;

        if (image[bytePos-3]<R4) image[bytePos-3]=R4;
        if (image[bytePos-2]<G4) image[bytePos-2]=G4;
        if (image[bytePos-1]<B4) image[bytePos-1]=B4;

        if (image[bytePos  ]<R2) image[bytePos  ]=R2;
        if (image[bytePos+1]<G2) image[bytePos+1]=G2;
        if (image[bytePos+2]<B2) image[bytePos+2]=B2;

        if (image[bytePos+3]<R4) image[bytePos+3]=R4;
        if (image[bytePos+4]<G4) image[bytePos+4]=G4;
        if (image[bytePos+5]<B4) image[bytePos+5]=B4;
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
    double dX[12],dY[12];
    static double dXmin,dXmax,dYmin,dYmax;
    double dXv[8],dYv[8];
    double dXc,dYc;
    double dGain;

    double m_Radius,m_RadiusOrig;
    double activation[12];

    static int m_maxRange;
    static double *Exponential;
    
    // scaled
    int x[12],y[12];
    int xv[8],yv[8];

    int nVerts;

    int xMin,xMax,yMin,yMax;

    int m_Width,m_Height;
};

#endif
