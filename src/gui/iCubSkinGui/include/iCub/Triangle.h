// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#define _USE_MATH_DEFINES
#include <math.h>

#include <memory.h>

#ifndef __ALE_TRIANGLE_H__
#define __ALE_TRIANGLE_H__

class Triangle
{
public:
    Triangle(double cx,double cy,double th)
    {
        double deg2rad=M_PI/180.0;
        double H=sin(deg2rad*60.0);
        double l=2.0*H/9.0;
        
        xm[6]=l*cos(deg2rad*30.0); xm[5]=0.5-xm[6]; xm[3]=0.5+xm[6]; xm[2]=1.0-xm[6];
        xm[7]=0.25; xm[4]=0.5; xm[1]=0.75;
        xm[8]=0.25+xm[6]; xm[12]=0.75-xm[6];
        xm[9]=xm[8]; xm[11]=xm[12];
        xm[10]=0.5;

        ym[6]=l*sin(deg2rad*30.0); ym[5]=ym[6]; ym[3]=ym[6]; ym[2]=ym[6];
        ym[7]=0.5*H-l; ym[4]=l; ym[1]=ym[7];
        ym[8]=0.5*H-l*sin(deg2rad*30.0); ym[12]=ym[8];
        ym[9]=0.5*H+l*sin(deg2rad*30.0); ym[11]=ym[9];
        ym[10]=H-l;

        for (int i=1; i<=12; ++i)
        {
            xm[i]=30.0*(xm[i]-0.5);
            ym[i]=30.0*ym[i]-10.0*H;
        }

        m_cx=cx;
        m_cy=cy;

        m_cst=cos(deg2rad*th);
        m_snt=sin(deg2rad*th);

        double dX,dY;

        for (int i=1; i<=12; ++i)
        {
            dX=m_cx+m_cst*xm[i]-m_snt*ym[i];
            dY=m_cy+m_snt*xm[i]+m_cst*ym[i];

            if (dX<xMin) xMin=dX;
            if (dX>xMax) xMax=dX;
            if (dY<yMin) yMin=dY;
            if (dY>yMax) yMax=dY;
        }
    }

    ~Triangle()
    {
        if (Exponential)
        {
            delete [] Exponential;
            Exponential=0;
        }
    }

    void resize(int width,int height,int margin=20)
    {
        double scaleX=double(width- 2*margin)/(xMax-xMin);
        double scaleY=double(height-2*margin)/(yMax-yMin);

        double scale=scaleX<scaleY?scaleX:scaleY;

        m_Radius=2.0*scale;

        double xMid=0.5*(xMin+xMax);
        double yMid=0.5*(yMin+yMax);

        int w2=width/2;
        int h2=height/2;

        for (int i=1; i<=12; ++i)
        {
            x[i]=w2+int(scale*((m_cx+m_cst*xm[i]-m_snt*ym[i])-xMid));
            y[i]=h2+int(scale*((m_cy+m_snt*xm[i]+m_cst*ym[i])-yMid));
        }

        m_x0=w2+int(scale*((m_cx-15.0*m_cst+8.66*m_snt)-xMid));
        m_y0=h2+int(scale*((m_cy-15.0*m_snt-8.66*m_cst)-yMid));

        m_x1=w2+int(scale*((m_cx+15.0*m_cst+8.66*m_snt)-xMid));
        m_y1=h2+int(scale*((m_cy+15.0*m_snt-8.66*m_cst)-yMid));

        m_x2=w2+int(scale*((m_cx-17.32*m_snt)-xMid));
        m_y2=h2+int(scale*((m_cy+17.32*m_cst)-yMid));

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

        m_Xa=w2+int(scale*(m_cx-15.0-xMid))-maxRange;
        m_Xb=w2+int(scale*(m_cx+15.0-xMid))+maxRange;
        m_Ya=h2+int(scale*(m_cy-15.0-yMid))-maxRange;
        m_Yb=h2+int(scale*(m_cy+15.0-yMid))+maxRange;

        if (m_Xa<0) m_Xa=0;
        if (m_Xb>width) m_Xb=width;
        if (m_Ya<0) m_Ya=0;
        if (m_Yb>height) m_Yb=height;

        m_Width=width;
        m_Height=height;
    }

    int Abs(int x)
    {
        return x>=0?x:-x;
    }

    void eval(double *image)
    {
        int dx,dy;

        for (int Y=m_Ya; Y<m_Yb; ++Y)
        {        
            int Ybase=(m_Width-Y-1)*m_Width;

            for (int X=m_Xa; X<m_Xb; ++X)
            {
                double &value=image[X+Ybase];

                for (int i=1; i<=12; ++i)
                {
                    dx=Abs(X-x[i]);
                    if (dx>=m_maxRange) continue;
                    dy=Abs(Y-y[i]);
                    if (dy>=m_maxRange) continue;

                    value+=activation[i]*Exponential[dx]*Exponential[dy];
                }
            }
        }
    }

    void setActivationFirst7(unsigned char* data)
    {
        for (int i=1; i<=7; ++i)
        {
            activation[i]=double(255-data[i]);
        }
    }
    void setActivationLast5(unsigned char* data)
    {
        for (int i=1; i<=5; ++i)
        {
            activation[i+7]=double(255-data[i]);
        }
    }

    void draw(unsigned char *image)
    {
        drawLine(image,m_x0,m_y0,m_x1,m_y1);
        drawLine(image,m_x1,m_y1,m_x2,m_y2);
        drawLine(image,m_x2,m_y2,m_x0,m_y0);

        for (int i=1; i<=12; ++i)
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
        int bytePos=(x+(m_Width-y-2)*m_Width)*3;

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

    double xm[16],ym[16];
    int x[16],y[16];

    int m_Width,m_Height;

    int m_Xa,m_Xb,m_Ya,m_Yb;

    double m_cx,m_cy,m_cst,m_snt;
    double activation[16];

    int m_x0,m_y0,m_x1,m_y1,m_x2,m_y2;
    double m_Radius;

    static int m_maxRange;
    static double *Exponential;
    static double xMin,xMax,yMin,yMax;
};

#endif
