/**
* 
* Support Library of the 3d position tracker implementing the particle filter.
* See \ref icub_pf3dtracker \endref
*
* Copyright (C) 2009 RobotCub Consortium
*
* Author: Matteo Taiana
*
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#include <iostream>

#include <iCub/pf3dTrackerSupport.hpp>

using namespace std;

void rgbToYuvBin(int &R, int &G, int &B, int &YBin, int &UBin, int &VBin)
{
    //I copied the transformation from the wikipedia. WARNING ??? !!!
    float Y, U, V;
    Y=  0.299F  *(float)R +0.587F * (float)G + 0.114F *(float)B;
    U= -0.147F*float(R) -0.289F*float(G) +0.436F*float(B); //-255*436<U<255*436
    U+=0.436F*255.0F; //0<U<255*2*0.436
    U=U/(2.0F*0.436F);//0<U<255
    V= ((0.615F*float(R) -0.515F*float(G)+0.100F*float(B)+0.615F*255.0F)/(2.0F*0.615F+0.1F));    
    
    YBin=(int)Y/ 64; //I want Y to vary between 0 and 3.
    UBin=(int)U/ 32; //I want U to vary between 0 and 7.
    VBin=(int)V/ 32; //I want V to vary between 0 and 7.
    
    if(YBin<0||YBin>3)
        yWarning()<<"something's wrong with Y: "<<YBin<<" "<<Y;
    if(UBin<0||UBin>7)
        yWarning()<<"something's wrong with U: "<<UBin<<" "<<U;
    if(VBin<0||VBin>7)
        yWarning()<<"something's wrong with V: "<<VBin<<" "<<V<<" R= "<<R<<" G= "<<G<<" B= "<<B;
}

void rgbToYuvBinImage(IplImage *image,IplImage* transformedImage)
{
    int a1,a2,r,g,b, s,t,u;
    for(a1=0;a1<transformedImage->width;a1++)
        for(a2=0;a2<transformedImage->height;a2++)
    {
        r=(((uchar*)(image->imageData + image->widthStep*a2))[a1*3+0]);
        g=(((uchar*)(image->imageData + image->widthStep*a2))[a1*3+1]);
        b=(((uchar*)(image->imageData + image->widthStep*a2))[a1*3+2]);
        rgbToYuvBin(r,g,b, s,t,u);
        (((uchar*)(transformedImage->imageData + transformedImage->widthStep*a2))[a1*3+0])=s;
        (((uchar*)(transformedImage->imageData + transformedImage->widthStep*a2))[a1*3+1])=t;
        (((uchar*)(transformedImage->imageData + transformedImage->widthStep*a2))[a1*3+2])=u;
        //yuvBinsImage[a1][a2][0], yuvBinsImage[a1][a2][1], yuvBinsImage[a1][a2][2]);
    }
}

void setPixel(int u, int v, int r, int g, int b, IplImage *image)
{
    //std::cout<<"u= "<<u<<"  v= "<<v<<std::endl;
    if(u>-1&&u<image->width && v>-1&&v<image->height)
    {
        (((uchar*)(image->imageData + image->widthStep*v))[u*3+0])=r;
        (((uchar*)(image->imageData + image->widthStep*v))[u*3+1])=g;
        (((uchar*)(image->imageData + image->widthStep*v))[u*3+2])=b;
    }
}      

void fillLut(Lut *lut)
{
    int r,g,b, y,u,v;
    int index;
    for(r=0;r<256;r++)
        for(g=0;g<256;g++)
            for(b=0;b<256;b++)
            {
                rgbToYuvBin(r,g,b, y,u,v);
                index=r*65536+g*256+b;
                lut[index].y=y;
                lut[index].u=u;
                lut[index].v=v;
            }
}

void rgbToYuvBinImageLut(IplImage *image,IplImage *transformedImage, Lut *lut)
{
    int a1,a2,r,g,b;
    int index;

    for(a1=0;a1<image->width;a1++)
        for(a2=0;a2<image->height;a2++)
    {
        r=(((uchar*)(image->imageData + image->widthStep*a2))[a1*3+0]);
        g=(((uchar*)(image->imageData + image->widthStep*a2))[a1*3+1]);
        b=(((uchar*)(image->imageData + image->widthStep*a2))[a1*3+2]);
        index=r*65536+g*256+b;
        (((uchar*)(transformedImage->imageData + transformedImage->widthStep*a2))[a1*3+0])=lut[index].y;
        (((uchar*)(transformedImage->imageData + transformedImage->widthStep*a2))[a1*3+1])=lut[index].u;
        (((uchar*)(transformedImage->imageData + transformedImage->widthStep*a2))[a1*3+2])=lut[index].v;
        //rgbToYuvBin(r,g,b, yuvBinsImage[a1][a2][0], yuvBinsImage[a1][a2][1], yuvBinsImage[a1][a2][2]);
    }

}

void rgbToYuvBinLut(int &R, int &G, int &B, int &YBin, int &UBin, int &VBin, Lut *lut)
{
    //I copied the transformation from the wikipedia. WARNING ??? !!!
    float Y, U, V;
    Y=  0.299F  *(float)R +0.587F * (float)G + 0.114F *(float)B;
    U= -0.147F*float(R) -0.289F*float(G) +0.436F*float(B); //-255*436<U<255*436
    U+=0.436F*255.0F; //0<U<255*2*0.436
    U=U/(2.0F*0.436F);//0<U<255
    V= ((0.615F*float(R) -0.515F*float(G)+0.100F*float(B)+0.615F*255.0F)/(2.0F*0.615F+0.1F));

    YBin=(int)Y/ 64; //I want Y to vary between 0 and 3.
    UBin=(int)U/ 32; //I want U to vary between 0 and 7.
    VBin=(int)V/ 32; //I want V to vary between 0 and 7.

    if(YBin<0||YBin>3)
        yWarning()<<"something's wrong with Y: "<<YBin<<" "<<Y;
    if(UBin<0||UBin>7)
        yWarning()<<"something's wrong with U: "<<UBin<<" "<<U;
    if(VBin<0||VBin>7)
        yWarning()<<"something's wrong with V: "<<VBin<<" "<<V<<" R= "<<R<<" G= "<<G<<" B= "<<B;
}
