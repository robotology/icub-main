/*
* Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
* Author: Marco Randazzo <marco.randazzo@iit.it>
* CopyPolicy: Released under the terms of the GPLv2 or later, see GPL.TXT
*/

#include <yarp/sig/Image.h>
#include <yarp/os/all.h>

#include <iostream>
#include <math.h>

using namespace yarp::os;
using namespace yarp::sig;
void printFrame(int h, int w, int c);

void merge(const ImageOf<PixelRgb> &imgR, const ImageOf<PixelRgb> &imgL, ImageOf<PixelRgb> &out, size_t start_lx, size_t start_ly, size_t start_rx, size_t start_ry, double alpha1,double alpha2)
{
    size_t max_w = (imgR.width() > imgL.width()) ? imgR.width() : imgL.width();
    size_t max_h = (imgR.height() > imgL.height()) ? imgR.height() : imgL.height();
    if (out.width()  != max_w || out.height() != max_h)  out.resize(max_w, max_h);

    start_lx = (start_lx < max_w) ? start_lx : max_w;
    start_ly = (start_ly < max_h) ? start_ly : max_h;
    start_rx = (start_rx < max_w) ? start_rx : max_w;
    start_ry = (start_ry < max_h) ? start_ry : max_h;
    size_t end_lx = (start_lx + imgL.width() < max_w) ? (start_lx + imgL.width()) : max_w;
    size_t end_ly = (start_ly + imgL.height() < max_h) ? (start_ly + imgL.height()) : max_h;
    size_t end_rx = (start_rx + imgR.width() < max_w) ? (start_rx + imgR.width()) : max_w;
    size_t end_ry = (start_ry + imgR.height() < max_h) ? (start_ry + imgR.height()) : max_h;

    //canvas
    for (size_t r_dst = 0; r_dst < max_h; r_dst++)
    {
        unsigned char       *tmp_dst = out.getRow(r_dst);

        for (size_t c_dst = 0; c_dst < max_w; c_dst++)
        {
            tmp_dst[0 + c_dst * 3] = 0;
            tmp_dst[1 + c_dst * 3] = 0;
            tmp_dst[2 + c_dst * 3] = 0;
        }
    }

    //left image
    for (size_t r_dst = start_ly, r_src = 0; r_dst < end_ly; r_dst++, r_src++)
    {
        unsigned char       *tmp_dst = out.getRow(r_dst);
        const unsigned char *tmp_src = imgL.getRow(r_src);

        for (size_t c_dst = start_lx, c_src = 0; c_dst < end_lx; c_dst++, c_src++)
        {
            tmp_dst[0 + c_dst * 3] = tmp_dst[0 + c_dst * 3] + (unsigned char)(double(tmp_src[0 + c_src * 3]) * alpha1);
            tmp_dst[1 + c_dst * 3] = tmp_dst[1 + c_dst * 3] + (unsigned char)(double(tmp_src[1 + c_src * 3]) * alpha1);
            tmp_dst[2 + c_dst * 3] = tmp_dst[2 + c_dst * 3] + (unsigned char)(double(tmp_src[2 + c_src * 3]) * alpha1);
        }
    }
    
    //right image
    for (size_t r_dst = start_ry, r_src = 0; r_dst < end_ry; r_dst++, r_src++)
    {
        unsigned char       *tmp_dst = out.getRow(r_dst);
        const unsigned char *tmp_src = imgR.getRow(r_src);

        for (size_t c_dst = start_rx, c_src = 0; c_dst < end_rx; c_dst++, c_src++)
        {
            tmp_dst[0 + c_dst * 3] = tmp_dst[0 + c_dst * 3] + (unsigned char)(double(tmp_src[0 + c_src * 3]) * alpha2);
            tmp_dst[1 + c_dst * 3] = tmp_dst[1 + c_dst * 3] + (unsigned char)(double(tmp_src[1 + c_src * 3]) * alpha2);
            tmp_dst[2 + c_dst * 3] = tmp_dst[2 + c_dst * 3] + (unsigned char)(double(tmp_src[2 + c_src * 3]) * alpha2);
        }
    }
}

int main(int argc, char *argv[])
{
    Network yarp;
    int c=0;
    ResourceFinder rf;
    rf.configure(argc, argv);
    
    BufferedPort<ImageOf<PixelRgb> > right;
    BufferedPort<ImageOf<PixelRgb> > left;
    BufferedPort<ImageOf<PixelRgb> > out;

    right.open("/imageBlending/right");
    left.open("/imageBlending/left");

    out.open("/imageBlending/out");

    size_t start_rx = 0;
    size_t start_ry = 0;
    size_t start_lx = 0;
    size_t start_ly = 0;
    double alpha1 = 0.5;
    double alpha2 = 0.5;
    if (rf.check("rx")) start_rx = (size_t) rf.find("rx").asDouble();
    if (rf.check("ry")) start_ry = (size_t) rf.find("ry").asDouble();
    if (rf.check("lx")) start_lx = (size_t) rf.find("lx").asDouble();
    if (rf.check("ly")) start_ly = (size_t) rf.find("ly").asDouble();
    if (rf.check("alpha1")) alpha1 = rf.find("alpha1").asDouble();
    if (rf.check("alpha2")) alpha2 = rf.find("alpha2").asDouble();
    yDebug("left offset:%lu,%lu right offset:%lu,%lu, alpha1:%f, alpha2:%f", start_lx, start_ly, start_rx, start_ry, alpha1, alpha2);
    if (rf.check("help"))
    {
        yDebug() << "Available options:";
        yDebug() << "rx";
        yDebug() << "ry";
        yDebug() << "lx";
        yDebug() << "ly";
        yDebug() << "alpha1";
        yDebug() << "alpha2";
        return 0;
    }

    while(true)
    {
        ImageOf< PixelRgb> *imgR=right.read(true);
        ImageOf< PixelRgb> *imgL=left.read(true);

        ImageOf< PixelRgb> &outImg=out.prepare();

        if (imgR!=0 && imgL!=0)
        {
            merge(*imgR, *imgL, outImg, start_lx, start_ly, start_rx, start_ry,alpha1,alpha2);

            out.write();
            c++;
        }
        printFrame(outImg.height(), outImg.width(), c);
    }
    return 0;
}

void printFrame(int h, int w, int c)
{   
    if (c%500 == 0)
        printf("Frame #%d %dx%d \n", c, h, w);
}
