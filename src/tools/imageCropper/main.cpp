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

void crop(const ImageOf<PixelRgb> &inImg, ImageOf<PixelRgb> &outImg, size_t start_x, size_t start_y, size_t width, size_t height)
{
    size_t end_x = (start_x + width)  < size_t(inImg.width()) ? (start_x + width) : (inImg.width());
    size_t end_y = (start_y + height) < size_t(inImg.height()) ? (start_y + height) : (inImg.height());
    outImg.resize(end_x-start_x, end_y-start_y);

    for (size_t r_src = start_y, r_dst = 0; r_src < end_y; r_dst++, r_src++)
    {
        unsigned char       *tmp_dst = outImg.getRow(r_dst);
        const unsigned char *tmp_src = inImg.getRow(r_src);

        for (size_t c_src = start_x, c_dst = 0; c_src < end_x; c_dst++, c_src++)
        {
            tmp_dst[0 + c_dst * 3] = tmp_src[0 + c_src * 3];
            tmp_dst[1 + c_dst * 3] = tmp_src[1 + c_src * 3];
            tmp_dst[2 + c_dst * 3] = tmp_src[2 + c_src * 3];
        }
    }
}

int main(int argc, char *argv[])
{
    Network yarp;
    int c=0;
    ResourceFinder rf;
    rf.configure(argc, argv);
    
    BufferedPort<ImageOf<PixelRgb> > input;
    BufferedPort<ImageOf<PixelRgb> > out;

    input.open("/imageCropper/in");
    out.open("/imageCropper/out");

    size_t start_x = 0;
    size_t start_y = 0;
    size_t width = 0;
    size_t height = 0;
    if (rf.check("x_off"))  start_x = (size_t) rf.find("x_off").asDouble();
    if (rf.check("y_off"))  start_y = (size_t) rf.find("y_off").asDouble();
    if (rf.check("width"))  width    = (size_t)rf.find("width").asDouble();
    if (rf.check("height")) height  = (size_t)rf.find("height").asDouble();
    yDebug("x_off %lu, y_off %lu, width %lu, height %lu", start_x, start_y, width, height);
    if (rf.check("help"))
    {
        yDebug() << "Available options:";
        yDebug() << "x_off";
        yDebug() << "y_off";
        yDebug() << "width";
        yDebug() << "height";
        return 0;
    }

    while(true)
    {
        ImageOf< PixelRgb> *inImg = input.read(true);
        ImageOf< PixelRgb> &outImg=out.prepare();

        if (inImg != 0)
        {
            crop(*inImg, outImg, start_x, start_y, width, height);
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
