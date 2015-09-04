/**
* Copyright: (C) 2009 RobotCub Consortium
* Authors: Matteo Taiana, Ugo Pattacini
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef _PF3DTRACKERSUPPORT_
#define _PF3DTRACKERSUPPORT_

#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <yarp/os/LogStream.h>

#ifdef _CH_
#pragma package <opencv>
#endif
#ifndef _EiC
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

struct Lut
{
    int y;
    int u;
    int v;
};

void rgbToYuvBin(int &R, int &G, int &B, int &YBin, int &UBin, int &VBin);

void rgbToYuvBinImage(IplImage *image,IplImage *yuvBinsImage);

void rgbToYuvBinImageLut(IplImage *image,IplImage *yuvBinsImage, Lut *lut);

void setPixel(int u, int v, int r, int g, int b, IplImage *image);

void fillLut(Lut *lut);

#endif /* _PF3DTRACKERSUPPORT_ */
