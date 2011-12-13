/**
*
@ingroup icub_tools
\defgroup icub_testStereoMatch testStereoMatch

Test how much left and right images are aligned.

\section intro_sec Description
Superimpose images from the left and right cameras. The two streams
go to different color channels (left image to red and right image to green), 
when images matches perfectly you see a yellow image.

\section lib_sec Libraries
YARP libs.

\section parameters_sec Parameters
No parameters.

\section portsa_sec Ports Accessed
The module does not assume ports are already created. You have to connect 
them manually. To properly work you will need two camera streams (e.g. 
/icub/cam/left or /icub/cam/right).

\section portsc_sec Ports Created
The module creates two ports to receive images, and one port to produce the merged
result.

Output ports:
- /teststereomatch/o: a color image which contains the result of the merge

Input ports:
- /teststereomatch/left: right image
- /teststereomatch/right: left image

\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module

Just run:

\code
testStereoMatch
yarpview --name /match/view
\endcode

and connect ports:

\code
yarp connect /icub/cam/right /teststereomatch/right
yarp connect /icub/cam/left /teststereomatch/left
yarp connect /teststereomatch/o /match/view
\endcode

\author Lorenzo Natale

Copyright (C) 2008 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/testStereoMatch/main.cpp
**/

#include <yarp/sig/Image.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <iostream>
#include <math.h>

using namespace yarp::os;
using namespace yarp::sig;

const int scale = 1;
void printFrame(int h, int w, int c);

void merge(const ImageOf<PixelRgb> &imgR, const ImageOf<PixelRgb> &imgL, ImageOf<PixelRgb> &out)
{
    if (out.width()!=imgR.width()/scale)
        out.resize(imgR.width()/scale, imgR.height()/scale);

    int rr=0;
    for(int r=0; r<out.height(); r++)
    {
        const unsigned char *tmpR=imgR.getRow(rr);
        const unsigned char *tmpL=imgL.getRow(rr);
        unsigned char *tmpO=out.getRow(r);

        for(int c=0; c<out.width(); c++)
        {
            tmpO[0]=(unsigned char) (1/3.0*(tmpL[0]+tmpL[1]+tmpL[2]));
            tmpO[1]=(unsigned char) (1/3.0*(tmpR[0]+tmpR[1]+tmpR[2]));

            tmpO+=3;
            tmpL+=(3*scale);
            tmpR+=(3*scale);
        }

        rr+=scale;
    }
}

int main(int argc, char *argv[])
{
    Network yarp;
    int c=0;

    BufferedPort<ImageOf<PixelRgb> > right;
    BufferedPort<ImageOf<PixelRgb> > left;

    BufferedPort<ImageOf<PixelRgb> > out;

    right.open("/teststereomatch/right");
    left.open("/teststereomatch/left");

    out.open("/teststereomatch/o");

    while(true)
    {
        ImageOf< PixelRgb> *imgR=right.read(true);
        ImageOf< PixelRgb> *imgL=left.read(true);

        ImageOf< PixelRgb> &outImg=out.prepare();

        if (imgR!=0 && imgL!=0)
                {
                    merge(*imgR, *imgL, outImg);

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
