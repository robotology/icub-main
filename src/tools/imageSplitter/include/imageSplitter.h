// Copyright (C) 2016 Istituto Italiano di Tecnologia - iCub Facility
// Author: Alberto Cardellino <alberto.cardellino@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <iostream>
#include <string>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/sig/Image.h>
#include <yarp/os/BufferedPort.h>

#ifndef ICUB_TOOLS_IMAGE_SPLITTER_H
#define ICUB_TOOLS_IMAGE_SPLITTER_H


class ImageSplitter: public yarp::os::RFModule
{
private:
    int method;  // tmp, just for testing different methods of filling the output images
    bool horizontal;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outLeftPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outRightPort;

    int inWidth, inHeight;
    int outWidth, outHeight;

public:
    ImageSplitter();
    ~ImageSplitter();
    bool configure(yarp::os::ResourceFinder &rf);
    bool interruptModule();                       
    bool close();                                 
    bool updateModule();
    double getPeriod(); 
};

#endif  // ICUB_TOOLS_IMAGE_SPLITTER_H
