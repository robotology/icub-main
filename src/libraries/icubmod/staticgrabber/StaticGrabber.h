// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */
 
#ifndef __ICUB_STATICGRABBER__
#define __ICUB_STATICGRABBER__

namespace yarp {
    namespace dev
    {
        class StaticGrabber;
    }
}

// yarp
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/dev/Drivers.h>

using namespace yarp::os;

/**
 * \file StaticGrabber
 * A simple image file grabber for testing purposes. The device grabs the *.ppm image specified by the --filename option repeatedly.\n
 * (A file grabber for a series of files can be found in yarp/examples/dev)
 */

class yarp::dev::StaticGrabber : 
                        public yarp::dev::IFrameGrabber,
                        public yarp::dev::IFrameGrabberImage, 
                        public yarp::dev::DeviceDriver {
private:

    yarp::os::ConstString filename;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> img;
    yarp::sig::ImageOf<yarp::sig::PixelMono> bayer;
    
    bool makeSimpleBayer(yarp::sig::ImageOf<yarp::sig::PixelRgb>& src, yarp::sig::ImageOf<yarp::sig::PixelMono>& bayer);

public:
    StaticGrabber() {
    }

    virtual bool open(yarp::os::Searchable& config) { 
        bool ok = true;
        filename = config.check("filename",
                                Value(""),
                                "The *.ppm file to grab (string).").asString();
        ok = yarp::sig::file::read(img, filename);
        if (ok) {
            makeSimpleBayer(img, bayer);
        }
        return ok;
    }

    virtual bool close();

    virtual bool getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image) {
        yarp::os::Time::delay(0.002);  // simulate waiting for hardware to report
        image.copy(img);
        return true;
    }

    virtual int height() const {
        return img.height();
    }

    virtual int width() const {
        return img.width();
    }

    // implement IFrameGrabber interface

    /**
     * Get the raw buffer from the frame grabber. The driver returns 
     * a copy of the internal memory buffer acquired by the frame grabber, no
     * post processing is applied (e.g. no color reconstruction/demosaicking).
     * The user must allocate the buffer; the size of the buffer, in bytes, 
     * is determined by calling getRawBufferSize().
     * @param buffer: pointer to the buffer to be filled (must be previously allocated)
     * @return true/false upon success/failure
     */
    virtual bool getRawBuffer(unsigned char *buffer);

    /**
     * Get the size of the card's internal buffer, the user should use this 
     * method to allocate the storage to contain a raw frame (getRawBuffer).
     * @return the size of the internal buffer, in bytes.
     **/
    virtual int getRawBufferSize() {
        return bayer.getRawImageSize();
    }
};

#endif
