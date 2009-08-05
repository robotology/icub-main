// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Author: Giorgio Metta
 * Copyright (C) 2009 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __LOGPOLARINTERFACES__
#define __LOGPOLARINTERFACES__

/* dev drivers */
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/dev/ServerFrameGrabber.h>

/* LATER: is it likely that some of these would move into iCub::dev namespace? */
namespace yarp{
    namespace dev {
        class ILogpolarFrameGrabberImage;
        class ILogpolarAPI;
    }
}

// additional vocabs defined for the ILogpolarFrameGrabberImage interface.
#define VOCAB_NECC VOCAB4('n','e','c','c')
#define VOCAB_NANG VOCAB4('n','a','n','g')
#define VOCAB_FOVEA VOCAB3('f','o','v')
#define VOCAB_OVERLAP VOCAB3('o','v','l')

/*
 * A logpolar frame grabber interfaces (specialized for the logpolar images).
 */
class yarp::dev::ILogpolarFrameGrabberImage {
public:
    virtual ~ILogpolarFrameGrabberImage() {}

    /**
     * get the number of eccentricities.
     * @return the number of eccentricities of the logpolar image.
     */
    virtual int necc(void) const = 0;

    /**
     * get the number of angles.
     * @return the number of angles of the logpolar image.
     */
    virtual int nang(void) const = 0;

    /**
     * get the overlap between receptive fields.
     * @return the size of the overlap (double).
     */
    virtual double overlap(void) const = 0;

    /**
     * get the fovea size (in pixels).
     * @return the size of the foveal image (square).
     */
    virtual int fovea(void) const = 0;

    /**
     * get the logpolar image.
     * @param image is the RGB image containing the logpolar subsampled image.
     * @return true iff successful.
     */
    virtual bool getLogpolarImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image) = 0;

    /**
     * get the foveal image (a small part of the centre of the image in full resolution).
     * @param image is the RGB image containing the foveal image.
     * @return true iff successful.
     */
    virtual bool getFovealImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image) = 0;
};

/**
 * interface to the logpolar library/API.
 */
class yarp::dev::ILogpolarAPI {
public:
    virtual ~ILogpolarAPI() {}

    /**
     * alloc the lookup tables and stores them in memory.
     * @param necc is the number of eccentricities of the logpolar image.
     * @param nang is the number of angles of the logpolar image.
     * @param w is the width of the original rectangular image.
     * @param h is the height of the original rectangular image.
     * @param overlap is the degree of overlap of the receptive fields (>0.).
     * @return true iff successful.
     */
    virtual bool allocLookupTables (int necc, int nang, int w, int h, double overlap) = 0;

    /**
     * free the lookup tables from memory.
     * @return true iff successful.
     */
    virtual bool freeLookupTables () = 0;

    /**
     * converts an image from rectangular to logpolar.
     * @param lp is the logpolar image (destination).
     * @param cart is the cartesian image (source data).
     * @return true iff successful. Beware that tables must be
     * allocated in advance.
     */
    virtual bool cartToLogpolar(yarp::sig::ImageOf<yarp::sig::PixelRgb>& lp, 
                                const yarp::sig::ImageOf<yarp::sig::PixelRgb>& cart) = 0;

    /**
     * converts an image from logpolar to cartesian (rectangular).
     * @param cart is the cartesian image (destination).
     * @param lp is the logpolar image (source).
     * @return true iff successful. Beware that tables must be
     * allocated in advance.
     */
    virtual bool logpolarToCart(yarp::sig::ImageOf<yarp::sig::PixelRgb>& cart,
                                const yarp::sig::ImageOf<yarp::sig::PixelRgb>& lp) = 0;
};


#endif /* __LOGPOLARINTERFACES__ */
