// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Giorgio Metta
 * email: giorgio.metta@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 * @file LogpolarInterfaces.h
 * @brief A collection of interfaces that provide logpolar image sampling to iCub devices.
 */

/**
 * \defgroup icub_icubDev iCubDev
 * \ingroup icub_libraries
 *
 * A library that collects device interfaces. This is similar to the 
 * libYARP_dev in YARP. To be populated.
 *
 * Author: Giorgio Metta
 * Copyright (C) 2009 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
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
    }
}

// additional vocabs defined for the ILogpolarFrameGrabberImage interface.
#define VOCAB_NECC VOCAB4('n','e','c','c')
#define VOCAB_NANG VOCAB4('n','a','n','g')
#define VOCAB_FOVEA VOCAB3('f','o','v')
#define VOCAB_OVERLAP VOCAB3('o','v','l')

/**
 * \ingroup icub_icubDev
 *
 * A logpolar grabber interface; it behaves like the standard grabber but with specialization for logpolar images.
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

#endif /* __LOGPOLARINTERFACES__ */
