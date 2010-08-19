// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Author: Giorgio Metta
 * Copyright (C) 2009 The RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/**
 * @file ClientLogpolarFrameGrabber.h
 * @brief A client device driver that connects to the corresponding ServerLogpolarFrameGrabber exporing the IFrameGrabberImage 
 * interface with the ability of obtaining standard, logpolar and foveal images.
 *
 */


#ifndef _YARP2_CLIENTLOGPOLARFRAMEGRABBER_
#define _YARP2_CLIENTLOGPOLARFRAMEGRABBER_

#include <yarp/dev/ServerFrameGrabber.h>
#include <yarp/dev/RemoteFrameGrabber.h>
#include <iCub/LogpolarInterfaces.h>
#include <yarp/os/Network.h>
#include <yarp/os/Semaphore.h>

#include <iCub/RC_DIST_FB_logpolar_mapper.h>

namespace yarp{
    namespace dev {
        class ClientLogpolarFrameGrabber;
    }
}

#define NSEMA ((yarp::os::Semaphore&)nmutex)

/**
 * @ingroup logpolar
 *
 * Connect to a ServerLogpolarFrameGrabber.  See ServerLogpolarFrameGrabber for
 * details of the network protocol in use. This class defines and implements the client side of a 
 * frame grabber in logpolar image format. This is a the client side of a network wrapper of any device 
 * providing the IFrameGrabberImage and IFrameGrabberControls interfaces.
 *
 */
class yarp::dev::ClientLogpolarFrameGrabber : 
                                public yarp::dev::ILogpolarFrameGrabberImage,
                                public yarp::dev::RemoteFrameGrabberDC1394 {
private:
    ClientLogpolarFrameGrabber(const ClientLogpolarFrameGrabber&);
    void operator=(const ClientLogpolarFrameGrabber&);

protected:
    yarp::os::Port portLogpolar;
    yarp::os::Port portFoveal;
    yarp::os::PortReaderBuffer<yarp::sig::ImageOf<yarp::sig::PixelRgb> > readerLogpolar;
    yarp::os::PortReaderBuffer<yarp::sig::ImageOf<yarp::sig::PixelRgb> > readerFoveal;
    yarp::os::Semaphore nmutex;

public:
    /**
     * Constructor.
     */
    ClientLogpolarFrameGrabber();

    /**
     * Destructor.
     */
    virtual ~ClientLogpolarFrameGrabber() {
        close();
    }

    /**
     * Configure with a set of options. These are:
     * <TABLE>
     * <TR><TD> local </TD><TD> Port name of this client. </TD></TR>
     * <TR><TD> remote </TD><TD> Port name of server to connect to. </TD></TR>
     * <TR><TD> stream </TD><TD> Carrier for the stream connection. </TD></TR>
     * </TABLE>
     *
     * @param config The options to use
     * @return true iff the object could be configured.
     */
    virtual bool open(yarp::os::Searchable& config);

    /**
     * Close the device driver.
     * @return true iff the operation is successful.
     */
    virtual bool close();

    // implement ILogpolarFrameGrabberImage.
    /**
     * Get the number of eccentricities.
     * @return the number of eccentricities of the logpolar image.
     */
    virtual int necc(void) const {
        NSEMA.wait();
        const int i = (int)getCommand(VOCAB_NECC);
        NSEMA.post();
        return i;
    }

    /**
     * Get the number of angles.
     * @return the number of angles of the logpolar image.
     */
    virtual int nang(void) const {
        NSEMA.wait();
        const int i = (int)getCommand(VOCAB_NANG);
        NSEMA.post();
        return i;
    }

    /**
     * Get the fovea size (in pixels).
     * @return the size of the foveal image (square).
     */
    virtual int fovea(void) const {
        NSEMA.wait();
        const int i = (int)getCommand(VOCAB_FOVEA);
        NSEMA.post();
        return i;
    }

    /**
     * Get the overlap between receptive fields.
     * @return the size of the overlap (double).
     */
    virtual double overlap(void) const {
        NSEMA.wait();
        const double x = getCommand(VOCAB_OVERLAP);
        NSEMA.post();
        return x;
    }

    /**
     * Get the logpolar image.
     * @param image is the RGB image containing the logpolar subsampled image.
     * @return true iff successful.
     */
    virtual bool getLogpolarImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image) {
        nmutex.wait();
        if (readerLogpolar.read(true) != NULL) {
            image = *(readerLogpolar.lastRead());
            nmutex.post();
            return true;
        }
        nmutex.post();
        return false;
    }

    /**
     * Get the foveal image (a small part of the centre of the image in full resolution).
     * @param image is the RGB image containing the foveal image.
     * @return true iff successful.
     */
    virtual bool getFovealImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image) {
        nmutex.wait();
        if (readerFoveal.read(true) != NULL) {
            image = *(readerFoveal.lastRead());
            nmutex.post();
            return true;
        }
        nmutex.post();
        return false;
    }
};

#undef NSEMA

#endif /* _YARP2_CLIENTLOGPOLARFRAMEGRABBER_ */


