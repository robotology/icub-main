// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Author: Giorgio Metta
 * Copyright (C) 2009 The RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/**
 * @file ClientLogpolarFrameGrabber.cpp
 * @brief Implementation of the network wrapper device driver for the logpolar subsampling of images.
 *
 * @cond
 *
 * @author Giorgio Metta
 */

#include <stdio.h>
#include <memory.h>

#include <yarp/os/impl/String.h>
#include <yarp/os/impl/Logger.h>

#include <yarp/dev/ServerLogpolarFrameGrabber.h>
#include <yarp/dev/ClientLogpolarFrameGrabber.h>
#include <iCub/RC_DIST_FB_logpolar_mapper.h>
#include <yarp/dev/PolyDriver.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

//
// LATER: move this to a separate class, this code is at the moment duplicated
// in the ServerLogpolarFrameGrabber.cpp file (I'm ashamed of this :)).
// 

namespace yarp {
    namespace dev {
        class logPolarLibrary;
        static logPolarLibrary *lpLibrary = 0;
    }
}

/*
 * provide access to the logpolar library.
 * LATER: add a proper class directly in the library.
 */
class yarp::dev::logPolarLibrary : public ILogpolarAPI {
protected:
    cart2LpPixel *c2lTable;
    lp2CartPixel *l2cTable;
    int count;

public:
    logPolarLibrary() {
        c2lTable = 0;
        l2cTable = 0;
        count = 0;
    }

    virtual ~logPolarLibrary() {
        freeLookupTables();
    }

    virtual const bool allocated() const {
        if (c2lTable != 0 || l2cTable != 0)
            return true;
        else
            return false;
    }

    virtual int addInstance() {
        count ++;
        return count;
    }

    virtual int removeInstance() {
        count --;
        return count;
    }

    virtual bool allocLookupTables(int necc, int nang, int w, int h, double overlap) {
        if (c2lTable == 0) {
            c2lTable = new cart2LpPixel[necc*nang];
            if (c2lTable == 0) {
                fprintf(stderr, "logPolarLibrary: can't allocate c2l lookup tables, wrong size?\n");
                return false;
            }
        }

        if (l2cTable == 0) {
            l2cTable = new lp2CartPixel[w*h];
            if (l2cTable == 0) {
                fprintf(stderr, "logPolarLibrary: can't allocate l2c lookup tables, wrong size?\n");
                if (c2lTable) 
                    delete[] c2lTable;
                c2lTable = 0;
                return false;
            }
        }

        const double scaleFact = RCcomputeScaleFactor (necc, nang, w, h, overlap);    
        // LATER: remove the dependency on the file.
        // saves the table to file.
        RCbuildC2LMap (necc, nang, w, h, overlap, scaleFact, ELLIPTICAL, "./");
        // reloads the table from file. :(
        RCallocateC2LTable (c2lTable, necc, nang, 0, "./");

        // saves the table to file.
        RCbuildL2CMap (necc, nang, w, h, overlap, scaleFact, 0, 0, ELLIPTICAL, "./");
        RCallocateL2CTable (l2cTable, w, h, "./");

        return true;
    }

    virtual bool freeLookupTables() {
        if (c2lTable)
            RCdeAllocateC2LTable (c2lTable);
        c2lTable = 0;
        if (l2cTable)
            RCdeAllocateL2CTable (l2cTable);
        l2cTable = 0;
        return true;
    }

    virtual bool cartToLogpolar(yarp::sig::ImageOf<yarp::sig::PixelRgb>& lp, 
                                const yarp::sig::ImageOf<yarp::sig::PixelRgb>& cart) {
        // adjust padding.
        if (cart.getPadding() != 0) {
            int i;
            const int byte = cart.width() * sizeof(PixelRgb);
            unsigned char *d = cart.getRawImage();
            for (i = 1; i < cart.height(); i ++) {
                unsigned char *s = (unsigned char *)cart.getRow(i);
                memmove(d, s, byte);
                d += byte;
            }
        }

        // LATER: assert whether lp & cart are effectively nang * necc as the c2lTable requires.
        RCgetLpImg (lp.getRawImage(), (unsigned char *)cart.getRawImage(), c2lTable, lp.height()*lp.width(), 0);

        // adjust padding.
        if (lp.getPadding()) {
            const int byte = lp.width() * sizeof(PixelRgb);
            int i;
            for (i = lp.height()-1; i >= 1; i--) {
                unsigned char *d = lp.getRow(i);
                unsigned char *s = lp.getRawImage() + i*byte;
                memmove(d, s, byte);
            }
        }
        return true;
    }

    virtual bool logpolarToCart(yarp::sig::ImageOf<yarp::sig::PixelRgb>& cart,
                                const yarp::sig::ImageOf<yarp::sig::PixelRgb>& lp) {
        // adjust padding.
        if (lp.getPadding() != 0) {
            int i;
            const int byte = lp.width() * sizeof(PixelRgb);
            unsigned char *d = lp.getRawImage();
            for (i = 1; i < lp.height(); i ++) {
                unsigned char *s = (unsigned char *)lp.getRow(i);
                memmove(d, s, byte);
                d += byte;
            }
        }

        // LATER: assert whether lp & cart are effectively of the correct size.
        RCgetCartImg (cart.getRawImage(), lp.getRawImage(), l2cTable, cart.width() * cart.height());

        // adjust padding.
        if (cart.getPadding() != 0) {
            const int byte = cart.width() * sizeof(PixelRgb);
            int i;
            for (i = cart.height()-1; i >= 1; i--) {
                unsigned char *d = cart.getRow(i);
                unsigned char *s = cart.getRawImage() + i*byte;
                memmove(d, s, byte);
            }
        }
        return true;
    }
};

// logpolar details and maps, pointer to access the lp library.
#define HELPER(x) (*((logPolarLibrary*)(x)))
/*
 * implementation of the ClientLogpolarFrameGrabber class.
 */

ClientLogpolarFrameGrabber::ClientLogpolarFrameGrabber() : RemoteFrameGrabberDC1394(), nmutex(1) {}

bool ClientLogpolarFrameGrabber::open(yarp::os::Searchable& config) {
    nmutex.wait();
    bool ok = RemoteFrameGrabberDC1394::open(config);
    if (!ok) {
        fprintf(stderr, "ClientLogpolarFrameGrabber: can't open the dc1394 controls device\n");
        nmutex.post();
        return false;
    }

    ConstString loc = local + "/logpolar";
    portLogpolar.open(loc);
    ConstString loc2 = local + "/fovea";
    portFoveal.open(loc2);

    // don't connect if !remote.
    if (remote != "") {
        yarp::os::ConstString carrier = 
            config.check("stream",yarp::os::Value("tcp"),
                         "carrier to use for streaming").asString();
         ConstString rem = remote + "/logpolar";
         ConstString rem2 = remote + "/fovea";
         bool ok = false;
         ok = Network::connect(rem, loc, carrier);
         ok &= Network::connect(rem2, loc2, carrier);
         if (!ok) {
             fprintf(stderr, "ClientLogpolarFrameGrabber: can't connect to the server\n");
             portLogpolar.close();
             portFoveal.close();
             nmutex.post();
             return false;
         }
    }
    
    // attach the readers.
    readerLogpolar.attach(portLogpolar);
    readerFoveal.attach(portFoveal);

    // alloc memory and tables.
    // don't forget to alloc the tables later.
    if (lpLibrary == 0) {
        lpLibrary = new logPolarLibrary;
    }
    lpLibrary->addInstance();

    nmutex.post();
    return true;
}

bool ClientLogpolarFrameGrabber::close() {
    nmutex.wait();
    portLogpolar.close();
    portFoveal.close();
 
    if (lpLibrary->removeInstance() == 0) {
        lpLibrary->freeLookupTables();
        delete lpLibrary;
        lpLibrary = 0;
    }

    nmutex.post();
    return RemoteFrameGrabberDC1394::close();
}

/*
 * implement the ILogpolarAPI interface.
 */
bool ClientLogpolarFrameGrabber::allocLookupTables(int necc, int nang, int w, int h, double overlap) {
    nmutex.wait();
    const bool ok = HELPER(lpLibrary).allocLookupTables(necc, nang, w, h, overlap);
    nmutex.post();
    return ok;
}

bool ClientLogpolarFrameGrabber::freeLookupTables() {
    nmutex.wait();
    const bool ok = HELPER(lpLibrary).freeLookupTables();
    nmutex.post();
    return ok;
}

bool ClientLogpolarFrameGrabber::cartToLogpolar(ImageOf<PixelRgb>& lp, const ImageOf<PixelRgb>& cart) {
    nmutex.wait();
    const bool ok = HELPER(lpLibrary).cartToLogpolar(lp, cart);
    nmutex.post();
    return ok;
}

bool ClientLogpolarFrameGrabber::logpolarToCart(ImageOf<PixelRgb>& cart, const ImageOf<PixelRgb>& lp) {
    nmutex.wait();
    const bool ok = HELPER(lpLibrary).logpolarToCart(cart, lp);
    nmutex.post();
    return ok;
}

/**
 * @endcond
 */

