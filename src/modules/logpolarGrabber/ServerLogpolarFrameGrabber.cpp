// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Author: Giorgio Metta
 * Copyright (C) 2009 The RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/**
 * @file ServerLogpolarFrameGrabber.cpp
 * @brief Implementation of the network wrapper device driver for the logpolar subsampling of images.
 *
 * @cond
 *
 * @author Giorgio Metta
 */

#include <stdio.h>
#include <memory.h>

#include <string>

#include <yarp/os/impl/Logger.h>

#include <yarp/dev/ServerLogpolarFrameGrabber.h>
#include <iCub/RC_DIST_FB_logpolar_mapper.h>
#include <yarp/dev/PolyDriver.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

/* 
 * acquisition size, fixed to the reasonable maximum available at the moment. 
 */
const int baseWidth = 640;
const int baseHeight = 480;

/* 
 * size of the logpolar image, resembling one of the cmos sensors developed
 * at LIRA-Lab. These are not meant to be parameters of the driver although the code
 * is pretty generic anyway.
 */
const int nEcc = 152;   // number of rows (multiple of 6 for the foveal arrangement).
const int nAng = 252;   // number of columns (same as above).
const int nFovea = 128; // size of the fovea.
const double baseOverlap = 1.0;

namespace yarp {
    namespace dev {
        class logPolarLibrary;
        static logPolarLibrary *lpLibrary = 0;
    }
}

/*
 * provide access to the logpolar library.
 * LATER: add a proper class directly in the library.
 * LATER: make this thread safe.
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
            const int byte = cart.width() * sizeof(PixelRgb);
            unsigned char *d = (unsigned char *)cart.getRawImage() + byte;
            int i;
            for (i = 1; i < cart.height(); i++) {
                unsigned char *s = (unsigned char *)cart.getRow(i);
                memmove(d, s, byte);
                d += byte; 
            }
        }

        // LATER: assert whether lp & cart are effectively nang * necc as the c2lTable requires.
        RCgetLpImg (lp.getRawImage(), (unsigned char *)cart.getRawImage(), c2lTable, lp.height()*lp.width(), 0);

        // adjust padding.
        if (lp.getPadding() != 0) {
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
            unsigned char *d = lp.getRawImage() + byte;
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
 * Constructor. 
 */ 
ServerLogpolarFrameGrabber::ServerLogpolarFrameGrabber() : RateThread(0), mutex(1) {
    fgImage = 0;
    fgCtrl = 0;
	fgTimed = 0;

    canDrop = false;
    addStamp = true;
    active = false;

    cwidth = 0;
    cheight = 0;

    inang = 0;
    inecc = 0;
    ifovea = 0;
    ioverlap = 0.;
}

/* 
 * Open the device with parameters in config.
 */
bool ServerLogpolarFrameGrabber::open(yarp::os::Searchable& config) {
    mutex.wait();
    if (active) {
        fprintf(stderr, "ServerLogpolarFrameGrabber: Did you just try to open the same ServerFrameGrabber twice?\n");
        mutex.post();
        return false;
    }

    // LATER: the responder works with the fstd port (remember this for the client side!)
    fstd.setReader(*this);

    // stores the size of the requested cartesian output.
    cwidth = config.check("width", Value(baseWidth), "the width of the output rectangular image").asInt();
    cheight = config.check("height", Value(baseHeight), "the height of the output rectangular image").asInt();

    // size of the logp output. Use the member var from now on (LATER: make these driver params).
    inecc = nEcc;
    inang = nAng;
    ifovea = nFovea;
    ioverlap = baseOverlap;

    yarp::os::Value *name;

    if (config.check("subdevice", name, "name (or nested configuration) of device to wrap")) {
        if (name->isString()) {
            // maybe user isn't doing nested configuration

            // LATER: I need also to hack the subdevice config to guarantee that I can acquire 
            // at max resolution for logpolar subsampling.
            yarp::os::Property p;
            p.setMonitor(config.getMonitor(),
                         name->toString().c_str()); // pass on any monitoring
            p.fromString(config.toString());
            p.put("device", name->toString());
            // forces the subdevice to go maximum resolution (assuming this is the max!).
            p.put("width", baseWidth);
            p.put("height", baseHeight);
            p.unput("subdevice");
            poly.open(p);
        } else {
            Bottle subdevice = config.findGroup("subdevice").tail();
            poly.open(subdevice);
        }
        if (!poly.isValid()) {
            fprintf(stderr, "ServerLogpolarFrameGrabber: Cannot make <%s>\n", name->toString().c_str());
            mutex.post();
            return false;
        }
    } else {
        fprintf(stderr, "ServerLogpolarFrameGrabber: \"--subdevice <name>\" not set for logpolarframegrabber\n");
        mutex.post();
        return false;
    }

    if (poly.isValid()) {
        poly.view(fgImage);
        poly.view(fgCtrl);
		poly.view(fgTimed);

        // we need at least the IFrameGrabberImage interface (controls & timed are optional).
        if (!fgImage) {
            fprintf(stderr, "ServerLogpolarFrameGrabber: Can't get the image grabber interface\n");
            mutex.post();
            return false;
        }
    }

    // the main image buffer.
    buffer.resize(fgImage->width(), fgImage->height());

    canDrop = !config.check("no_drop", "if present, use strict policy for sending data");
    addStamp = config.check("stamp", "if present, add timestamps to data");

    string portname = config.check("name", Value("/grabber"), 
                               "name of port to send cartesian images on").asString().c_str();
    fstd.open(portname.c_str());
    fstd.initProcessingMode(canDrop, addStamp, fgTimed);
    fstd.setProcessingSize(cwidth, cheight);

    string namelp = portname;
    namelp += "/logpolar";
    flogp.open(namelp.c_str());
    flogp.initProcessingMode(canDrop, addStamp, fgTimed);
    flogp.setProcessingSize(inang, inecc);

    string namefov = portname;
    namefov += "/fovea";
    ffov.open(namefov.c_str());
    ffov.initProcessingMode(canDrop, addStamp, fgTimed);
    ffov.setProcessingSize(ifovea, ifovea);
    
    // allocate logpolar library helper class.
    if (lpLibrary == 0) {
        fprintf (stderr, "ServerLogpolarFrameGrabber: starting logpolar table allocation, please wait...\n");
        lpLibrary = new logPolarLibrary;
        lpLibrary->allocLookupTables(inecc, inang, baseWidth, baseHeight, ioverlap);
        fprintf (stderr, "ServerLogpolarFrameGrabber: logpolar table allocation completed\n");
    }

    lpLibrary->addInstance();

    active = true;

    // LATER: help information about the device driver (to be completed).
    DeviceResponder::makeUsage();
    addUsage("[set] [bri] $fBrightness", "set brightness");
    addUsage("[set] [expo] $fExposure", "set exposure");    
    addUsage("[set] [shar] $fSharpness", "set sharpness");
    addUsage("[set] [whit] $fBlue $fRed", "set white balance");    
    addUsage("[set] [hue] $fHue", "set hue");
    addUsage("[set] [satu] $fSaturation", "set saturation");    
    addUsage("[set] [gamm] $fGamma", "set gamma");        
    addUsage("[set] [shut] $fShutter", "set shutter");        
    addUsage("[set] [gain] $fGain", "set gain");
    addUsage("[set] [iris] $fIris", "set iris");

    addUsage("[get] [bri]",  "get brightness");
    addUsage("[get] [expo]", "get exposure");    
    addUsage("[get] [shar]", "get sharpness");
    addUsage("[get] [whit]", "get white balance");    
    addUsage("[get] [hue]",  "get hue");
    addUsage("[get] [satu]", "get saturation");    
    addUsage("[get] [gamm]", "get gamma");        
    addUsage("[get] [shut]", "get shutter");        
    addUsage("[get] [gain]", "get gain");
    addUsage("[get] [iris]", "get iris");
    
    addUsage("[get] [w]", "get width of image");
    addUsage("[get] [h]", "get height of image");

    addUsage("[get] [necc]", "get the number of eccentricities of a logpolar image");
    addUsage("[get] [nang]", "get the number of angles of a logpolar image");
    addUsage("[get] [fov]", "get the size of the fovea of a logpolar image");
    addUsage("[get] [ovl]", "get the overlap between receptive fields in the logpolar image");

    // configure and start acquisition thread.
    double framerate = config.check("framerate",Value("0")).asDouble(); // this is the period in fact (not the rate)!
    if (framerate < 0) framerate = 0.;
    RateThread::setRate((int)(framerate+.5));
    if (!RateThread::start()) {
        fprintf(stderr, "ServerLogpolarFrameGrabber: Troubles starting the grabber thread\n");
        // LATER: here I need to delete all objects, close ports, etc.
        active = false;
        mutex.post();
        return false;
    }
 
    mutex.post();
    return true;
}

/*
 * close the device.
 */
bool ServerLogpolarFrameGrabber::close() {
    mutex.wait();
    if (!active) {
        mutex.post();
        return false;
    }

    active = false;
    RateThread::stop();

    if (lpLibrary->removeInstance() == 0) {
        lpLibrary->freeLookupTables();
        delete lpLibrary;
        lpLibrary = 0;
    }

    fstd.close();
    flogp.close();
    ffov.close();

    poly.close();
    fgImage = 0;
    fgCtrl = 0;
    fgTimed = 0;

    mutex.post();
    return true;
}

/*
 * implement the main thread loop.
 * LATER: no-multithread access at the moment (e.g. on rawBuffer).
 */ 
void ServerLogpolarFrameGrabber::run() {
    mutex.wait();

    if (fgImage == 0) {
        mutex.post();
        return;
    }

    fgImage->getImage(buffer);
    if (fstd.getOutputCount() > 0) fstd.process(buffer);
    if (flogp.getOutputCount() > 0) flogp.process(buffer);
    if (ffov.getOutputCount() > 0) ffov.process(buffer);

    mutex.post();
}

/*
 * implement the respond method of the DeviceResponder base class.
 * there's no mutex protection here since the method only accesses the device via
 * other calls that are themselves protected.
 */
bool ServerLogpolarFrameGrabber::respond(const yarp::os::Bottle& cmd, 
                                        yarp::os::Bottle& response) {
    
    int code = cmd.get(0).asVocab();
    bool rec = false;
    bool ok = false;

    // NOTE: this manages also the logpolar interfaces and I need to keep 
    // the DC1394 here because we might be in fact using the 1394 cameras as subdevice
    //
	IFrameGrabberControlsDC1394* fgCtrlDC1394=dynamic_cast<IFrameGrabberControlsDC1394*>(fgCtrl);

	switch (code) 
	{
	case VOCAB_SET:
		//printf("set command received\n");
		{
			switch(cmd.get(1).asVocab()) 
			{
			case VOCAB_BRIGHTNESS:
				ok = setBrightness(cmd.get(2).asDouble());
				rec = true;
				break;
			case VOCAB_EXPOSURE:
				ok = setExposure(cmd.get(2).asDouble());
				rec = true;
				break;
			case VOCAB_SHARPNESS:
				ok = setSharpness(cmd.get(2).asDouble());
				rec = true;
				break;
			case VOCAB_WHITE:
				ok = setWhiteBalance(cmd.get(2).asDouble(),cmd.get(3).asDouble());
				rec = true;
				break;
			case VOCAB_HUE:
				ok = setHue(cmd.get(2).asDouble());
				rec = true;
				break;
			case VOCAB_SATURATION:
				ok = setSaturation(cmd.get(2).asDouble());
				rec = true;
				break;
			case VOCAB_GAMMA:
				ok = setGamma(cmd.get(2).asDouble());
				rec = true;
				break;
			case VOCAB_SHUTTER:
				ok = setShutter(cmd.get(2).asDouble());
				rec = true;
				break;
			case VOCAB_GAIN:
				ok = setGain(cmd.get(2).asDouble());
				rec = true;
				break;
			case VOCAB_IRIS:
				ok = setIris(cmd.get(2).asDouble());
				rec = true;
				break;
			}
		}
		break;
	case VOCAB_GET:
		//printf("get command received\n");
		{
			response.addVocab(VOCAB_IS);
			response.add(cmd.get(1));
			switch(cmd.get(1).asVocab()) 
			{
			case VOCAB_BRIGHTNESS:
				ok = true;
				response.addDouble(getBrightness());
				rec = true;
				break;
			case VOCAB_EXPOSURE:
				ok = true;
				response.addDouble(getExposure());
				rec = true;
				break;
			case VOCAB_SHARPNESS:
				ok = true;
				response.addDouble(getSharpness());
				rec = true;
				break;
			case VOCAB_WHITE:
				{
					ok = true;
					double b=0;
					double r=0;

					getWhiteBalance(b,r);
					response.addDouble(b);
					response.addDouble(r);
					rec=true;
				}
				break;                
			case VOCAB_HUE:
				ok = true;
				response.addDouble(getHue());
				rec = true;
				break;
			case VOCAB_SATURATION:
				ok = true;
				response.addDouble(getSaturation());
				rec = true;
				break;   
			case VOCAB_GAMMA:
				ok = true;
				response.addDouble(getGamma());
				rec = true;
				break;   
			case VOCAB_SHUTTER:
				ok = true;
				response.addDouble(getShutter());
				rec = true;
				break;
			case VOCAB_GAIN:
				ok = true;
				response.addDouble(getGain());
				rec = true;
				break;
			case VOCAB_IRIS:
				ok = true;
				response.addDouble(getIris());
				rec = true;
				break;
			case VOCAB_WIDTH:
				// normally, this would come from stream information
				ok = true;
				response.addInt(width());
				rec = true;
				break;
			case VOCAB_HEIGHT:
				// normally, this would come from stream information
				ok = true;
				response.addInt(height());
				rec = true;
				break;

            // interface ILogpolarFrameGrabberImage.
            case VOCAB_NECC:
                ok = true;
                response.addInt(necc());
                rec = true;
                break;

            case VOCAB_NANG:
                ok = true;
                response.addInt(nang());
                rec = true;
                break;

            case VOCAB_FOVEA:   
                ok = true;
                response.addInt(fovea());
                rec = true;
                break;

            case VOCAB_OVERLAP:
                ok = true;
                response.addDouble(overlap());
                rec = true;
                break;
			}

			if (!ok) {
				// leave answer blank
			}
		}
		break;

    //
    // These are the DC1394 commands. Very misterious code by Ale Scalzo!
    //
	default:
		if (fgCtrlDC1394) 
        switch(code)    
		{
			case VOCAB_DRHASFEA: // VOCAB_DRHASFEA 00
				response.addInt(int(fgCtrlDC1394->hasFeatureDC1394(cmd.get(1).asInt())));
				return true;
			case VOCAB_DRSETVAL: // VOCAB_DRSETVAL 01
				return fgCtrlDC1394->setFeatureDC1394(cmd.get(1).asInt(),cmd.get(2).asDouble());
			case VOCAB_DRGETVAL: // VOCAB_DRGETVAL 02
				response.addDouble(fgCtrlDC1394->getFeatureDC1394(cmd.get(1).asInt()));
				return true;

			case VOCAB_DRHASACT: // VOCAB_DRHASACT 03
				response.addInt(int(fgCtrlDC1394->hasOnOffDC1394(cmd.get(1).asInt())));
				return true;
			case VOCAB_DRSETACT: // VOCAB_DRSETACT 04
				return fgCtrlDC1394->setActiveDC1394(cmd.get(1).asInt(),bool(cmd.get(2).asInt()));
			case VOCAB_DRGETACT: // VOCAB_DRGETACT 05
				response.addInt(int(fgCtrlDC1394->getActiveDC1394(cmd.get(1).asInt())));
				return true;

			case VOCAB_DRHASMAN: // VOCAB_DRHASMAN 06
				response.addInt(int(fgCtrlDC1394->hasManualDC1394(cmd.get(1).asInt())));
				return true;
			case VOCAB_DRHASAUT: // VOCAB_DRHASAUT 07
				response.addInt(int(fgCtrlDC1394->hasAutoDC1394(cmd.get(1).asInt())));
				return true;
			case VOCAB_DRHASONP: // VOCAB_DRHASONP 08
				response.addInt(int(fgCtrlDC1394->hasOnePushDC1394(cmd.get(1).asInt())));
				return true;
			case VOCAB_DRSETMOD: // VOCAB_DRSETMOD 09
				return fgCtrlDC1394->setModeDC1394(cmd.get(1).asInt(),bool(cmd.get(2).asInt()));
			case VOCAB_DRGETMOD: // VOCAB_DRGETMOD 10
				response.addInt(int(fgCtrlDC1394->getModeDC1394(cmd.get(1).asInt())));
				return true;
			case VOCAB_DRSETONP: // VOCAB_DRSETONP 11
				return fgCtrlDC1394->setOnePushDC1394(cmd.get(1).asInt());

			case VOCAB_DRGETMSK: // VOCAB_DRGETMSK 12
				response.addInt(int(fgCtrlDC1394->getVideoModeMaskDC1394()));
				return true;
			case VOCAB_DRGETVMD: // VOCAB_DRGETVMD 13
				response.addInt(int(fgCtrlDC1394->getVideoModeDC1394()));
				return true;
			case VOCAB_DRSETVMD: // VOCAB_DRSETVMD 14
				return fgCtrlDC1394->setVideoModeDC1394(cmd.get(1).asInt());

			case VOCAB_DRGETFPM: // VOCAB_DRGETFPM 15
				response.addInt(int(fgCtrlDC1394->getFPSMaskDC1394()));
				return true;
			case VOCAB_DRGETFPS: // VOCAB_DRGETFPS 16
				response.addInt(int(fgCtrlDC1394->getFPSDC1394()));
				return true;
			case VOCAB_DRSETFPS: // VOCAB_DRSETFPS 17
				return fgCtrlDC1394->setFPSDC1394(cmd.get(1).asInt());


			case VOCAB_DRGETISO: // VOCAB_DRGETISO 18
				response.addInt(int(fgCtrlDC1394->getISOSpeedDC1394()));
				return true;
			case VOCAB_DRSETISO: // VOCAB_DRSETISO 19
				return fgCtrlDC1394->setISOSpeedDC1394(cmd.get(1).asInt());

			case VOCAB_DRGETCCM: // VOCAB_DRGETCCM 20
				response.addInt(int(fgCtrlDC1394->getColorCodingMaskDC1394(cmd.get(1).asInt())));
				return true;
			case VOCAB_DRGETCOD: // VOCAB_DRGETCOD 21
				response.addInt(int(fgCtrlDC1394->getColorCodingDC1394()));
				return true;
			case VOCAB_DRSETCOD: // VOCAB_DRSETCOD 22
				return fgCtrlDC1394->setColorCodingDC1394(cmd.get(1).asInt());

			case VOCAB_DRSETWHB: // VOCAB_DRSETWHB 23
				return fgCtrlDC1394->setWhiteBalanceDC1394(cmd.get(1).asDouble(),cmd.get(2).asDouble());
			case VOCAB_DRGETWHB: // VOCAB_DRGETWHB 24
				{
					double b,r;
					fgCtrlDC1394->getWhiteBalanceDC1394(b,r);
					response.addDouble(b);
					response.addDouble(r);
				}
				return true;

			case VOCAB_DRGETF7M: // VOCAB_DRGETF7M 25
				{
					unsigned int xstep,ystep,xdim,ydim;
					fgCtrlDC1394->getFormat7MaxWindowDC1394(xdim,ydim,xstep,ystep);
					response.addInt(xdim);
					response.addInt(ydim);
					response.addInt(xstep);
					response.addInt(ystep);
				}
				return true;
			case VOCAB_DRGETWF7: // VOCAB_DRGETWF7 26
				{
					unsigned int xdim,ydim;
					fgCtrlDC1394->getFormat7WindowDC1394(xdim,ydim);
					response.addInt(xdim);
					response.addInt(ydim);
				}
				return true;
			case VOCAB_DRSETWF7: // VOCAB_DRSETWF7 27
				return fgCtrlDC1394->setFormat7WindowDC1394(cmd.get(1).asInt(),cmd.get(2).asInt());

			case VOCAB_DRSETOPM: // VOCAB_DRSETOPM 28
				return fgCtrlDC1394->setOperationModeDC1394(bool(cmd.get(1).asInt()));

			case VOCAB_DRGETOPM: // VOCAB_DRGETOPM 29
				response.addInt(fgCtrlDC1394->getOperationModeDC1394());
				return true;

			case VOCAB_DRSETTXM: // VOCAB_DRSETTXM 30
				return fgCtrlDC1394->setTransmissionDC1394(bool(cmd.get(1).asInt()));

			case VOCAB_DRGETTXM: // VOCAB_DRGETTXM 31
				response.addInt(fgCtrlDC1394->getTransmissionDC1394());
				return true;
			/*
			case VOCAB_DRSETBAY: // VOCAB_DRSETBAY 32
			return fgCtrlDC1394->setBayerDC1394(bool(cmd.get(1).asInt()));

			case VOCAB_DRGETBAY: // VOCAB_DRGETBAY 33
			response.addInt(fgCtrlDC1394->getBayerDC1394());
			return true;
			*/
			case VOCAB_DRSETBCS: // VOCAB_DRSETBCS 34
				return fgCtrlDC1394->setBroadcastDC1394(bool(cmd.get(1).asInt()));
			case VOCAB_DRSETDEF: // VOCAB_DRSETDEF 35
				return fgCtrlDC1394->setDefaultsDC1394();
			case VOCAB_DRSETRST: // VOCAB_DRSETRST 36
				return fgCtrlDC1394->setResetDC1394();
			case VOCAB_DRSETPWR: // VOCAB_DRSETPWR 37
				return fgCtrlDC1394->setPowerDC1394(bool(cmd.get(1).asInt()));

			case VOCAB_DRSETCAP: // VOCAB_DRSETCAP 38
				return fgCtrlDC1394->setCaptureDC1394(bool(cmd.get(1).asInt()));

			case VOCAB_DRSETBPP: // VOCAB_DRSETCAP 39
				return fgCtrlDC1394->setBytesPerPacketDC1394(cmd.get(1).asInt());	

			case VOCAB_DRGETBPP: // VOCAB_DRGETTXM 40
				response.addInt(fgCtrlDC1394->getBytesPerPacketDC1394());
				return true;
		}
	}
    if (!rec) {
        ok = DeviceResponder::respond(cmd,response);
    }

    return ok;
}


/*
 * implement the IFrameGrabberImage interface.
 */
bool ServerLogpolarFrameGrabber::getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image) {
    mutex.wait();
    StdImageFormatter fmt;
    image.resize (cwidth, cheight);
    const bool ok = fmt.format(buffer, image);
    mutex.post();
    return ok;
}

// this is only meant to remove the "const" when using the sema in a const method.
#define NSEMA ((Semaphore&)mutex)

int ServerLogpolarFrameGrabber::height() const {
    NSEMA.wait();
    if (fgImage == 0) { 
        NSEMA.post();
        return 0; 
    }
    const int h = cheight;
    NSEMA.post();
    return h;
}

int ServerLogpolarFrameGrabber::width() const {
    NSEMA.wait();
    if (fgImage == 0) { 
        NSEMA.post();
        return 0; 
    }
    const int w = cwidth;
    NSEMA.post();
    return w;
}

// just saving some typing, watch out at the #define :)
#define CHECKIFACE(i) \
    if (i == 0) { mutex.post(); return false; }


/*
 * implement the IFrameGrabberControls interface (set/get methods).
 */
bool ServerLogpolarFrameGrabber::setBrightness(double v) {
    mutex.wait();
    CHECKIFACE(fgCtrl);
    const bool ok = fgCtrl->setBrightness(v);
    mutex.post();
    return ok;
}

bool ServerLogpolarFrameGrabber::setExposure(double v) {
    mutex.wait();
    CHECKIFACE(fgCtrl);
    const bool ok = fgCtrl->setExposure(v);
    mutex.post();
    return ok;
}	

bool ServerLogpolarFrameGrabber::setSharpness(double v) {
    mutex.wait();
    CHECKIFACE(fgCtrl);
    const bool ok = fgCtrl->setSharpness(v);
    mutex.post();
    return ok;
}

bool ServerLogpolarFrameGrabber::setWhiteBalance(double blue, double red) {
    mutex.wait();
    CHECKIFACE(fgCtrl);
    const bool ok = fgCtrl->setWhiteBalance(blue,red);
    mutex.post();
    return ok;
}

bool ServerLogpolarFrameGrabber::setHue(double v) {
    mutex.wait();
    CHECKIFACE(fgCtrl);
    const bool ok = fgCtrl->setHue(v);
    mutex.post();
    return ok;
}

bool ServerLogpolarFrameGrabber::setSaturation(double v) {
    mutex.wait();
    CHECKIFACE(fgCtrl);
    const bool ok = fgCtrl->setSaturation(v);
    mutex.post();
    return ok;
}

bool ServerLogpolarFrameGrabber::setGamma(double v) {
    mutex.wait();
    CHECKIFACE(fgCtrl);
    const bool ok = fgCtrl->setGamma(v);
    mutex.post();
    return ok;
}

bool ServerLogpolarFrameGrabber::setShutter(double v) {
    mutex.wait();
    CHECKIFACE(fgCtrl);
    const bool ok = fgCtrl->setShutter(v);
    mutex.post();
    return ok;
}

bool ServerLogpolarFrameGrabber::setGain(double v) {
    mutex.wait();
    CHECKIFACE(fgCtrl);
    const bool ok = fgCtrl->setGain(v);
    mutex.post();
    return ok;
}

bool ServerLogpolarFrameGrabber::setIris(double v) {
    mutex.wait();
    CHECKIFACE(fgCtrl);
    const bool ok = fgCtrl->setIris(v);
    mutex.post();
    return ok;
}
    
double ServerLogpolarFrameGrabber::getBrightness() {
    mutex.wait();
    CHECKIFACE(fgCtrl);
    const double x = fgCtrl->getBrightness();
    mutex.post();
    return x;
}

double ServerLogpolarFrameGrabber::getExposure() {
    mutex.wait();
    CHECKIFACE(fgCtrl);
    const double x = fgCtrl->getExposure();
    mutex.post();
    return x;
}

double ServerLogpolarFrameGrabber::getSharpness() {
    mutex.wait();
    CHECKIFACE(fgCtrl);
    const double x = fgCtrl->getSharpness();
    mutex.post();
    return x;
}

bool ServerLogpolarFrameGrabber::getWhiteBalance(double &blue, double &red) {
    mutex.wait();
    CHECKIFACE(fgCtrl);
    const bool ok = fgCtrl->getWhiteBalance(blue,red);
    mutex.post();
    return ok;
}

double ServerLogpolarFrameGrabber::getHue() {
    mutex.wait();
    CHECKIFACE(fgCtrl);
    const double x = fgCtrl->getHue();
    mutex.post();
    return x;
}	

double ServerLogpolarFrameGrabber::getSaturation() {
    mutex.wait();
    CHECKIFACE(fgCtrl);
    const double x = fgCtrl->getSaturation();
    mutex.post();
    return x;
}

double ServerLogpolarFrameGrabber::getGamma() {
    mutex.wait();
    CHECKIFACE(fgCtrl);
    const double x = fgCtrl->getGamma();
    mutex.post();
    return x;
}

double ServerLogpolarFrameGrabber::getShutter() {
    mutex.wait();
    CHECKIFACE(fgCtrl);
    const double x = fgCtrl->getShutter();
    mutex.post();
    return x;
}

double ServerLogpolarFrameGrabber::getGain() {
    mutex.wait();
    CHECKIFACE(fgCtrl);
    const double x = fgCtrl->getGain();
    mutex.post();
    return x;
}

double ServerLogpolarFrameGrabber::getIris() {
    mutex.wait();
    CHECKIFACE(fgCtrl);
    const double x = fgCtrl->getIris();
    mutex.post();
    return x;
}
    
/*
 * implementation of the IService interface.
 */
bool ServerLogpolarFrameGrabber::startService() {
    return active;
}

bool ServerLogpolarFrameGrabber::stopService() {
    return close();
}

bool ServerLogpolarFrameGrabber::updateService() {
    return active;
}

/*
 * implement the ILogpolarFrameGrabberImage interface.
 */
int ServerLogpolarFrameGrabber::necc(void) const { 
    NSEMA.wait();
    const int i = inecc;
    NSEMA.post();
    return  i;
}

int ServerLogpolarFrameGrabber::nang(void) const { 
    NSEMA.wait();
    const int i = inang;
    NSEMA.post();
    return i; 
}

int ServerLogpolarFrameGrabber::fovea(void) const {
    NSEMA.wait();
    const int i = ifovea;
    NSEMA.post();
    return i;
}

double ServerLogpolarFrameGrabber::overlap(void) const {
    NSEMA.wait();
    const double x = ioverlap;
    NSEMA.post();
    return x;
}

bool ServerLogpolarFrameGrabber::getLogpolarImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image) { 
    mutex.wait();
    LogpolarImageFormatter fmt;
    image.resize (inang, inecc);
    const bool ok = fmt.format(buffer, image);
    mutex.post();
    return ok;
}

bool ServerLogpolarFrameGrabber::getFovealImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image) { 
    mutex.wait();
    FovealImageFormatter fmt;
    image.resize (ifovea, ifovea);
    const bool ok = fmt.format(buffer, image);
    mutex.post();
    return ok; 
}

/*
 * implement the ILogpolarAPI interface.
 */
bool ServerLogpolarFrameGrabber::allocLookupTables(int necc, int nang, int w, int h, double overlap) {
    mutex.wait();
    const bool ok = HELPER(lpLibrary).allocLookupTables(necc, nang, w, h, overlap);
    mutex.post();
    return ok;
}

bool ServerLogpolarFrameGrabber::freeLookupTables() {
    mutex.wait();
    const bool ok = HELPER(lpLibrary).freeLookupTables();
    mutex.post();
    return ok;
}

bool ServerLogpolarFrameGrabber::cartToLogpolar(ImageOf<PixelRgb>& lp, const ImageOf<PixelRgb>& cart) {
    mutex.wait();
    const bool ok = HELPER(lpLibrary).cartToLogpolar(lp, cart);
    mutex.post();
    return ok;
}

bool ServerLogpolarFrameGrabber::logpolarToCart(ImageOf<PixelRgb>& cart, const ImageOf<PixelRgb>& lp) {
    mutex.wait();
    const bool ok = HELPER(lpLibrary).logpolarToCart(cart, lp);
    mutex.post();
    return ok;
}

/*
 * implement the image formatters.
 */
bool StdImageFormatter::format(const yarp::sig::ImageOf<yarp::sig::PixelRgb>& buffer, 
                               yarp::sig::ImageOf<yarp::sig::PixelRgb>& formatted) {
    const int w = formatted.width();
    const int h = formatted.height();
    // LATER: replace this with a proper filtering + subsampling.
    formatted.copy(buffer, w, h);   // copy & resize.
    return true;
}

bool LogpolarImageFormatter::format(const yarp::sig::ImageOf<yarp::sig::PixelRgb>& buffer, 
                                    yarp::sig::ImageOf<yarp::sig::PixelRgb>& formatted) {
    HELPER(lpLibrary).cartToLogpolar(formatted, (ImageOf<PixelRgb>&)buffer);
    return true;
}

bool FovealImageFormatter::format(const yarp::sig::ImageOf<yarp::sig::PixelRgb>& buffer, 
                                  yarp::sig::ImageOf<yarp::sig::PixelRgb>& formatted) {
    return subsampleFovea(formatted, buffer);
}

bool FovealImageFormatter::subsampleFovea(yarp::sig::ImageOf<yarp::sig::PixelRgb>& dst, 
                                          const yarp::sig::ImageOf<yarp::sig::PixelRgb>& src) {
    const int fov = dst.width();
    const int offset = baseHeight/2-fov/2;
    const int col = baseWidth/2-fov/2;
    const int bytes = fov*sizeof(PixelRgb);

    int i;
    for (i = 0; i < fov; i++) {
        unsigned char *s = (unsigned char *)src.getRow(i+offset)+col*sizeof(PixelRgb);
        unsigned char *d = dst.getRow(i);
        memcpy(d, s, bytes);
    }
    return true;
}

/**
 * @endcond
 */
