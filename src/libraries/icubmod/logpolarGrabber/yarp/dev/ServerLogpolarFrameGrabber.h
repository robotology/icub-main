// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Author: Giorgio Metta
 * Copyright (C) 2009 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/**
 * @file ServerLogpolarFrameGrabber.h
 * @brief A device driver that wraps logpolar sampling around a grabber device exporing the IFrameGrabberImage 
 * interface and a number of ports with standard, logpolar and foveal images.
 *
 * 
 * @defgroup logpolar logpolar drivers
 * @ingroup icub_hardware_modules
 *
 * Classes that define and implement a frame grabber in logpolar image format. This is a
 * network wrapper of any device providing the IFrameGrabberImage and IFrameGrabberControls
 * interfaces.
 *
 * The network interface is built on multiple Ports (logpolar and foveal).
 * Images are streamed out from those Ports -- RemoteFrameGrabber
 * uses these streams to provide the IFrameGrabberImage/ILogpolarFrameGrabberImage 
 * interfaces.
 * The IFrameGrabberControls and ILogpolarFrameGrabberControls functionality are 
 * provided via RPC.
 * 
 * @section dep_sec Dependencies 
 * - the logPolar library 
 *
 * @author Giorgio Metta
 */

#ifndef __YARPSERVERLOGPOLARFRAMEGRABBER__
#define __YARPSERVERLOGPOLARFRAMEGRABBER__

#include <stdio.h>

/* dev drivers */
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/dev/ServerFrameGrabber.h>
#include <iCub/LogpolarInterfaces.h>

/* std os */
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Semaphore.h>

namespace yarp {
    namespace dev {
        class ServerLogpolarFrameGrabber;
        template <class T, class F> class ImageFormatter;
        template <class T> class BaseFormatter;
        class StdImageFormatter;
        class LogpolarImageFormatter;
        class FovealImageFormatter;
    }
}

/**
 * @ingroup logpolar
 *
 * A template base class with format method. Pass this as F argument to the ImageFormatter template.
 */
template <class T>
class yarp::dev::BaseFormatter {
public:
    /**
     * The format method takes a raw buffer image and formats according to the
     * code provided in the format method.
     *
     * @param buffer is the raw input buffer, e.g. an rgb image.
     * @param formatted is the output image of type T.
     * @return true iff the call is successful.
     */
    virtual bool format(const T& buffer, T& formatted) { return false; }
};

/**
 * @ingroup logpolar
 *
 * Customization to the standard image formatter to provide a rectangular subsampled image.
 */
class yarp::dev::StdImageFormatter : public yarp::dev::BaseFormatter<yarp::sig::ImageOf<yarp::sig::PixelRgb> > {
public:
    /**
     * The format method takes a raw buffer image and formats according to the
     * code provided in the format method.
     *
     * @param buffer is the raw input buffer, e.g. bayer pattern image.
     * @param formatted is the output image of type T.
     * @return true iff the call is successful.
     */
    virtual bool format(const yarp::sig::ImageOf<yarp::sig::PixelRgb>& buffer, 
                        yarp::sig::ImageOf<yarp::sig::PixelRgb>& formatted);
};

/**
 * @ingroup logpolar
 *
 * Customization of the standard image formatter to provide a logpolar subsampled image output.
 */
class yarp::dev::LogpolarImageFormatter : public yarp::dev::BaseFormatter<yarp::sig::ImageOf<yarp::sig::PixelRgb> > {
protected:
public:
    /**
     * The format method takes a raw buffer image and formats according to the
     * code provided in the format method.
     *
     * @param buffer is the raw input buffer, e.g. bayer pattern image.
     * @param formatted is the output image of type T.
     * @return true iff the call is successful.
     */
    virtual bool format(const yarp::sig::ImageOf<yarp::sig::PixelRgb>& buffer, 
                        yarp::sig::ImageOf<yarp::sig::PixelRgb>& formatted);
};

/**
 * @ingroup logpolar
 * 
 * Customization of the standard formatter to provide a foveal image output (full res cartesian image of the fovea).
 */
class yarp::dev::FovealImageFormatter : public yarp::dev::BaseFormatter<yarp::sig::ImageOf<yarp::sig::PixelRgb> > {
protected:
public:
    /**
     * The format method takes a raw buffer image and formats according to the
     * code provided in the format method.
     *
     * @param buffer is the raw input buffer, e.g. bayer pattern image.
     * @param formatted is the output image of type T.
     * @return true iff the call is successful.
     */
    virtual bool format(const yarp::sig::ImageOf<yarp::sig::PixelRgb>& buffer, 
                        yarp::sig::ImageOf<yarp::sig::PixelRgb>& formatted);

    /**
     * Get the fovea from the raw full-res buffer.
     * @param dst is the destination image (Yarp image).
     * @param src is the source image buffer.
     * @return true iff successful.
     */
    virtual bool subsampleFovea(yarp::sig::ImageOf<yarp::sig::PixelRgb>& dst, 
                                const yarp::sig::ImageOf<yarp::sig::PixelRgb>& src);
};

/**
 * @ingroup logpolar
 *
 * A flexible image formatter template class. This takes raw images as input, formats them
 * formats and writes them to an output port.
 */
template <class T, class F>
class yarp::dev::ImageFormatter : public yarp::os::Port {
private:
    ImageFormatter(const ImageFormatter&);
    void operator=(const ImageFormatter&);

protected:
    yarp::os::PortWriterBuffer<T> writer;

    bool canDrop;
    bool addStamp;
    IPreciselyTimed *pPrecTime;
    int width;
    int height;

    F fmt;

public:
    /**
     * Constructor.
     */
    ImageFormatter() {
        canDrop = false;
        addStamp = true;
        width = 0;
        height = 0;
        writer.attach(*this);
    }

    /**
     * Prepare the formatter and port to processing.
     * @param name of the Port connection, i.e. call to open.
     * @param canDrop indicates whether packets can be dropped (strict protocol).
     * @param addStamp indicates whether time stamps should be added to the messages.
     * @param pt is an interface of type IPreciselyTimed to access the drive time stamps.
     * @return always true.
     */
    virtual bool initProcessingMode(bool drp, bool stamp, IPreciselyTimed *pt = 0) {
        canDrop = drp;
        addStamp = stamp;
        pPrecTime = pt;
        return true;
    }

    /**
     * Prepare the formatter with image-related information.
     * @param w is the input image width.
     * @param h is the input image height.
     * @return always true.
     */
    virtual bool setProcessingSize(int w, int h) {
        width = w;
        height = h;
        return true;
    }

    /**
     * Destructor, close the internal Port object.
     */
    virtual ~ImageFormatter() {
        yarp::os::Port::close();
    }

    /**
     * This method calls format to process the image buffer and then 
     * sends the result across the network using the internal Port object.
     *
     * @param buffer is the raw input buffer, e.g. bayer pattern image.
     * @return true iff the call is successful.
     */
    virtual bool process(const T& buffer) {
        yarp::os::Stamp stamp;
        T& datum = writer.get();
        datum.resize(width, height);
        bool ok = fmt.format(buffer, datum);
        if (ok) {
            if (addStamp) {
			    if (pPrecTime)
			    {
				    stamp = pPrecTime->getLastInputStamp();
			    }
			    else
			    {
				    stamp.update();
			    }
                Port::setEnvelope(stamp);
            }

            writer.write(!canDrop);
            ok = true;
        }

        return ok;
    }
};


/**
 * @ingroup logpolar
 *
 * Export a frame grabber in logpolar image format to the network.  
 * Provides the IFrameGrabberImage, IFrameGrabberControls, 
 * ILogpolarFrameGrabberImage, and ILogpolarFrameGrabberControls
 * interfaces.  The corresponding client is a RemoteLogpolarFrameGrabber.
 *
 * The network interface is on multiple Ports (logpolar and foveal).
 * Images are streamed out from those Ports -- RemoteFrameGrabber
 * uses these streams to provide the IFrameGrabberImage/ILogpolarFrameGrabberImage 
 * interfaces.
 * The IFrameGrabberControls and ILogpolarFrameGrabberControls functionality are 
 * provided via RPC.
 *
 * Here's a command-line example:
 * \verbatim
 [terminal A] icubmoddev --device logpolargrabber --width 320 --height 240 --framerate 33 --subdevice test_grabber
 [terminal B] yarpview --name /view1 --x 0 --y 50 --synch
 [terminal C] yarp connect /grabber\logpolar /view1
 \endverbatim
 *
 * The yarpdev line starts a TestFrameGrabber wrapped in a ServerLogpolarFrameGrabber.
 * Parameters are:
 * --width, --height set the size of the frame in pixels
 * --name portname set the name of the output port
 * --framerate set the frequency (Hz) at which images will be read and boradcast to 
 * the network; if the parameter is not set images are provided at the maximum speed
 * supported by the device. Notice that the maximum frame rate is determined by
 * the device.
 * 
 * After the "yarp connect" line, images will show up in the yarpview window.
 *
 * The "yarp rpc" command can be used to query the parameters of the test_grabber.
 *
 * <TABLE>
 * <TR><TD> Command (text form) </TD><TD> Response </TD><TD> Code equivalent </TD></TR>
 * <TR><TD> [set] [bri] 1.0 </TD><TD> none </TD><TD> setBrightness() </TD></TR>
 * <TR><TD> [set] [gain] 1.0 </TD><TD> none </TD><TD> setGain() </TD></TR>
 * <TR><TD> [set] [shut] 1.0 </TD><TD> none </TD><TD> setShutter() </TD></TR>
 * <TR><TD> [get] [bri] </TD><TD> [is] [bri] 1.0 </TD><TD> getBrightness() </TD></TR>
 * <TR><TD> [get] [gain] </TD><TD> [is] [gain] 1.0 </TD><TD> getGain() </TD></TR>
 * <TR><TD> [get] [shut] </TD><TD> [is] [shut] 1.0 </TD><TD> getShutter() </TD></TR>
 * </TABLE>
 *
 */
class yarp::dev::ServerLogpolarFrameGrabber : public DeviceDriver, 
                public DeviceResponder,
                public IFrameGrabberImage,
                public ILogpolarFrameGrabberImage,
                public IFrameGrabberControls,
                public IService,
                public ILogpolarAPI,
                public yarp::os::RateThread
{
private:
    ServerLogpolarFrameGrabber(const ServerLogpolarFrameGrabber&);
    void operator=(const ServerLogpolarFrameGrabber&);

protected:
    // we have three ports, std to maintain compatibility with cartesian image processing
    // logp and fovea to stream the logpolar images and fovea respectively. These data
    // are stored in the ImageFormatter template class which is specialized to the three 
    // types of image "subsampling".
    ImageFormatter<yarp::sig::ImageOf<yarp::sig::PixelRgb>, yarp::dev::StdImageFormatter> fstd;
    ImageFormatter<yarp::sig::ImageOf<yarp::sig::PixelRgb>, yarp::dev::LogpolarImageFormatter> flogp;
    ImageFormatter<yarp::sig::ImageOf<yarp::sig::PixelRgb>, yarp::dev::FovealImageFormatter> ffov;

    PolyDriver poly;
    IFrameGrabberImage *fgImage;
    IFrameGrabberControls *fgCtrl;
    IPreciselyTimed *fgTimed;

    yarp::os::Semaphore mutex;

    // keep one copy of the image buffer and provide various "views" of it depending on 
    // the request (std image, logpolar, foveal).
    yarp::sig::ImageOf<yarp::sig::PixelRgb> buffer;

    // cartesian output image specs.
    int cwidth;
    int cheight;

    // logpolar output image specs.
    int inecc;
    int inang;
    int ifovea;
    double ioverlap;

    bool canDrop;
    bool addStamp;
    bool active;

public:
    /**
     * Constructor, take care of the default initialization.
     */
    ServerLogpolarFrameGrabber();

    /**
     * Configure with a set of options. These are:
     * <TABLE>
     * <TR><TD> subdevice </TD><TD> Common name of device to wrap (e.g. "test_grabber"). </TD></TR>
     * <TR><TD> name </TD><TD> Port name to assign to this server (default /grabber). </TD></TR>
     * <TR><TD> width </TD><TD> Width of the cartesian output image. </TD></TR>
     * <TR><TD> height </TD><TD> Height of the cartesian output image. </TD></TR>
     * <TR><TD> framerate </TD><TD> Period of the acquisition thread (e.g. 33ms). </TD></TR>
     * </TABLE>
     *
     * @param config The options to use
     * @return true iff the object could be configured.
     */
    virtual bool open(yarp::os::Searchable& config);

    /**
     * Closes the device and communication ports.
     * @return true if the was active.
     */
    virtual bool close();
    
    /**
     * Implement the respond method from the DeviceResponder base class.
     * @param command, the request to the device arriving from the port attached to the responder.
     * @param reply the reply message to the request.
     * @return true iff the message was recognized ok.
     */
    virtual bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    /**
     * Implement the RateThread main loop.
     */
    virtual void run();

    // the IFrameGrabberImage interface, the server manages a unique image buffer that 
    // is sampled to logpolar and/or copied without extra processing.

    /**
     * Get the last image from the acquisition buffer.
     * @param image is filled with the last acquired buffer in Rgb format.
     * @return true iff the operation is successful.
     */
    virtual bool getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image);
    
    /**
     * Get the height of the acquired buffer (number of rows) in pixels.
     * @return the height of the image in pixels.
     */
    virtual int height() const;

    /**
     * Get the width of the acquired buffer (number of rows) in pixels.
     * @return the width of the image in pixels.
     */
    virtual int width() const;

    // the IFrameGrabberControls interface (set methods).

    /**
     * Set the brightness parameter of the acquisition device.
     * @param v is a double whose range depends on the specific camera device.
     * @return true iff the operation is successful.
     */
	virtual bool setBrightness(double v);

    /**
     * Set the exposure parameter of the acquisition device.
     * @param v is a double whose range depends on the specific camera device.
     * @return true iff the operation is successful.
     */
    virtual bool setExposure(double v);

    /**
     * Set the sharpness parameter of the acquisition device.
     * @param v is a double whose range depends on the specific camera device.
     * @return true iff the operation is successful.
     */
	virtual bool setSharpness(double v);

    /**
     * Set the white balance parameters of the acquisition device.
     * @param blue is a double value which controls the amount of blue in the image.
     * @param red is a double value which controls the amount of red in the image.
     * @return true iff the operation is successful.
     */
	virtual bool setWhiteBalance(double blue, double red);

    /**
     * Set the hue parameter of the acquisition device.
     * @param v is a double value which changes the hue in the image.
     * @return true iff the operation is successful.
     */
	virtual bool setHue(double v);

    /**
     * Set the saturation (color) parameter of the acquisition device.
     * @param v is a double value which changes the color saturation in the image.
     * @return true iff the operation is successful.
     */
	virtual bool setSaturation(double v);

    /**
     * Set the gamma value (color correction) parameter of the acquisition device.
     * @param v is a double value which changes the gamma of the image.
     * @return true iff the operation is successful.
     */
	virtual bool setGamma(double v);

    /**
     * Set the shutter parameter of the acquisition device.
     * @param v is a double value whose range depends on the specific device.
     * @return true iff the operation is successful.
     */
    virtual bool setShutter(double v);

    /**
     * Set the acquisition amplifier gain of the device.
     * @param v is a double value whose range depends on the specific device.
     * @return true iff the operation is successful.
     */
    virtual bool setGain(double v);

    /**
     * Set the iris parameter of the acquisition device.
     * @param v is a double value whose range depends on the specifics of the device.
     * @return true iff the operation is successful.
     */
    virtual bool setIris(double v);

    // the IFrameGrabberControls interface (get methods).

    /**
     * Get the brightness control value of the acquisition device.
     * @return the brightness value if successful, 0 otherwise.
     */
	virtual double getBrightness();

    /**
     * Get the exposure control value of the camera acquisition device.
     * @return the exposure value if successful, 0 otherwise.
     */
	virtual double getExposure();

    /**
     * Get the sharpness control value of the camera acquisition device.
     * @return the sharpness value if successful, 0 otherwise.
     */
	virtual double getSharpness();

    /**
     * Get the white balance control values of the camera acquisition device.
     * @param blue is a double precision number containing the current blue balance value.
     * @param red is a double precision number containing the current red balance value.
     * @return true iff successful.
     */
    virtual bool getWhiteBalance(double &blue, double &red);

    /**
     * Get the hue (color) control value of the camera acquisition device.
     * @return the value of the hue if successful, 0 otherwise.
     */
	virtual double getHue();

    /**
     * Get the saturation (color) control value of the camera acquisition device.
     * @return the value of the saturation if successful, 0 otherwise.
     */
	virtual double getSaturation();

    /**
     * Get the gamma compensation control value of the camera acquisition device.
     * @return the value of the gamma compensation if successful, 0 otherwise.
     */
	virtual double getGamma();

    /**
     * Get the shutter control value of the camera acquisition device.
     * @return the value of the shutter if successful, 0 otherwise.
     */
    virtual double getShutter();

    /**
     * Get the amplifier gain value of the camera acquisition device.
     * @return the value of the amplifier gain if successful, 0 otherwise.
     */
    virtual double getGain();

    /**
     * Get the iris control value of the camera acquisition device.
     * @return the value of the iris value if successful, 0 otherwise.
     */
    virtual double getIris();
    
    // implementation of the IService interface.
    // NOTE: this interface seems useless!
    virtual bool startService();
    virtual bool stopService();
    virtual bool updateService();

    // implementation of the ILogpolarFrameGrabberImage interface.

    /**
     * Get the number of eccentricities.
     * @return the number of eccentricities of the logpolar image.
     */
    virtual int necc(void) const;

    /**
     * Get the number of angles.
     * @return the number of angles of the logpolar image.
     */
    virtual int nang(void) const;

    /**
     * Get the fovea size (in pixels).
     * @return the size of the foveal image (square).
     */
    virtual int fovea(void) const;

    /**
     * Get the overlap between receptive fields.
     * @return the size of the overlap (double).
     */
    virtual double overlap(void) const;

    /**
     * Get the logpolar image.
     * @param image is the RGB image containing the logpolar subsampled image.
     * @return true iff successful.
     */
    virtual bool getLogpolarImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image);

    /**
     * Get the foveal image (a small part of the centre of the image in full resolution).
     * @param image is the RGB image containing the foveal image.
     * @return true iff successful.
     */
    virtual bool getFovealImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image);

    // implementation of the ILogpolarAPI interface.

    /**
     * Alloc the lookup tables and stores them in memory.
     * @param necc is the number of eccentricities of the logpolar image.
     * @param nang is the number of angles of the logpolar image.
     * @param w is the width of the original rectangular image.
     * @param h is the height of the original rectangular image.
     * @param overlap is the degree of overlap of the receptive fields (>0.).
     * @return true iff successful.
     */
    virtual bool allocLookupTables (int necc, int nang, int w, int h, double overlap);

    /**
     * Free the lookup tables from memory.
     * @return true iff successful.
     */
    virtual bool freeLookupTables ();

    /**
     * Converts an image from rectangular to logpolar.
     * @param lp is the logpolar image (destination).
     * @param cart is the cartesian image (source data).
     * @return true iff successful. Beware that tables must be
     * allocated in advance.
     */
    virtual bool cartToLogpolar(yarp::sig::ImageOf<yarp::sig::PixelRgb>& lp, 
                                const yarp::sig::ImageOf<yarp::sig::PixelRgb>& cart);

    /**
     * Converts an image from logpolar to cartesian (rectangular).
     * @param cart is the cartesian image (destination).
     * @param lp is the logpolar image (source).
     * @return true iff successful. Beware that tables must be
     * allocated in advance.
     */
    virtual bool logpolarToCart(yarp::sig::ImageOf<yarp::sig::PixelRgb>& cart,
                                const yarp::sig::ImageOf<yarp::sig::PixelRgb>& lp);
};

#endif /* __YARPSERVERLOGPOLARFRAMEGRABBER__ */


