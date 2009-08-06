// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006 Paul Fitzpatrick, Giorgio Metta
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

//
// $Id: DragonflyDeviceDriver2.h,v 1.1 2009/03/18 17:03:16 ale-scalzo Exp $
//
//

#ifndef __DragonflyDeviceDriver2h__
#define __DragonflyDeviceDriver2h__

//=============================================================================
// YARP Includes
//=============================================================================

// May 06, readapted for YARP2 by nat

#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/PreciselyTimed.h>

namespace yarp {
    namespace dev {
        class Dragonfly2OpenParameters;
        class DragonflyDeviceDriver2;
    }
}

/**
 * \file FirewireCamera.h device driver for managing the 
 * IEEE-1394-DR2 Camera
 */

/**
 * Structure for defining the open() parameters of the camera.
 */
class yarp::dev::Dragonfly2OpenParameters
{
public:
	// Parameters
	unsigned int _unit_number;
    unsigned int _port_number;
	unsigned int _size_x;
	unsigned int _size_y;
	unsigned int _video_type;

	double _brightness;
	double _exposure;
	double _sharpness;
	double _whiteB,_whiteR;
	double _hue;
	double _saturation;
	double _gamma;
	double _shutter;
	double _gain;
	double _iris;

	bool _DR2; // Dragonfly2 model

	bool _fleacr;  //FLEA color reconstruction flag

	/**
	 * Constructor. Add here the parameters for the open().
	 */
	Dragonfly2OpenParameters()
	{
		// parameters initialization
        _port_number = 0;
		_unit_number = 0;
		_size_x = 0;
		_size_y = 0;
		_video_type = 0;

		//uninitialized - inherit registry stored values
		_brightness=-1.0;
	    _exposure=-1.0;
	    _sharpness=-1.0;
		_whiteB=_whiteR=-1.0;
	    _hue=-1.0;
	    _saturation=-1.0;	    
	    _gamma=-1.0;	    
		_shutter=-1.0;
		_gain=-1.0;
		_iris=-1.0;

		_DR2 = false;

		// FLEA cameras are compatible with DRAGONFLY's but ...
		// the color reconstruction method is different 
		// (GBRG instead of RGGB)
		// The default is to use Dragonsfly's method
		_fleacr = false;

	}

};

/**
 * @ingroup dev_impl_media
 *
 * A generic firewire digital camera (or, on Linux, any digital camera).
 */

class yarp::dev::DragonflyDeviceDriver2 : 
    public IFrameGrabber, 
    public IPreciselyTimed,
    public IFrameGrabberRgb, public IFrameGrabberImage, public IFrameGrabberControlsDC1394, public DeviceDriver
{
private:
	DragonflyDeviceDriver2(const DragonflyDeviceDriver2&);
	void operator=(const DragonflyDeviceDriver2&);

public:
	/**
	 * Constructor.
	 */
	DragonflyDeviceDriver2();

	/**
	 * Destructor.
	 */
	virtual ~DragonflyDeviceDriver2();

    // temp: here for debug purposes only
    void recColorFSBilinear(const unsigned char *src, unsigned char *out);
    void recColorFSNN(const unsigned char *src, unsigned char *out);
    void recColorHSBilinear(const unsigned char *src, unsigned char *out);

    /**
	 * Open the device driver.
     * @param par parameters for the device driver
	 * @return returns true on success, false on failure.
	 */
    bool open(const Dragonfly2OpenParameters& par);

    virtual bool open(yarp::os::Searchable& config)
	{
        Dragonfly2OpenParameters params;
		yarp::os::Value *value;
		if (config.check("port_number",value)) {
			params._port_number = value->asInt();
		}		
        if (config.check("unit_number",value)||config.check("d",value)) {
			params._unit_number = value->asInt();
		}
		if (config.check("size_x",value)||config.check("width",value)){
			params._size_x  = value->asInt();
		}
		if (config.check("size_y",value)||config.check("height",value)){
			params._size_y  = value->asInt();
		}

		params._video_type=0;
		if (config.check("video_type",value))
		{
			params._video_type = value->asInt();
		}

		//params._offset_y = config.find("offset_y").asInt();
		//params._offset_x = config.find("offset_x").asInt();
		//params._alfa = (float)config.find("alfa").asInt();

		if (config.check("brightness", value)){
			params._brightness=value->asDouble();
		}
		if (config.check("exposure", value)){
			params._exposure=value->asDouble();
		}
		if (config.check("sharpness", value)){
			params._sharpness=value->asDouble();
		}
		yarp::os::Bottle& white_balance = config.findGroup("white_balance");
		if (!white_balance.isNull()) {
			params._whiteR = white_balance.get(1).asDouble();
			params._whiteB = white_balance.get(2).asDouble();
		}
		if (config.check("hue", value)){
			params._hue=value->asDouble();
		}
		if (config.check("saturation", value)){
			params._saturation=value->asDouble();
		}
		if (config.check("gamma", value)){
			params._gamma=value->asDouble();
		}
		if (config.check("shutter", value)){
			params._shutter=value->asDouble();
		}
		if (config.check("gain", value)){
			params._gain=value->asDouble();
		}
 		if (config.check("iris", value)){
			params._iris=value->asDouble();
		}       

		params._DR2 = config.check("DR2","If present indicates Dragonfly2 camera model");

		params._fleacr = config.check("flea", "If present indicates to use Flea color reconstruction ");
		

		return open(params);
    }

	/**
	 * Closes the device driver.
	 * @return returns true/false on success/failure.
	 */
	virtual bool close(void);

    /**
     * Implements FrameGrabber basic interface.
     * @param buffer the pointer to the array to store the last frame.
     * @return returns true/false on success/failure.
     */
    virtual bool getRawBuffer(unsigned char *buffer);

    /**
     * Implements the Frame grabber basic interface.
     * @return the size of the raw buffer (for the Dragonfly
     * camera this is 1x640x480).
     */
    virtual int getRawBufferSize();

    /**
     * Implements FrameGrabber basic interface.
     */
    virtual int height() const;
    
    /**
     * Implements FrameGrabber basic interface.
     */
    virtual int width() const;

    /** 
     * FrameGrabber bgr interface, returns the last acquired frame as
     * a buffer of bgr triplets. A demosaicking method is applied to 
     * reconstuct the color from the Bayer pattern of the sensor.
     * @param buffer pointer to the array that will contain the last frame.
     * @return true/false upon success/failure
     */
    virtual bool getRgbBuffer(unsigned char *buffer);

    /** 
     * FrameGrabber image interface, returns the last acquired frame as
     * an rgb image. A demosaicking method is applied to 
     * reconstuct the color from the Bayer pattern of the sensor.
     * @param image that will store the last frame.
     * @return true/false upon success/failure
     */
    virtual bool getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image);

    /** 
     * Implements the IPreciselyTimed interface.
     * @return the yarp::os::Stamp of the last image acquired
     */
    virtual yarp::os::Stamp getLastInputStamp();

    /**
     * Set image normalized brightness [0.0 : 1.0].
     */
    virtual bool setBrightness(double v);
    /**
     * Set image normalized exposure [0.0 : 1.0].
     */
	virtual bool setExposure(double v);
    /**
     * Set image normalized sharpness [0.0 : 1.0].
     */
	virtual bool setSharpness(double v);
	/**
     * Set normalized white balance [0.0 : 1.0].
     */
	virtual bool setWhiteBalance(double blue, double red);
    /**
     * Set image normalized hue [0.0 : 1.0].
     */
	virtual bool setHue(double v);
    /**
     * Set image normalized saturation [0.0 : 1.0].
     */
	virtual bool setSaturation(double v);
    /**
     * Set image normalized gamma [0.0 : 1.0].
     */
	virtual bool setGamma(double v);		
    /**
     * Set normalized shutter time [0.0 : 1.0].
     */
    virtual bool setShutter(double v);
    /**
     * Set normalized gain [0.0 : 1.0].
     */
    virtual bool setGain(double v);
    /**
     * Set normalized iris [0.0 : 1.0].
     */
    virtual bool setIris(double v);
    //virtual bool setTemperature(double v);
    //virtual bool setWhiteShading(double r,double g,double b);
    //virtual bool setOpticalFilter(double v);
    //virtual bool setCaptureQuality(double v);

    /**
     * Get normalized image brightness [0.0 : 1.0].
     */
    virtual double getBrightness();
    /**
     * Get normalized image exposure [0.0 : 1.0].
     */
    virtual double getExposure();    
    /**
     * Get normalized image sharpness [0.0 : 1.0].
     */
    virtual double getSharpness();    
    /**
     * Get normalized image white balance [0.0 : 1.0].
     */
	virtual bool getWhiteBalance(double &blue, double &red);
    /**
     * Get normalized hue [0.0 : 1.0].
     */
	virtual double getHue();
    /**
     * Get normalized saturation [0.0 : 1.0].
     */
	virtual double getSaturation();
    /**
     * Get normalized gamma [0.0 : 1.0].
     */
	virtual double getGamma();
	/**
     * Get normalized shutter time [0.0 : 1.0].
     */
    virtual double getShutter();
    /**
     * Get normalized gain [0.0 : 1.0].
     */
    virtual double getGain();
    /**
     * Get normalized iris [0.0 : 1.0].
     */
    virtual double getIris();


    //virtual double getTemperature() const;
    //virtual double getWhiteShading() const;
    //virtual double getOpticalFilter() const;
    //virtual double getCaptureQuality() const;

	//virtual bool setAutoBrightness(bool bAuto=true);

	//virtual bool setAutoGain(bool bAuto=true);

	//virtual bool setAutoShutter(bool bAuto=true);

	//virtual bool setAutoWhiteBalance(bool bAuto=true);

	//virtual bool setAuto(bool bAuto=true);

	//virtual void PrintSettings();

	/////////
	// DC1394
	/////////

	virtual bool hasFeatureDC1394(int feature);
	virtual bool setFeatureDC1394(int feature,double value);
	virtual double getFeatureDC1394(int feature);
	virtual bool hasOnOffDC1394(int feature);
	virtual bool setActiveDC1394(int feature, bool onoff);
	virtual bool getActiveDC1394(int feature);
	virtual bool hasAutoDC1394(int feature);
	virtual bool hasManualDC1394(int feature);
	virtual bool hasOnePushDC1394(int feature);
	virtual bool setModeDC1394(int feature, bool auto_onoff);
	virtual bool getModeDC1394(int feature);
	virtual bool setOnePushDC1394(int feature);
	virtual unsigned int getVideoModeMaskDC1394();
	virtual unsigned int getVideoModeDC1394();
	virtual bool setVideoModeDC1394(int video_mode);
	virtual unsigned int getFPSMaskDC1394();
	virtual unsigned int getFPSDC1394();
	virtual bool setFPSDC1394(int fps);
	virtual unsigned int getISOSpeedDC1394();
	virtual bool setISOSpeedDC1394(int speed);
	virtual unsigned int getColorCodingMaskDC1394(unsigned int video_mode);
	virtual unsigned int getColorCodingDC1394();
	virtual bool setColorCodingDC1394(int coding);
	virtual bool setWhiteBalanceDC1394(double b, double r);
	virtual bool getWhiteBalanceDC1394(double &b, double &r);
	virtual bool getFormat7MaxWindowDC1394(unsigned int &xdim,unsigned int &ydim,unsigned int &xstep,unsigned int &ystep);
	virtual bool getFormat7WindowDC1394(unsigned int &xdim,unsigned int &ydim);
	virtual bool setFormat7WindowDC1394(unsigned int xdim,unsigned int ydim);
	virtual bool setOperationModeDC1394(bool b1394b);
	virtual bool getOperationModeDC1394(); 
	virtual bool setTransmissionDC1394(bool bTxON);
	virtual bool getTransmissionDC1394();
	//virtual bool setBayerDC1394(bool bON);
	//virtual bool getBayerDC1394();
	virtual bool setBroadcastDC1394(bool onoff);
	virtual bool setDefaultsDC1394();
	virtual bool setResetDC1394();
	virtual bool setPowerDC1394(bool onoff);
	virtual bool setCaptureDC1394(bool bON);
	virtual unsigned int getBytesPerPacketDC1394();
	virtual bool setBytesPerPacketDC1394(unsigned int bpp);

protected:
	void *system_resources;
};

/**
 * @ingroup dev_runtime
 * \defgroup cmd_device_firewirecamera firewirecamera

 A streaming digital camera source, see yarp::dev::DragonflyDeviceDriver.

*/


#endif
