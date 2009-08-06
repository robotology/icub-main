// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "common/DragonflyDeviceDriver2.h"

#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <ace/Log_Msg.h>
#include <ace/OS.h>

using namespace yarp::os;
using namespace yarp::dev;

#include "FirewireCameraSet2.h"

class FirewireCamera2Resources
{
public:
	FirewireCamera2Resources (void)
	{
	}

	virtual ~FirewireCamera2Resources () 
	{ 
        _uninitialize();
	}
	
	// Hardware-dependant variables
	enum { _num_buffers = 3 };
		
    static CFWCameraSet2* m_pCameraSet;
    static Semaphore m_InitCloseMutex;

    //unsigned char *img;
    
    unsigned int unit_number;

	int width(){ return m_pCameraSet->width(unit_number); }
	int height(){ return m_pCameraSet->height(unit_number); }
	int getRawBufferSize(){ return m_pCameraSet->getRawBufferSize(unit_number); }

	//Semaphore mutexArray[_num_buffers];
	//Semaphore _newFrameMutex;
	//Semaphore _convImgMutex;
	
	inline bool _initialize(const Dragonfly2OpenParameters & params);
	inline bool _uninitialize(); 

    /*
    inline void _printSettings()
    {
        m_pCameraSet->PrintSettings(unit_number);
    }
    */
	
    inline bool _capture(unsigned char *buff)
    {
        return m_pCameraSet->CaptureRgb(unit_number,buff);
    }

    const yarp::os::Stamp& getLastInputStamp()
	{
	    return m_pCameraSet->getLastInputStamp(unit_number);
	}

    inline bool _capture_raw(unsigned char *buff)
    {
        return m_pCameraSet->CaptureRaw(unit_number,buff);
    }

	/////////
	// DC1394
	/////////

	// 00 01 02
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
};

CFWCameraSet2* FirewireCamera2Resources::m_pCameraSet=NULL;
Semaphore FirewireCamera2Resources::m_InitCloseMutex;

/// full initialize and startup of the grabber.
inline bool FirewireCamera2Resources::_initialize(const Dragonfly2OpenParameters& params)
{
    m_InitCloseMutex.wait();

    if (!m_pCameraSet)
    {
        m_pCameraSet=new CFWCameraSet2();
        
        if (!m_pCameraSet->Init())
        {
            delete m_pCameraSet;
            m_pCameraSet=0;
            m_InitCloseMutex.post(); 
            return false;
        }
    }

    unit_number=params._unit_number;

	if (!m_pCameraSet->StartCamera(unit_number,params._size_x,params._size_y,params._DR2,params._video_type))
	{
		ACE_OS::fprintf(stderr, "FirewireCamera2Resources: can't open camera %d",unit_number);
		m_InitCloseMutex.post();
		return false;
	}
    
    m_InitCloseMutex.post();

	// Setup Camera Parameters, Magic Numbers :-)
	
	if (params._brightness>=0.0)
    {
        setActiveDC1394(DC1394_FEATURE_BRIGHTNESS-DC1394_FEATURE_MIN,true);
        setModeDC1394(DC1394_FEATURE_BRIGHTNESS-DC1394_FEATURE_MIN,false);
        setFeatureDC1394(DC1394_FEATURE_BRIGHTNESS-DC1394_FEATURE_MIN,params._brightness);
    }	
    if (params._exposure>=0.0)
    {
        setActiveDC1394(DC1394_FEATURE_EXPOSURE-DC1394_FEATURE_MIN,true);
        setModeDC1394(DC1394_FEATURE_EXPOSURE-DC1394_FEATURE_MIN,false);
        setFeatureDC1394(DC1394_FEATURE_EXPOSURE,  params._exposure);
    }	
	if (params._sharpness>=0.0)
    {
        setActiveDC1394(DC1394_FEATURE_SHARPNESS-DC1394_FEATURE_MIN,true);
        setModeDC1394(DC1394_FEATURE_SHARPNESS-DC1394_FEATURE_MIN,false);
        setFeatureDC1394(DC1394_FEATURE_SHARPNESS-DC1394_FEATURE_MIN, params._sharpness);
    }	
	if (params._whiteB>=0.0 && params._whiteR>=0.0)
    { 
        setActiveDC1394(DC1394_FEATURE_WHITE_BALANCE-DC1394_FEATURE_MIN,true);
        setModeDC1394(DC1394_FEATURE_WHITE_BALANCE-DC1394_FEATURE_MIN,false);
        setWhiteBalanceDC1394(params._whiteB,params._whiteR);
    }
	if (params._hue>=0.0)
    {
        setActiveDC1394(DC1394_FEATURE_HUE-DC1394_FEATURE_MIN,true);
        setModeDC1394(DC1394_FEATURE_HUE-DC1394_FEATURE_MIN,false);
        setFeatureDC1394(DC1394_FEATURE_HUE-DC1394_FEATURE_MIN,params._hue);
    }
	if (params._saturation>=0.0)
    {
        setActiveDC1394(DC1394_FEATURE_SATURATION-DC1394_FEATURE_MIN,true);
        setModeDC1394(DC1394_FEATURE_SATURATION-DC1394_FEATURE_MIN,false);
        setFeatureDC1394(DC1394_FEATURE_SATURATION-DC1394_FEATURE_MIN,params._saturation);
    }
	if (params._gamma>=0.0)
    {
        setActiveDC1394(DC1394_FEATURE_GAMMA-DC1394_FEATURE_MIN,true);
        setModeDC1394(DC1394_FEATURE_GAMMA-DC1394_FEATURE_MIN,false);
        setFeatureDC1394(DC1394_FEATURE_GAMMA-DC1394_FEATURE_MIN,params._gamma);
    }   
	if (params._shutter>=0.0)
    {
        setActiveDC1394(DC1394_FEATURE_SHUTTER-DC1394_FEATURE_MIN,true);
        setModeDC1394(DC1394_FEATURE_SHUTTER-DC1394_FEATURE_MIN,false);
        setFeatureDC1394(DC1394_FEATURE_SHUTTER-DC1394_FEATURE_MIN,params._shutter);
    }
	if (params._gain>=0.0) 
    {
        setActiveDC1394(DC1394_FEATURE_GAIN-DC1394_FEATURE_MIN,true);
        setModeDC1394(DC1394_FEATURE_GAIN-DC1394_FEATURE_MIN,false);
        setFeatureDC1394(DC1394_FEATURE_GAIN-DC1394_FEATURE_MIN,params._gain);
    }
	
    return true;
}

inline bool FirewireCamera2Resources::_uninitialize()
{
    m_InitCloseMutex.wait();
    
    if (m_pCameraSet)
    {
        m_pCameraSet->ShutdownCamera(unit_number);
    
        if (m_pCameraSet->GetCameraNum()<=0)
        {
            m_pCameraSet->Close();
            delete m_pCameraSet;
            m_pCameraSet=0;
        }
    }
    
    m_InitCloseMutex.post();
    
    return true;
}

//////////////////////////////////////////////
inline FirewireCamera2Resources& RES(void *res) { return *(FirewireCamera2Resources *)res; }
//////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


DragonflyDeviceDriver2::DragonflyDeviceDriver2(void)
{
	system_resources=(void *)new FirewireCamera2Resources;
	ACE_ASSERT(system_resources!=NULL);
}

DragonflyDeviceDriver2::~DragonflyDeviceDriver2()
{
	if (system_resources != NULL)
		delete (FirewireCamera2Resources *)system_resources;
	
	system_resources=NULL;
}

///
bool DragonflyDeviceDriver2::open (const Dragonfly2OpenParameters &par)
{
	FirewireCamera2Resources& d=RES(system_resources);
	
	return d._initialize(par);
}

bool DragonflyDeviceDriver2::close (void)
{
	FirewireCamera2Resources& d=RES(system_resources);

	return d._uninitialize();
}

bool DragonflyDeviceDriver2::getRawBuffer(unsigned char *buff)
{
    FirewireCamera2Resources& d = RES(system_resources);

    return d._capture_raw(buff);
}

bool DragonflyDeviceDriver2::getRgbBuffer(unsigned char *buff)
{
    FirewireCamera2Resources& d=RES(system_resources);

    return d._capture(buff);
}

bool DragonflyDeviceDriver2::getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image) 
{
    FirewireCamera2Resources& d = RES(system_resources);
    image.resize(d.width(),d.height());
    d._capture(image.getRawImage());
    return true;
}

yarp::os::Stamp DragonflyDeviceDriver2::getLastInputStamp()
{
	return RES(system_resources).getLastInputStamp();
}

int DragonflyDeviceDriver2::getRawBufferSize()
{
    return RES(system_resources).getRawBufferSize();
}

int DragonflyDeviceDriver2::width () const
{
	return RES(system_resources).width();
}

int DragonflyDeviceDriver2::height () const
{
	return RES(system_resources).height();
}

// SET

bool DragonflyDeviceDriver2::setBrightness(double value)
{
	return RES(system_resources).setFeatureDC1394(DC1394_FEATURE_BRIGHTNESS,value);
}
bool DragonflyDeviceDriver2::setExposure(double value)
{
	return RES(system_resources).setFeatureDC1394(DC1394_FEATURE_EXPOSURE,value);
}
bool DragonflyDeviceDriver2::setSharpness(double value)
{
	return RES(system_resources).setFeatureDC1394(DC1394_FEATURE_SHARPNESS,value);
}
bool DragonflyDeviceDriver2::setWhiteBalance(double blue, double red)
{
	return RES(system_resources).setWhiteBalanceDC1394(blue,red);
}
bool DragonflyDeviceDriver2::setHue(double value)
{
	return RES(system_resources).setFeatureDC1394(DC1394_FEATURE_HUE,value);
}
bool DragonflyDeviceDriver2::setSaturation(double value)
{
	return RES(system_resources).setFeatureDC1394(DC1394_FEATURE_SATURATION,value);
}
bool DragonflyDeviceDriver2::setGamma(double value)
{
	return RES(system_resources).setFeatureDC1394(DC1394_FEATURE_GAMMA,value);
}
bool DragonflyDeviceDriver2::setShutter(double value)
{
	return RES(system_resources).setFeatureDC1394(DC1394_FEATURE_SHUTTER,value);
}
bool DragonflyDeviceDriver2::setGain(double value)
{
	return RES(system_resources).setFeatureDC1394(DC1394_FEATURE_GAIN,value);
}
bool DragonflyDeviceDriver2::setIris(double value)
{
	return RES(system_resources).setFeatureDC1394(DC1394_FEATURE_IRIS,value);
}

// GET

double DragonflyDeviceDriver2::getBrightness()
{
	return RES(system_resources).getFeatureDC1394(DC1394_FEATURE_BRIGHTNESS);
}
double DragonflyDeviceDriver2::getExposure()
{
	return RES(system_resources).getFeatureDC1394(DC1394_FEATURE_EXPOSURE);
}
double DragonflyDeviceDriver2::getSharpness()
{
	return RES(system_resources).getFeatureDC1394(DC1394_FEATURE_SHARPNESS);
}
bool DragonflyDeviceDriver2::getWhiteBalance(double &blue, double &red)
{
	return RES(system_resources).getWhiteBalanceDC1394(blue,red);
}
double DragonflyDeviceDriver2::getHue()
{
	return RES(system_resources).getFeatureDC1394(DC1394_FEATURE_HUE);
}
double DragonflyDeviceDriver2::getSaturation()
{
	return RES(system_resources).getFeatureDC1394(DC1394_FEATURE_SATURATION);
}
double DragonflyDeviceDriver2::getGamma()
{
	return RES(system_resources).getFeatureDC1394(DC1394_FEATURE_GAMMA);
}
double DragonflyDeviceDriver2::getShutter()
{
	return RES(system_resources).getFeatureDC1394(DC1394_FEATURE_SHUTTER);
}
double DragonflyDeviceDriver2::getGain()
{
	return RES(system_resources).getFeatureDC1394(DC1394_FEATURE_GAIN);
}
double DragonflyDeviceDriver2::getIris()
{
	return RES(system_resources).getFeatureDC1394(DC1394_FEATURE_IRIS);
}

/////////
// DC1394
/////////

///////////////////////////////////////////////////////////////////////
bool DragonflyDeviceDriver2::hasFeatureDC1394(int feature)
{
	return RES(system_resources).hasFeatureDC1394(feature);
}
bool DragonflyDeviceDriver2::setFeatureDC1394(int feature,double value)
{
	return RES(system_resources).setFeatureDC1394(feature,value);
}
double DragonflyDeviceDriver2::getFeatureDC1394(int feature)
{
	return RES(system_resources).getFeatureDC1394(feature);
}
bool DragonflyDeviceDriver2::hasOnOffDC1394(int feature)
{
	return RES(system_resources).hasOnOffDC1394(feature);
}
bool DragonflyDeviceDriver2::setActiveDC1394(int feature, bool onoff)
{
	return RES(system_resources).setActiveDC1394(feature,onoff);
}
bool DragonflyDeviceDriver2::getActiveDC1394(int feature)
{
	return RES(system_resources).getActiveDC1394(feature);
}
bool DragonflyDeviceDriver2::hasAutoDC1394(int feature)
{
	return RES(system_resources).hasAutoDC1394(feature);
}
bool DragonflyDeviceDriver2::hasManualDC1394(int feature)
{
	return RES(system_resources).hasManualDC1394(feature);
}
bool DragonflyDeviceDriver2::hasOnePushDC1394(int feature)
{
	return RES(system_resources).hasOnePushDC1394(feature);
}
bool DragonflyDeviceDriver2::setModeDC1394(int feature, bool auto_onoff)
{
	return RES(system_resources).setModeDC1394(feature,auto_onoff);
}
bool DragonflyDeviceDriver2::getModeDC1394(int feature)
{
	return RES(system_resources).getModeDC1394(feature);
}
bool DragonflyDeviceDriver2::setOnePushDC1394(int feature)
{
	return RES(system_resources).setOnePushDC1394(feature);
}
unsigned int DragonflyDeviceDriver2::getVideoModeMaskDC1394()
{
	return RES(system_resources).getVideoModeMaskDC1394();
}
bool DragonflyDeviceDriver2::setVideoModeDC1394(int video_mode)
{
	return RES(system_resources).setVideoModeDC1394(video_mode);
}
unsigned int DragonflyDeviceDriver2::getVideoModeDC1394()
{
	return RES(system_resources).getVideoModeDC1394();
}
unsigned int DragonflyDeviceDriver2::getFPSMaskDC1394()
{
	return RES(system_resources).getFPSMaskDC1394();
}
unsigned int DragonflyDeviceDriver2::getFPSDC1394()
{
	return RES(system_resources).getFPSDC1394();
}
bool DragonflyDeviceDriver2::setFPSDC1394(int fps)
{
	return RES(system_resources).setFPSDC1394(fps);
}
unsigned int DragonflyDeviceDriver2::getISOSpeedDC1394()
{
	return RES(system_resources).getISOSpeedDC1394();
}
bool DragonflyDeviceDriver2::setISOSpeedDC1394(int speed)
{
	return RES(system_resources).setISOSpeedDC1394(speed);
}
unsigned int DragonflyDeviceDriver2::getColorCodingMaskDC1394(unsigned int video_mode)
{
	return RES(system_resources).getColorCodingMaskDC1394(video_mode);
}
unsigned int DragonflyDeviceDriver2::getColorCodingDC1394()
{
	return RES(system_resources).getColorCodingDC1394();
}
bool DragonflyDeviceDriver2::setColorCodingDC1394(int coding)
{
	return RES(system_resources).setColorCodingDC1394(coding);
}
bool DragonflyDeviceDriver2::setWhiteBalanceDC1394(double b, double r)
{
	return RES(system_resources).setWhiteBalanceDC1394(b,r);
}
bool DragonflyDeviceDriver2::getWhiteBalanceDC1394(double &b, double &r)
{
	return RES(system_resources).getWhiteBalanceDC1394(b,r);
}
bool DragonflyDeviceDriver2::getFormat7MaxWindowDC1394(unsigned int &xdim,unsigned int &ydim,unsigned int &xstep,unsigned int &ystep)
{
	return RES(system_resources).getFormat7MaxWindowDC1394(xdim,ydim,xstep,ystep);
}
bool DragonflyDeviceDriver2::setFormat7WindowDC1394(unsigned int xdim,unsigned int ydim)
{
	return RES(system_resources).setFormat7WindowDC1394(xdim,ydim);
}
bool DragonflyDeviceDriver2::getFormat7WindowDC1394(unsigned int &xdim,unsigned int &ydim)
{
	return RES(system_resources).getFormat7WindowDC1394(xdim,ydim);
}
bool DragonflyDeviceDriver2::setOperationModeDC1394(bool b1394b)
{
	return RES(system_resources).setOperationModeDC1394(b1394b);
}
bool DragonflyDeviceDriver2::getOperationModeDC1394()
{
	return RES(system_resources).getOperationModeDC1394();
}
bool DragonflyDeviceDriver2::setTransmissionDC1394(bool bTxON)
{
	return RES(system_resources).setTransmissionDC1394(bTxON);
}
bool DragonflyDeviceDriver2::getTransmissionDC1394()
{
	return RES(system_resources).getTransmissionDC1394();
}
/*
bool DragonflyDeviceDriver2::setBayerDC1394(bool bON)
{
	return RES(system_resources).setBayerDC1394(bON);
}
bool DragonflyDeviceDriver2::getBayerDC1394()
{
	return RES(system_resources).getBayerDC1394();
}
*/
bool DragonflyDeviceDriver2::setBroadcastDC1394(bool onoff)
{
	return RES(system_resources).setBroadcastDC1394(onoff);
}
bool DragonflyDeviceDriver2::setDefaultsDC1394()
{
	return RES(system_resources).setDefaultsDC1394();
}
bool DragonflyDeviceDriver2::setResetDC1394()
{
	return RES(system_resources).setResetDC1394();
}
bool DragonflyDeviceDriver2::setPowerDC1394(bool onoff)
{
	return RES(system_resources).setPowerDC1394(onoff);
}
bool DragonflyDeviceDriver2::setCaptureDC1394(bool bON)
{
	return RES(system_resources).setCaptureDC1394(bON);
}
bool DragonflyDeviceDriver2::setBytesPerPacketDC1394(unsigned int bpp)
{
	return RES(system_resources).setBytesPerPacketDC1394(bpp);
}
unsigned int DragonflyDeviceDriver2::getBytesPerPacketDC1394()
{
	return RES(system_resources).getBytesPerPacketDC1394();
}


///////////////////////////////////////////////////////////////////////

bool FirewireCamera2Resources::hasFeatureDC1394(int feature)
{
    //printf("FirewireCamera2Resources::hasFeatureDC1394(%d)\n",feature);
	return m_pCameraSet->getControls(unit_number)->hasFeatureDC1394(feature);
}
bool FirewireCamera2Resources::setFeatureDC1394(int feature,double value)
{
	return m_pCameraSet->getControls(unit_number)->setFeatureDC1394(feature,value);
}
double FirewireCamera2Resources::getFeatureDC1394(int feature)
{
	return m_pCameraSet->getControls(unit_number)->getFeatureDC1394(feature);
}
bool FirewireCamera2Resources::hasOnOffDC1394(int feature)
{
	return m_pCameraSet->getControls(unit_number)->hasOnOffDC1394(feature);
}
bool FirewireCamera2Resources::setActiveDC1394(int feature, bool onoff)
{
	return m_pCameraSet->getControls(unit_number)->setActiveDC1394(feature,onoff);
}
bool FirewireCamera2Resources::getActiveDC1394(int feature)
{
	return m_pCameraSet->getControls(unit_number)->getActiveDC1394(feature);
}
bool FirewireCamera2Resources::hasAutoDC1394(int feature)
{
	return m_pCameraSet->getControls(unit_number)->hasAutoDC1394(feature);
}
bool FirewireCamera2Resources::hasManualDC1394(int feature)
{
	return m_pCameraSet->getControls(unit_number)->hasManualDC1394(feature);
}
bool FirewireCamera2Resources::hasOnePushDC1394(int feature)
{
	return m_pCameraSet->getControls(unit_number)->hasOnePushDC1394(feature);
}
bool FirewireCamera2Resources::setModeDC1394(int feature, bool auto_onoff)
{
	return m_pCameraSet->getControls(unit_number)->setModeDC1394(feature,auto_onoff);
}
bool FirewireCamera2Resources::getModeDC1394(int feature)
{
	return m_pCameraSet->getControls(unit_number)->getModeDC1394(feature);
}
bool FirewireCamera2Resources::setOnePushDC1394(int feature)
{
	return m_pCameraSet->getControls(unit_number)->setOnePushDC1394(feature);
}
unsigned int FirewireCamera2Resources::getVideoModeMaskDC1394()
{
	return m_pCameraSet->getControls(unit_number)->getVideoModeMaskDC1394();
}
bool FirewireCamera2Resources::setVideoModeDC1394(int video_mode)
{
	return m_pCameraSet->getControls(unit_number)->setVideoModeDC1394(video_mode);
}
unsigned int FirewireCamera2Resources::getVideoModeDC1394()
{
	return m_pCameraSet->getControls(unit_number)->getVideoModeDC1394();
}
unsigned int FirewireCamera2Resources::getFPSMaskDC1394()
{
	return m_pCameraSet->getControls(unit_number)->getFPSMaskDC1394();
}
unsigned int FirewireCamera2Resources::getFPSDC1394()
{
	return m_pCameraSet->getControls(unit_number)->getFPSDC1394();
}
bool FirewireCamera2Resources::setFPSDC1394(int fps)
{
	return m_pCameraSet->getControls(unit_number)->setFPSDC1394(fps);
}
unsigned int FirewireCamera2Resources::getISOSpeedDC1394()
{
	return m_pCameraSet->getControls(unit_number)->getISOSpeedDC1394();
}
bool FirewireCamera2Resources::setISOSpeedDC1394(int speed)
{
	return m_pCameraSet->getControls(unit_number)->setISOSpeedDC1394(speed);
}
unsigned int FirewireCamera2Resources::getColorCodingMaskDC1394(unsigned int video_mode)
{
	return m_pCameraSet->getControls(unit_number)->getColorCodingMaskDC1394(video_mode);
}
unsigned int FirewireCamera2Resources::getColorCodingDC1394()
{
	return m_pCameraSet->getControls(unit_number)->getColorCodingDC1394();
}
bool FirewireCamera2Resources::setColorCodingDC1394(int coding)
{
	return m_pCameraSet->getControls(unit_number)->setColorCodingDC1394(coding);
}
bool FirewireCamera2Resources::setWhiteBalanceDC1394(double b, double r)
{
	return m_pCameraSet->getControls(unit_number)->setWhiteBalanceDC1394(b,r);
}
bool FirewireCamera2Resources::getWhiteBalanceDC1394(double &b, double &r)
{
	return m_pCameraSet->getControls(unit_number)->getWhiteBalanceDC1394(b,r);
}
bool FirewireCamera2Resources::getFormat7MaxWindowDC1394(unsigned int &xdim,unsigned int &ydim,unsigned int &xstep,unsigned int &ystep)
{
	return m_pCameraSet->getControls(unit_number)->getFormat7MaxWindowDC1394(xdim,ydim,xstep,ystep);
}
bool FirewireCamera2Resources::setFormat7WindowDC1394(unsigned int xdim,unsigned int ydim)
{
	return m_pCameraSet->getControls(unit_number)->setFormat7WindowDC1394(xdim,ydim);
}
bool FirewireCamera2Resources::getFormat7WindowDC1394(unsigned int &xdim,unsigned int &ydim)
{
	return m_pCameraSet->getControls(unit_number)->getFormat7WindowDC1394(xdim,ydim);
}
bool FirewireCamera2Resources::setOperationModeDC1394(bool b1394b)
{
	return m_pCameraSet->getControls(unit_number)->setOperationModeDC1394(b1394b);
}
bool FirewireCamera2Resources::getOperationModeDC1394()
{
	return m_pCameraSet->getControls(unit_number)->getOperationModeDC1394();
}
bool FirewireCamera2Resources::setTransmissionDC1394(bool bTxON)
{
	return m_pCameraSet->getControls(unit_number)->setTransmissionDC1394(bTxON);
}
bool FirewireCamera2Resources::getTransmissionDC1394()
{
	return m_pCameraSet->getControls(unit_number)->getTransmissionDC1394();
}
/*
bool FirewireCamera2Resources::setBayerDC1394(bool bON)
{
	return m_pCameraSet->getControls(unit_number)->setBayerDC1394(bON);
}
bool FirewireCamera2Resources::getBayerDC1394()
{
	return m_pCameraSet->getControls(unit_number)->getBayerDC1394();
}
*/
bool FirewireCamera2Resources::setBroadcastDC1394(bool onoff)
{
	return m_pCameraSet->getControls(unit_number)->setBroadcastDC1394(onoff);
}
bool FirewireCamera2Resources::setDefaultsDC1394()
{
	return m_pCameraSet->getControls(unit_number)->setDefaultsDC1394();
}
bool FirewireCamera2Resources::setResetDC1394()
{
	return m_pCameraSet->getControls(unit_number)->setResetDC1394();
}
bool FirewireCamera2Resources::setPowerDC1394(bool onoff)
{
	return m_pCameraSet->getControls(unit_number)->setPowerDC1394(onoff);
}
bool FirewireCamera2Resources::setCaptureDC1394(bool bON)
{
	return m_pCameraSet->getControls(unit_number)->setCaptureDC1394(bON);
}
bool FirewireCamera2Resources::setBytesPerPacketDC1394(unsigned int bpp)
{
	return m_pCameraSet->getControls(unit_number)->setBytesPerPacketDC1394(bpp);
}
unsigned int FirewireCamera2Resources::getBytesPerPacketDC1394()
{
	return m_pCameraSet->getControls(unit_number)->getBytesPerPacketDC1394();
}
