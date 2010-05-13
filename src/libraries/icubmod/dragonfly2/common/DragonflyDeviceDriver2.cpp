// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006 Robotcub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Authors: Paul Fitzpatrick, Alessandro Scalzo
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

#include "FirewireCameraDC1394-DR2_2.h"

////////////////////////////////////////////////////////////////////////
inline CFWCamera_DR2_2* RES(void *res) { return (CFWCamera_DR2_2*)res; }
////////////////////////////////////////////////////////////////////////

DragonflyDeviceDriver2::DragonflyDeviceDriver2(void)
{
	system_resources=(void*)new CFWCamera_DR2_2;
	ACE_ASSERT(system_resources!=NULL);
}

DragonflyDeviceDriver2::~DragonflyDeviceDriver2()
{
	if (system_resources)
	{
		delete (CFWCamera_DR2_2*)system_resources;
	    system_resources=NULL;
	}
}

bool DragonflyDeviceDriver2::open(yarp::os::Searchable& config)
{
	if (!RES(system_resources)->Create(config))
	{
		fprintf(stderr,"DragonflyDeviceDriver2: can't open camera\n");
		return false;
	}
	
    return true;
}

bool DragonflyDeviceDriver2::close(void)
{
	if (system_resources)
	{
	    RES(system_resources)->Close();
		return true;
	}

	return false;
}

bool DragonflyDeviceDriver2::getRawBuffer(unsigned char *buff)
{
    return RES(system_resources)->CaptureRaw(buff);
}

bool DragonflyDeviceDriver2::getRgbBuffer(unsigned char *buff)
{
    return RES(system_resources)->CaptureRgb(buff);
}

bool DragonflyDeviceDriver2::getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image) 
{
    return RES(system_resources)->CaptureImage(image);
}

yarp::os::Stamp DragonflyDeviceDriver2::getLastInputStamp()
{
	return RES(system_resources)->getLastInputStamp();
}

int DragonflyDeviceDriver2::getRawBufferSize()
{
    return RES(system_resources)->getRawBufferSize();
}

int DragonflyDeviceDriver2::width () const
{
	return RES(system_resources)->width();
}

int DragonflyDeviceDriver2::height () const
{
	return RES(system_resources)->height();
}

// SET

bool DragonflyDeviceDriver2::setBrightness(double value)
{
	return RES(system_resources)->setBrightness(value);
}
bool DragonflyDeviceDriver2::setExposure(double value)
{
    return RES(system_resources)->setExposure(value);
}
bool DragonflyDeviceDriver2::setSharpness(double value)
{
	return RES(system_resources)->setSharpness(value);
}
bool DragonflyDeviceDriver2::setWhiteBalance(double blue, double red)
{
	return RES(system_resources)->setWhiteBalance(blue,red);
}
bool DragonflyDeviceDriver2::setHue(double value)
{
	return RES(system_resources)->setHue(value);
}
bool DragonflyDeviceDriver2::setSaturation(double value)
{
	return RES(system_resources)->setSaturation(value);
}
bool DragonflyDeviceDriver2::setGamma(double value)
{
	return RES(system_resources)->setGamma(value);
}
bool DragonflyDeviceDriver2::setShutter(double value)
{
	return RES(system_resources)->setShutter(value);
}
bool DragonflyDeviceDriver2::setGain(double value)
{
	return RES(system_resources)->setGain(value);
}
bool DragonflyDeviceDriver2::setIris(double value)
{
	return RES(system_resources)->setIris(value);
}

// GET

double DragonflyDeviceDriver2::getBrightness()
{
	return RES(system_resources)->getBrightness();
}
double DragonflyDeviceDriver2::getExposure()
{
	return RES(system_resources)->getExposure();
}
double DragonflyDeviceDriver2::getSharpness()
{
	return RES(system_resources)->getSharpness();
}
bool DragonflyDeviceDriver2::getWhiteBalance(double &blue, double &red)
{
	return RES(system_resources)->getWhiteBalance(blue,red);
}
double DragonflyDeviceDriver2::getHue()
{
	return RES(system_resources)->getHue();
}
double DragonflyDeviceDriver2::getSaturation()
{
	return RES(system_resources)->getSaturation();
}
double DragonflyDeviceDriver2::getGamma()
{
	return RES(system_resources)->getGamma();
}
double DragonflyDeviceDriver2::getShutter()
{
	return RES(system_resources)->getShutter();
}
double DragonflyDeviceDriver2::getGain()
{
	return RES(system_resources)->getGain();
}
double DragonflyDeviceDriver2::getIris()
{
	return RES(system_resources)->getIris();
}

/////////
// DC1394
/////////

///////////////////////////////////////////////////////////////////////
bool DragonflyDeviceDriver2::hasFeatureDC1394(int feature)
{
	return RES(system_resources)->hasFeatureDC1394(feature);
}
bool DragonflyDeviceDriver2::setFeatureDC1394(int feature,double value)
{
	return RES(system_resources)->setFeatureDC1394(feature,value);
}
double DragonflyDeviceDriver2::getFeatureDC1394(int feature)
{
	return RES(system_resources)->getFeatureDC1394(feature);
}
bool DragonflyDeviceDriver2::hasOnOffDC1394(int feature)
{
	return RES(system_resources)->hasOnOffDC1394(feature);
}
bool DragonflyDeviceDriver2::setActiveDC1394(int feature, bool onoff)
{
	return RES(system_resources)->setActiveDC1394(feature,onoff);
}
bool DragonflyDeviceDriver2::getActiveDC1394(int feature)
{
	return RES(system_resources)->getActiveDC1394(feature);
}
bool DragonflyDeviceDriver2::hasAutoDC1394(int feature)
{
	return RES(system_resources)->hasAutoDC1394(feature);
}
bool DragonflyDeviceDriver2::hasManualDC1394(int feature)
{
	return RES(system_resources)->hasManualDC1394(feature);
}
bool DragonflyDeviceDriver2::hasOnePushDC1394(int feature)
{
	return RES(system_resources)->hasOnePushDC1394(feature);
}
bool DragonflyDeviceDriver2::setModeDC1394(int feature, bool auto_onoff)
{
	return RES(system_resources)->setModeDC1394(feature,auto_onoff);
}
bool DragonflyDeviceDriver2::getModeDC1394(int feature)
{
	return RES(system_resources)->getModeDC1394(feature);
}
bool DragonflyDeviceDriver2::setOnePushDC1394(int feature)
{
	return RES(system_resources)->setOnePushDC1394(feature);
}
unsigned int DragonflyDeviceDriver2::getVideoModeMaskDC1394()
{
	return RES(system_resources)->getVideoModeMaskDC1394();
}
bool DragonflyDeviceDriver2::setVideoModeDC1394(int video_mode)
{
	return RES(system_resources)->setVideoModeDC1394(video_mode);
}
unsigned int DragonflyDeviceDriver2::getVideoModeDC1394()
{
	return RES(system_resources)->getVideoModeDC1394();
}
unsigned int DragonflyDeviceDriver2::getFPSMaskDC1394()
{
	return RES(system_resources)->getFPSMaskDC1394();
}
unsigned int DragonflyDeviceDriver2::getFPSDC1394()
{
	return RES(system_resources)->getFPSDC1394();
}
bool DragonflyDeviceDriver2::setFPSDC1394(int fps)
{
	return RES(system_resources)->setFPSDC1394(fps);
}
unsigned int DragonflyDeviceDriver2::getISOSpeedDC1394()
{
	return RES(system_resources)->getISOSpeedDC1394();
}
bool DragonflyDeviceDriver2::setISOSpeedDC1394(int speed)
{
	return RES(system_resources)->setISOSpeedDC1394(speed);
}
unsigned int DragonflyDeviceDriver2::getColorCodingMaskDC1394(unsigned int video_mode)
{
	return RES(system_resources)->getColorCodingMaskDC1394(video_mode);
}
unsigned int DragonflyDeviceDriver2::getColorCodingDC1394()
{
	return RES(system_resources)->getColorCodingDC1394();
}
bool DragonflyDeviceDriver2::setColorCodingDC1394(int coding)
{
	return RES(system_resources)->setColorCodingDC1394(coding);
}
bool DragonflyDeviceDriver2::setWhiteBalanceDC1394(double b, double r)
{
	return RES(system_resources)->setWhiteBalanceDC1394(b,r);
}
bool DragonflyDeviceDriver2::getWhiteBalanceDC1394(double &b, double &r)
{
	return RES(system_resources)->getWhiteBalanceDC1394(b,r);
}
bool DragonflyDeviceDriver2::getFormat7MaxWindowDC1394(unsigned int &xdim,unsigned int &ydim,unsigned int &xstep,unsigned int &ystep)
{
	return RES(system_resources)->getFormat7MaxWindowDC1394(xdim,ydim,xstep,ystep);
}
bool DragonflyDeviceDriver2::setFormat7WindowDC1394(unsigned int xdim,unsigned int ydim)
{
	return RES(system_resources)->setFormat7WindowDC1394(xdim,ydim);
}
bool DragonflyDeviceDriver2::getFormat7WindowDC1394(unsigned int &xdim,unsigned int &ydim)
{
	return RES(system_resources)->getFormat7WindowDC1394(xdim,ydim);
}
bool DragonflyDeviceDriver2::setOperationModeDC1394(bool b1394b)
{
	return RES(system_resources)->setOperationModeDC1394(b1394b);
}
bool DragonflyDeviceDriver2::getOperationModeDC1394()
{
	return RES(system_resources)->getOperationModeDC1394();
}
bool DragonflyDeviceDriver2::setTransmissionDC1394(bool bTxON)
{
	return RES(system_resources)->setTransmissionDC1394(bTxON);
}
bool DragonflyDeviceDriver2::getTransmissionDC1394()
{
	return RES(system_resources)->getTransmissionDC1394();
}
/*
bool DragonflyDeviceDriver2::setBayerDC1394(bool bON)
{
	return RES(system_resources)->setBayerDC1394(bON);
}
bool DragonflyDeviceDriver2::getBayerDC1394()
{
	return RES(system_resources)->getBayerDC1394();
}
*/
bool DragonflyDeviceDriver2::setBroadcastDC1394(bool onoff)
{
	return RES(system_resources)->setBroadcastDC1394(onoff);
}
bool DragonflyDeviceDriver2::setDefaultsDC1394()
{
	return RES(system_resources)->setDefaultsDC1394();
}
bool DragonflyDeviceDriver2::setResetDC1394()
{
	return RES(system_resources)->setResetDC1394();
}
bool DragonflyDeviceDriver2::setPowerDC1394(bool onoff)
{
	return RES(system_resources)->setPowerDC1394(onoff);
}
bool DragonflyDeviceDriver2::setCaptureDC1394(bool bON)
{
	return RES(system_resources)->setCaptureDC1394(bON);
}
bool DragonflyDeviceDriver2::setBytesPerPacketDC1394(unsigned int bpp)
{
	return RES(system_resources)->setBytesPerPacketDC1394(bpp);
}
unsigned int DragonflyDeviceDriver2::getBytesPerPacketDC1394()
{
	return RES(system_resources)->getBytesPerPacketDC1394();
}

