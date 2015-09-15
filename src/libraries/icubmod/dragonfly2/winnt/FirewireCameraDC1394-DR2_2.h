// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

//     W   W   I   N   N
//     W   W   I   NN  N
//     W W W   I   N N N
//     WW WW   I   N  NN
//     W   W   I   N   N

#ifndef __FIREWIRE_CAMERA_DR2_H__
#define __FIREWIRE_CAMERA_DR2_H__

#include <stdio.h>
#include <memory.h>
#include <Flycapture2.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/FrameGrabberInterfaces.h>

#define DR_UNINIT           0
#define DR_RGB_HALF_RES     1
#define DR_RGB_FULL_RES     2
#define DR_BAYER_FULL_RES   3

#define NUM_DMA_BUFFERS 8


class CFWCamera_DR2_2 : public yarp::dev::IFrameGrabberControlsDC1394
{
public:   
    CFWCamera_DR2_2(bool raw) : mRawDriver(raw)
	{
	}
	
	virtual ~CFWCamera_DR2_2()
	{
		Close();
	}

	int width() { return m_XDim; }
	int height(){ return m_YDim; }

	int getRawBufferSize(){ return m_pFrame->GetDataSize(); }

	const yarp::os::Stamp& getLastInputStamp(){ return m_Stamp; }

	bool SetVideoMode(FlyCapture2::VideoMode video_mode);
	bool SetF7(int mode,int xdim,int ydim,int pixel_format,int speed,int x0,int y0);

    bool Create(yarp::os::Searchable& config);

    virtual void Close();

	bool CaptureImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image)
	{
		return Capture(&image);
	}

	bool CaptureImage(yarp::sig::ImageOf<yarp::sig::PixelMono>& image)
	{
		return Capture(&image);
	}

	bool CaptureRgb(unsigned char* pBuffer)
	{
		return Capture(0,pBuffer);
	}

	bool CaptureRaw(unsigned char* pBuffer)
	{
		return Capture(0,pBuffer,true);
	}

    bool Capture(yarp::sig::ImageOf<yarp::sig::PixelRgb>* pImage,unsigned char *pBuffer=0,bool bRaw=false);
    bool Capture(yarp::sig::ImageOf<yarp::sig::PixelMono>* pImage);

protected:
    bool mRawDriver;

	FlyCapture2::BusManager *m_pBusManager;
	FlyCapture2::Camera *m_pCamera;
	FlyCapture2::Error error;

    bool mHires;
	FlyCapture2::CameraInfo m_CameraInfo;
	int m_BusSpeedBS;
	
	FlyCapture2::FC2Config m_CamConfig;

	FlyCapture2::Format7ImageSettings m_F7ImageSettings;
	float m_F7PercentSpeed;
	unsigned int m_F7PacketSize;
	FlyCapture2::Format7Info m_F7Info;
	bool m_bTxOn;

    unsigned int m_nNumCameras;
    int m_nActiveCams;

	/////////////////////

    bool m_bFrameIsValid;
    bool m_bCameraOn;
 
	unsigned int m_XDim,m_YDim;

	unsigned int m_iMin[FlyCapture2::UNSPECIFIED_PROPERTY_TYPE],m_iMax[FlyCapture2::UNSPECIFIED_PROPERTY_TYPE];

	FlyCapture2::Image *m_pFrame,*m_pBayer;
	yarp::os::Semaphore m_AcqMutex;
	yarp::os::Stamp m_Stamp;

	int DC2Fly(int feature);

	inline unsigned int NormToValue(double& dVal,int feature);
    inline double ValueToNorm(unsigned int iVal,int feature);

    int maxFPS(FlyCapture2::Mode mode,FlyCapture2::PixelFormat pixelFormat);
    double bytesPerPixel(FlyCapture2::PixelFormat pixelFormat);

    bool manage(FlyCapture2::Error& error,yarp::os::Semaphore *pToUnlock=NULL)
    {
        if (error.GetType()!=FlyCapture2::PGRERROR_OK)
        {
            yError("%s\n",error.GetDescription());
            if (pToUnlock)
            {
                pToUnlock->post();
            }
		    return true;
	    }

        return false;
    }

    int checkInt(yarp::os::Searchable& config,const char* key)
    {
        if (config.check(key))
        {
            return config.find(key).asInt();
        }

        return 0;
    }

    double checkDouble(yarp::os::Searchable& config,const char* key)
    {
        if (config.check(key))
        {
            return config.find(key).asDouble();
        }

        return -1.0;
    }

	///////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////

public:

	////////////////////
	// feature functions
	////////////////////

	// 00
	virtual bool hasFeatureDC1394(int feature);

	// 01
	virtual bool setFeatureDC1394(int feature,double value);

	// 02
	virtual double getFeatureDC1394(int feature);

	// 03
	virtual bool hasOnOffDC1394(int feature);

	// 04
	virtual bool setActiveDC1394(int feature, bool onoff);

	// 05
	virtual bool getActiveDC1394(int feature);

	// 06
	virtual bool hasManualDC1394(int feature);

	// 07
	virtual bool hasAutoDC1394(int feature);

	// 08
	virtual bool hasOnePushDC1394(int feature);

	// 09
	virtual bool setModeDC1394(int feature, bool auto_onoff);

	// 10
	virtual bool getModeDC1394(int feature);

	// 11
	virtual bool setOnePushDC1394(int feature);

	// 23
	virtual bool setWhiteBalanceDC1394(double b, double r);

	// 24
	virtual bool getWhiteBalanceDC1394(double &b, double &r);

	/////////////////////////
	// end features functions
	/////////////////////////

	// 12
	virtual unsigned int getVideoModeMaskDC1394();

	// 13
	virtual bool setVideoModeDC1394(int video_mode);

	// 14
	virtual unsigned int getVideoModeDC1394();

	// 15
	virtual unsigned int getFPSMaskDC1394();

	// 16
	virtual unsigned int getFPSDC1394();

	// 17
	virtual bool setFPSDC1394(int fps);

	// 18
	virtual unsigned int getISOSpeedDC1394();

	// 19
	virtual bool setISOSpeedDC1394(int speed);

	// 20
	virtual unsigned int getColorCodingMaskDC1394(unsigned int video_mode);
	virtual unsigned int getActualColorCodingMaskDC1394();

	// 21
	virtual unsigned int getColorCodingDC1394();

	// 22
	virtual bool setColorCodingDC1394(int coding);	

	// 25
	virtual bool getFormat7MaxWindowDC1394(unsigned int &xdim,unsigned int &ydim,unsigned int &xstep,unsigned int &ystep,unsigned int &xoffstep,unsigned int &yoffstep);

	// 26
	virtual bool setFormat7WindowDC1394(unsigned int xdim,unsigned int ydim,int x0,int y0);

	// 27
	virtual bool getFormat7WindowDC1394(unsigned int &xdim,unsigned int &ydim,int &x0,int &y0);	
	
	// 28
	virtual bool setOperationModeDC1394(bool b1394b);

	// 29
	virtual bool getOperationModeDC1394();

	// 30
	virtual bool setTransmissionDC1394(bool bTxON);

	// 31
	virtual bool getTransmissionDC1394();

	// 32 setBayer
	// 33 getBayer

	// 34
	virtual bool setBroadcastDC1394(bool onoff);

	// 35
	virtual bool setDefaultsDC1394();

	// 36
	virtual bool setResetDC1394();

	// 37
	virtual bool setPowerDC1394(bool onoff);

	// 38
	virtual bool setCaptureDC1394(bool bON);

	// 39
	virtual unsigned int getBytesPerPacketDC1394();

	// 40
	virtual bool setBytesPerPacketDC1394(unsigned int bpp);

	///////////////////////////////////////////////
	// base class implementation
	///////////////////////////////////////////////
	
	// SET
		
	virtual bool setBrightness(double v);
    virtual bool setExposure(double v);
	virtual bool setSharpness(double v);
	virtual bool setWhiteBalance(double blue, double red);
	virtual bool setHue(double v);
	virtual bool setSaturation(double v);
	virtual bool setGamma(double v);
	virtual bool setShutter(double v);
    virtual bool setGain(double v);
    virtual bool setIris(double v);
    
    // GET

	virtual double getBrightness();
	virtual double getExposure();	
	virtual double getSharpness();
    virtual bool getWhiteBalance(double &blue, double &red);	
	virtual double getHue();	
	virtual double getSaturation();
	virtual double getGamma();
    virtual double getShutter();
    virtual double getGain();
    virtual double getIris();
};
 
#endif
