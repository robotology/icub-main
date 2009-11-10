// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
* Author: Alessandro Scalzo
*/

//  L      I  N   N  U   U  X   X
//  L      I  NN  N  U   U   X X
//  L      I  N N N  U   U    X
//  L      I  N  NN  U   U   X X
//  LLLLL  I  N   N   UUU   X   X

#ifndef __FIREWIRE_CAMERA_DR2_H__
#define __FIREWIRE_CAMERA_DR2_H__

#include <stdio.h>
#include <memory.h>
#include <dc1394/dc1394.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/FrameGrabberInterfaces.h>

#define NUM_DMA_BUFFERS 10

class CFWCamera_DR2_2 : public yarp::dev::IFrameGrabberControlsDC1394
{
public:   
    CFWCamera_DR2_2()
    {
    }

    virtual ~CFWCamera_DR2_2()
    {
        if (m_pCamera) Close(); 
    }

    inline int width() { return m_XDim; }
    inline int height(){ return m_YDim; }
    inline int getRawBufferSize(){ return m_RawBufferSize; }

    inline const yarp::os::Stamp& getLastInputStamp(){ return m_Stamp; }

    // formats
#define DR_UNINIT           0 // not initialized
#define DR_RGB_HALF_RES     1 // 320 x 240 RGB8
#define DR_RGB_FULL_RES     2 // 640 x 480 RGB8
#define DR_BAYER_FULL_RES   3 // 640 x 480 RAW + software Bayer decoding
    bool Create(unsigned int idCamera,unsigned int size_x,unsigned int size_y,bool bDR2,int format);

    virtual void Close();

    bool CaptureImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image)
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

protected:
    dc1394_t *m_dc1394_handle;
    dc1394camera_list_t *m_pCameraList;
    dc1394video_frame_t m_ConvFrame;

    int m_nNumCameras;
    int m_nActiveCams;

    bool m_bFrameIsValid;
    bool m_bCameraOn;

    bool m_bIsRaw,m_bIsRgb,m_bIsFormat7;
    bool m_bDR2;
    unsigned int m_RawBufferSize;
    unsigned int m_XDim,m_YDim;

    int m_IsoSpeed;

    unsigned int m_MinBPP,m_MaxBPP,m_RelBPP;
    dc1394video_mode_t m_video_mode;
    dc1394color_coding_t m_color_coding;
    uint32_t m_iMin[DC1394_FEATURE_NUM],m_iMax[DC1394_FEATURE_NUM];

    dc1394video_frame_t *m_pFrame;
    yarp::os::Semaphore m_AcqMutex;
    yarp::os::Stamp m_Stamp;

    dc1394camera_t *m_pCamera;

    virtual void LoadSettings(unsigned int size_x=0,unsigned int size_y=0);
    virtual void UpdateNonFormat7Window();
    virtual void UpdateFormat7Window(int xdim=0,int ydim=0);
    //inline void Bayer(unsigned char* dst,unsigned char* src);

    inline uint32_t NormToValue(double& dVal,int feature);
    inline double ValueToNorm(uint32_t iVal,int feature);


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

    /////////////////////////
    // format functions
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
    virtual bool getFormat7MaxWindowDC1394(unsigned int &xdim,unsigned int &ydim,unsigned int &xstep,unsigned int &ystep);

    // 26
    virtual bool setFormat7WindowDC1394(unsigned int xdim,unsigned int ydim);

    // 27
    virtual bool getFormat7WindowDC1394(unsigned int &xdim,unsigned int &ydim);	

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

    /////////////////////////
    // end format functions
    /////////////////////////

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
