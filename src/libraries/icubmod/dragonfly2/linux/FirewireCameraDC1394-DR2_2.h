// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

//  L      I  N   N  U   U  X   X
//  L      I  NN  N  U   U   X X
//  L      I  N N N  U   U    X
//  L      I  N  NN  U   U   X X
//  LLLLL  I  N   N   UUU   X   X

// Added modes 3,4,5 by Alexis Maldonado and Federico Ruiz Ugalde

#ifndef __FIREWIRE_CAMERA_DR2_H__
#define __FIREWIRE_CAMERA_DR2_H__

#include <stdio.h>
#include <memory.h>
#include <mutex>
#include <dc1394/dc1394.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>
#include <yarp/dev/IRgbVisualParams.h>
#include <yarp/dev/IFrameGrabberControlsDC1394.h>
#include <yarp/dev/IFrameGrabberControls.h>

#define NUM_DMA_BUFFERS 4

// formats

#define DR_UNINIT                0
#define DR_RGB_320x240           1
#define DR_RGB_640x480           2
#define DR_BAYER_640x480         3
#define DR_BAYER16_640x480       4
#define DR_YUV_640x480           5

//#define DR_YUV_320x240
#define DR_RGB_512x384           6
#define DR_RGB_800x600           7
#define DR_YUV_800x600           8
#define DR_RGB_1024x768          9
#define DR_YUV_1024x768          10
#define DR_BAYER_1024x768        11

class CFWCamera_DR2_2 : public yarp::dev::IFrameGrabberControlsDC1394,
                        public yarp::dev::IRgbVisualParams
{
public:
    CFWCamera_DR2_2(bool raw);

    virtual ~CFWCamera_DR2_2()
    {
        if (m_pCamera) Close();
    }

    int width();
    int height();
    int getRawBufferSize();

    const yarp::os::Stamp& getLastInputStamp();

    bool Create(yarp::os::Searchable& config);

    virtual void Close();

    bool CaptureImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image);

    bool CaptureImage(yarp::sig::ImageOf<yarp::sig::PixelMono>& image);

    bool CaptureRgb(unsigned char* pBuffer);

    bool CaptureRaw(unsigned char* pBuffer);

    bool SetVideoMode(dc1394video_mode_t videoMode);
    bool SetF7(int newVideoMode,int newXdim,int newYdim,int newColorCoding,int newSpeed,int x0,int y0);

    bool Capture(yarp::sig::ImageOf<yarp::sig::PixelRgb>* pImage,unsigned char *pBuffer=0,bool bRaw=false);
    bool Capture(yarp::sig::ImageOf<yarp::sig::PixelMono>* pImage);

protected:
    bool mRawDriver;

    dc1394_t *m_dc1394_handle;
    dc1394camera_list_t *m_pCameraList;
    dc1394video_frame_t m_ConvFrame;
    dc1394video_frame_t m_ConvFrame_tmp;

    int m_nNumCameras;
    int m_nActiveCams;

    int m_nInvalidFrames;
    bool m_bCameraOn;
    bool mHires;
    bool mUseHardwareTimestamp;

    unsigned int m_RawBufferSize;
    unsigned int m_XDim,m_YDim;
    int m_Framerate;

    unsigned int m_GainSaveValue,m_ShutterSaveValue;
    dc1394feature_mode_t m_GainSaveModeAuto,m_ShutterSaveModeAuto;

    uint32_t m_iMin[DC1394_FEATURE_NUM],m_iMax[DC1394_FEATURE_NUM];

    dc1394video_frame_t *m_pFrame,*m_pFramePoll;
    std::mutex m_AcqMutex;
    yarp::os::Stamp m_Stamp;
    int m_LastSecond;
    double m_SecondOffset;

    dc1394camera_t *m_pCamera;

    double          horizontalFov;
    double          verticalFov;
    yarp::os::Property intrinsic;
    bool configFx,configFy;
    bool configPPx,configPPy;
    bool configRet,configDistM;
    bool configIntrins;

    inline uint32_t NormToValue(double& dVal,int feature);
    inline double ValueToNorm(uint32_t iVal,int feature);

    int TRANSL(yarp::dev::cameraFeature_id_t feature);

    bool manage(dc1394error_t error);

    int checkInt(yarp::os::Searchable& config,const char* key);

    double checkDouble(yarp::os::Searchable& config,const char* key);

    int maxFPS(dc1394video_mode_t mode,dc1394color_coding_t pixelFormat);
    double bytesPerPixel(dc1394color_coding_t pixelFormat);

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
    virtual bool setFeatureDC1394(int feature, double value);

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
    virtual yarp::dev::ReturnValue getVideoModeMaskDC1394(unsigned int& val);

    // 13
    virtual yarp::dev::ReturnValue setVideoModeDC1394(int video_mode);

    // 14
    virtual yarp::dev::ReturnValue getVideoModeDC1394(unsigned int& val);

    // 15
    virtual yarp::dev::ReturnValue getFPSMaskDC1394(unsigned int& val);

    // 16
    virtual yarp::dev::ReturnValue getFPSDC1394(unsigned int& val);

    // 17
    virtual yarp::dev::ReturnValue setFPSDC1394(int fps);

    // 18
    virtual yarp::dev::ReturnValue getISOSpeedDC1394(unsigned int& val);

    // 19
    virtual yarp::dev::ReturnValue setISOSpeedDC1394(int speed);

    // 20
    virtual yarp::dev::ReturnValue getColorCodingMaskDC1394(unsigned int video_mode,unsigned int& val);
    virtual unsigned int getActualColorCodingMaskDC1394();

    // 21
    virtual yarp::dev::ReturnValue getColorCodingDC1394(unsigned int& val);

    // 22
    virtual yarp::dev::ReturnValue setColorCodingDC1394(int coding);

    // 25
    virtual yarp::dev::ReturnValue getFormat7MaxWindowDC1394(unsigned int &xdim,unsigned int &ydim,unsigned int &xstep,unsigned int &ystep,unsigned int &xoffstep,unsigned int &yoffstep);

    // 26
    virtual yarp::dev::ReturnValue setFormat7WindowDC1394(unsigned int xdim,unsigned int ydim,int x0,int y0);

    // 27
    virtual yarp::dev::ReturnValue getFormat7WindowDC1394(unsigned int &xdim,unsigned int &ydim,int &x0,int &y0);

    // 28
    virtual yarp::dev::ReturnValue setOperationModeDC1394(bool b1394b);

    // 29
    virtual yarp::dev::ReturnValue getOperationModeDC1394(bool& b1394);

    // 30
    virtual yarp::dev::ReturnValue setTransmissionDC1394(bool bTxON);

    // 31
    virtual yarp::dev::ReturnValue getTransmissionDC1394(bool& bTxON);

    // 32 setBayer
    // 33 getBayer

    // 34
    virtual yarp::dev::ReturnValue setBroadcastDC1394(bool onoff);

    // 35
    virtual yarp::dev::ReturnValue setDefaultsDC1394();

    // 36
    virtual yarp::dev::ReturnValue setResetDC1394();

    // 37
    virtual yarp::dev::ReturnValue setPowerDC1394(bool onoff);

    // 38
    virtual yarp::dev::ReturnValue setCaptureDC1394(bool bON);

    // 39
    virtual yarp::dev::ReturnValue getBytesPerPacketDC1394(unsigned int& bpp);

    // 40
    virtual yarp::dev::ReturnValue setBytesPerPacketDC1394(unsigned int bpp);

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

    /*Implementation of IRgbVisualParams interface*/
    virtual int getRgbHeight();
    virtual int getRgbWidth();
    virtual yarp::dev::ReturnValue getRgbSupportedConfigurations(std::vector<yarp::dev::CameraConfig>& configurations);
    virtual yarp::dev::ReturnValue getRgbResolution(int &width, int &height);
    virtual yarp::dev::ReturnValue setRgbResolution(int width, int height);
    virtual yarp::dev::ReturnValue getRgbFOV(double &horizontalFov, double &verticalFov);
    virtual yarp::dev::ReturnValue setRgbFOV(double horizontalFov, double verticalFov);
    virtual yarp::dev::ReturnValue getRgbIntrinsicParam(yarp::os::Property &intrinsic);
    virtual yarp::dev::ReturnValue getRgbMirroring(bool &mirror);
    virtual yarp::dev::ReturnValue setRgbMirroring(bool mirror);

};

#endif
