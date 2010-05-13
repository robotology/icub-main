// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
* Author: Alessandro Scalzo
*/

//     W   W   I   N   N
//     W   W   I   NN  N
//     W W W   I   N N N
//     WW WW   I   N  NN
//     W   W   I   N   N

#include "winnt/FirewireCameraDC1394-DR2_2.h"

#define NOT_PRESENT -1
int CFWCamera_DR2_2::DC2Fly(int feature)
{
    switch (feature)
    {
    case DC1394_FEATURE_BRIGHTNESS: return FlyCapture2::BRIGHTNESS;
    case DC1394_FEATURE_EXPOSURE: return FlyCapture2::AUTO_EXPOSURE;
    case DC1394_FEATURE_SHARPNESS: return FlyCapture2::SHARPNESS;
    case DC1394_FEATURE_WHITE_BALANCE: return FlyCapture2::WHITE_BALANCE;
    case DC1394_FEATURE_HUE: return FlyCapture2::HUE;
    case DC1394_FEATURE_SATURATION: return FlyCapture2::SATURATION;
    case DC1394_FEATURE_GAMMA: return FlyCapture2::GAMMA;
    case DC1394_FEATURE_SHUTTER: return FlyCapture2::SHUTTER;
    case DC1394_FEATURE_GAIN: return FlyCapture2::GAIN;
    case DC1394_FEATURE_IRIS: return FlyCapture2::IRIS;
    case DC1394_FEATURE_FOCUS: return FlyCapture2::FOCUS;
    case DC1394_FEATURE_TEMPERATURE: return FlyCapture2::TEMPERATURE;
    case DC1394_FEATURE_TRIGGER: return FlyCapture2::TRIGGER_MODE;
    case DC1394_FEATURE_TRIGGER_DELAY: return FlyCapture2::TRIGGER_DELAY;
    case DC1394_FEATURE_FRAME_RATE: return FlyCapture2::FRAME_RATE;
    case DC1394_FEATURE_ZOOM: return FlyCapture2::ZOOM;
    case DC1394_FEATURE_PAN: return FlyCapture2::PAN;
    case DC1394_FEATURE_TILT: return FlyCapture2::TILT;
    }

    return NOT_PRESENT;
}

int CFWCamera_DR2_2::maxFPS(FlyCapture2::Mode mode,FlyCapture2::PixelFormat pixelFormat)
{
    switch (mode)
    {
    case FlyCapture2::MODE_0:
        switch (pixelFormat)
        {
        case FlyCapture2::PIXEL_FORMAT_MONO8:
        case FlyCapture2::PIXEL_FORMAT_RAW8:  
        case FlyCapture2::PIXEL_FORMAT_411YUV8: return 59;
        
        case FlyCapture2::PIXEL_FORMAT_MONO16:
        case FlyCapture2::PIXEL_FORMAT_RAW16:
        case FlyCapture2::PIXEL_FORMAT_422YUV8: return 47;
        
        case FlyCapture2::PIXEL_FORMAT_444YUV8:
        case FlyCapture2::PIXEL_FORMAT_RGB8:    return 31;
        }
        return 0;
    case FlyCapture2::MODE_1:
        switch (pixelFormat)
        {
        case FlyCapture2::PIXEL_FORMAT_MONO8:
        case FlyCapture2::PIXEL_FORMAT_MONO16:  return 100;

        case FlyCapture2::PIXEL_FORMAT_411YUV8:
        case FlyCapture2::PIXEL_FORMAT_422YUV8:
        case FlyCapture2::PIXEL_FORMAT_444YUV8:
        case FlyCapture2::PIXEL_FORMAT_RGB8:    return 59;
        }
        return 0;
    case FlyCapture2::MODE_2:
        switch (pixelFormat)
        {
        case FlyCapture2::PIXEL_FORMAT_MONO8:
        case FlyCapture2::PIXEL_FORMAT_MONO16:
        case FlyCapture2::PIXEL_FORMAT_411YUV8:
        case FlyCapture2::PIXEL_FORMAT_422YUV8:
        case FlyCapture2::PIXEL_FORMAT_444YUV8:
        case FlyCapture2::PIXEL_FORMAT_RGB8:    return 59;
        }
        return 0;
    }

    return 0;
}

double CFWCamera_DR2_2::bytesPerPixel(FlyCapture2::PixelFormat pixelFormat)
{
    switch (pixelFormat)
    {
        case FlyCapture2::PIXEL_FORMAT_MONO8:   return 1.0;
        case FlyCapture2::PIXEL_FORMAT_MONO16:  return 2.0;
        case FlyCapture2::PIXEL_FORMAT_411YUV8: return 1.5;
        case FlyCapture2::PIXEL_FORMAT_422YUV8: return 2.0;
        case FlyCapture2::PIXEL_FORMAT_444YUV8: return 3.0;
        case FlyCapture2::PIXEL_FORMAT_RGB8:    return 3.0;
        case FlyCapture2::PIXEL_FORMAT_RAW8:    return 1.0;
        case FlyCapture2::PIXEL_FORMAT_RAW16:   return 2.0;
    }

    return 0.0;
}

unsigned int CFWCamera_DR2_2::NormToValue(double& dVal,int feature)
{
    if (dVal<0.0) dVal=0.0;
    if (dVal>1.0) dVal=1.0;

    unsigned int iVal=m_iMin[feature]+(unsigned int)(dVal*double(m_iMax[feature]-m_iMin[feature]));

    if (iVal<m_iMin[feature]) iVal=m_iMin[feature];
    if (iVal>m_iMax[feature]) iVal=m_iMax[feature];

    return iVal;
}

double CFWCamera_DR2_2::ValueToNorm(unsigned int iVal,int feature)
{ 
    double dVal=double(iVal-m_iMin[feature])/double(m_iMax[feature]-m_iMin[feature]);

    if (dVal<0.0) return 0.0;
    if (dVal>1.0) return 1.0;

    return dVal;
}

bool CFWCamera_DR2_2::Create(yarp::os::Searchable& config)
{
    m_bDR2=config.check("DR2");

    int size_x=checkInt(config,"width");   
    int size_y=checkInt(config,"height");
    int format=checkInt(config,"video_type");
    unsigned int idCamera=checkInt(config,"d");
    
    m_pFrame=new FlyCapture2::Image();
    m_pBayer=new FlyCapture2::Image();

    //FlyCapture2::Image::SetDefaultColorProcessing(FlyCapture2::HQ_LINEAR);

    m_bCameraOn=false;
    m_bFrameIsValid=false;
    m_pCamera=NULL;
    m_pBusManager=NULL;
    m_nNumCameras=0;
    m_bTxOn=true;

    if (!(m_pBusManager=new FlyCapture2::BusManager()))
    {
        fprintf(stderr,"ERROR: failed to open Firewire Bus Manager\n");
        return false;
    }

    error=m_pBusManager->GetNumOfCameras(&m_nNumCameras);
    if (manage(error)) return false;

    if (!m_nNumCameras)
    {
        fprintf(stderr,"ERROR: no active cameras\n");
        return false;
    }

    if (idCamera<0 || idCamera>=m_nNumCameras)
    {
        fprintf(stderr,"ERROR: invalid camera number\n");
        return false;       
    }

    if (!(m_pCamera=new FlyCapture2::Camera()))
    {
        fprintf(stderr,"ERROR: failed to create camera\n");
        return false;
    }

    FlyCapture2::PGRGuid Guid;
    error=m_pBusManager->GetCameraFromIndex(idCamera,&Guid);
    if (manage(error)) return false;	

    error=m_pCamera->Connect(&Guid);
    if (manage(error)) return false;

    error=m_pCamera->GetCameraInfo(&m_CameraInfo);
    if (manage(error)) return false;

    switch (m_CameraInfo.maximumBusSpeed)
    {
    case FlyCapture2::BUSSPEED_S100:  m_BusSpeedBS= 10000000; break;
    case FlyCapture2::BUSSPEED_S200:  m_BusSpeedBS= 20000000; break;
    case FlyCapture2::BUSSPEED_S400:  m_BusSpeedBS= 40000000; break;
    case FlyCapture2::BUSSPEED_S480:  m_BusSpeedBS= 48000000; break;
    case FlyCapture2::BUSSPEED_S800:  m_BusSpeedBS= 80000000; break;
    case FlyCapture2::BUSSPEED_S1600: m_BusSpeedBS=160000000; break;
    case FlyCapture2::BUSSPEED_S3200: m_BusSpeedBS=320000000; break;
    default: m_BusSpeedBS=0;
    }

    error=m_pCamera->GetConfiguration(&m_CamConfig);
    if (manage(error)) return false;

    // CONFIGURE

    //error=m_pCamera->RestoreFromMemoryChannel(0);
    //if (manage(error)) return false;

    m_CamConfig.isochBusSpeed=FlyCapture2::BUSSPEED_S_FASTEST;
    m_CamConfig.asyncBusSpeed=FlyCapture2::BUSSPEED_S_FASTEST;
    m_CamConfig.grabMode=FlyCapture2::DROP_FRAMES;
    m_CamConfig.grabTimeout=2000; //FlyCapture2::TIMEOUT_INFINITE; 
    m_CamConfig.bandwidthAllocation=FlyCapture2::BANDWIDTH_ALLOCATION_ON;
    m_CamConfig.numBuffers=NUM_DMA_BUFFERS;
    m_CamConfig.numImageNotifications=1;

    error=m_pCamera->SetConfiguration(&m_CamConfig);
    if (manage(error)) return false;

    switch (format)
    {
    case DR_UNINIT:
        break;

    case DR_RGB_HALF_RES:
        if (!size_x) { size_x=320; }
        if (!size_y) { size_y=240; }
        SetF7(FlyCapture2::MODE_1,size_x,size_y,FlyCapture2::PIXEL_FORMAT_RGB,50);
        break;

    case DR_RGB_FULL_RES:
        if (!size_x) { size_x=640; }
        if (!size_y) { size_y=480; }

        if (size_x==640 && size_y==480)
        {
            SetVideoMode(FlyCapture2::VIDEOMODE_640x480RGB);
        }
        else
        {
            SetF7(FlyCapture2::MODE_0,size_x,size_y,FlyCapture2::PIXEL_FORMAT_RGB,50);
        }
        break;

    case DR_BAYER_FULL_RES:
        if (!size_x) { size_x=640; }
        if (!size_y) { size_y=480; }
        SetF7(FlyCapture2::MODE_0,size_x,size_y,FlyCapture2::PIXEL_FORMAT_RAW8,50);
        break;

    default:
        fprintf(stderr,"Reading video format from camera\n");    
    }

    for (int f=FlyCapture2::BRIGHTNESS; f<=FlyCapture2::TEMPERATURE; ++f)
    {
        FlyCapture2::PropertyInfo propInfo;
        propInfo.type=(FlyCapture2::PropertyType)f;
        error=m_pCamera->GetPropertyInfo(&propInfo);
        if (error.GetType()==FlyCapture2::PGRERROR_OK)
        {
            m_iMin[f]=(int)propInfo.min;
            m_iMax[f]=(int)propInfo.max;

            if (propInfo.present)
            {
                FlyCapture2::Property prop;
                prop.type=(FlyCapture2::PropertyType)f;
                error=m_pCamera->GetProperty(&prop);
                if (error.GetType()!=FlyCapture2::PGRERROR_OK)
                {
                    fprintf(stderr,"WARNING: feature %d %s\n",f,error.GetDescription());
                }
                prop.onOff=(f!=FlyCapture2::AUTO_EXPOSURE && f!=FlyCapture2::IRIS && f<FlyCapture2::TRIGGER_MODE);
                prop.autoManualMode=false;
                prop.absControl=false;
                error=m_pCamera->SetProperty(&prop);
                if (error.GetType()!=FlyCapture2::PGRERROR_OK)
                {
                    fprintf(stderr,"WARNING: feature %d %s\n",f,error.GetDescription());
                }
            }
            else
            {
                fprintf(stderr,"Feature %d not present\n");
            }
        }
        else
        {
            fprintf(stderr,"Feature %d %s\n",f,error.GetDescription());
        }
    }

    error=m_pCamera->GetConfiguration(&m_CamConfig);
    if (manage(error)) return false;

    error=m_pCamera->StartCapture();
    if (manage(error)) return false;

    m_bFrameIsValid=true;
    m_bCameraOn=true;

    // parameters

    setBrightness(checkDouble(config,"brightness"));
    setExposure(checkDouble(config,"exposure"));
    setSharpness(checkDouble(config,"sharpness"));
    yarp::os::Bottle& white_balance=config.findGroup("white_balance");
    if (!white_balance.isNull()) 
    {
        setWhiteBalance(white_balance.get(2).asDouble(),white_balance.get(1).asDouble());
    }
    setHue(checkDouble(config,"hue"));
    setSaturation(checkDouble(config,"saturation"));
    setGamma(checkDouble(config,"gamma"));
    setShutter(checkDouble(config,"shutter"));
    setGain(checkDouble(config,"gain"));
    setIris(checkDouble(config,"iris"));

    return true;
}

void CFWCamera_DR2_2::Close()
{
    if (m_pCamera)
    {
        m_pCamera->StopCapture();
        m_pCamera->Disconnect();
        delete m_pCamera;
        m_pCamera=0;
    }

    if (m_pFrame)
    {
        delete m_pFrame;
        m_pFrame=0;
    }

    if (m_pBayer)
    {
        delete m_pBayer;
        m_pBayer=0;
    }

    if (m_pBusManager)
    {
        delete m_pBusManager;
        m_pBusManager=0;
    }
}

bool CFWCamera_DR2_2::SetVideoMode(FlyCapture2::VideoMode video_mode)
{
    if (!m_pCamera) return false;

    int xdim,ydim,buff_dim;

    // calculate raw image size at given video mode
    switch (video_mode)
    {
    case FlyCapture2::VIDEOMODE_160x120YUV444: xdim=160; ydim=120; buff_dim= xdim*ydim*3;    break;
    case FlyCapture2::VIDEOMODE_320x240YUV422: xdim=320; ydim=240; buff_dim= xdim*ydim*2;    break;
    case FlyCapture2::VIDEOMODE_640x480YUV411: xdim=640; ydim=480; buff_dim=(xdim*ydim*3)/2; break;
    case FlyCapture2::VIDEOMODE_640x480YUV422: xdim=640; ydim=480; buff_dim= xdim*ydim*2;    break;
    case FlyCapture2::VIDEOMODE_640x480RGB:    xdim=640; ydim=480; buff_dim= xdim*ydim*3;    break;
    case FlyCapture2::VIDEOMODE_640x480Y8:     xdim=640; ydim=480; buff_dim= xdim*ydim;		 break;
    case FlyCapture2::VIDEOMODE_640x480Y16:	   xdim=640; ydim=480; buff_dim= xdim*ydim*2;    break;
    default: return false;
    }

    error=m_pCamera->GetConfiguration(&m_CamConfig);
    if (manage(error)) return false;

    // get ISO bandwidth
    switch (m_CamConfig.isochBusSpeed)
    {
    case FlyCapture2::BUSSPEED_S100:  m_BusSpeedBS= 10000000; break;
    case FlyCapture2::BUSSPEED_S200:  m_BusSpeedBS= 20000000; break;
    case FlyCapture2::BUSSPEED_S400:  m_BusSpeedBS= 40000000; break;
    case FlyCapture2::BUSSPEED_S480:  m_BusSpeedBS= 48000000; break;
    case FlyCapture2::BUSSPEED_S800:  m_BusSpeedBS= 80000000; break;
    case FlyCapture2::BUSSPEED_S1600: m_BusSpeedBS=160000000; break;
    case FlyCapture2::BUSSPEED_S3200: m_BusSpeedBS=320000000; break;
    default: m_BusSpeedBS=0;
    }

    // calculate maximum allowed framerate at given image format
    static const double two_cams=0.5; // only half bandwith available with two cams
    double fpsMax=two_cams*double(m_BusSpeedBS)/double(buff_dim);

    FlyCapture2::FrameRate frame_rate;

    // choose framerate according to maximum allowed
    if (fpsMax<1.785){ return false; }
    else if (fpsMax<3.75) { frame_rate=FlyCapture2::FRAMERATE_1_875; }
    else if (fpsMax<7.5)  { frame_rate=FlyCapture2::FRAMERATE_3_75; }
    else if (fpsMax<15.0) { frame_rate=FlyCapture2::FRAMERATE_7_5; }
    else if (fpsMax<30.0) { frame_rate=FlyCapture2::FRAMERATE_15; }
    else if (fpsMax<60.0) { frame_rate=FlyCapture2::FRAMERATE_30; }
    else if (fpsMax<120.0){ frame_rate=FlyCapture2::FRAMERATE_60; }
    else if (fpsMax<120.0){ frame_rate=FlyCapture2::FRAMERATE_60; }
    else if (fpsMax<240.0){ frame_rate=FlyCapture2::FRAMERATE_120; }
    else                  { frame_rate=FlyCapture2::FRAMERATE_240; }

    // is frame_rate supported? choose the best
    for (int fr=frame_rate; fr>=FlyCapture2::FRAMERATE_1_875; --fr)
    {
        bool bSupported;
        error=m_pCamera->GetVideoModeAndFrameRateInfo(video_mode,(FlyCapture2::FrameRate)fr,&bSupported);
        if (manage(error)) return false;

        if (bSupported)
        {
            error=m_pCamera->SetVideoModeAndFrameRate(video_mode,(FlyCapture2::FrameRate)fr);
            if (manage(error)) return false;

            m_XDim=xdim;
            m_YDim=ydim;

            return true;
        }
    }

    return false;
}

#define SKIP -1
bool CFWCamera_DR2_2::SetF7(int mode,int xdim,int ydim,int pixel_format,int speed)
{
    if (!m_pCamera) return false;

    FlyCapture2::VideoMode vm;
    FlyCapture2::FrameRate fr;
    error=m_pCamera->GetVideoModeAndFrameRate(&vm,&fr);
    if (manage(error)) return false;

    m_F7PercentSpeed=0.5f;

    if (vm==FlyCapture2::VIDEOMODE_FORMAT7)
    {
        error=m_pCamera->GetFormat7Configuration(&m_F7ImageSettings,&m_F7PacketSize,&m_F7PercentSpeed);
        if (manage(error)) return false;
    }
    else
    {
        if (mode==SKIP) // we're not in F7 mode and no mode is specified!
        {
            fprintf(stderr,"ERROR: no format 7 mode specified\n");
            return false;
        }

        m_F7ImageSettings.pixelFormat=FlyCapture2::PIXEL_FORMAT_RGB8;
    }

    m_F7Info.mode= mode==SKIP ? m_F7ImageSettings.mode : (FlyCapture2::Mode)mode;

    bool bSupported;
    error=m_pCamera->GetFormat7Info(&m_F7Info,&bSupported);
    if (manage(error)) return false;

    // is given mode supported?
    if (!bSupported)
    {
        fprintf(stderr,"ERROR: format 7 mode %d not supported\n",m_F7Info.mode);
        return false;
    }

    // is given pixel format supported?
    if (! (m_F7ImageSettings.pixelFormat & m_F7Info.pixelFormatBitField))
    {
        if (pixel_format==SKIP)
        {
            pixel_format=FlyCapture2::PIXEL_FORMAT_RGB8;
        }
        else
        {
            fprintf(stderr,"ERROR: invalid format 7 pixel format %d\n",m_F7ImageSettings.pixelFormat);
            return false;
        }
    }

    if (pixel_format==SKIP) pixel_format=m_F7ImageSettings.pixelFormat;

    if (vm!=FlyCapture2::VIDEOMODE_FORMAT7)
    {
        if (xdim==SKIP) xdim=(int)m_F7Info.maxWidth;
        if (ydim==SKIP) ydim=(int)m_F7Info.maxHeight;
    }
    else
    {
        if (xdim==SKIP) xdim=m_F7ImageSettings.width;
        if (ydim==SKIP) ydim=m_F7ImageSettings.height;
    }

    // adjust image size to allowed in this format
    if (xdim>(int)m_F7Info.maxWidth)  { xdim=m_F7Info.maxWidth;  }
    if (ydim>(int)m_F7Info.maxHeight) { ydim=m_F7Info.maxHeight; }

    if (m_F7Info.mode==FlyCapture2::MODE_1)
    {
        xdim=(xdim/(2*m_F7Info.imageHStepSize))*(2*m_F7Info.imageHStepSize);
    }
    else
    {
        xdim=(xdim/m_F7Info.imageHStepSize)*m_F7Info.imageHStepSize;
    }
    ydim=(ydim/m_F7Info.imageVStepSize)*m_F7Info.imageVStepSize;

    // calculate offset
    int xoff=(m_F7Info.maxWidth -xdim)/2;
    int yoff=(m_F7Info.maxHeight-ydim)/2;
    xoff=(xoff/m_F7Info.offsetHStepSize)*m_F7Info.offsetHStepSize;
    yoff=(yoff/m_F7Info.offsetVStepSize)*m_F7Info.offsetVStepSize;

    // prepare image settings
    m_F7ImageSettings.mode=m_F7Info.mode;
    if (pixel_format!=SKIP) m_F7ImageSettings.pixelFormat=(FlyCapture2::PixelFormat)pixel_format;
    m_F7ImageSettings.width=(unsigned)xdim;
    m_F7ImageSettings.offsetX=xoff;
    m_F7ImageSettings.height=(unsigned)ydim;		
    m_F7ImageSettings.offsetY=yoff;

    // validate image settings
    bool bSettingsAreValid;
    FlyCapture2::Format7PacketInfo packetInfo;
    error=m_pCamera->ValidateFormat7Settings(&m_F7ImageSettings,&bSettingsAreValid,&packetInfo);
    if (manage(error)) return false;

    if (!bSettingsAreValid)
    {
        fprintf(stderr,"ERROR: invalid format 7 settings\n");
        return false;
    }

    // speed

    error=m_pCamera->GetConfiguration(&m_CamConfig);
    if (manage(error)) return false;

    // get ISO bandwidth
    switch (m_CamConfig.isochBusSpeed)
    {
    case FlyCapture2::BUSSPEED_S100:  m_BusSpeedBS= 10000000; break;
    case FlyCapture2::BUSSPEED_S200:  m_BusSpeedBS= 20000000; break;
    case FlyCapture2::BUSSPEED_S400:  m_BusSpeedBS= 40000000; break;
    case FlyCapture2::BUSSPEED_S480:  m_BusSpeedBS= 48000000; break;
    case FlyCapture2::BUSSPEED_S800:  m_BusSpeedBS= 80000000; break;
    case FlyCapture2::BUSSPEED_S1600: m_BusSpeedBS=160000000; break;
    case FlyCapture2::BUSSPEED_S3200: m_BusSpeedBS=320000000; break;
    default: m_BusSpeedBS=0;
    }

    int fps=maxFPS(m_F7ImageSettings.mode,m_F7ImageSettings.pixelFormat);
    double bpp=bytesPerPixel(m_F7ImageSettings.pixelFormat);
    double maxBandOcc=double(fps*xdim*ydim)*bpp;

    double margin=double(m_BusSpeedBS)/maxBandOcc;

    if (speed==SKIP)
    {
        m_F7PacketSize=(unsigned int)(m_F7PercentSpeed*packetInfo.maxBytesPerPacket);
    }
    else
    {
        m_F7PacketSize=(unsigned int)(0.01*double(speed*packetInfo.maxBytesPerPacket)*margin);
        //m_F7PacketSize=(unsigned int)((speed*packetInfo.maxBytesPerPacket)/100);
        printf("maxBandOcc=%f   m_BusSpeedBS=%d    margin=%f    m_F7PacketSize=%d\n",maxBandOcc,m_BusSpeedBS,margin,m_F7PacketSize);
    }

    m_F7PacketSize=(m_F7PacketSize/packetInfo.unitBytesPerPacket)*packetInfo.unitBytesPerPacket;
    if (m_F7PacketSize>packetInfo.maxBytesPerPacket)
    {
        m_F7PacketSize=packetInfo.maxBytesPerPacket;
    }

    // set image configuration
    error=m_pCamera->SetFormat7Configuration(&m_F7ImageSettings,m_F7PacketSize);
    if (manage(error)) return false;

    error=m_pCamera->GetFormat7Configuration(&m_F7ImageSettings,&m_F7PacketSize,&m_F7PercentSpeed);
    if (manage(error)) return false;

    printf("\nPacket size=%d - Speed=%f\n\n",m_F7PacketSize,m_F7PercentSpeed);

    m_XDim=m_F7ImageSettings.width;
    m_YDim=m_F7ImageSettings.height;

    return true;
}

bool CFWCamera_DR2_2::Capture(yarp::sig::ImageOf<yarp::sig::PixelRgb>* pImage,unsigned char *pBuffer,bool bRaw)
{
    m_AcqMutex.wait();

    if (!m_bCameraOn)
    {
        m_AcqMutex.post();
        return false;
    }

    error=m_pCamera->RetrieveBuffer(m_pFrame);
    if (manage(error,&m_AcqMutex)) return false;

    if (!m_bFrameIsValid)
    {
        m_bFrameIsValid=true;
        m_AcqMutex.post();
        return false;
    }

    m_Stamp.update();

    if (pImage)
    {
        pImage->resize(m_pFrame->GetCols(),m_pFrame->GetRows());
        pBuffer=pImage->getRawImage();
    }

    if (m_pFrame->GetPixelFormat()==FlyCapture2::PIXEL_FORMAT_RGB8 || bRaw)
    {
        memcpy(pBuffer,m_pFrame->GetData(),m_pFrame->GetRows()*m_pFrame->GetCols()*(bRaw?1:3));
    }
    else
    {
        error=m_pFrame->Convert(FlyCapture2::PIXEL_FORMAT_RGB8,m_pBayer);
        if (manage(error,&m_AcqMutex))
        {
            return false;
        }
        else
        {
            memcpy(pBuffer,m_pBayer->GetData(),m_pBayer->GetRows()*m_pBayer->GetCols()*3);
        }
    }

    m_AcqMutex.post();
    return true;
}

////////////////////
// feature functions
////////////////////

// 00
bool CFWCamera_DR2_2::hasFeatureDC1394(int feature)
{
    if ((feature=DC2Fly(feature))==NOT_PRESENT) return false;
    FlyCapture2::PropertyInfo info;
    info.type=(FlyCapture2::PropertyType)feature;
    error=m_pCamera->GetPropertyInfo(&info);
    if (error.GetType()!=FlyCapture2::PGRERROR_OK) return false;
    return info.present;
}
// 01
bool CFWCamera_DR2_2::setFeatureDC1394(int feature,double value)
{
    if (value<0.0 || value>1.0) return false;

    m_AcqMutex.wait();

    if (!m_pCamera)
    {
        m_AcqMutex.post();
        return false;
    }

    if ((feature=DC2Fly(feature))==NOT_PRESENT)
    {
        m_AcqMutex.post();
        return false;
    }

    FlyCapture2::Property prop;
    prop.type=(FlyCapture2::PropertyType)feature;
    error=m_pCamera->GetProperty(&prop);
    if (manage(error,&m_AcqMutex)) return false;
   
    prop.valueA=NormToValue(value,feature);
    error=m_pCamera->SetProperty(&prop);
    if (manage(error,&m_AcqMutex)) return false;

    m_AcqMutex.post();
    return true;
}
// 02
double CFWCamera_DR2_2::getFeatureDC1394(int feature)
{
    if ((feature=DC2Fly(feature))==NOT_PRESENT) return -1.0;
    if (!m_pCamera) return -1.0;
    FlyCapture2::Property prop;
    prop.type=(FlyCapture2::PropertyType)feature;
    error=m_pCamera->GetProperty(&prop);
    if (manage(error)) return -1.0;
    return ValueToNorm(prop.valueA,feature);
}

// 03
bool CFWCamera_DR2_2::hasOnOffDC1394(int feature)
{
    if ((feature=DC2Fly(feature))==NOT_PRESENT) return false;
    if (!m_pCamera) return false;
    FlyCapture2::PropertyInfo info;
    info.type=(FlyCapture2::PropertyType)feature;
    error=m_pCamera->GetPropertyInfo(&info);
    if (manage(error)) return false;
    return info.onOffSupported;
}
// 04
bool CFWCamera_DR2_2::setActiveDC1394(int feature, bool onoff)
{
    if ((feature=DC2Fly(feature))==NOT_PRESENT) return false;
    if (!m_pCamera) return false;
    FlyCapture2::Property prop;
    prop.type=(FlyCapture2::PropertyType)feature;
    error=m_pCamera->GetProperty(&prop);
    if (manage(error)) return false;
    prop.onOff=onoff;
    m_pCamera->SetProperty(&prop);
    if (manage(error)) return false;
    return true;
}
// 05
bool CFWCamera_DR2_2::getActiveDC1394(int feature)
{
    if ((feature=DC2Fly(feature))==NOT_PRESENT) return false;
    if (!m_pCamera) return false;
    FlyCapture2::Property prop;
    prop.type=(FlyCapture2::PropertyType)feature;
    error=m_pCamera->GetProperty(&prop); 
    if (manage(error)) return false;
    return prop.onOff;
}

// 06
bool CFWCamera_DR2_2::hasManualDC1394(int feature)
{
    if ((feature=DC2Fly(feature))==NOT_PRESENT) return false;
    if (!m_pCamera) return false;
    FlyCapture2::PropertyInfo info;
    info.type=(FlyCapture2::PropertyType)feature;
    error=m_pCamera->GetPropertyInfo(&info);
    if (manage(error)) return false;
    return info.manualSupported;
}
// 07
bool CFWCamera_DR2_2::hasAutoDC1394(int feature)
{
    if ((feature=DC2Fly(feature))==NOT_PRESENT) return false;
    if (!m_pCamera) return false;
    FlyCapture2::PropertyInfo info;
    info.type=(FlyCapture2::PropertyType)feature;
    error=m_pCamera->GetPropertyInfo(&info);
    if (manage(error)) return false;
    return info.autoSupported;
}	
// 08
bool CFWCamera_DR2_2::hasOnePushDC1394(int feature)
{
    if ((feature=DC2Fly(feature))==NOT_PRESENT) return false;
    if (!m_pCamera) return false;
    FlyCapture2::PropertyInfo info;
    info.type=(FlyCapture2::PropertyType)feature;
    error=m_pCamera->GetPropertyInfo(&info);
    if (manage(error)) return false;
    return info.onePushSupported;
}

// 09
bool CFWCamera_DR2_2::setModeDC1394(int feature, bool auto_onoff)
{
    if ((feature=DC2Fly(feature))==NOT_PRESENT) return false;
    if (!m_pCamera) return false;
    FlyCapture2::Property prop;
    prop.type=(FlyCapture2::PropertyType)feature;
    error=m_pCamera->GetProperty(&prop);
    if (manage(error)) return false;
    prop.autoManualMode=auto_onoff;
    error=m_pCamera->SetProperty(&prop);
    if (manage(error)) return false;
    return true;
}
// 10
bool CFWCamera_DR2_2::getModeDC1394(int feature)
{
    if ((feature=DC2Fly(feature))==NOT_PRESENT) return false;
    if (!m_pCamera) return false;
    FlyCapture2::Property prop;
    prop.type=(FlyCapture2::PropertyType)feature;
    error=m_pCamera->GetProperty(&prop); 
    if (manage(error)) return false;
    return prop.autoManualMode;
}
// 11
bool CFWCamera_DR2_2::setOnePushDC1394(int feature)
{
    if ((feature=DC2Fly(feature))==NOT_PRESENT) return false;
    if (!m_pCamera) return false;
    FlyCapture2::Property prop;
    prop.type=(FlyCapture2::PropertyType)feature;
    error=m_pCamera->GetProperty(&prop);
    if (manage(error)) return false;
    prop.onePush=true;
    error=m_pCamera->SetProperty(&prop);
    if (manage(error)) return false;
    return true;
}

// 23
bool CFWCamera_DR2_2::setWhiteBalanceDC1394(double b, double r)
{
    if (b<0.0 || b>1.0 || r<0.0 || r>1.0) return false;
    if (!m_pCamera) return false;
    FlyCapture2::Property prop;
    prop.type=FlyCapture2::WHITE_BALANCE;
    error=m_pCamera->GetProperty(&prop);
    if (manage(error)) return false;
    prop.valueA=NormToValue(r,FlyCapture2::WHITE_BALANCE);
    prop.valueB=NormToValue(b,FlyCapture2::WHITE_BALANCE);
    error=m_pCamera->SetProperty(&prop);
    if (manage(error)) return false;
    return true;
}
// 24
bool CFWCamera_DR2_2::getWhiteBalanceDC1394(double &b, double &r)
{
    if (!m_pCamera) return false;
    FlyCapture2::Property prop;
    prop.type=FlyCapture2::WHITE_BALANCE;
    error=m_pCamera->GetProperty(&prop);
    if (manage(error)) return false;
    r=ValueToNorm(prop.valueA,FlyCapture2::WHITE_BALANCE);
    b=ValueToNorm(prop.valueB,FlyCapture2::WHITE_BALANCE);
    return true;
}

////////////////////////
// end feature functions
////////////////////////

// 12
unsigned int CFWCamera_DR2_2::getVideoModeMaskDC1394()
{
    bool bSupported;
    unsigned int mask=0;

    for (int m=FlyCapture2::VIDEOMODE_160x120YUV444; m<FlyCapture2::NUM_VIDEOMODES; ++m)
    {
        error=m_pCamera->GetVideoModeAndFrameRateInfo((FlyCapture2::VideoMode)m,FlyCapture2::FRAMERATE_15,&bSupported);
        if (error.GetType()==FlyCapture2::PGRERROR_OK && bSupported) 
        {
            mask|=1<<m;
        }
    }

    if (mask & FlyCapture2::VIDEOMODE_FORMAT7)
    {
        mask &= ~(1<<FlyCapture2::VIDEOMODE_FORMAT7);

        bool bSupported;

        for (int m=FlyCapture2::MODE_0; m<FlyCapture2::NUM_MODES; ++m)
        {
            m_F7Info.mode=(FlyCapture2::Mode)m;

            error=m_pCamera->GetFormat7Info(&m_F7Info,&bSupported);
            if (manage(error)) return 0;

            if (bSupported) mask|=1<<(1+m+FlyCapture2::VIDEOMODE_FORMAT7);
        }
    }

    printf("video mode mask: %x\n",mask);
    fflush(stdout);

    return mask;
}
// 13
bool CFWCamera_DR2_2::setVideoModeDC1394(int video_mode)
{
    m_AcqMutex.wait();

    printf("setVideoModeDC1394(%d)\n",video_mode);

    if (!m_pCamera)
    {
        m_AcqMutex.post();
        return false;
    }

    m_bFrameIsValid=false;

    error=m_pCamera->StopCapture();
    if (manage(error,&m_AcqMutex)) return false;

    if (video_mode<FlyCapture2::VIDEOMODE_FORMAT7)
    {
        if (!SetVideoMode((FlyCapture2::VideoMode)video_mode))
        {
            m_AcqMutex.post();
            return false;
        }
    }
    else
    {
        FlyCapture2::Mode mode=(FlyCapture2::Mode)((int)video_mode-1-(int)FlyCapture2::VIDEOMODE_FORMAT7);
        if (!SetF7(mode,SKIP,SKIP,SKIP,SKIP))
        {
            m_AcqMutex.post();
            return false;
        }
    }

    error=m_pCamera->StartCapture();
    if (manage(error,&m_AcqMutex)) return false;

    m_AcqMutex.post();
    return true;
}
// 14
unsigned int CFWCamera_DR2_2::getVideoModeDC1394()
{ 
    FlyCapture2::VideoMode vm;
    FlyCapture2::FrameRate fr;
    error=m_pCamera->GetVideoModeAndFrameRate(&vm,&fr);
    if (manage(error)) return 0;

    if (vm==FlyCapture2::VIDEOMODE_FORMAT7)
    {
        error=m_pCamera->GetFormat7Configuration(&m_F7ImageSettings,&m_F7PacketSize,&m_F7PercentSpeed);
        if (manage(error)) return 0;

        return 1+vm+m_F7ImageSettings.mode;
    }

    return vm;
}

// 15
unsigned int CFWCamera_DR2_2::getFPSMaskDC1394()
{
    if (!m_pCamera) return 0;

    FlyCapture2::VideoMode vm;
    FlyCapture2::FrameRate fr;
    error=m_pCamera->GetVideoModeAndFrameRate(&vm,&fr);
    if (manage(error)) return 0;

    if (vm==FlyCapture2::VIDEOMODE_FORMAT7)
    {
        return 0;
    }

    bool bSupported;
    unsigned int mask=0;

    for (int f=FlyCapture2::FRAMERATE_1_875; f<FlyCapture2::NUM_FRAMERATES; ++f)
    {
        error=m_pCamera->GetVideoModeAndFrameRateInfo(vm,(FlyCapture2::FrameRate)f,&bSupported);
        if (error.GetType()==FlyCapture2::PGRERROR_OK && bSupported) 
        {
            mask|=1<<f;
        }
    }

    return mask;
}
// 16
unsigned int CFWCamera_DR2_2::getFPSDC1394()
{
    if (!m_pCamera) return 0;

    FlyCapture2::VideoMode vm;
    FlyCapture2::FrameRate fr;
    error=m_pCamera->GetVideoModeAndFrameRate(&vm,&fr);
    if (manage(error)) return 0;

    if (vm==FlyCapture2::VIDEOMODE_FORMAT7)
    {
        return 0;
    }

    return fr; 
}
// 17
bool CFWCamera_DR2_2::setFPSDC1394(int fps)
{
    m_AcqMutex.wait();

    if (!m_pCamera)
    {
        m_AcqMutex.post();
        return false;
    }

    FlyCapture2::VideoMode vm;
    FlyCapture2::FrameRate fr;
    error=m_pCamera->GetVideoModeAndFrameRate(&vm,&fr);
    if (manage(error,&m_AcqMutex)) return false;

    if (vm==FlyCapture2::VIDEOMODE_FORMAT7)
    {
        m_AcqMutex.post();
        return false;
    }

    error=m_pCamera->StopCapture();
    if (manage(error,&m_AcqMutex)) return false;

    error=m_pCamera->SetVideoModeAndFrameRate(vm,(FlyCapture2::FrameRate)fps);
    if (manage(error,&m_AcqMutex)) return false;

    error=m_pCamera->StartCapture();
    if (manage(error,&m_AcqMutex)) return false;

    m_AcqMutex.post();
    return true;
}

// 18
unsigned int CFWCamera_DR2_2::getISOSpeedDC1394()
{
    if (!m_pCamera) return 0;

    error=m_pCamera->GetConfiguration(&m_CamConfig);
    if (manage(error)) return 0;

    if (m_CamConfig.isochBusSpeed>FlyCapture2::BUSSPEED_S400)
    {
        return (int)m_CamConfig.isochBusSpeed-1;
    }

    return (int)m_CamConfig.isochBusSpeed;
}
// 19
bool CFWCamera_DR2_2::setISOSpeedDC1394(int speed)
{ 	
    m_AcqMutex.wait();

    if (!m_pCamera)
    {
        m_AcqMutex.post();
        return false;
    }

    m_bFrameIsValid=false;

    error=m_pCamera->StopCapture();
    if (manage(error,&m_AcqMutex)) return false;

    FlyCapture2::VideoMode vm;
    FlyCapture2::FrameRate fr;
    error=m_pCamera->GetVideoModeAndFrameRate(&vm,&fr);
    if (manage(error,&m_AcqMutex)) return false;

    error=m_pCamera->GetConfiguration(&m_CamConfig);
    if (manage(error,&m_AcqMutex)) return false;

    if (speed>FlyCapture2::BUSSPEED_S400) ++speed;

    m_CamConfig.isochBusSpeed=(FlyCapture2::BusSpeed)speed;

    error=m_pCamera->SetConfiguration(&m_CamConfig);
    if (manage(error,&m_AcqMutex)) return false;

    if (vm<FlyCapture2::VIDEOMODE_FORMAT7)
    {
        if (!SetVideoMode(vm))
        {
            m_AcqMutex.post();
            return false;
        }
    }
    else
    {
        if (!SetF7(SKIP,SKIP,SKIP,SKIP,SKIP))
        {
            m_AcqMutex.post();
            return false;
        }
    }

    error=m_pCamera->StartCapture();
    if (manage(error,&m_AcqMutex)) return false;

    m_AcqMutex.post();
    return true;
}

// 20
unsigned int CFWCamera_DR2_2::getColorCodingMaskDC1394(unsigned int video_mode)
{
    if (!m_pCamera) return 0;

    if (video_mode<FlyCapture2::VIDEOMODE_FORMAT7)
    {
        return 0;
    }

    bool bSupported;
    m_F7Info.mode=(FlyCapture2::Mode)(video_mode-1-FlyCapture2::VIDEOMODE_FORMAT7);

    error=m_pCamera->GetFormat7Info(&m_F7Info,&bSupported);
    if (manage(error)) return 0;

    if (!bSupported)
    {
        return 0;
    }

    unsigned int mask=0;

    for (int i=0; i<32; ++i)
    {
        if (m_F7Info.pixelFormatBitField & (1<<i))
        {
            mask |= (1<<(31-i));
        }
    }

    return mask;
}

unsigned int CFWCamera_DR2_2::getActualColorCodingMaskDC1394()
{
    if (!m_pCamera) return 0;

    FlyCapture2::VideoMode vm;
    FlyCapture2::FrameRate fr;
    error=m_pCamera->GetVideoModeAndFrameRate(&vm,&fr);
    if (manage(error)) return 0;

    if (vm<FlyCapture2::VIDEOMODE_FORMAT7)
    {
        return 0;
    }

    error=m_pCamera->GetFormat7Configuration(&m_F7ImageSettings,&m_F7PacketSize,&m_F7PercentSpeed);
    if (manage(error)) return 0;

    bool bSupported;
    m_F7Info.mode=m_F7ImageSettings.mode;

    error=m_pCamera->GetFormat7Info(&m_F7Info,&bSupported);
    if (manage(error)) return 0;

    if (!bSupported)
    {
        return 0;
    }

    return m_F7Info.pixelFormatBitField;
}

// 21
unsigned int CFWCamera_DR2_2::getColorCodingDC1394()
{
    if (!m_pCamera) return 0;

    FlyCapture2::VideoMode vm;
    FlyCapture2::FrameRate fr;
    error=m_pCamera->GetVideoModeAndFrameRate(&vm,&fr);
    if (manage(error)) return 0;

    if (vm!=FlyCapture2::VIDEOMODE_FORMAT7)
    {
        return 0;
    }

    error=m_pCamera->GetFormat7Configuration(&m_F7ImageSettings,&m_F7PacketSize,&m_F7PercentSpeed);
    if (manage(error)) return 0;

    for (int i=0; i<32; ++i)
    {
        if ((1<<i) & m_F7ImageSettings.pixelFormat)
        {
            return 31-i;
        }
    }

    return 0;
}
// 22
bool CFWCamera_DR2_2::setColorCodingDC1394(int coding)
{
    m_AcqMutex.wait();

    fprintf(stdout,"setColorCodingDC1394(%d)\n",coding);

    if (!m_pCamera)
    {
        m_AcqMutex.post();
        return false;
    }

    m_bFrameIsValid=false;

    error=m_pCamera->StopCapture();
    if (manage(error,&m_AcqMutex)) return false;

    FlyCapture2::PixelFormat pixel_format=(FlyCapture2::PixelFormat)(1<<(31-coding));

    if (!SetF7(SKIP,SKIP,SKIP,pixel_format,SKIP))
    {
        m_AcqMutex.post();
        return false;
    }

    error=m_pCamera->StartCapture();
    if (manage(error,&m_AcqMutex)) return false;

    m_AcqMutex.post();
    return true;
}	

// 25
bool CFWCamera_DR2_2::getFormat7MaxWindowDC1394(unsigned int &xdim,unsigned int &ydim,unsigned int &xstep,unsigned int &ystep)
{
    if (!m_pCamera) return false;

    FlyCapture2::VideoMode vm;
    FlyCapture2::FrameRate fr;
    error=m_pCamera->GetVideoModeAndFrameRate(&vm,&fr);
    if (manage(error)) return false;

    if (vm!=FlyCapture2::VIDEOMODE_FORMAT7)
    {
        xdim=ydim=xstep=ystep=0;
        return true;
    }

    error=m_pCamera->GetFormat7Configuration(&m_F7ImageSettings,&m_F7PacketSize,&m_F7PercentSpeed);
    if (manage(error)) return false;

    m_F7Info.mode=m_F7ImageSettings.mode;
    bool m_bSupported;
    error=m_pCamera->GetFormat7Info(&m_F7Info,&m_bSupported);
    if (manage(error)) return false;

    if (!m_bSupported)
    {
        return false;
    }

    xdim=m_F7Info.maxWidth;
    ydim=m_F7Info.maxHeight;

    xstep=m_F7Info.imageHStepSize;
    ystep=m_F7Info.imageVStepSize;

    if (m_F7Info.mode==1) xstep*=2;

    return true;
}
// 26
bool CFWCamera_DR2_2::setFormat7WindowDC1394(unsigned int xdim,unsigned int ydim)
{
    m_AcqMutex.wait();

    fprintf(stderr,"setFormat7WindowDC1394(%d,%d)\n",xdim,ydim);

    if (!m_pCamera)
    {
        m_AcqMutex.post();
        return false;
    }

    m_bFrameIsValid=false;

    error=m_pCamera->StopCapture();
    if (manage(error,&m_AcqMutex)) return false;

    if (!SetF7(SKIP,xdim,ydim,SKIP,SKIP))
    {
        m_AcqMutex.post();
        return false;
    }

    error=m_pCamera->StartCapture();
    if (manage(error,&m_AcqMutex)) return false;

    m_AcqMutex.post();
    return true;
}
// 27
bool CFWCamera_DR2_2::getFormat7WindowDC1394(unsigned int &xdim,unsigned int &ydim)
{
    if (!m_pCamera) return false;

    FlyCapture2::VideoMode vm;
    FlyCapture2::FrameRate fr;
    error=m_pCamera->GetVideoModeAndFrameRate(&vm,&fr);
    if (manage(error)) return false;

    if (vm!=FlyCapture2::VIDEOMODE_FORMAT7)
    {
        xdim=ydim=0;
        return true;
    }

    error=m_pCamera->GetFormat7Configuration(&m_F7ImageSettings,&m_F7PacketSize,&m_F7PercentSpeed);
    if (manage(error)) return false;

    xdim=m_F7ImageSettings.width;
    ydim=m_F7ImageSettings.height;

    return true;
}	

// 28
bool CFWCamera_DR2_2::setOperationModeDC1394(bool b1394b)
{
    return true;
}

// 29
bool CFWCamera_DR2_2::getOperationModeDC1394()
{
    //if (!m_pCamera || !m_bDR2) return false;
    return true;
}

// 30
bool CFWCamera_DR2_2::setTransmissionDC1394(bool bTxON)
{
    if (!m_pCamera) return false;

    if (bTxON)
    {
        error=m_pCamera->StartCapture();
        if (manage(error)) return false;
        m_bTxOn=true;
    }
    else
    {
        error=m_pCamera->StopCapture();
        if (manage(error)) return false;
        m_bTxOn=false;
    }

    return true;
}

// 31
bool CFWCamera_DR2_2::getTransmissionDC1394()
{
    if (!m_pCamera) return false;

    return m_bTxOn;
}

// 32 setBayer
// 33 getBayer

// 34
bool CFWCamera_DR2_2::setBroadcastDC1394(bool onoff)
{
    if (!m_pCamera) return false;
    return false;
}
// 35
bool CFWCamera_DR2_2::setDefaultsDC1394()
{
    m_AcqMutex.wait();

    if (!m_pCamera)
    {
        m_AcqMutex.post();
        return false;
    }

    m_bFrameIsValid=false;

    error=m_pCamera->StopCapture();
    if (manage(error,&m_AcqMutex)) return false;

    error=m_pCamera->RestoreFromMemoryChannel(0);
    if (manage(error,&m_AcqMutex)) return false;

    error=m_pCamera->StartCapture();
    if (manage(error,&m_AcqMutex)) return false;

    m_AcqMutex.post();
    return true;
}
// 36
bool CFWCamera_DR2_2::setResetDC1394()
{
    return false;
}
// 37
bool CFWCamera_DR2_2::setPowerDC1394(bool onoff)
{
    return false;
}

// 38
bool CFWCamera_DR2_2::setCaptureDC1394(bool bON)
{
    if (!m_pCamera) return false;

    if (bON)
    {
        error=m_pCamera->StartCapture();
        if (manage(error)) return false;
        m_bTxOn=true;
    }
    else
    {
        error=m_pCamera->StopCapture();
        if (manage(error)) return false;
        m_bTxOn=false;
    }

    return true;
}

// 39
unsigned int CFWCamera_DR2_2::getBytesPerPacketDC1394()
{
    if (!m_pCamera) return 0;

    FlyCapture2::VideoMode vm;
    FlyCapture2::FrameRate fr;
    error=m_pCamera->GetVideoModeAndFrameRate(&vm,&fr);
    if (manage(error)) return 0;

    if (vm!=FlyCapture2::VIDEOMODE_FORMAT7)
    {
        return 0;
    }

    error=m_pCamera->GetFormat7Configuration(&m_F7ImageSettings,&m_F7PacketSize,&m_F7PercentSpeed);
    if (manage(error)) return 0;

    return (unsigned int)(100.0f*m_F7PercentSpeed);
}

// 40
bool CFWCamera_DR2_2::setBytesPerPacketDC1394(unsigned int bpp)
{
    m_AcqMutex.wait();

    fprintf(stderr,"setBytesPerPacketDC1394(%d)\n",bpp);

    if (!m_pCamera)
    {
        m_AcqMutex.post();
        return false;
    }

    m_bFrameIsValid=false;

    error=m_pCamera->StopCapture();
    if (manage(error,&m_AcqMutex)) return false;

    if (!SetF7(SKIP,SKIP,SKIP,SKIP,(int)bpp))
    {
        m_AcqMutex.post();
        return false;
    }

    error=m_pCamera->StartCapture();
    if (manage(error,&m_AcqMutex)) return false;

    m_AcqMutex.post();
    return true;
}

// base class implementation

bool CFWCamera_DR2_2::setBrightness(double v)
{
    if (v<0.0 || v>1.0) return false;
    setActiveDC1394(DC1394_FEATURE_BRIGHTNESS,true);
    setModeDC1394(DC1394_FEATURE_BRIGHTNESS,false);  
    return setFeatureDC1394(DC1394_FEATURE_BRIGHTNESS,v); 
}
bool CFWCamera_DR2_2::setExposure(double v)
{
    if (v<0.0 || v>1.0) return false;
    setActiveDC1394(DC1394_FEATURE_EXPOSURE,true);
    setModeDC1394(DC1394_FEATURE_EXPOSURE,false);  
    return setFeatureDC1394(DC1394_FEATURE_EXPOSURE,v); 
}
bool CFWCamera_DR2_2::setSharpness(double v)
{ 
    if (v<0.0 || v>1.0) return false;
    setActiveDC1394(DC1394_FEATURE_SHARPNESS,true);
    setModeDC1394(DC1394_FEATURE_SHARPNESS,false);
    return setFeatureDC1394(DC1394_FEATURE_SHARPNESS,v); 
}
bool CFWCamera_DR2_2::setWhiteBalance(double blue, double red)
{
    if (blue<0.0 || blue>1.0 || red<0.0 || red>1.0) return false; 
    setActiveDC1394(DC1394_FEATURE_WHITE_BALANCE,true);
    setModeDC1394(DC1394_FEATURE_WHITE_BALANCE,false);
    return setWhiteBalanceDC1394(blue,red); 
}
bool CFWCamera_DR2_2::setHue(double v)
{
    if (v<0.0 || v>1.0) return false; 
    setActiveDC1394(DC1394_FEATURE_HUE,true);
    setModeDC1394(DC1394_FEATURE_HUE,false);
    return setFeatureDC1394(DC1394_FEATURE_HUE,v); 
}
bool CFWCamera_DR2_2::setSaturation(double v)
{
    if (v<0.0 || v>1.0) return false;
    setActiveDC1394(DC1394_FEATURE_SATURATION,true);
    setModeDC1394(DC1394_FEATURE_SATURATION,false);
    return setFeatureDC1394(DC1394_FEATURE_SATURATION,v); 
}
bool CFWCamera_DR2_2::setGamma(double v)
{
    if (v<0.0 || v>1.0) return false;
    setActiveDC1394(DC1394_FEATURE_GAMMA,true);
    setModeDC1394(DC1394_FEATURE_GAMMA,false);
    return setFeatureDC1394(DC1394_FEATURE_GAMMA,v); 
}
bool CFWCamera_DR2_2::setShutter(double v)
{
    if (v<0.0 || v>1.0) return false; 
    setActiveDC1394(DC1394_FEATURE_SHUTTER,true);
    setModeDC1394(DC1394_FEATURE_SHUTTER,false);
    return setFeatureDC1394(DC1394_FEATURE_SHUTTER,v); 
}
bool CFWCamera_DR2_2::setGain(double v)
{
    if (v<0.0 || v>1.0) return false; 
    setActiveDC1394(DC1394_FEATURE_GAIN,true);
    setModeDC1394(DC1394_FEATURE_GAIN,false);
    return setFeatureDC1394(DC1394_FEATURE_GAIN,v); 
}
bool CFWCamera_DR2_2::setIris(double v)
{
    if (v<0.0 || v>1.0) return false; 
    setActiveDC1394(DC1394_FEATURE_IRIS,true);
    setModeDC1394(DC1394_FEATURE_IRIS,false);
    return setFeatureDC1394(DC1394_FEATURE_IRIS,v); 
}

// GET

double CFWCamera_DR2_2::getBrightness()
{ 
    return getFeatureDC1394(DC1394_FEATURE_BRIGHTNESS); 
}
double CFWCamera_DR2_2::getExposure()
{ 
    return getFeatureDC1394(DC1394_FEATURE_EXPOSURE); 
}	
double CFWCamera_DR2_2::getSharpness()
{ 
    return getFeatureDC1394(DC1394_FEATURE_SHARPNESS); 
}
bool CFWCamera_DR2_2::getWhiteBalance(double &blue, double &red)
{ 
    return getWhiteBalance(blue,red); 
}	
double CFWCamera_DR2_2::getHue()
{ 
    return getFeatureDC1394(DC1394_FEATURE_HUE); 
}	
double CFWCamera_DR2_2::getSaturation()
{ 
    return CFWCamera_DR2_2::getFeatureDC1394(DC1394_FEATURE_SATURATION); 
}
double CFWCamera_DR2_2::getGamma()
{ 
    return getFeatureDC1394(DC1394_FEATURE_GAMMA); 
}
double CFWCamera_DR2_2::getShutter()
{ 
    return getFeatureDC1394(DC1394_FEATURE_SHUTTER);
}
double CFWCamera_DR2_2::getGain()
{ 
    return getFeatureDC1394(DC1394_FEATURE_GAIN);
}
double CFWCamera_DR2_2::getIris()
{ 
    return getFeatureDC1394(DC1394_FEATURE_IRIS); 
}
