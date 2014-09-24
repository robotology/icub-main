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

#include <stdlib.h>
#include "linux/FirewireCameraDC1394-DR2_2.h"
#include <arpa/inet.h>

#define POINTGREY_REGISTER_TIMESTAMP 0x12F8

static dc1394error_t set_embedded_timestamp(dc1394camera_t *camera, 
                                            bool enable) {
    uint32_t value;
    dc1394error_t err;

    err = dc1394_get_control_register(camera, 
                                      POINTGREY_REGISTER_TIMESTAMP, 
                                      &value);
    DC1394_ERR_RTN(err, "No embedded timestamp capability");
    
    bool current = (value & 0x1U)!=0;
    
    if(enable==current) {
        return DC1394_SUCCESS;
    }
    value ^= 0x1;
    err = dc1394_set_control_register(camera, 
                                      POINTGREY_REGISTER_TIMESTAMP, 
                                      value);
    DC1394_ERR_RTN(err, "Cannot configure embedded timestamp");
    err = dc1394_get_control_register(camera, 
                                      POINTGREY_REGISTER_TIMESTAMP, 
                                      &value);
    DC1394_ERR_RTN(err, "Configuration for embedded timestamp won't stick");

    current = value & 0x1;

    return (enable==current)?DC1394_SUCCESS: DC1394_FAILURE;
}

double CFWCamera_DR2_2::bytesPerPixel(dc1394color_coding_t pixelFormat)
{
    switch (pixelFormat)
    {
        case DC1394_COLOR_CODING_MONO8:   return 1.0;
        case DC1394_COLOR_CODING_MONO16:  return 2.0;
        case DC1394_COLOR_CODING_YUV411:  return 1.5;
        case DC1394_COLOR_CODING_YUV422:  return 2.0;
        case DC1394_COLOR_CODING_YUV444:  return 3.0;
        case DC1394_COLOR_CODING_RGB8:    return 3.0;
        case DC1394_COLOR_CODING_RAW8:    return 1.0;
        case DC1394_COLOR_CODING_RAW16:   return 2.0;
    }

    return 0.0;
}

int CFWCamera_DR2_2::maxFPS(dc1394video_mode_t mode,dc1394color_coding_t pixelFormat)
{
    if (mHires)
    {
        switch (mode)
        {
        case DC1394_VIDEO_MODE_FORMAT7_0:
            switch (pixelFormat)
            {
            case DC1394_COLOR_CODING_MONO8:  return 31;
            case DC1394_COLOR_CODING_RAW8:   return 31;
            case DC1394_COLOR_CODING_YUV411: return 26;

            case DC1394_COLOR_CODING_MONO16: return 15;
            case DC1394_COLOR_CODING_RAW16:  return 15;
            case DC1394_COLOR_CODING_YUV422: return 15;

            case DC1394_COLOR_CODING_YUV444: return 11;
            case DC1394_COLOR_CODING_RGB8:   return 11;
            }
            return 0;
        case DC1394_VIDEO_MODE_FORMAT7_1:
            switch (pixelFormat)
            {
            case DC1394_COLOR_CODING_MONO8:  return 54;
            case DC1394_COLOR_CODING_MONO16: return 54;

            case DC1394_COLOR_CODING_YUV411: return 31;
            case DC1394_COLOR_CODING_YUV422: return 31;
            case DC1394_COLOR_CODING_YUV444: return 31;
            case DC1394_COLOR_CODING_RGB8:   return 31;
            }
            return 0;
        case DC1394_VIDEO_MODE_FORMAT7_2:
            switch (pixelFormat)
            {
            case DC1394_COLOR_CODING_MONO8:  return 31;
            case DC1394_COLOR_CODING_MONO16: return 26;
            case DC1394_COLOR_CODING_YUV411: return 31;
            case DC1394_COLOR_CODING_YUV422: return 26;
            case DC1394_COLOR_CODING_YUV444: return 17;
            case DC1394_COLOR_CODING_RGB8:   return 17;
            }
            return 0;
        }
    }
    else // lores
    {
        switch (mode)
        {
        case DC1394_VIDEO_MODE_FORMAT7_0:
            switch (pixelFormat)
            {
            case DC1394_COLOR_CODING_MONO8:
            case DC1394_COLOR_CODING_RAW8:  
            case DC1394_COLOR_CODING_YUV411: return 59;
        
            case DC1394_COLOR_CODING_MONO16:
            case DC1394_COLOR_CODING_RAW16:
            case DC1394_COLOR_CODING_YUV422: return 47;
        
            case DC1394_COLOR_CODING_YUV444:
            case DC1394_COLOR_CODING_RGB8:    return 31;
            }
            return 0;
        case DC1394_VIDEO_MODE_FORMAT7_1:
            switch (pixelFormat)
            {
            case DC1394_COLOR_CODING_MONO8:
            case DC1394_COLOR_CODING_MONO16:  return 100;

            case DC1394_COLOR_CODING_YUV411:
            case DC1394_COLOR_CODING_YUV422:
            case DC1394_COLOR_CODING_YUV444:
            case DC1394_COLOR_CODING_RGB8:    return 59;
            }
            return 0;
        case DC1394_VIDEO_MODE_FORMAT7_2:
            switch (pixelFormat)
            {
            case DC1394_COLOR_CODING_MONO8:
            case DC1394_COLOR_CODING_MONO16:
            case DC1394_COLOR_CODING_YUV411:
            case DC1394_COLOR_CODING_YUV422:
            case DC1394_COLOR_CODING_YUV444:
            case DC1394_COLOR_CODING_RGB8:    return 59;
            }
            return 0;
        }
    }

    return 0;
}

bool CFWCamera_DR2_2::Create(yarp::os::Searchable& config)
{
    //bool bDR2=config.check("DR2");

    //bool bDR2=true;

    int size_x=checkInt(config,"width");   
    int size_y=checkInt(config,"height");
    int off_x=checkInt(config,"xoff");
    int off_y=checkInt(config,"yoff");
    int format=checkInt(config,"video_type");

    if (!config.check("use_network_time"))
    {
        mUseHardwareTimestamp = false;
    }
    else
    {
        mUseHardwareTimestamp = !checkInt(config,"use_network_time");
    }

    unsigned int idCamera=0;

    m_Framerate=checkInt(config,"framerate");
    
    fprintf(stderr,"Format = %d\n",format);

	m_XDim=m_YDim=0;
	m_RawBufferSize=0;
   
    m_bCameraOn=false;
    m_pCamera=NULL;
    m_pCameraList=NULL;
    m_dc1394_handle=NULL;
    m_nNumCameras=0;
    m_nInvalidFrames=0;
    m_ConvFrame.image=new unsigned char[1032*776*3*2];
    m_ConvFrame_tmp.image=new unsigned char[1032*776*2];

    if (!(m_dc1394_handle=dc1394_new()))
    {
        fprintf(stderr,"ERROR: failed to open Firewire Bus Manager\n");
        fprintf(stderr,"LINE: %d\n",__LINE__);
        return false;
    }

    dc1394error_t error;
    
    error=dc1394_camera_enumerate(m_dc1394_handle,&m_pCameraList);
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

    m_nNumCameras=m_pCameraList->num;

    if (!m_nNumCameras)
    {
        fprintf(stderr,"ERROR: no active cameras\n");
        fprintf(stderr,"LINE: %d\n",__LINE__); 
        return false;
    }

	if (config.check("guid"))
	{
		yarp::os::ConstString sguid=config.find("guid").asString();

		uint64_t guid=strtoull(sguid.c_str(),NULL,16);

		for (idCamera=0; idCamera<m_nNumCameras; ++idCamera)
		{
			if (guid==m_pCameraList->ids[idCamera].guid)
			{
				break;
			}
		}

		if (idCamera==m_nNumCameras)
		{
			fprintf(stderr,"ERROR: GUID=%s camera not found\n",sguid.c_str());
			fprintf(stderr,"LINE: %d\n",__LINE__);
            return false;  
		}
	}
    else if (config.check("d"))
    {
        fprintf(stderr,"WARNING: --d <unit_number> parameter is deprecated, use --guid <64_bit_global_unique_identifier> instead\n");
        idCamera=config.find("d").asInt(); 
    }

    if (idCamera<0 || idCamera>=m_nNumCameras)
    {
        fprintf(stderr,"ERROR: invalid camera number\n");
        fprintf(stderr,"LINE: %d\n",__LINE__); 
        return false;       
    }

    if (!(m_pCamera=dc1394_camera_new(m_dc1394_handle,m_pCameraList->ids[idCamera].guid)))
	{
		fprintf(stderr,"ERROR: can't create camera\n");
        fprintf(stderr,"LINE: %d\n",__LINE__); 
		return false;
	}

	error=dc1394_camera_reset(m_pCamera);
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

    uint32_t channel_in_use=0;
    dc1394_video_get_iso_channel(m_pCamera,&channel_in_use);
    dc1394_video_set_iso_channel(m_pCamera,idCamera);

    /*
    if (channel_in_use != 0)
    {                
        dc1394_iso_release_channel(m_pCamera,idCamera);        
        const uint64_t ISO_CHANNEL_MASK=idCamera;
        int allocated_channel;        
        dc1394_iso_allocate_channel(m_pCamera,ISO_CHANNEL_MASK,&allocated_channel);
        dc1394_video_set_iso_channel(m_pCamera,idCamera);		 
        dc1394_iso_release_channel(m_pCamera,channel_in_use);
    }
    */

    // if previous instance crashed we need to clean up
    // allocated bandwidth -- Added Lorenzo Natale, 9/2/2010.
#ifdef _ENABLE_LEGACY_STACK_
    // ifdeffed by Alberto Cardellino on 08/01/2014 in
    // order to enable compatibility with new firewire stack into kernel
    // and avoid recompiling the kernel itself with old legacy support.
    // Pay attention to see if the cleanup problem shows up again
    const int BANDWIDTH_MAX=0x7FFFFFFF;
    error=dc1394_iso_release_bandwidth(m_pCamera, BANDWIDTH_MAX);
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }
#endif

	dc1394speed_t isoSpeed;
	error=dc1394_video_get_iso_speed(m_pCamera,&isoSpeed);
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

    dc1394_video_set_operation_mode(m_pCamera,DC1394_OPERATION_MODE_1394B);       
    //if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

    error=dc1394_video_set_iso_speed(m_pCamera,DC1394_ISO_SPEED_400);
	if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

    error=dc1394_camera_print_info(m_pCamera,stdout); 
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

    // CONFIGURE

    //error=m_pCamera->RestoreFromMemoryChannel(0);
    //if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

    mHires=(strcmp("Dragonfly2 DR2-08S2C-EX",m_pCamera->model)==0);

    if (mHires) printf("Hires camera found\n");

// formats
/*
#define DR_UNINIT                0
#define DR_RGB_320x240           1
#define DR_RGB_640x480           2
#define DR_BAYER_640x480         3
#define DR_BAYER_RAW16_640x480   4
#define DR_YUV422_640x480        5

#define DR_RGB_512x384           6

#define DR_RGB_800x600           7
#define DR_YUV_800x600           8

#define DR_RGB_1024x768          8
#define DR_YUV_1024x768          9
#define DR_BAYER_1024x768        10
*/

    if (mHires)
    {
        if (!format) format=DR_BAYER_1024x768;

        if (mRawDriver) format=DR_BAYER_1024x768;
    
        switch (format)
        {
        case DR_UNINIT:
            break;

        case DR_RGB_512x384:
            if (!size_x) { size_x=512; }
            if (!size_y) { size_y=384; }
            SetF7(DC1394_VIDEO_MODE_FORMAT7_1,size_x,size_y,DC1394_COLOR_CODING_RGB8,44,off_x,off_y);
            break; 

        case DR_RGB_640x480:
            SetVideoMode(DC1394_VIDEO_MODE_640x480_RGB8);
            break;

        case DR_YUV_640x480:
            SetVideoMode(DC1394_VIDEO_MODE_640x480_YUV422);
            break;

        case DR_RGB_800x600:
            SetVideoMode(DC1394_VIDEO_MODE_800x600_RGB8);
            break; 

        case DR_YUV_800x600:
            SetVideoMode(DC1394_VIDEO_MODE_800x600_YUV422);
            break; 
        
        case DR_RGB_1024x768:
            if (!size_x) { size_x=1024; }
            if (!size_y) { size_y=768; }
            SetF7(DC1394_VIDEO_MODE_FORMAT7_0,size_x,size_y,DC1394_COLOR_CODING_RGB8,44,off_x,off_y);
            break;
        
        case DR_YUV_1024x768:
            if (!size_x) { size_x=1024; }
            if (!size_y) { size_y=768; }
            SetF7(DC1394_VIDEO_MODE_FORMAT7_0,size_x,size_y,DC1394_COLOR_CODING_YUV422,44,off_x,off_y);
            break;

        case DR_BAYER_1024x768:
            if (!size_x) { size_x=1024; }
            if (!size_y) { size_y=768; }
            SetF7(DC1394_VIDEO_MODE_FORMAT7_0,size_x,size_y,DC1394_COLOR_CODING_RAW8,44,off_x,off_y);
            break;

        case DR_RGB_320x240:
            SetVideoMode(DC1394_VIDEO_MODE_320x240_YUV422);
            break;

        default:
            fprintf(stderr,"Unsupported video format, camera configuration will be used.\n");     
        }
    }
    else // lores
    {
        if (!format) format=DR_BAYER_640x480;

        if (mRawDriver) format=DR_BAYER_640x480;

        switch (format)
        {
        case DR_UNINIT:
            break;

        case DR_RGB_320x240:
            if (!size_x) { size_x=320; }
            if (!size_y) { size_y=240; }
            SetF7(DC1394_VIDEO_MODE_FORMAT7_1,size_x,size_y,DC1394_COLOR_CODING_RGB8,44,off_x,off_y);
            break;

        case DR_RGB_640x480:
            if (!size_x) { size_x=640; }
            if (!size_y) { size_y=480; }
            SetF7(DC1394_VIDEO_MODE_FORMAT7_0,size_x,size_y,DC1394_COLOR_CODING_RGB8,44,off_x,off_y);
            break;

        case DR_BAYER_640x480:
            if (!size_x) { size_x=640; }
            if (!size_y) { size_y=480; }
            SetF7(DC1394_VIDEO_MODE_FORMAT7_0,size_x,size_y,DC1394_COLOR_CODING_RAW8,44,off_x,off_y);
            break;

        case DR_BAYER16_640x480:
            if (!size_x) { size_x=640; }
            if (!size_y) { size_y=480; }
            SetF7(DC1394_VIDEO_MODE_FORMAT7_0,size_x,size_y,DC1394_COLOR_CODING_RAW16,44,off_x,off_y);
            break;

        case DR_YUV_640x480:
            if (!size_x) { size_x=640; }
            if (!size_y) { size_y=480; }
            SetF7(DC1394_VIDEO_MODE_FORMAT7_0,size_x,size_y,DC1394_COLOR_CODING_YUV422,44,off_x,off_y);
            break;

        default:
            fprintf(stderr,"Unsupported video format, camera configuration will be used.\n");    
        }
    }

	for (int f=DC1394_FEATURE_MIN; f<=DC1394_FEATURE_MAX; ++f)
	{	
		dc1394bool_t bPresent;
		error=dc1394_feature_is_present(m_pCamera,(dc1394feature_t)f,&bPresent);

        if (error==DC1394_SUCCESS)
        {
		    if (bPresent)
		    {
			    dc1394_feature_get_boundaries(m_pCamera,(dc1394feature_t)f,&m_iMin[f-DC1394_FEATURE_MIN],&m_iMax[f-DC1394_FEATURE_MIN]);

			    dc1394bool_t bSwitch;
			    dc1394_feature_is_switchable(m_pCamera,(dc1394feature_t)f,&bSwitch);

			    if (bSwitch)
			    {
                    dc1394switch_t turnOn=(dc1394switch_t)(f!=DC1394_FEATURE_EXPOSURE && f!=DC1394_FEATURE_IRIS && f<DC1394_FEATURE_TEMPERATURE);
			        dc1394_feature_set_power(m_pCamera,(dc1394feature_t)f,turnOn);
                }
                dc1394_feature_set_mode(m_pCamera,(dc1394feature_t)f,DC1394_FEATURE_MODE_MANUAL);
			    dc1394_feature_set_absolute_control(m_pCamera,(dc1394feature_t)f,DC1394_OFF);
		    }
            else
            {
                fprintf(stderr,"Feature %d not present\n",f);
            }
        }
        else
        {
            fprintf(stderr,"Feature %d error %d\n",f,error);
        }
	}

    error=dc1394_capture_setup(m_pCamera,NUM_DMA_BUFFERS,DC1394_CAPTURE_FLAGS_DEFAULT);
	if (error!=DC1394_SUCCESS)
	{
        fprintf(stderr,"ERROR: %d can't setup capture\n",error);
		dc1394_camera_free(m_pCamera);
		m_pCamera=NULL;
		return false;
	}

    error=dc1394_video_set_transmission(m_pCamera,DC1394_ON);
	if (error!=DC1394_SUCCESS)
	{
		fprintf(stderr,"ERROR: %d can't start transmission\n");
		dc1394_camera_free(m_pCamera);
		m_pCamera=NULL;
		return false;
	}

	setBroadcastDC1394(false);

    m_bCameraOn=true;

    // parameters

    m_GainSaveModeAuto=DC1394_FEATURE_MODE_MANUAL;
    m_ShutterSaveModeAuto=DC1394_FEATURE_MODE_MANUAL;
    
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

    dc1394_feature_get_value(m_pCamera,DC1394_FEATURE_SHUTTER,&m_ShutterSaveValue);
    dc1394_feature_get_value(m_pCamera,DC1394_FEATURE_GAIN,&m_GainSaveValue);

    if (mUseHardwareTimestamp) {
        set_embedded_timestamp(m_pCamera,true);
    }

    return true;
}

void CFWCamera_DR2_2::Close()
{
	if (m_pCamera)
	{
		dc1394_video_set_transmission(m_pCamera,DC1394_OFF);
		dc1394_capture_stop(m_pCamera);
        dc1394_iso_release_all(m_pCamera);
		dc1394_camera_free(m_pCamera);
	}
	m_pCamera=NULL;

	if (m_pCameraList) dc1394_camera_free_list(m_pCameraList);
	m_pCameraList=NULL;

	if (m_dc1394_handle) dc1394_free(m_dc1394_handle);
	m_dc1394_handle=NULL;

	if (m_ConvFrame.image) delete [] m_ConvFrame.image;
	m_ConvFrame.image=NULL;
}

bool CFWCamera_DR2_2::SetVideoMode(dc1394video_mode_t videoMode)
{
    if (!m_pCamera) return false;

    if (mRawDriver) return false;

    int xdim,ydim,buffDim;
    dc1394framerate_t maxFramerate;

    // calculate raw image size at given video mode
    if (mHires)
    {
        switch (videoMode)
        {
        case DC1394_VIDEO_MODE_160x120_YUV444: 
            xdim=160; 
            ydim=120; 
            buffDim=xdim*ydim*3;
            maxFramerate=DC1394_FRAMERATE_30; 
            break;
        case DC1394_VIDEO_MODE_320x240_YUV422: 
            xdim=320; 
            ydim=240; 
            buffDim=xdim*ydim*2;   
            maxFramerate=DC1394_FRAMERATE_30; 
            break;
        case DC1394_VIDEO_MODE_640x480_YUV411: 
            xdim=640; 
            ydim=480; 
            buffDim=(xdim*ydim*3)/2; 
            maxFramerate=DC1394_FRAMERATE_30; 
            break;
        case DC1394_VIDEO_MODE_640x480_YUV422: 
            xdim=640; 
            ydim=480; 
            buffDim=xdim*ydim*2;    
            maxFramerate=DC1394_FRAMERATE_30; 
            break;
        case DC1394_VIDEO_MODE_640x480_RGB8:
            xdim=640; 
            ydim=480; 
            buffDim=xdim*ydim*3;
            maxFramerate=DC1394_FRAMERATE_30;
            break;
        case DC1394_VIDEO_MODE_640x480_MONO8:  
            xdim=640; 
            ydim=480; 
            buffDim=xdim*ydim;
            maxFramerate=DC1394_FRAMERATE_30;
            break;
        case DC1394_VIDEO_MODE_640x480_MONO16: 
            xdim=640; 
            ydim=480; 
            buffDim=xdim*ydim*2;
            maxFramerate=DC1394_FRAMERATE_30;
            break;

        case DC1394_VIDEO_MODE_800x600_YUV422:
            xdim=800;
            ydim=600;
            buffDim=xdim*ydim*2;
            maxFramerate=DC1394_FRAMERATE_30;
            break;
        case DC1394_VIDEO_MODE_800x600_RGB8:
            xdim=800;
            ydim=600;
            buffDim=xdim*ydim*3;
            maxFramerate=DC1394_FRAMERATE_15;
            break;
        case DC1394_VIDEO_MODE_800x600_MONO8:
            xdim=800;
            ydim=600;
            buffDim=xdim*ydim;
            maxFramerate=DC1394_FRAMERATE_30;
            break;
        case DC1394_VIDEO_MODE_800x600_MONO16:
            xdim=800;
            ydim=600;
            buffDim=xdim*ydim*2;
            maxFramerate=DC1394_FRAMERATE_30;
            break;

        case DC1394_VIDEO_MODE_1024x768_YUV422:
            xdim=1024;
            ydim=768;
            buffDim=xdim*ydim*2;
            maxFramerate=DC1394_FRAMERATE_15;
            break;
        case DC1394_VIDEO_MODE_1024x768_RGB8:
            xdim=1024;
            ydim=768;
            buffDim=xdim*ydim*3;
            maxFramerate=DC1394_FRAMERATE_7_5;
            break;
        case DC1394_VIDEO_MODE_1024x768_MONO8:
            xdim=1024;
            ydim=768;
            buffDim=xdim*ydim;
            maxFramerate=DC1394_FRAMERATE_30;
            break;
        case DC1394_VIDEO_MODE_1024x768_MONO16:
            xdim=1024;
            ydim=768;
            buffDim=xdim*ydim*2;
            maxFramerate=DC1394_FRAMERATE_15;
            break;
        /*
        case DC1394_VIDEO_MODE_FORMAT7_0: 
            xdim=1024; 
            ydim=768; 
            buffDim=xdim*ydim*3;
            maxFramerate=DC1394_FRAMERATE_;
            break;
        */
        default: return false;
        }
    }
    else
    {
        switch (videoMode)
        {
        case DC1394_VIDEO_MODE_160x120_YUV444: 
            xdim=160; 
            ydim=120; 
            buffDim=xdim*ydim*3;
            maxFramerate=DC1394_FRAMERATE_60; 
            break;
        case DC1394_VIDEO_MODE_320x240_YUV422: 
            xdim=320; 
            ydim=240; 
            buffDim=xdim*ydim*2;   
            maxFramerate=DC1394_FRAMERATE_60; 
            break;
        case DC1394_VIDEO_MODE_640x480_YUV411: 
            xdim=640; 
            ydim=480; 
            buffDim=(xdim*ydim*3)/2; 
            maxFramerate=DC1394_FRAMERATE_60; 
            break;
        case DC1394_VIDEO_MODE_640x480_YUV422: 
            xdim=640; 
            ydim=480; 
            buffDim=xdim*ydim*2;    
            maxFramerate=DC1394_FRAMERATE_30; 
            break;
        case DC1394_VIDEO_MODE_640x480_RGB8:
            xdim=640; 
            ydim=480; 
            buffDim=xdim*ydim*3;
            maxFramerate=DC1394_FRAMERATE_30;
            break;
        case DC1394_VIDEO_MODE_640x480_MONO8:  
            xdim=640; 
            ydim=480; 
            buffDim=xdim*ydim;
            maxFramerate=DC1394_FRAMERATE_30;
            break;
        case DC1394_VIDEO_MODE_640x480_MONO16: 
            xdim=640; 
            ydim=480; 
            buffDim=xdim*ydim*2;
            maxFramerate=DC1394_FRAMERATE_60;
            break;
        /*
        case DC1394_VIDEO_MODE_FORMAT7_0: 
            xdim=640; 
            ydim=480; 
            buffDim=xdim*ydim*3;
            maxFramerate=DC1394_FRAMERATE_60;
            break;
        */
        default: return false;
        }
    }

	dc1394speed_t isoSpeed;
	dc1394error_t error;
	error=dc1394_video_get_iso_speed(m_pCamera,&isoSpeed);
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

    // get ISO bandwidth
    int busSpeed=10000000<<(isoSpeed-DC1394_ISO_SPEED_MIN);

    // calculate maximum allowed framerate at given image format
    static const double twoCams=0.5; // only half bandwith available with two cams
    double fpsMax=twoCams*double(busSpeed)/double(buffDim);
    printf("fpsMax = %f\n",fpsMax);

    dc1394framerate_t framerate;
    
    if (m_Framerate)
    { 
        if (m_Framerate<(int)fpsMax)
        {
            fpsMax=double(m_Framerate);
        }
        else
        {
            fprintf(stderr,"WARNING: framerate %d is too high, it will be set to the maximum available\n",m_Framerate);
        }
        
        m_Framerate=0;
    }

    // choose framerate according to maximum allowed
    if (fpsMax<1.785){ return false; }
    else if (fpsMax<3.75) { framerate=DC1394_FRAMERATE_1_875; }
    else if (fpsMax<7.5)  { framerate=DC1394_FRAMERATE_3_75; }
    else if (fpsMax<15.0) { framerate=DC1394_FRAMERATE_7_5; }
    else if (fpsMax<30.0) { framerate=DC1394_FRAMERATE_15; }
    else if (fpsMax<60.0) { framerate=DC1394_FRAMERATE_30; }
    else if (fpsMax<120.0){ framerate=DC1394_FRAMERATE_60; }
    else if (fpsMax<120.0){ framerate=DC1394_FRAMERATE_60; }
    else if (fpsMax<240.0){ framerate=DC1394_FRAMERATE_120; }
    else                  { framerate=DC1394_FRAMERATE_240; }

    if (framerate>maxFramerate)
    {
        framerate=maxFramerate;
    }
    
    fprintf(stderr,"Framerate = %f\n",1.875*double(1<<(framerate-DC1394_FRAMERATE_MIN)));

    error=dc1394_video_set_mode(m_pCamera,videoMode);
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

    m_XDim=xdim;
    m_YDim=ydim;
    m_RawBufferSize=buffDim;

    error=dc1394_video_set_framerate(m_pCamera,framerate);
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

    return true;
}

#define SKIP 0x80000000
bool CFWCamera_DR2_2::SetF7(int newVideoMode,int newXdim,int newYdim,int newColorCoding,int newBandPercent,int x0,int y0)
{
    if (!m_pCamera) return false;

    if (mRawDriver)
    {
        if (newVideoMode!=DC1394_VIDEO_MODE_FORMAT7_0 || newColorCoding!=DC1394_COLOR_CODING_RAW8) return false;
    }

    dc1394color_coding_t actColorCoding=DC1394_COLOR_CODING_RGB8;
    unsigned int actPacketSize=0;

    dc1394video_mode_t actVideoMode;
    dc1394error_t error;
    error=dc1394_video_get_mode(m_pCamera,&actVideoMode);
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

    if (actVideoMode>=DC1394_VIDEO_MODE_FORMAT7_0 && actVideoMode<=DC1394_VIDEO_MODE_FORMAT7_2)
    {
        error=dc1394_format7_get_packet_size(m_pCamera,actVideoMode,&actPacketSize);
        if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }
        error=dc1394_format7_get_color_coding(m_pCamera,actVideoMode,&actColorCoding);
        if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }
    }
    else if (newVideoMode==SKIP) // we're not in F7 mode and no mode is specified!
    {
        fprintf(stderr,"ERROR: no format 7 mode specified\n");
        return false;
    }

    ///////////////////////////
    // is given video mode supported?
    //
    if (newVideoMode==SKIP)
    {
        newVideoMode=actVideoMode;
    }
    dc1394video_modes_t modes;
	dc1394_video_get_supported_modes(m_pCamera,&modes);
    bool bSupported=false;
	for (unsigned int m=0; m<modes.num; ++m)
    {
		if (modes.modes[m]==(dc1394video_mode_t)newVideoMode)
        {
            bSupported=true;
            break;
        }
    }
    if (!bSupported)
    {
        fprintf(stderr,"ERROR: format 7 video mode %d not supported\n",newVideoMode-DC1394_VIDEO_MODE_FORMAT7_MIN);
        return false;
    }
    //
    ////////////////////////////

    ////////////////////////////
    // is given pixel format supported?
    //
    if (newColorCoding==SKIP)
    {
        newColorCoding=actColorCoding;
    }
	dc1394color_codings_t codings;
	dc1394_format7_get_color_codings(m_pCamera,(dc1394video_mode_t)newVideoMode,&codings);
	bSupported=false;
	for (unsigned int m=0; m<codings.num; ++m)
    {
		if (codings.codings[m]==(dc1394color_coding_t)newColorCoding)
        {
            bSupported=true;
            break;
        }
    }
    if (!bSupported)
    {
        fprintf(stderr,"ERROR: invalid format 7 pixel format %d\n",newColorCoding-DC1394_COLOR_CODING_MIN);
        return false;
    }
    //
    //////////////////////////

    unsigned int maxWidth,maxHeight,wStep,hStep,xStep,yStep;

    error=dc1394_format7_get_max_image_size(m_pCamera,(dc1394video_mode_t)newVideoMode,&maxWidth,&maxHeight);
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

    error=dc1394_format7_get_unit_size(m_pCamera,(dc1394video_mode_t)newVideoMode,&wStep,&hStep);
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

    error=dc1394_format7_get_unit_position(m_pCamera,(dc1394video_mode_t)newVideoMode,&xStep,&yStep);
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

    if (newVideoMode==DC1394_VIDEO_MODE_FORMAT7_1) 
    {
        wStep*=2;
    }

    if (actVideoMode<DC1394_VIDEO_MODE_FORMAT7_0)
    {
        if (newXdim==SKIP) newXdim=(int)maxWidth;
        if (newYdim==SKIP) newYdim=(int)maxHeight;
        if (x0==SKIP) x0=0;
        if (y0==SKIP) y0=0;
    }
    else
    {
        unsigned int xdim,ydim,xpos,ypos;
        error=dc1394_format7_get_image_size(m_pCamera,actVideoMode,&xdim,&ydim);
        if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }
        error=dc1394_format7_get_image_position(m_pCamera,actVideoMode,&xpos,&ypos);
        if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }
        
        if (newXdim==SKIP) newXdim=xdim;
        if (newYdim==SKIP) newYdim=ydim;
        if (x0==SKIP) x0=xpos;
        if (y0==SKIP) y0=ypos;
    }

    // adjust image size to allowed in this format
    if (newXdim>(int)maxWidth)  newXdim=(int)maxWidth;
    if (newYdim>(int)maxHeight) newYdim=(int)maxHeight;

    newXdim=(newXdim/wStep)*wStep;
    newYdim=(newYdim/hStep)*hStep;

    // calculate offset
    int xOff=(maxWidth -newXdim)/2+x0;
    int yOff=(maxHeight-newYdim)/2+y0;
    xOff=(xOff/xStep)*xStep;
    yOff=(yOff/yStep)*yStep;
    
    if (xOff<0) xOff=0;
    if (yOff<0) yOff=0;

    if (xOff+newXdim>maxWidth)  xOff=maxWidth -newXdim;
    if (yOff+newYdim>maxHeight) yOff=maxHeight-newYdim;

    error=dc1394_video_set_mode(m_pCamera,(dc1394video_mode_t)newVideoMode);
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }
    error=dc1394_format7_set_color_coding(m_pCamera,(dc1394video_mode_t)newVideoMode,(dc1394color_coding_t)newColorCoding);
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

    //////////////////
    // speed
    //////////////////
    if (newBandPercent<0)
    { 
        newBandPercent=40;
    } 
    else if (newBandPercent>100) 
    {
        newBandPercent=100;
    }
    
    dc1394speed_t isoSpeed;
    error=dc1394_video_get_iso_speed(m_pCamera,&isoSpeed);
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

    // get ISO bandwidth
    int busBand=10000000<<(isoSpeed-DC1394_ISO_SPEED_MIN);
    int fpsMax=maxFPS((dc1394video_mode_t)newVideoMode,(dc1394color_coding_t)newColorCoding);
    double bpp=bytesPerPixel((dc1394color_coding_t)newColorCoding);
    double newBandOcc=double(fpsMax*newXdim*newYdim)*bpp;
    
    int fps=fpsMax;
    
    unsigned int roundUp=0;
    if (m_Framerate)
    { 
        roundUp=1;
        
        if (m_Framerate<fps)
        {
            fps=m_Framerate;
        }
        else
        {
            fprintf(stderr,"WARNING: framerate %d is too high, it will be set to the maximum available %d\n",m_Framerate,fps);
        }
        
        m_Framerate=0;
    }
    
    unsigned int bytesPerPacket,maxBytesPerPacket,unitBytesPerPacket;
    error=dc1394_format7_get_packet_parameters(m_pCamera,(dc1394video_mode_t)newVideoMode,&unitBytesPerPacket,&maxBytesPerPacket);
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }
    bytesPerPacket=(unsigned int)(0.01*double(fps)/double(fpsMax)*double(newBandPercent)*double(busBand)*double(maxBytesPerPacket)/newBandOcc);
    bytesPerPacket=(roundUp+bytesPerPacket/unitBytesPerPacket)*unitBytesPerPacket;
    
    if (bytesPerPacket>maxBytesPerPacket)
    {
        bytesPerPacket=maxBytesPerPacket;
    }
    else if (bytesPerPacket<unitBytesPerPacket)
    {
        bytesPerPacket=unitBytesPerPacket;
    }
    
    printf("\nfps=%d newBandOcc=%f bpp=%f bytesPerPacket=%d maxBytesPerPacket=%d\n\n",fps,newBandOcc,bpp,bytesPerPacket,maxBytesPerPacket);
    
    error=dc1394_format7_set_roi(m_pCamera,(dc1394video_mode_t)newVideoMode,(dc1394color_coding_t)newColorCoding,bytesPerPacket,xOff,yOff,newXdim,newYdim);
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

    m_XDim=newXdim;
    m_YDim=newYdim;
    m_RawBufferSize=(unsigned int)(double(newXdim*newYdim)*bpp);

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

    dc1394error_t ret=dc1394_capture_dequeue(m_pCamera,DC1394_CAPTURE_POLICY_WAIT,&m_pFrame); 

	if (ret!=DC1394_SUCCESS)
	{
        if (ret<0)
        {
            fprintf(stderr,"\nWARNING: dc1394_capture_dequeue returned %d\n\n",ret);
        }
        else
        {
            fprintf(stderr,"\nERROR: dc1394_capture_dequeue returned %d\n\n",ret);
        }
		
        m_AcqMutex.post();
		return false;
	}
	
	m_pFramePoll=0;	
	while (dc1394_capture_dequeue(m_pCamera,DC1394_CAPTURE_POLICY_POLL,&m_pFramePoll)==DC1394_SUCCESS && m_pFramePoll)
	{
	    dc1394_capture_enqueue(m_pCamera,m_pFrame);
	    m_pFrame=m_pFramePoll;
	    m_pFramePoll=0; 
	}

	if (m_nInvalidFrames)
	{
		--m_nInvalidFrames;
		dc1394_capture_enqueue(m_pCamera,m_pFrame);
		m_AcqMutex.post();
		return false;
	}

    if (mUseHardwareTimestamp) {
        uint32_t v = ntohl(*((uint32_t*)m_pFrame->image));
        int nSecond = (v >> 25) & 0x7f;
        int nCycleCount  = (v >> 12) & 0x1fff;
        int nCycleOffset = (v >> 0) & 0xfff;

        if (m_LastSecond>nSecond) {
            // we got a wrap-around event, losing 128 seconds
            m_SecondOffset += 128;
        }
        m_LastSecond = nSecond;

        m_Stamp.update(m_SecondOffset+(double)nSecond + (((double)nCycleCount+((double)nCycleOffset/3072.0))/8000.0));
        //m_Stamp.update(m_pFrame->timestamp/1000000.0);
    } else {
        m_Stamp.update();
    }

	if (pImage)
	{
		pImage->resize(m_pFrame->size[0],m_pFrame->size[1]);
		pBuffer=pImage->getRawImage();
	}

	if (m_pFrame->color_coding==DC1394_COLOR_CODING_RGB8 || bRaw)
	{
		memcpy(pBuffer,m_pFrame->image,m_pFrame->size[0]*m_pFrame->size[1]*(bRaw?1:3));
	}
	else if (m_pFrame->color_coding==DC1394_COLOR_CODING_RAW8)
	{
		dc1394_debayer_frames(m_pFrame,&m_ConvFrame,DC1394_BAYER_METHOD_BILINEAR);
		memcpy(pBuffer,m_ConvFrame.image,m_ConvFrame.size[0]*m_ConvFrame.size[1]*3);
	}
	else if (m_pFrame->color_coding==DC1394_COLOR_CODING_RAW16)
	{
		dc1394_debayer_frames(m_pFrame,&m_ConvFrame,DC1394_BAYER_METHOD_BILINEAR);
		m_ConvFrame_tmp.size[0]=m_pFrame->size[0];
		m_ConvFrame_tmp.size[1]=m_pFrame->size[1];
		m_ConvFrame_tmp.position[0]=0;
		m_ConvFrame_tmp.position[1]=0;
		m_ConvFrame_tmp.color_coding=DC1394_COLOR_CODING_RGB8;
		m_ConvFrame_tmp.data_depth=24;
		m_ConvFrame_tmp.image_bytes=m_ConvFrame_tmp.total_bytes=3*m_pFrame->size[0]*m_pFrame->size[1];
		m_ConvFrame_tmp.padding_bytes=0;
		m_ConvFrame_tmp.stride=3*m_pFrame->size[0];
		m_ConvFrame_tmp.data_in_padding=DC1394_FALSE;
		m_ConvFrame_tmp.little_endian=m_pFrame->little_endian;
		dc1394_convert_frames(&m_ConvFrame,&m_ConvFrame_tmp);
		memcpy(pBuffer,m_ConvFrame_tmp.image,m_ConvFrame_tmp.size[0]*m_ConvFrame_tmp.size[1]*3);
	}
	else
	{
		m_ConvFrame.size[0]=m_pFrame->size[0];
		m_ConvFrame.size[1]=m_pFrame->size[1];
		m_ConvFrame.position[0]=0;
		m_ConvFrame.position[1]=0;
		m_ConvFrame.color_coding=DC1394_COLOR_CODING_RGB8;
		m_ConvFrame.data_depth=24;
		m_ConvFrame.image_bytes=m_ConvFrame.total_bytes=3*m_pFrame->size[0]*m_pFrame->size[1];
		m_ConvFrame.padding_bytes=0;
		m_ConvFrame.stride=3*m_pFrame->size[0];
		m_ConvFrame.data_in_padding=DC1394_FALSE;
		m_ConvFrame.little_endian=m_pFrame->little_endian;

		dc1394_convert_frames(m_pFrame,&m_ConvFrame);

		memcpy(pBuffer,m_ConvFrame.image,m_ConvFrame.size[0]*m_ConvFrame.size[1]*3);
	}

	dc1394_capture_enqueue(m_pCamera,m_pFrame);
	m_AcqMutex.post();
	return true;
}

bool CFWCamera_DR2_2::Capture(yarp::sig::ImageOf<yarp::sig::PixelMono>* pImage)
{
	m_AcqMutex.wait();

	if (!m_bCameraOn || dc1394_capture_dequeue(m_pCamera,DC1394_CAPTURE_POLICY_WAIT,&m_pFrame)!=DC1394_SUCCESS)
	{
		m_AcqMutex.post();
		return false;
	}
	
	m_pFramePoll=0;	
	while (dc1394_capture_dequeue(m_pCamera,DC1394_CAPTURE_POLICY_POLL,&m_pFramePoll)==DC1394_SUCCESS && m_pFramePoll)
	{
	    dc1394_capture_enqueue(m_pCamera,m_pFrame);
	    m_pFrame=m_pFramePoll;
	    m_pFramePoll=0; 
	}

	if (m_nInvalidFrames)
	{
		--m_nInvalidFrames;
		dc1394_capture_enqueue(m_pCamera,m_pFrame);
		m_AcqMutex.post();
		return false;
	}

    if (mUseHardwareTimestamp) {
        uint32_t v = ntohl(*((uint32_t*)m_pFrame->image));
        int nSecond = (v >> 25) & 0x7f;
        int nCycleCount  = (v >> 12) & 0x1fff;
        int nCycleOffset = (v >> 0) & 0xfff;

        if (m_LastSecond>nSecond) {
            // we got a wrap-around event, losing 128 seconds
            m_SecondOffset += 128;
        }
        m_LastSecond = nSecond;

        m_Stamp.update(m_SecondOffset+(double)nSecond + (((double)nCycleCount+((double)nCycleOffset/3072.0))/8000.0));
        //m_Stamp.update(m_pFrame->timestamp/1000000.0);
    } else {
        m_Stamp.update();
    }

	if (pImage)
	{
		pImage->resize(m_pFrame->size[0],m_pFrame->size[1]);
	    memcpy(pImage->getRawImage(),m_pFrame->image,m_pFrame->size[0]*m_pFrame->size[1]);
    }

	dc1394_capture_enqueue(m_pCamera,m_pFrame);
	m_AcqMutex.post();
	return true;
}

uint32_t CFWCamera_DR2_2::NormToValue(double& dVal,int feature)
{
	int f=feature-DC1394_FEATURE_MIN;

	if (dVal<0.0) dVal=0.0;
	if (dVal>1.0) dVal=1.0;

	uint32_t iVal=m_iMin[f]+(uint32_t)(dVal*double(m_iMax[f]-m_iMin[f]));

	if (iVal<m_iMin[f]) iVal=m_iMin[f];
	if (iVal>m_iMax[f]) iVal=m_iMax[f];

	return iVal;
}

double CFWCamera_DR2_2::ValueToNorm(uint32_t iVal,int feature)
{ 
	int f=feature-DC1394_FEATURE_MIN;

	double dVal=double(iVal-m_iMin[f])/double(m_iMax[f]-m_iMin[f]);

	if (dVal<0.0) return 0.0;
	if (dVal>1.0) return 1.0;

	return dVal;
}

////////////////////
// feature functions
////////////////////

// 00
bool CFWCamera_DR2_2::hasFeatureDC1394(int feature)
{
	if (!m_pCamera) return false;
	feature+=DC1394_FEATURE_MIN;
	dc1394bool_t value;
	dc1394_feature_is_present(m_pCamera,(dc1394feature_t)feature,&value);
	return value;
}

// 01
bool CFWCamera_DR2_2::setFeatureDC1394(int feature,double value)
{
    if (value<0.0 || value>1.0) return false;
	if (!m_pCamera) return false;
	feature+=DC1394_FEATURE_MIN;
	return DC1394_SUCCESS==dc1394_feature_set_value(m_pCamera,(dc1394feature_t)feature,NormToValue(value,feature));
}

// 02
double CFWCamera_DR2_2::getFeatureDC1394(int feature)
{
	if (!m_pCamera) return -1.0;
	feature+=DC1394_FEATURE_MIN;
	uint32_t iVal;
	dc1394_feature_get_value(m_pCamera,(dc1394feature_t)feature,&iVal); 
	return ValueToNorm(iVal,feature);
}

// 03
bool CFWCamera_DR2_2::hasOnOffDC1394(int feature)
{
	if (!m_pCamera) return false;
	feature+=DC1394_FEATURE_MIN;
	dc1394bool_t value;
	dc1394_feature_is_switchable(m_pCamera,(dc1394feature_t)feature,&value);
	return value;
}

// 04
bool CFWCamera_DR2_2::setActiveDC1394(int feature, bool onoff)
{
	if (!m_pCamera) return false;
	feature+=DC1394_FEATURE_MIN;
	
	if (feature==DC1394_FEATURE_EXPOSURE)
	{   
	    if (onoff)
	    {
	        dc1394_feature_set_power(m_pCamera,DC1394_FEATURE_GAIN,DC1394_ON);
	        dc1394_feature_set_power(m_pCamera,DC1394_FEATURE_SHUTTER,DC1394_ON);

            dc1394_feature_get_mode(m_pCamera,DC1394_FEATURE_GAIN,&m_GainSaveModeAuto);
            if (m_GainSaveModeAuto==DC1394_FEATURE_MODE_MANUAL)
            {
                dc1394_feature_get_value(m_pCamera,DC1394_FEATURE_GAIN,&m_GainSaveValue);
            }

            dc1394_feature_get_mode(m_pCamera,DC1394_FEATURE_SHUTTER,&m_ShutterSaveModeAuto);
            if (m_ShutterSaveModeAuto==DC1394_FEATURE_MODE_MANUAL)
            {
                dc1394_feature_get_value(m_pCamera,DC1394_FEATURE_SHUTTER,&m_ShutterSaveValue);
            }

	        dc1394_feature_set_mode(m_pCamera,DC1394_FEATURE_GAIN,DC1394_FEATURE_MODE_AUTO);
	        dc1394_feature_set_mode(m_pCamera,DC1394_FEATURE_SHUTTER,DC1394_FEATURE_MODE_AUTO);
	    }
	    else
	    {
	    	dc1394_feature_set_mode(m_pCamera,DC1394_FEATURE_GAIN,m_GainSaveModeAuto);
	    	if (m_GainSaveModeAuto==DC1394_FEATURE_MODE_MANUAL)
	    	{
	    	    dc1394_feature_set_value(m_pCamera,DC1394_FEATURE_GAIN,m_GainSaveValue);
	    	}
	        dc1394_feature_set_mode(m_pCamera,DC1394_FEATURE_SHUTTER,m_ShutterSaveModeAuto);
	        if (m_ShutterSaveModeAuto==DC1394_FEATURE_MODE_MANUAL)
	    	{
	    	    dc1394_feature_set_value(m_pCamera,DC1394_FEATURE_SHUTTER,m_ShutterSaveValue);
	    	}
	    }
	}
	
	return DC1394_SUCCESS==dc1394_feature_set_power(m_pCamera,(dc1394feature_t)feature,onoff?DC1394_ON:DC1394_OFF);
}

// 05
bool CFWCamera_DR2_2::getActiveDC1394(int feature)
{
	if (!m_pCamera) return false;
	feature+=DC1394_FEATURE_MIN;
	dc1394switch_t pwr;
	dc1394_feature_get_power(m_pCamera,(dc1394feature_t)feature,&pwr);
	return pwr;
}

// 06
bool CFWCamera_DR2_2::hasManualDC1394(int feature)
{
	if (!m_pCamera) return false;
	feature+=DC1394_FEATURE_MIN;
	dc1394feature_modes_t modes;
	if (DC1394_SUCCESS!=dc1394_feature_get_modes(m_pCamera,(dc1394feature_t)feature,&modes)) return false;
	for (uint32_t num=0; num<modes.num; ++num)
	{
	    if (modes.modes[num]==DC1394_FEATURE_MODE_MANUAL)
	    {
	        return true;
	    }
    }	
    return false;
}

// 07
bool CFWCamera_DR2_2::hasAutoDC1394(int feature)
{
	if (!m_pCamera) return false;
	feature+=DC1394_FEATURE_MIN;
	dc1394feature_modes_t modes;
	if (DC1394_SUCCESS!=dc1394_feature_get_modes(m_pCamera,(dc1394feature_t)feature,&modes)) return false;
	for (uint32_t num=0; num<modes.num; ++num)
	{
		if (modes.modes[num]==DC1394_FEATURE_MODE_AUTO)
		{
		    return true;
		}
	}
	return false;
}	

// 08
bool CFWCamera_DR2_2::hasOnePushDC1394(int feature)
{
	if (!m_pCamera) return false;
	feature+=DC1394_FEATURE_MIN;
	dc1394feature_modes_t modes;
	if (DC1394_SUCCESS!=dc1394_feature_get_modes(m_pCamera,(dc1394feature_t)feature,&modes)) return false;
	for (uint32_t num=0; num<modes.num; ++num)
	{
		if (modes.modes[num]==DC1394_FEATURE_MODE_ONE_PUSH_AUTO)
		{
		    return true;
		}
	}
	return false;
}

// 09
bool CFWCamera_DR2_2::setModeDC1394(int feature, bool auto_onoff)
{
	if (!m_pCamera) return false;
	feature+=DC1394_FEATURE_MIN;
	
	return DC1394_SUCCESS==dc1394_feature_set_mode(m_pCamera,(dc1394feature_t)feature,auto_onoff?DC1394_FEATURE_MODE_AUTO:DC1394_FEATURE_MODE_MANUAL);
}

// 10
bool CFWCamera_DR2_2::getModeDC1394(int feature)
{
	if (!m_pCamera) return false;
	feature+=DC1394_FEATURE_MIN;
	dc1394feature_mode_t mode; 
	dc1394_feature_get_mode(m_pCamera,(dc1394feature_t)feature,&mode);
	return mode==DC1394_FEATURE_MODE_AUTO;
}

// 11
bool CFWCamera_DR2_2::setOnePushDC1394(int feature)
{
	if (!m_pCamera) return false;
	feature+=DC1394_FEATURE_MIN;
	return DC1394_SUCCESS==dc1394_feature_set_mode(m_pCamera,(dc1394feature_t)feature,DC1394_FEATURE_MODE_ONE_PUSH_AUTO);
}

// 23
bool CFWCamera_DR2_2::setWhiteBalanceDC1394(double b, double r)
{
    if (b<0.0 || b>1.0 || r<0.0 || r>1.0 || !m_pCamera)
    {
        return false;
    }
    
	return DC1394_SUCCESS==dc1394_feature_whitebalance_set_value(m_pCamera,
		                                                         NormToValue(b,DC1394_FEATURE_WHITE_BALANCE),
		                                                         NormToValue(r,DC1394_FEATURE_WHITE_BALANCE));
}

// 24
bool CFWCamera_DR2_2::getWhiteBalanceDC1394(double &b, double &r)
{
	unsigned int iB,iR;
	bool ok=DC1394_SUCCESS==dc1394_feature_whitebalance_get_value(m_pCamera,&iB,&iR);
	b=ValueToNorm(iB,DC1394_FEATURE_WHITE_BALANCE);
	r=ValueToNorm(iR,DC1394_FEATURE_WHITE_BALANCE);
	return ok;
}

////////////////////////
// end feature functions
////////////////////////

// 12
unsigned int CFWCamera_DR2_2::getVideoModeMaskDC1394()
{
    if (mRawDriver)
    {
        return 1<<(DC1394_VIDEO_MODE_FORMAT7_0-DC1394_VIDEO_MODE_MIN);
    }

	dc1394video_modes_t modes;
	dc1394_video_get_supported_modes(m_pCamera,&modes);

	unsigned int mask=0;
	for (unsigned int m=0; m<modes.num; ++m)
	{
		mask|=1<<(modes.modes[m]-DC1394_VIDEO_MODE_MIN);
    }
	fprintf(stderr,"video mode mask: %x\n",mask);
	fflush(stdout);

	return mask;
}

// 13
bool CFWCamera_DR2_2::setVideoModeDC1394(int newVideoMode)
{
    if (mRawDriver) return false;

	fprintf(stderr,"SET VIDEO MODE %d\n",newVideoMode);

	m_AcqMutex.wait();

	if (!m_pCamera)
	{
		m_AcqMutex.post();
		return false;
	}

    m_nInvalidFrames=NUM_DMA_BUFFERS;

	dc1394_video_set_transmission(m_pCamera,DC1394_OFF);
	dc1394_capture_stop(m_pCamera);

	dc1394video_mode_t videoModeToSet=(dc1394video_mode_t)((int)DC1394_VIDEO_MODE_MIN+newVideoMode);

	if (videoModeToSet<DC1394_VIDEO_MODE_FORMAT7_MIN)
	{
		fprintf(stderr,"Attempting to set NON format 7\n");
		SetVideoMode(videoModeToSet);
	}
	else
	{
		fprintf(stderr,"Attempting to set format 7\n");
		SetF7(videoModeToSet,SKIP,SKIP,SKIP,SKIP,SKIP,SKIP);
	}

	if (dc1394_capture_setup(m_pCamera,NUM_DMA_BUFFERS,DC1394_CAPTURE_FLAGS_DEFAULT)!=DC1394_SUCCESS)
	{
		fprintf(stderr,"ERROR: Can't set DC1394_CAPTURE_FLAGS_DEFAULT\n");
		dc1394_camera_free(m_pCamera);
		m_pCamera=NULL;
		m_AcqMutex.post();
		return false;
	}

	if (dc1394_video_set_transmission(m_pCamera,DC1394_ON)!=DC1394_SUCCESS)
	{
		fprintf(stderr,"ERROR: Can't set DC1394_ON\n");
		dc1394_camera_free(m_pCamera);
		m_pCamera=NULL;
		m_AcqMutex.post();
		return false;
	}

	m_AcqMutex.post();
	return true;
}

// 14
unsigned int CFWCamera_DR2_2::getVideoModeDC1394()
{ 
    dc1394video_mode_t videoMode;
	dc1394_video_get_mode(m_pCamera,&videoMode);
	return videoMode-DC1394_VIDEO_MODE_MIN;
}

// 15
unsigned int CFWCamera_DR2_2::getFPSMaskDC1394()
{
	if (!m_pCamera) return 0;

    dc1394video_mode_t videoMode;
    dc1394error_t error=dc1394_video_get_mode(m_pCamera,&videoMode);
    if (manage(error) || videoMode>=DC1394_VIDEO_MODE_FORMAT7_MIN)
    { 
        fprintf(stderr,"LINE: %d\n",__LINE__); 
        return 0; 
    }

	dc1394framerates_t fps;
	dc1394_video_get_supported_framerates(m_pCamera,videoMode,&fps);

	unsigned int mask=0;
	for (unsigned int f=0; f<fps.num; ++f)
	{
		mask|=1<<(fps.framerates[f]-DC1394_FRAMERATE_MIN);
    }
    
	return mask;
}

// 16
unsigned int CFWCamera_DR2_2::getFPSDC1394()
{
	if (!m_pCamera) return 0;

    dc1394video_mode_t videoMode;
    dc1394error_t error=dc1394_video_get_mode(m_pCamera,&videoMode);
    if (manage(error) || videoMode>=DC1394_VIDEO_MODE_FORMAT7_MIN)
    { 
        fprintf(stderr,"LINE: %d\n",__LINE__); 
        return 0;
    }

	dc1394framerate_t fps; 
	dc1394_video_get_framerate(m_pCamera,&fps);

	return fps-DC1394_FRAMERATE_MIN;
}

// 17
bool CFWCamera_DR2_2::setFPSDC1394(int fps)
{
	if (!m_pCamera) return false;

    dc1394video_mode_t videoMode;
    dc1394error_t error=dc1394_video_get_mode(m_pCamera,&videoMode);
    if (manage(error) || videoMode>=DC1394_VIDEO_MODE_FORMAT7_MIN)
    { 
        fprintf(stderr,"LINE: %d\n",__LINE__); 
        return false; 
    }

	return DC1394_SUCCESS==dc1394_video_set_framerate(m_pCamera,(dc1394framerate_t)((int)fps+DC1394_FRAMERATE_MIN));	
}

// 18
unsigned int CFWCamera_DR2_2::getISOSpeedDC1394()
{
	if (!m_pCamera) return false;
	dc1394speed_t speed;
	dc1394_video_get_iso_speed(m_pCamera,&speed);
	return speed-DC1394_ISO_SPEED_MIN;
}

// 19
bool CFWCamera_DR2_2::setISOSpeedDC1394(int speed)
{ 	
	if (!m_pCamera) return false;
	return DC1394_SUCCESS==dc1394_video_set_iso_speed(m_pCamera,(dc1394speed_t)(speed+DC1394_ISO_SPEED_MIN));
}

// 20
unsigned int CFWCamera_DR2_2::getColorCodingMaskDC1394(unsigned int videoMode)
{
	if (!m_pCamera) return 0;

    if (mRawDriver) return 1<<(DC1394_COLOR_CODING_RAW8-DC1394_COLOR_CODING_MIN);

	dc1394video_mode_t vm=(dc1394video_mode_t)(videoMode+DC1394_VIDEO_MODE_MIN);

	if (vm<DC1394_VIDEO_MODE_FORMAT7_MIN) return 0;

	dc1394color_codings_t codings;
	dc1394_format7_get_color_codings(m_pCamera,vm,&codings);

	unsigned int mask=0;
	for (unsigned int m=0; m<codings.num; ++m)
	{
		mask|=1<<(codings.codings[m]-DC1394_COLOR_CODING_MIN);
    }
    
	fprintf(stderr,"color coding mask for video mode %d is %x\n",videoMode,mask);

	return mask;
}
unsigned int CFWCamera_DR2_2::getActualColorCodingMaskDC1394()
{
	if (!m_pCamera) return false;

    if (mRawDriver) return 1<<(DC1394_COLOR_CODING_RAW8-DC1394_COLOR_CODING_MIN);

    dc1394video_mode_t videoMode;
    dc1394error_t error=dc1394_video_get_mode(m_pCamera,&videoMode);
    if (manage(error) || videoMode<DC1394_VIDEO_MODE_FORMAT7_MIN)
    { 
        fprintf(stderr,"LINE: %d\n",__LINE__); 
        return 0; 
    }

	dc1394color_codings_t codings;
	dc1394_format7_get_color_codings(m_pCamera,videoMode,&codings);

	unsigned int mask=0;
	for (unsigned int m=0; m<codings.num; ++m)
	{
		mask|=1<<(codings.codings[m]-DC1394_COLOR_CODING_MIN);
    }
    
	fprintf(stderr,"actual color coding mask %x\n",mask);

	return mask;
}

// 21
unsigned int CFWCamera_DR2_2::getColorCodingDC1394()
{
	if (!m_pCamera) return false;

    dc1394video_mode_t videoMode;
    dc1394error_t error=dc1394_video_get_mode(m_pCamera,&videoMode);
    if (manage(error) || videoMode<DC1394_VIDEO_MODE_FORMAT7_MIN)
    { 
        fprintf(stderr,"LINE: %d\n",__LINE__); 
        return 0; 
    }

	dc1394color_coding_t coding;
	dc1394_format7_get_color_coding(m_pCamera,videoMode,&coding);
	return coding-DC1394_COLOR_CODING_MIN;
}

// 22
bool CFWCamera_DR2_2::setColorCodingDC1394(int coding)
{
    if (mRawDriver) return false;

	m_AcqMutex.wait();

	if (!m_pCamera)
	{
		m_AcqMutex.post();
		return false;
	}
	
	dc1394video_mode_t videoMode;
    dc1394error_t error=dc1394_video_get_mode(m_pCamera,&videoMode);
    if (manage(error) || videoMode<DC1394_VIDEO_MODE_FORMAT7_MIN)
    {
        fprintf(stderr,"LINE: %d\n",__LINE__);
        m_AcqMutex.post();
        return false;
    }
    
	m_nInvalidFrames=NUM_DMA_BUFFERS;

	dc1394_video_set_transmission(m_pCamera,DC1394_OFF);
	dc1394_capture_stop(m_pCamera);

	dc1394color_coding_t cc=(dc1394color_coding_t)((int)coding+DC1394_COLOR_CODING_MIN);
	SetF7(SKIP,SKIP,SKIP,cc,SKIP,SKIP,SKIP);

	bool bRetVal=true;

	if (dc1394_capture_setup(m_pCamera,NUM_DMA_BUFFERS,DC1394_CAPTURE_FLAGS_DEFAULT)!=DC1394_SUCCESS)
	{
		fprintf(stderr,"ERROR: Can't set DC1394_CAPTURE_FLAGS_DEFAULT\n");
		dc1394_camera_free(m_pCamera);
		m_pCamera=NULL;
		bRetVal=false;
	}

	if (bRetVal && dc1394_video_set_transmission(m_pCamera,DC1394_ON)!=DC1394_SUCCESS)
	{
		fprintf(stderr,"ERROR: Can't set DC1394_ON\n");
		dc1394_camera_free(m_pCamera);
		m_pCamera=NULL;
		bRetVal=false;
	}

	m_AcqMutex.post();

	return bRetVal;
}	

//experimental cleanup function, Lorenzo

// 25
bool CFWCamera_DR2_2::getFormat7MaxWindowDC1394(unsigned int &xdim,unsigned int &ydim,unsigned int &xstep,unsigned int &ystep,unsigned int &xoffstep,unsigned int &yoffstep)
{
	if (!m_pCamera) return false;

	dc1394video_mode_t videoMode;
    dc1394error_t error=dc1394_video_get_mode(m_pCamera,&videoMode);
    if (manage(error))
    {
        fprintf(stderr,"LINE: %d\n",__LINE__);
        xdim=ydim=0;
        xstep=ystep=2;
        xoffstep=yoffstep=2;
        return false;
    }
    
    if (videoMode<DC1394_VIDEO_MODE_FORMAT7_MIN)
	{
		xdim=m_XDim;
		ydim=m_YDim;
		xoffstep=yoffstep=2;
		return true;
	}

	error=dc1394_format7_get_unit_size(m_pCamera,videoMode,&xstep,&ystep);
	if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }
	error=dc1394_format7_get_max_image_size(m_pCamera,videoMode,&xdim,&ydim);
	if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }
	error=dc1394_format7_get_unit_position(m_pCamera,videoMode,&xoffstep,&yoffstep);
	if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

	return true;
}

// 26
bool CFWCamera_DR2_2::setFormat7WindowDC1394(unsigned int xdim,unsigned int ydim,int x0,int y0)
{
	m_AcqMutex.wait();

	if (!m_pCamera)
	{
		m_AcqMutex.post();
		return false;
	}
	
	dc1394video_mode_t videoMode;
    dc1394error_t error=dc1394_video_get_mode(m_pCamera,&videoMode);
	if (manage(error) || videoMode<DC1394_VIDEO_MODE_FORMAT7_MIN)
    {
        fprintf(stderr,"LINE: %d\n",__LINE__);
		m_AcqMutex.post();
		return false;
	}

	m_nInvalidFrames=NUM_DMA_BUFFERS;
	dc1394_video_set_transmission(m_pCamera,DC1394_OFF);
	dc1394_capture_stop(m_pCamera);

	SetF7(SKIP,xdim,ydim,SKIP,SKIP,x0,y0);

	bool bRetVal=true;

	if (dc1394_capture_setup(m_pCamera,NUM_DMA_BUFFERS,DC1394_CAPTURE_FLAGS_DEFAULT)!=DC1394_SUCCESS)
	{
		fprintf(stderr,"ERROR: Can't set DC1394_CAPTURE_FLAGS_DEFAULT\n");
		dc1394_camera_free(m_pCamera);
		m_pCamera=NULL;
		bRetVal=false;
	}

	if (bRetVal && dc1394_video_set_transmission(m_pCamera,DC1394_ON)!=DC1394_SUCCESS)
	{
		fprintf(stderr,"ERROR: Can't set DC1394_ON\n");
		dc1394_camera_free(m_pCamera);
		m_pCamera=NULL;
		bRetVal=false;
	}

	m_AcqMutex.post();

	return bRetVal;
}

// 27
bool CFWCamera_DR2_2::getFormat7WindowDC1394(unsigned int &xdim,unsigned int &ydim,int &x0,int &y0)
{
	if (!m_pCamera) return false;

	//xdim=m_XDim;
	//ydim=m_YDim;
	
	dc1394video_mode_t actVideoMode;
    dc1394error_t error;
    
    error=dc1394_video_get_mode(m_pCamera,&actVideoMode);
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

    if (actVideoMode<DC1394_VIDEO_MODE_FORMAT7_MIN)
	{
		xdim=m_XDim;
		ydim=m_YDim;
		x0=y0=0;
		return true;
	}
    
    unsigned int xmaxdim,ymaxdim;
    error=dc1394_format7_get_max_image_size(m_pCamera,actVideoMode,&xmaxdim,&ymaxdim);
	if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }
	
	unsigned int xoff,yoff;
    error=dc1394_format7_get_image_position(m_pCamera,actVideoMode,&xoff,&yoff);
	if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }
	
	error=dc1394_format7_get_image_size(m_pCamera,actVideoMode,&xdim,&ydim);
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }
    
    x0=(int)xoff-(xmaxdim-xdim)/2;
    y0=(int)yoff-(ymaxdim-ydim)/2;

	return true;
}	

// 28
bool CFWCamera_DR2_2::setOperationModeDC1394(bool b1394b)
{
	m_AcqMutex.wait();

	if (!m_pCamera)
	{
		m_AcqMutex.post();
		return false;
	}

	m_nInvalidFrames=NUM_DMA_BUFFERS;

	dc1394_video_set_transmission(m_pCamera,DC1394_OFF);
	dc1394_capture_stop(m_pCamera);
	dc1394_video_set_operation_mode(m_pCamera,b1394b?DC1394_OPERATION_MODE_1394B:DC1394_OPERATION_MODE_LEGACY);
	dc1394_video_set_iso_speed(m_pCamera,DC1394_ISO_SPEED_400);

	if (dc1394_capture_setup(m_pCamera,NUM_DMA_BUFFERS,DC1394_CAPTURE_FLAGS_DEFAULT)!=DC1394_SUCCESS)
	{
		fprintf(stderr,"ERROR: Can't set DC1394_CAPTURE_FLAGS_DEFAULT\n");
		dc1394_camera_free(m_pCamera);
		m_pCamera=NULL;
		m_AcqMutex.post();
		return false;
	}

	if (dc1394_video_set_transmission(m_pCamera,DC1394_ON)!=DC1394_SUCCESS)
	{
		fprintf(stderr,"ERROR: Can't set DC1394_ON\n");
		dc1394_camera_free(m_pCamera);
		m_pCamera=NULL;
		m_AcqMutex.post();
		return false;
	}

	m_AcqMutex.post();
	return true;
}

// 29
bool CFWCamera_DR2_2::getOperationModeDC1394()
{
	if (!m_pCamera) return false;
	dc1394operation_mode_t b1394b;
	dc1394_video_get_operation_mode(m_pCamera,&b1394b);
	return b1394b==DC1394_OPERATION_MODE_1394B;
}

// 30
bool CFWCamera_DR2_2::setTransmissionDC1394(bool bTxON)
{
	if (!m_pCamera) return false;
	return DC1394_SUCCESS==dc1394_video_set_transmission(m_pCamera,(dc1394switch_t)bTxON);
}

// 31
bool CFWCamera_DR2_2::getTransmissionDC1394()
{
	if (!m_pCamera) return false;
	dc1394switch_t bTxON;
	dc1394_video_get_transmission(m_pCamera,&bTxON);
	return bTxON;
}

// 32 setBayer
// 33 getBayer

// 34
bool CFWCamera_DR2_2::setBroadcastDC1394(bool onoff)
{
	if (!m_pCamera) return false;
	return DC1394_SUCCESS==dc1394_camera_set_broadcast(m_pCamera,(dc1394bool_t)onoff);
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

	m_nInvalidFrames=NUM_DMA_BUFFERS;

	bool bRetVal=DC1394_SUCCESS==dc1394_memory_load(m_pCamera,0);
	
	dc1394video_mode_t videoMode;
	dc1394_video_get_mode(m_pCamera,&videoMode);
	
	SetVideoMode(videoMode);

	m_AcqMutex.post();

	return bRetVal;
}

// 36
bool CFWCamera_DR2_2::setResetDC1394()
{
	m_AcqMutex.wait();

	if (!m_pCamera)
	{
		m_AcqMutex.post();
		return false;
	}

	dc1394_video_set_transmission(m_pCamera,DC1394_OFF);
	dc1394_capture_stop(m_pCamera);
	
	dc1394_camera_reset(m_pCamera);
	
	if (dc1394_capture_setup(m_pCamera,NUM_DMA_BUFFERS,DC1394_CAPTURE_FLAGS_DEFAULT)!=DC1394_SUCCESS)
	{
		fprintf(stderr,"ERROR: Can't set DC1394_CAPTURE_FLAGS_DEFAULT\n");
		dc1394_camera_free(m_pCamera);
		m_pCamera=NULL;
		m_AcqMutex.post();
		return false;
	}

	if (dc1394_video_set_transmission(m_pCamera,DC1394_ON)!=DC1394_SUCCESS)
	{
		fprintf(stderr,"ERROR: Can't set DC1394_ON\n");
		dc1394_camera_free(m_pCamera);
		m_pCamera=NULL;
		m_AcqMutex.post();
		return false;
	}

	m_AcqMutex.post();
	return true;
}

// 37
bool CFWCamera_DR2_2::setPowerDC1394(bool onoff)
{
	m_AcqMutex.wait();

	if (!m_pCamera)
	{
		m_AcqMutex.post();
		return false;
	}

	if (onoff)
	{
		dc1394_camera_set_power(m_pCamera,(dc1394switch_t)onoff);

		if (dc1394_capture_setup(m_pCamera,NUM_DMA_BUFFERS,DC1394_CAPTURE_FLAGS_DEFAULT)!=DC1394_SUCCESS)
		{
			fprintf(stderr,"ERROR: Can't set DC1394_CAPTURE_FLAGS_DEFAULT\n");
			dc1394_camera_free(m_pCamera);
			m_pCamera=NULL;
			m_AcqMutex.post();
			return false;
		}

		if (dc1394_video_set_transmission(m_pCamera,DC1394_ON)!=DC1394_SUCCESS)
		{
			fprintf(stderr,"ERROR: Can't set DC1394_ON\n");
			dc1394_camera_free(m_pCamera);
			m_pCamera=NULL;
			m_AcqMutex.post();
			return false;
		}

		m_bCameraOn=true;
	}
	else
	{
		m_bCameraOn=false;
		dc1394_video_set_transmission(m_pCamera,DC1394_OFF);
		dc1394_capture_stop(m_pCamera);
		dc1394_camera_set_power(m_pCamera,(dc1394switch_t)onoff);
	}

	m_AcqMutex.post();
	return true;
}

// 38
bool CFWCamera_DR2_2::setCaptureDC1394(bool bON)
{
	if (!m_pCamera) return false;

	if (bON) 
		return DC1394_SUCCESS==dc1394_capture_setup(m_pCamera,NUM_DMA_BUFFERS,DC1394_CAPTURE_FLAGS_DEFAULT);
	else
		return DC1394_SUCCESS==dc1394_capture_stop(m_pCamera);
}

// 39
unsigned int CFWCamera_DR2_2::getBytesPerPacketDC1394()
{
	if (!m_pCamera) return 0;
	
	dc1394video_mode_t videoMode;
    dc1394error_t error=dc1394_video_get_mode(m_pCamera,&videoMode);
	if (manage(error) || videoMode<DC1394_VIDEO_MODE_FORMAT7_MIN)
    { 
        fprintf(stderr,"LINE: %d\n",__LINE__); 
        return 0; 
    }
    
    dc1394speed_t isoSpeed;
    error=dc1394_video_get_iso_speed(m_pCamera,&isoSpeed);
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return 0; }

    // get ISO bandwidth
    int busBand=10000000<<(isoSpeed-DC1394_ISO_SPEED_MIN);

    dc1394color_coding_t colorCoding;
    unsigned int dummy,bytesPerPacket,xDim,yDim;
    error=dc1394_format7_get_roi(m_pCamera,videoMode,&colorCoding,&bytesPerPacket,&dummy,&dummy,&xDim,&yDim);	
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return 0; }

    unsigned int maxBytesPerPacket,unitBytesPerPacket;
    error=dc1394_format7_get_packet_parameters(m_pCamera,videoMode,&unitBytesPerPacket,&maxBytesPerPacket);
    if (manage(error)) { fprintf(stderr,"LINE: %d\n",__LINE__); return 0; }

    int fps=maxFPS(videoMode,colorCoding);
    double bpp=bytesPerPixel(colorCoding);
    double maxBandOcc=double(fps*xDim*yDim)*bpp;
    double margin=double(busBand)/maxBandOcc;

    int bandPercent=(int)(100.0*double(bytesPerPacket)/(double(maxBytesPerPacket)*margin));

    if (bandPercent<0) bandPercent=0; else if (bandPercent>100) bandPercent=100;

    return (unsigned)bandPercent;
}

// 40
bool CFWCamera_DR2_2::setBytesPerPacketDC1394(unsigned int newBandPercent)
{	
	m_AcqMutex.wait();

	if (!m_pCamera)
	{
		m_AcqMutex.post();
		return false;
	}
    
    dc1394error_t error;

	error=dc1394_video_set_transmission(m_pCamera,DC1394_OFF);
    if (manage(error,&m_AcqMutex)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }
    
	error=dc1394_capture_stop(m_pCamera);	
	if (manage(error,&m_AcqMutex)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }
    
	//
	
	dc1394video_mode_t videoMode;
    error=dc1394_video_get_mode(m_pCamera,&videoMode);
	if (manage(error) || videoMode<DC1394_VIDEO_MODE_FORMAT7_MIN)
    {
        fprintf(stderr,"LINE: %d\n",__LINE__);
        m_AcqMutex.post();
        return false;
    }

	if (newBandPercent<0)
	{
	    newBandPercent=0;
	}
	else if (newBandPercent>100)
    {
        newBandPercent=100;
    }
    
    dc1394speed_t isoSpeed;
    error=dc1394_video_get_iso_speed(m_pCamera,&isoSpeed);
    if (manage(error,&m_AcqMutex)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }
    
    // get ISO bandwidth
    int busBand=10000000<<(isoSpeed-DC1394_ISO_SPEED_MIN);

    dc1394color_coding_t colorCoding;
    unsigned int dummy,xDim,yDim;
    error=dc1394_format7_get_roi(m_pCamera,videoMode,&colorCoding,&dummy,&dummy,&dummy,&xDim,&yDim);	
    if (manage(error,&m_AcqMutex)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

    unsigned int maxBytesPerPacket,unitBytesPerPacket;
    error=dc1394_format7_get_packet_parameters(m_pCamera,videoMode,&unitBytesPerPacket,&maxBytesPerPacket);
    if (manage(error,&m_AcqMutex)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }

    int fps=maxFPS(videoMode,colorCoding);
    double bpp=bytesPerPixel(colorCoding);
    double newBandOcc=double(fps*xDim*yDim)*bpp;

    unsigned int bytesPerPacket=(unsigned int)(0.01*double(newBandPercent)*double(busBand)*double(maxBytesPerPacket)/newBandOcc);
    bytesPerPacket=(bytesPerPacket/unitBytesPerPacket)*unitBytesPerPacket;
    if (bytesPerPacket>maxBytesPerPacket)
    {
        bytesPerPacket=maxBytesPerPacket;
    }
    else if (bytesPerPacket<unitBytesPerPacket)
    {
        bytesPerPacket=unitBytesPerPacket;
    }
    
    printf("videoMode %d band percnet %d bytesPerPacket %d\n",videoMode,newBandPercent,bytesPerPacket);
    
    error=dc1394_format7_set_packet_size(m_pCamera,videoMode,bytesPerPacket);
    if (manage(error,&m_AcqMutex)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }
    
    error=dc1394_format7_get_packet_size(m_pCamera,videoMode,&bytesPerPacket);
    if (manage(error,&m_AcqMutex)) { fprintf(stderr,"LINE: %d\n",__LINE__); return false; }
          
    if (dc1394_capture_setup(m_pCamera,NUM_DMA_BUFFERS,DC1394_CAPTURE_FLAGS_DEFAULT)!=DC1394_SUCCESS)
	{
		fprintf(stderr,"ERROR: Can't set DC1394_CAPTURE_FLAGS_DEFAULT\n");
		dc1394_camera_free(m_pCamera);
		m_pCamera=NULL;
		m_AcqMutex.post();
		return false;
	}

	if (dc1394_video_set_transmission(m_pCamera,DC1394_ON)!=DC1394_SUCCESS)
	{
		fprintf(stderr,"ERROR: Can't set DC1394_ON\n");
		dc1394_camera_free(m_pCamera);
		m_pCamera=NULL;
		m_AcqMutex.post();
		return false;
	}

	m_AcqMutex.post();
    printf("bytesPerPacket %d\n",bytesPerPacket);
    
    return true;
}

// base class implementation

#define TRANSL(feature) ((int)feature-(int)DC1394_FEATURE_MIN)

bool CFWCamera_DR2_2::setBrightness(double v)
{
	if (v<0.0 || v>1.0) return false;
	int feature=TRANSL(DC1394_FEATURE_BRIGHTNESS);
	setActiveDC1394(feature,true);
	setModeDC1394(feature,false);  
	return setFeatureDC1394(feature,v); 
}
bool CFWCamera_DR2_2::setExposure(double v)
{
	if (v<0.0 || v>1.0) return false;
	int feature=TRANSL(DC1394_FEATURE_EXPOSURE);
	setActiveDC1394(feature,true);
	setModeDC1394(feature,false);  
	return setFeatureDC1394(feature,v); 
}
bool CFWCamera_DR2_2::setSharpness(double v)
{ 
	if (v<0.0 || v>1.0) return false;
	int feature=TRANSL(DC1394_FEATURE_SHARPNESS);
	setActiveDC1394(feature,true);
	setModeDC1394(feature,false);  
	return setFeatureDC1394(feature,v); 
}
bool CFWCamera_DR2_2::setWhiteBalance(double blue, double red)
{
	if (blue<0.0 || blue>1.0 || red<0.0 || red>1.0) return false;
	int feature=TRANSL(DC1394_FEATURE_WHITE_BALANCE); 
	setActiveDC1394(feature,true);
	setModeDC1394(feature,false);
	return setWhiteBalanceDC1394(blue,red); 
}
bool CFWCamera_DR2_2::setHue(double v)
{
	if (v<0.0 || v>1.0) return false;
	int feature=TRANSL(DC1394_FEATURE_HUE);
	setActiveDC1394(feature,true);
	setModeDC1394(feature,false);  
	return setFeatureDC1394(feature,v);  
}
bool CFWCamera_DR2_2::setSaturation(double v)
{
	if (v<0.0 || v>1.0) return false;
	int feature=TRANSL(DC1394_FEATURE_SATURATION);
	setActiveDC1394(feature,true);
	setModeDC1394(feature,false);  
	return setFeatureDC1394(feature,v);  
}
bool CFWCamera_DR2_2::setGamma(double v)
{
	if (v<0.0 || v>1.0) return false;
	int feature=TRANSL(DC1394_FEATURE_GAMMA);
	setActiveDC1394(feature,true);
	setModeDC1394(feature,false);  
	return setFeatureDC1394(feature,v);  
}
bool CFWCamera_DR2_2::setShutter(double v)
{
	if (v<0.0 || v>1.0) return false;
	int feature=TRANSL(DC1394_FEATURE_SHUTTER);
	setActiveDC1394(feature,true);
	setModeDC1394(feature,false);  
	return setFeatureDC1394(feature,v);  
}
bool CFWCamera_DR2_2::setGain(double v)
{
	if (v<0.0 || v>1.0) return false;
	int feature=TRANSL(DC1394_FEATURE_GAIN);
	setActiveDC1394(feature,true);
	setModeDC1394(feature,false);  
	return setFeatureDC1394(feature,v);  
}
bool CFWCamera_DR2_2::setIris(double v)
{
	if (v<0.0 || v>1.0) return false;
	int feature=TRANSL(DC1394_FEATURE_IRIS);
	setActiveDC1394(feature,true);
	setModeDC1394(feature,false);  
	return setFeatureDC1394(feature,v);  
}

// GET

double CFWCamera_DR2_2::getBrightness()
{ 
	return getFeatureDC1394(TRANSL(DC1394_FEATURE_BRIGHTNESS)); 
}
double CFWCamera_DR2_2::getExposure()
{ 
	return getFeatureDC1394(TRANSL(DC1394_FEATURE_EXPOSURE)); 
}	
double CFWCamera_DR2_2::getSharpness()
{ 
	return getFeatureDC1394(TRANSL(DC1394_FEATURE_SHARPNESS)); 
}
bool CFWCamera_DR2_2::getWhiteBalance(double &blue, double &red)
{ 
	return getWhiteBalance(blue,red); 
}	
double CFWCamera_DR2_2::CFWCamera_DR2_2::getHue()
{ 
	return getFeatureDC1394(TRANSL(DC1394_FEATURE_HUE)); 
}	
double CFWCamera_DR2_2::getSaturation()
{ 
	return CFWCamera_DR2_2::getFeatureDC1394(TRANSL(DC1394_FEATURE_SATURATION)); 
}
double CFWCamera_DR2_2::getGamma()
{ 
	return getFeatureDC1394(TRANSL(DC1394_FEATURE_GAMMA)); 
}
double CFWCamera_DR2_2::getShutter()
{ 
	return getFeatureDC1394(TRANSL(DC1394_FEATURE_SHUTTER));
}
double CFWCamera_DR2_2::getGain()
{ 
	return getFeatureDC1394(TRANSL(DC1394_FEATURE_GAIN));
}
double CFWCamera_DR2_2::getIris()
{ 
	return getFeatureDC1394(TRANSL(DC1394_FEATURE_IRIS)); 
}
