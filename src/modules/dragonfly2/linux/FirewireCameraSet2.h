// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alessandro Scalzo
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __FIREWIRE_CAMERA_SET2_H__
#define __FIREWIRE_CAMERA_SET2_H__

#include "FirewireCameraDC1394-DR2_2.h"

class CFWCameraSet2
{
public:
    CFWCameraSet2()
    {
		m_dc1394_handle=NULL;
		m_pCameraList=NULL;
		m_apCamera=NULL;
   
        m_nNumCameras=0;
        m_nActiveCams=0;
    }
    
    virtual ~CFWCameraSet2()
    {   
        Close();
    }    

    ///////////////////////////////////////////////////////
    // Init/Close functions
    ///////////////////////////////////////////////////////
            
    bool Init();
    void Close();
    void Restart();
    bool StartCamera(int idCamera,unsigned int size_x,unsigned int size_y,bool bDR2,int format);
    void ShutdownCamera(int idCamera);
    //void GetCameraInfo(int idCamera);

    ///////////////////////////////////////////////////////
    // Capture functions
    ///////////////////////////////////////////////////////

    bool CaptureRgb(int idCamera,unsigned char* image);
    bool CaptureRaw(int idCamera,unsigned char* image);
    
    const yarp::os::Stamp& getLastInputStamp(int idCamera)
	{
	    return m_apCamera[idCamera]->getLastInputStamp();
	}

    ///////////////////////////////////////////////////////
    // camera controls
    ///////////////////////////////////////////////////////
    
	yarp::dev::IFrameGrabberControlsDC1394* getControls(int camera)
	{
		return m_apCamera[camera];
	}

    //bool SetFeature(int camera,dc1394feature_t feature,double dVal);
    //bool SetTemperature(int camera,double dTargetTemperature);
    //bool SetWhiteBalance(int camera,double dUB, double dVR);
    //bool SetWhiteShading(int camera,double dR, double dG, double dB);    
    
    //double GetFeature(int camera,dc1394feature_t feature);
    //bool   GetTemperature(int camera,double& dTargetTemperature,double& dTemperature);
    //bool   GetWhiteBalance(int camera,double& dUB, double& dVR);
    //bool   GetWhiteShading(int camera,double& dR, double& dG, double &dB);    
    
	int width (int camera){ return m_apCamera[camera]->width(); }
	int height(int camera){ return m_apCamera[camera]->height(); }
	int getRawBufferSize(int camera){ return m_apCamera[camera]->getRawBufferSize(); }

    int GetCameraNum()
    {
        return m_nActiveCams;
    }

    /*
    bool PrintSettings(int idCamera)
    {
        if (!IsCameraReady(idCamera)) return false;

        m_apCamera[idCamera]->settings.PrintSettings();
		return true;
    }
    */

protected:
	dc1394_t *m_dc1394_handle;
	dc1394camera_list_t *m_pCameraList;
    CFWCamera_DR2_2** m_apCamera;
	
    int m_nNumCameras;
    int m_nActiveCams;
            
    bool IsCameraReady(int idCamera)
    {
        return m_dc1394_handle && idCamera>=0 && idCamera<m_nNumCameras && m_apCamera && m_apCamera[idCamera];
    }

    //unsigned char* CaptureFrame(int camera);
};

#endif
