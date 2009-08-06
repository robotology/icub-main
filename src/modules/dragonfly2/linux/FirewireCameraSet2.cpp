// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2007 Alessandro Scalzo
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <unistd.h>
#include <stdio.h>
#include "FirewireCameraSet2.h"

///////////////////////////////////////////////////////
// Init/Close functions
///////////////////////////////////////////////////////
            
bool CFWCameraSet2::Init()
{
    if (m_dc1394_handle)
    {
        printf("ERROR: Firewire cameras already open, shutdown first or restart\n");
        fflush(stdout);
        return false;
    }
    
    m_dc1394_handle=dc1394_new();
	if (!m_dc1394_handle)
	{
		return false;
	}

	if (dc1394_camera_enumerate(m_dc1394_handle,&m_pCameraList)!=DC1394_SUCCESS)
	{
		return false;
	}

	m_nNumCameras=m_pCameraList->num;

	if (m_nNumCameras==0)
	{
		return false;
	}

	m_apCamera=new CFWCamera_DR2_2*[m_nNumCameras];

    for (int cam=0; cam<m_nNumCameras; ++cam) m_apCamera[cam]=NULL;
    
    return true;
}

void CFWCameraSet2::Close()
{
	if (!m_dc1394_handle) return;

	if (m_pCameraList)
	{
		dc1394_camera_free_list(m_pCameraList);
		m_pCameraList=NULL;
	}

	if (m_apCamera)
	{
		for (int cam=0; cam<m_nNumCameras; ++cam)
			ShutdownCamera(cam);
	
		delete [] m_apCamera;
		m_apCamera=NULL;
	}

	dc1394_free(m_dc1394_handle);
    m_dc1394_handle=NULL;

	m_nNumCameras=0;
	m_nActiveCams=0;
}
    
void CFWCameraSet2::Restart()
{
    Close();
    Init();
}

bool CFWCameraSet2::StartCamera(int idCamera,unsigned int size_x,unsigned int size_y,bool bDR2,int format)
{
    if (idCamera<0 || idCamera>=m_nNumCameras)
    {
        printf("Invalid camera number\n");
        fflush(stdout);
        return false;       
    }
        
    if (m_apCamera[idCamera])
    {
        printf("ERROR: camera already started!\n");
        fflush(stdout);
        return false;        
    }

	m_apCamera[idCamera]=new CFWCamera_DR2_2();

    bool bOk=m_apCamera[idCamera]->Create(m_dc1394_handle,m_pCameraList->ids[idCamera].guid,size_x,size_y,bDR2,format);
        
    if (!bOk)
    {
        delete m_apCamera[idCamera];
        m_apCamera[idCamera]=NULL;
        
        printf("ERROR: can't start camera %d\n",idCamera);
        fflush(stdout);
        return false;
    }                

    ++m_nActiveCams;

    return true;
}
    
void CFWCameraSet2::ShutdownCamera(int idCamera)
{
    if (IsCameraReady(idCamera))
    {
        m_apCamera[idCamera]->Close();
        delete m_apCamera[idCamera];   
        m_apCamera[idCamera]=NULL;
        --m_nActiveCams;
    }       
}

///////////////////////////////////////////////////////
// Capture functions
///////////////////////////////////////////////////////

bool CFWCameraSet2::CaptureRgb(int idCamera,unsigned char* image)
{
    if (!IsCameraReady(idCamera)) return false;
        
    return m_apCamera[idCamera]->CaptureRgb(image);
}

bool CFWCameraSet2::CaptureRaw(int idCamera,unsigned char* image)
{
    if (!IsCameraReady(idCamera)) return false;
        
    return m_apCamera[idCamera]->CaptureRaw(image);
}
    
////////////////////////////////////////////////////////////////////////
// camera controls
////////////////////////////////////////////////////////////////////////

/*    
bool CFWCameraSet2::SetFeature(int camera,dc1394feature_t feature,double dValue)
{
    if (!IsCameraReady(camera)) return false;
        
    m_apCamera[camera]->settings.SetFeature(feature,dValue);
        
    return true;
}
*/
/*    
bool CFWCameraSet2::SetTemperature(int camera,double dTargetTemperature)
{
    if (!IsCameraReady(camera)) return false;       

    m_apCamera[camera]->settings.SetTemperature(dTargetTemperature);

    return true;
} 
*/
/*  
bool CFWCameraSet2::SetWhiteBalance(int camera,double dUB, double dVR)
{
    if (!IsCameraReady(camera)) return false;       

    m_apCamera[camera]->settings.SetWhiteBalance(dUB,dVR);

    return true;
}
*/      
/*
bool CFWCameraSet2::SetWhiteShading(int camera, double dR, double dG, double dB)
{
    if (!IsCameraReady(camera)) return false;       

    m_apCamera[camera]->settings.SetWhiteShading(dR,dG,dB);

    return true;
}
*/
/*
double CFWCameraSet2::GetFeature(int camera,dc1394feature_t feature)
{
    if (!IsCameraReady(camera)) return -1.0;
        
    return m_apCamera[camera]->settings.GetFeature(feature);
}
*/
/*    
bool CFWCameraSet2::GetTemperature(int camera,double& dTargetTemperature, double& dTemperature)
{
    if (!IsCameraReady(camera)) return false;       

    m_apCamera[camera]->settings.GetTemperature(dTargetTemperature,dTemperature);
    
    return true;
}
*/
/*  
bool CFWCameraSet2::GetWhiteBalance(int camera,double& dUB, double& dVR)
{
    if (!IsCameraReady(camera)) return false;       

    m_apCamera[camera]->settings.GetWhiteBalance(dUB,dVR);
    
    return true;
}
*/
/*
bool CFWCameraSet2::GetWhiteShading(int camera,double& dR, double& dG,double& dB)
{
    if (!IsCameraReady(camera)) return false;       

    m_apCamera[camera]->settings.GetWhiteShading(dR,dG,dB);
    
    return true;
}
*/
