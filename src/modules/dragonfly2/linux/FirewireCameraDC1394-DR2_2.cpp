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

#include "linux/FirewireCameraDC1394-DR2_2.h"

bool CFWCamera_DR2_2::Create(unsigned int idCamera,unsigned int size_x,unsigned int size_y,bool bDR2,int format)
{
	m_pCamera=NULL;
	m_pCameraList=NULL;
	m_dc1394_handle=0;
	m_nNumCameras=0;

	m_ConvFrame.image=new unsigned char[648*488*3];

	if (!(m_dc1394_handle=dc1394_new())) return false;

	if (dc1394_camera_enumerate(m_dc1394_handle,&m_pCameraList)!=DC1394_SUCCESS) return false;

	if (!(m_nNumCameras=m_pCameraList->num)) return false;

	if (idCamera<0 || idCamera>=m_nNumCameras)
	{
		fprintf(stderr,"ERROR: invalid camera number\n");
		return false;       
	}

	m_IsoSpeed=2;

	m_bCameraOn=false;
	m_XDim=m_YDim=0;
	m_RawBufferSize=0;
	m_color_coding=DC1394_COLOR_CODING_RGB8;

	m_nInvalidFrames=0;

	m_bDR2=bDR2;

	if (!(m_pCamera=dc1394_camera_new(m_dc1394_handle,m_pCameraList->ids[idCamera].guid)))
	{
		fprintf(stderr,"ERROR: can't create camera\n");
		return false;
	}

	dc1394_camera_reset(m_pCamera);

	dc1394speed_t speed;
	dc1394_video_get_iso_speed(m_pCamera,&speed);
	fprintf(stderr,"ISO SPEED = %d\n",speed); 

	if (m_bDR2)
	{
		dc1394_video_set_operation_mode(m_pCamera,DC1394_OPERATION_MODE_1394B);
		dc1394_video_set_iso_speed(m_pCamera,DC1394_ISO_SPEED_400);
	}        
	else
	{
		dc1394_video_set_operation_mode(m_pCamera,DC1394_OPERATION_MODE_LEGACY);
		dc1394_video_set_iso_speed(m_pCamera,DC1394_ISO_SPEED_400);
	}       

	dc1394_camera_print_info(m_pCamera,stdout); 

	m_color_coding=DC1394_COLOR_CODING_RGB8;

	switch (format)
	{
	case DR_UNINIT:
		break;

	case DR_RGB_HALF_RES:
		dc1394_video_set_mode(m_pCamera,m_video_mode=DC1394_VIDEO_MODE_FORMAT7_1);
		dc1394_format7_set_color_coding(m_pCamera,m_video_mode,m_color_coding);
		break;

	case DR_RGB_FULL_RES:
		if ((!size_x || size_x==640) && (!size_y || size_y==480))
		{ 
			dc1394_video_set_mode(m_pCamera,m_video_mode=DC1394_VIDEO_MODE_640x480_RGB8);
		}
		else
		{
			dc1394_video_set_mode(m_pCamera,m_video_mode=DC1394_VIDEO_MODE_FORMAT7_0);
			dc1394_format7_set_color_coding(m_pCamera,m_video_mode,m_color_coding);
		}
		dc1394_video_set_framerate(m_pCamera,DC1394_FRAMERATE_15);
		break;

	case DR_BAYER_FULL_RES:
		dc1394_video_set_mode(m_pCamera,m_video_mode=DC1394_VIDEO_MODE_FORMAT7_0);
		dc1394_format7_set_color_coding(m_pCamera,m_video_mode,m_color_coding=m_bDR2?DC1394_COLOR_CODING_RAW8:DC1394_COLOR_CODING_MONO8);
		break;

	default:
		fprintf(stderr,"invalid video format, reading from camera\n");    
	}

	LoadSettings(size_x,size_y);

	for (int f=DC1394_FEATURE_MIN; f<=DC1394_FEATURE_MAX; ++f)
	{	
		dc1394bool_t bPresent;
		dc1394_feature_is_present(m_pCamera,(dc1394feature_t)f,&bPresent);

		if (bPresent)
		{
			dc1394_feature_get_boundaries(m_pCamera,(dc1394feature_t)f,&m_iMin[f-DC1394_FEATURE_MIN],&m_iMax[f-DC1394_FEATURE_MIN]);

			dc1394bool_t bSwitch;
			dc1394_feature_is_switchable(m_pCamera,(dc1394feature_t)f,&bSwitch);
			if (bSwitch)
			{
				if (f!=DC1394_FEATURE_EXPOSURE && f!=DC1394_FEATURE_IRIS && f<DC1394_FEATURE_TEMPERATURE)
				{
					dc1394_feature_set_power(m_pCamera,(dc1394feature_t)f,DC1394_ON);
				}
				else
				{
					dc1394_feature_set_power(m_pCamera,(dc1394feature_t)f,DC1394_OFF);
				}
			}

			dc1394_feature_set_mode(m_pCamera,(dc1394feature_t)f,DC1394_FEATURE_MODE_MANUAL);
			dc1394_feature_set_absolute_control(m_pCamera,(dc1394feature_t)f,DC1394_OFF);
		}
	}

	if (dc1394_capture_setup(m_pCamera,NUM_DMA_BUFFERS,DC1394_CAPTURE_FLAGS_DEFAULT)!=DC1394_SUCCESS)
	{
		fprintf(stderr,"ERROR: Can't set DC1394_CAPTURE_FLAGS_DEFAULT\n");
		dc1394_camera_free(m_pCamera);
		m_pCamera=NULL;
		return false;
	}

	if (dc1394_video_set_transmission(m_pCamera,DC1394_ON)!=DC1394_SUCCESS)
	{
		fprintf(stderr,"ERROR: Can't set DC1394_ON\n");
		dc1394_camera_free(m_pCamera);
		m_pCamera=NULL;
		return false;
	}

	setBroadcastDC1394(false);
	//setBytesPerPacketDC1394(45);

	m_bCameraOn=true;

	return true;
}

void CFWCamera_DR2_2::Close()
{
	if (m_pCamera)
	{
		dc1394_video_set_transmission(m_pCamera,DC1394_OFF);
		dc1394_capture_stop(m_pCamera);
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

void CFWCamera_DR2_2::LoadSettings(unsigned int size_x,unsigned int size_y)
{
	// video format
	dc1394_video_get_mode(m_pCamera,&m_video_mode);
	fprintf(stderr,"video mode %d\n",m_video_mode-DC1394_VIDEO_MODE_MIN);

	if (m_video_mode<DC1394_VIDEO_MODE_FORMAT7_MIN)
	{
		m_RelBPP=100;

		UpdateNonFormat7Window();
	}
	else // FORMAT 7
	{
		// color coding
		dc1394_format7_get_color_coding(m_pCamera,m_video_mode,&m_color_coding);
		fprintf(stderr,"color coding %d\n",m_color_coding-DC1394_COLOR_CODING_MIN);	

		m_bIsFormat7=true;
		m_bIsRaw=(m_bDR2 && m_color_coding==DC1394_COLOR_CODING_RAW8 || 
			!m_bDR2 && m_color_coding==DC1394_COLOR_CODING_MONO8);

		m_bIsRgb=m_color_coding==DC1394_COLOR_CODING_RGB8;

		// max image size in this mode
		unsigned int xmax,ymax;
		dc1394_format7_get_max_image_size(m_pCamera,m_video_mode,&xmax,&ymax);
		fprintf(stderr,"max image size %d x %d\n",xmax,ymax);

		dc1394_format7_get_image_size(m_pCamera,m_video_mode,&m_XDim,&m_YDim);
		fprintf(stderr,"image size %d x %d\n",m_XDim,m_YDim);

		// actual image size
		if (size_x>0 && size_y>0)
		{
			if (size_x>xmax)
				fprintf(stderr,"Invalid image width, setting to %d\n",size_x=xmax);

			if (size_y>ymax)
				fprintf(stderr,"Invalid image height, setting to %d\n",size_y=ymax);

			m_XDim=size_x;
			m_YDim=size_y;

			dc1394_format7_set_image_size(m_pCamera,m_video_mode,m_XDim,m_YDim);
			fprintf(stderr,"new image size %d x %d\n",m_XDim,m_YDim);

			int xcorn=(xmax-m_XDim)/2;
			int ycorn=(ymax-m_YDim)/2;

			dc1394_format7_set_image_position(m_pCamera,m_video_mode,xcorn,ycorn);
			fprintf(stderr,"new image pos %d x %d\n",xcorn,ycorn);
		}

		uint64_t bufsize;
		dc1394_format7_get_total_bytes(m_pCamera,m_video_mode,&bufsize);
		m_RawBufferSize=(unsigned int)bufsize;

		fprintf(stderr,"raw buffer size %d\n",m_RawBufferSize);

		// granularity
		unsigned int x_unit_pos,y_unit_pos;
		dc1394_format7_get_unit_position(m_pCamera,m_video_mode,&x_unit_pos,&y_unit_pos);
		fprintf(stderr,"position units %d %d\n",x_unit_pos,y_unit_pos);

		unsigned int x_unit_size,y_unit_size;
		dc1394_format7_get_unit_size(m_pCamera,m_video_mode,&x_unit_size,&y_unit_size);
		fprintf(stderr,"size units %d %d\n",x_unit_size,y_unit_size);

		//getBytesPerPacketDC1394();
		//fprintf(stderr,"relative bpp %d\n",m_RelBPP);
		setBytesPerPacketDC1394(100);           
	}  
}

void CFWCamera_DR2_2::UpdateNonFormat7Window()
{
	m_bIsRgb=false;
	m_bIsRaw=false;
	m_bIsFormat7=false;

	switch (m_video_mode-DC1394_VIDEO_MODE_MIN)
	{
	case 0:
		m_RawBufferSize=(m_XDim=160)*(m_YDim=120)*3;
		break;   
	case 1:
		m_RawBufferSize=(m_XDim=320)*(m_YDim=240)*2;
		break;
	case 2:  
		m_RawBufferSize=((m_XDim=640)*(m_YDim=480)*3)/2;
		break;   
	case 3:
		m_RawBufferSize=(m_XDim=640)*(m_YDim=480)*2;
		break;
	case 4:
		m_bIsRgb=true;  
		m_RawBufferSize=(m_XDim=640)*(m_YDim=480)*3;
		break;   
	case 5:
		m_RawBufferSize=(m_XDim=640)*(m_YDim=480);
		break;
	case 6:
		m_RawBufferSize=(m_XDim=640)*(m_YDim=480)*2;
		break;              
	}
}

void CFWCamera_DR2_2::UpdateFormat7Window(int xdim,int ydim)
{
	if (!((1<<(m_color_coding-DC1394_COLOR_CODING_MIN)) & getActualColorCodingMaskDC1394()))
	{
		fprintf(stderr,"Color coding %d not supported in this video mode, setting RGB8\n",m_color_coding-DC1394_COLOR_CODING_MIN);
		m_color_coding=DC1394_COLOR_CODING_RGB8;
	}

	// color coding
	dc1394_format7_set_color_coding(m_pCamera,m_video_mode,m_color_coding);
	fprintf(stderr,"color coding %d\n",m_color_coding-DC1394_COLOR_CODING_MIN);

	m_bIsFormat7=true;
	m_bIsRaw=(m_bDR2 && m_color_coding==DC1394_COLOR_CODING_RAW8 || 
		!m_bDR2 && m_color_coding==DC1394_COLOR_CODING_MONO8);

	m_bIsRgb=m_color_coding==DC1394_COLOR_CODING_RGB8;

	// max image size in this mode
	unsigned int xmax,ymax;
	dc1394_format7_get_max_image_size(m_pCamera,m_video_mode,&xmax,&ymax);
	fprintf(stderr,"max image size %d x %d\n",xmax,ymax);

	if (xdim && ydim)
	{
		m_XDim=xdim;
		m_YDim=ydim;
	}
	else
	{
		dc1394_format7_get_image_size(m_pCamera,m_video_mode,&m_XDim,&m_YDim);
	}		
	fprintf(stderr,"image size %d x %d\n",m_XDim,m_YDim);

	if (m_XDim>xmax)
		fprintf(stderr,"Invalid image width, setting to %d\n",m_XDim=xmax);

	if (m_YDim>ymax)
		fprintf(stderr,"Invalid image height, setting to %d\n",m_YDim=ymax);

	dc1394_format7_set_image_size(m_pCamera,m_video_mode,m_XDim,m_YDim);
	fprintf(stderr,"new image size %d x %d\n",m_XDim,m_YDim);

	int xcorn=(xmax-m_XDim)/2;
	int ycorn=(ymax-m_YDim)/2;

	dc1394_format7_set_image_position(m_pCamera,m_video_mode,xcorn,ycorn);
	fprintf(stderr,"new image pos %d x %d\n",xcorn,ycorn);

	uint64_t bufsize;
	dc1394_format7_get_total_bytes(m_pCamera,m_video_mode,&bufsize);
	m_RawBufferSize=(unsigned int)bufsize;

	setBytesPerPacketDC1394(100);
}  

bool CFWCamera_DR2_2::Capture(yarp::sig::ImageOf<yarp::sig::PixelRgb>* pImage,unsigned char *pBuffer,bool bRaw)
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

	m_Stamp.update();

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

/*
void CFWCamera_DR2_2::Bayer(unsigned char* dst,unsigned char* src)
{
int xstart=2;
int ystart=2;
int _2XDim=m_XDim*2;

const int row_len=m_XDim*3+12;
const int x2=m_XDim/2-2,y2=m_YDim/2-2;

unsigned char *g11start=src+ystart*m_XDim+xstart,*r12start=g11start+1,		      *g13start=r12start+1,           *r10start=g11start-1;
unsigned char *b01start=g11start-m_XDim,         *g02start=r12start-m_XDim, *b03start=g13start-m_XDim, *g00start=r10start-m_XDim;

unsigned char *g00=g00start,*b01=b01start,*g02=g02start,*b03=b03start;
unsigned char *r10=r10start,*g11=g11start,*r12=r12start,*g13=g13start;
unsigned char *g20=g00start+_2XDim,*b21=b01start+_2XDim,*g22=g02start+_2XDim,*b23=b03start+_2XDim;
unsigned char *r30=r10start+_2XDim,*g31=g11start+_2XDim,*r32=r12start+_2XDim,*g33=g13start+_2XDim;

unsigned char *R00=dst+6*m_XDim+6,*G00=R00+1,*B00=G00+1;
unsigned char *R01=B00+1,*G01=R01+1,*B01=G01+1;
unsigned char *R10=R00+3*m_XDim,*G10=R10+1,*B10=G10+1;
unsigned char *R11=B10+1,*G11=R11+1,*B11=G11+1;

for (int y=0; y<y2; ++y)
{
for (int x=0; x<x2; ++x)
{
*R00=(*r10+*r12+1)>>1; *G00=*g11; *B00=(*b01+*b21+1)>>1;

*R01=*r12; *G01=(*g02+*g11+*g13+*g22+2)>>2; *B01=(*b01+*b03+*b21+*b23+2)>>2;				

*R10=(*r10+*r12+*r30+*r32+2)>>2; *G10=(*g11+*g20+*g22+*g31+2)>>2; *B10=*b21;

*R11=(*r12+*r32+1)>>1; *G11=*g22; *B11=(*b21+*b23+1)>>1;

R00+=6; G00+=6; B00+=6;
R01+=6; G01+=6; B01+=6;	
R10+=6; G10+=6; B10+=6;
R11+=6; G11+=6; B11+=6;

g00=g02; b01=b03; g02+=2; b03+=2;
r10=r12; g11=g13; r12+=2; g13+=2;
g20=g22; b21=b23; g22+=2; b23+=2;
r30=r32; g31=g33; r32+=2; g33+=2;
}

R00+=row_len; G00+=row_len; B00+=row_len;
R01+=row_len; G01+=row_len; B01+=row_len;		
R10+=row_len; G10+=row_len; B10+=row_len;
R11+=row_len; G11+=row_len; B11+=row_len;

g20=(g00=(g00start+=_2XDim))+_2XDim; b21=(b01=(b01start+=_2XDim))+_2XDim; g22=(g02=(g02start+=_2XDim))+_2XDim; b23=(b03=(b03start+=_2XDim))+_2XDim;
r30=(r10=(r10start+=_2XDim))+_2XDim; g31=(g11=(g11start+=_2XDim))+_2XDim; r32=(r12=(r12start+=_2XDim))+_2XDim; g33=(g13=(g13start+=_2XDim))+_2XDim;
}
}
*/

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
		if (modes.modes[num]==DC1394_FEATURE_MODE_MANUAL) return true;
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
		if (modes.modes[num]==DC1394_FEATURE_MODE_AUTO) return true;
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
		if (modes.modes[num]==DC1394_FEATURE_MODE_ONE_PUSH_AUTO) return true;
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
	if (!m_pCamera) return false;
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
	dc1394video_modes_t modes;
	dc1394_video_get_supported_modes(m_pCamera,&modes);

	unsigned int mask=0;
	for (unsigned int m=0; m<modes.num; ++m)
		mask|=1<<(modes.modes[m]-DC1394_VIDEO_MODE_MIN);

	fprintf(stderr,"video mode mask: %x\n",mask);
	fflush(stdout);

	return mask;
}

// 13
bool CFWCamera_DR2_2::setVideoModeDC1394(int video_mode)
{
	fprintf(stderr,"SET VIDEO MODE %d\n",video_mode);

	m_AcqMutex.wait();

	if (!m_pCamera)
	{
		m_AcqMutex.post();
		return false;
	}

    m_nInvalidFrames=NUM_DMA_BUFFERS;

	dc1394_video_set_transmission(m_pCamera,DC1394_OFF);
	dc1394_capture_stop(m_pCamera);

	dc1394video_mode_t vm=(dc1394video_mode_t)((int)DC1394_VIDEO_MODE_MIN+video_mode);
	bool bRetVal=DC1394_SUCCESS==dc1394_video_set_mode(m_pCamera,vm);

	if (bRetVal)
	{
		m_video_mode=vm;

		if (vm<DC1394_VIDEO_MODE_FORMAT7_MIN)
		{
			fprintf(stderr,"Attempting to set NON format 7\n");
			UpdateNonFormat7Window();
		}
		else
		{
			fprintf(stderr,"Attempting to set format 7\n");
			UpdateFormat7Window();
		}
	}

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
	//setBytesPerPacketDC1394(m_RelBPP);

	return bRetVal;
}

// 14
unsigned int CFWCamera_DR2_2::getVideoModeDC1394()
{ 
	dc1394_video_get_mode(m_pCamera,&m_video_mode);
	return m_video_mode-DC1394_VIDEO_MODE_MIN;
}

// 15
unsigned int CFWCamera_DR2_2::getFPSMaskDC1394()
{
	if (!m_pCamera || m_bIsFormat7) return 0;

	dc1394framerates_t fps;
	dc1394_video_get_supported_framerates(m_pCamera,m_video_mode,&fps);

	unsigned int mask=0;
	for (unsigned int f=0; f<fps.num; ++f)
		mask|=1<<(fps.framerates[f]-DC1394_FRAMERATE_MIN);

	return mask;
}

// 16
unsigned int CFWCamera_DR2_2::getFPSDC1394()
{
	if (!m_pCamera || m_bIsFormat7) return 0;

	dc1394framerate_t fps; 
	dc1394_video_get_framerate(m_pCamera,&fps);

	return fps-DC1394_FRAMERATE_MIN;
}

// 17
bool CFWCamera_DR2_2::setFPSDC1394(int fps)
{
	if (!m_pCamera || m_bIsFormat7) return false;

	return DC1394_SUCCESS==dc1394_video_set_framerate(m_pCamera,(dc1394framerate_t)((int)fps+DC1394_FRAMERATE_MIN));	
}

// 18
unsigned int CFWCamera_DR2_2::getISOSpeedDC1394()
{
	if (!m_pCamera) return false;
	dc1394speed_t speed;
	dc1394_video_get_iso_speed(m_pCamera,&speed);
	return m_IsoSpeed=(speed-DC1394_ISO_SPEED_MIN);
}

// 19
bool CFWCamera_DR2_2::setISOSpeedDC1394(int speed)
{ 	
	if (!m_pCamera) return false;
	return DC1394_SUCCESS==dc1394_video_set_iso_speed(m_pCamera,(dc1394speed_t)((m_IsoSpeed=speed)+(int)DC1394_ISO_SPEED_MIN));
}

// 20
unsigned int CFWCamera_DR2_2::getColorCodingMaskDC1394(unsigned int video_mode)
{
	if (!m_pCamera) return 0;

	dc1394video_mode_t vm=(dc1394video_mode_t)(video_mode+DC1394_VIDEO_MODE_MIN);

	if (vm<DC1394_VIDEO_MODE_FORMAT7_MIN) return 0;

	dc1394color_codings_t codings;
	dc1394_format7_get_color_codings(m_pCamera,vm,&codings);

	unsigned int mask=0;
	for (unsigned int m=0; m<codings.num; ++m)
		mask|=1<<(codings.codings[m]-DC1394_COLOR_CODING_MIN);

	fprintf(stderr,"color coding mask for video mode %d is %x\n",video_mode,mask);

	return mask;
}
unsigned int CFWCamera_DR2_2::getActualColorCodingMaskDC1394()
{
	if (!m_pCamera || !m_bIsFormat7) return 0;

	dc1394color_codings_t codings;
	dc1394_format7_get_color_codings(m_pCamera,m_video_mode,&codings);

	unsigned int mask=0;
	for (unsigned int m=0; m<codings.num; ++m)
		mask|=1<<(codings.codings[m]-DC1394_COLOR_CODING_MIN);

	fprintf(stderr,"actual color coding mask %x\n",mask);

	return mask;
}

// 21
unsigned int CFWCamera_DR2_2::getColorCodingDC1394()
{
	if (!m_pCamera || !m_bIsFormat7) return 0;

	dc1394color_coding_t coding;
	//dc1394_video_get_mode(m_pCamera,&m_video_mode); 
	dc1394_format7_get_color_coding(m_pCamera,m_video_mode,&coding);
	return coding-DC1394_COLOR_CODING_MIN;
}

// 22
bool CFWCamera_DR2_2::setColorCodingDC1394(int coding)
{
	m_AcqMutex.wait();

	if (!m_pCamera || !m_bIsFormat7)
	{
		m_AcqMutex.post();
		return false;
	}

	m_nInvalidFrames=NUM_DMA_BUFFERS;

	dc1394_video_set_transmission(m_pCamera,DC1394_OFF);
	dc1394_capture_stop(m_pCamera);

	m_color_coding=(dc1394color_coding_t)((int)coding+DC1394_COLOR_CODING_MIN);
	UpdateFormat7Window();

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

	//setBytesPerPacketDC1394(m_RelBPP);

	return bRetVal;
}	

// 25
bool CFWCamera_DR2_2::getFormat7MaxWindowDC1394(unsigned int &xdim,unsigned int &ydim,unsigned int &xstep,unsigned int &ystep)
{
	if (!m_pCamera) return false;

	if (!m_bIsFormat7)
	{
		xdim=m_XDim;
		ydim=m_YDim;
		xstep=ystep=2;
		return true;
	}

	bool ok=DC1394_SUCCESS==dc1394_format7_get_unit_size(m_pCamera,m_video_mode,&xstep,&ystep);
	ok&=DC1394_SUCCESS==dc1394_format7_get_max_image_size(m_pCamera,m_video_mode,&xdim,&ydim);

	return ok;
}

// 26
bool CFWCamera_DR2_2::setFormat7WindowDC1394(unsigned int xdim,unsigned int ydim)
{
	m_AcqMutex.wait();

	if (!m_pCamera || !m_bIsFormat7)
	{
		m_AcqMutex.post();
		return false;
	}

	m_nInvalidFrames=NUM_DMA_BUFFERS;
	dc1394_video_set_transmission(m_pCamera,DC1394_OFF);
	dc1394_capture_stop(m_pCamera);

	UpdateFormat7Window(xdim,ydim);

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

	//setBytesPerPacketDC1394(m_RelBPP);

	return bRetVal;
}

// 27
bool CFWCamera_DR2_2::getFormat7WindowDC1394(unsigned int &xdim,unsigned int &ydim)
{
	if (!m_pCamera) return false;

	xdim=m_XDim;
	ydim=m_YDim;

	//bool ok=DC1394_SUCCESS==dc1394_format7_get_image_size(m_pCamera,m_video_mode,&xdim,&ydim);

	return true;
}	

// 28
bool CFWCamera_DR2_2::setOperationModeDC1394(bool b1394b)
{
	m_AcqMutex.wait();

	if (!m_pCamera || !m_bDR2)
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
	if (!m_pCamera || !m_bDR2) return false;
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

	LoadSettings();

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
	if (!m_pCamera || !m_bIsFormat7) return 0;

	return m_RelBPP;

	// minimum and maximum BPP
	dc1394_format7_get_packet_parameters(m_pCamera,m_video_mode,&m_MinBPP,&m_MaxBPP);
	fprintf(stderr,"min and max BPP %d %d\n",m_MinBPP,m_MaxBPP);

	unsigned int bpp;
	dc1394_format7_get_packet_size(m_pCamera,m_video_mode,&bpp);
	fprintf(stderr,"actual BPP %d\n",bpp);
	return m_RelBPP=(m_MaxBPP!=m_MinBPP)?(100*(bpp-m_MinBPP))/(m_MaxBPP-m_MinBPP):45;		
}

// 40
bool CFWCamera_DR2_2::setBytesPerPacketDC1394(unsigned int bpp)
{
	if (!m_pCamera || !m_bIsFormat7) return false;

	if (bpp<0) bpp=0; else if (bpp>100) bpp=100;

	m_RelBPP=bpp;

	// minimum and maximum BPP
	dc1394_format7_get_packet_parameters(m_pCamera,m_video_mode,&m_MinBPP,&m_MaxBPP);
	fprintf(stderr,"min and max BPP %d %d\n",m_MinBPP,m_MaxBPP);

	unsigned int act_bpp;
	dc1394_format7_get_packet_size(m_pCamera,m_video_mode,&act_bpp);
	fprintf(stderr,"bytes per packet now %d\n",act_bpp);

	unsigned int max_iso_bpp=512<<m_IsoSpeed;
	unsigned int max_bpp=m_MaxBPP/2;

	bpp=(bpp*(max_bpp<max_iso_bpp?max_bpp:max_iso_bpp))/100;

	bpp=(bpp/m_MinBPP)*m_MinBPP;

	if (bpp<m_MinBPP) bpp=m_MinBPP;

	fprintf(stderr,"setting %d bytes per packet\n",bpp);
	bool bRetVal=DC1394_SUCCESS==dc1394_format7_set_packet_size(m_pCamera,m_video_mode,bpp);

	return bRetVal;
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
