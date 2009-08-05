// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino (VisLab/ISR/IST)
 */

#ifndef _LogPolarGlobalDisparity_h
#define _LogPolarGlobalDisparity_h

#include <float.h>
#include "DisparityChannel.h"
#include<stdio.h>



class CLogPolarGlobalDisparity
{	
	typedef struct {
		int vert_coord;
		int horz_coord;
	}	img_coord_pair;

private:
	typedef CDisparityChannel*	DISPARITY_CHANNEL_PTR;
	typedef DISPARITY_CHANNEL_PTR*	DISP_CHANN_PTR_VEC;
	DISP_CHANN_PTR_VEC m_pDisparityLUTArray;
	int m_iNumChannels;
	bool m_b_initialized;
	int m_a;

public:
	CLogPolarGlobalDisparity() : 
		m_iNumChannels(0), 
		m_b_initialized(false),
		m_pDisparityLUTArray(NULL)
	{	
	};

	virtual ~CLogPolarGlobalDisparity()
	{
	};

	int GetNumChannels()			const {return m_iNumChannels;};
	int GetDisparity(int ch);
	bool CreateLogPolarGlobalDisparity( int num_channels, int *disparities, CLogPolarSensor *lps );
	bool CloseLogPolarGlobalDisparity();
	
	// COMPUTATION FUNCTIONS - CAREFULL - THE FOLLOWING FUNCTIONS WORK FOR SIZES < 16k !

//	template <class intype>
	int ComputeDisparity( unsigned char *img1, unsigned char *img2, int size,  int n_color_channels, int *index);

//	template <class intype>
	double ComputeChannelOutput( int channel, unsigned char *img1, unsigned char *img2, int size, int n_color_channels );

	double SSD_channel(unsigned char *img1, int *avg1, unsigned char *img2, int *avg2, int size,  int channel, int n_color_channels );
	int SSDComputeDisparity( unsigned char *img1, unsigned char *img2, int size,  int n_color_channels, int *index);


protected:
//	template <class intype>
	double NormalizedCrossCorrelation( unsigned char *img1, int avg1, unsigned char *img2, int avg2, int size, int n_color_channels );

//	template <class intype>
	double NCC_channel(unsigned char *img1, int avg1, unsigned char *img2, int avg2, int size,  int channel, int n_color_channels );

	int SumOfElements( unsigned char *img, int size, int n_color_channels );
	void SumOfElementsperchannel( unsigned char *img, int size, int n_color_channels, int *avg );
};
#endif //_LogPolarGlobalDisparity_h
