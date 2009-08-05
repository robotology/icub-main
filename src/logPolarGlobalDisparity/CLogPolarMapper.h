// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino (VisLab/ISR/IST)
 */

#ifndef CLOGPOLARMAPPER_H
#define CLOGPOLARMAPPER_H

/// Type specific classes for logpolar mapping and image allocation.

#include "CLogPolarSensor.h"

typedef unsigned char byte;

class CLogPolarMapper : public CLogPolarSensor
{
public:
	CLogPolarMapper()
	{
	};
	virtual ~CLogPolarMapper()
	{
	};

	void Create();
	void Destroy();
	
	/// computes the number of image lines to allocate, including extra space to store fovea pixels
	int GetImageBufferLines();
	
	/*
	int compute_warp_field_2( CHomography * h, CImg * wf2);
	int compute_warp_field_1(CHomography * h, CImg * wf1);
	int get_fov(CImg * fov);
	int get_log(CImg * log);
	int invmap(CImg * log, CImg * fov, CImg *cart);
	int logmap(CImg * cart);
	*/

	//int compute_warp_fields(/*[in]*/ float *h, /*[out]*/ float *wf1, /*[out]*/ float *wf2);

	int logmap8U( byte * cart, byte * log, bool bMapFovea);
	int invmap8U( byte * log, byte * cart, bool bMapFovea );
	int logmap8U_C3(byte * cart, byte * log, bool bMapFovea);
	int invmap8U_C3(byte * log, byte * cart, bool bMapFovea);
	int logmap8U_AC4( byte * cart, byte * log, bool bMapFovea );
	int invmap8U_AC4( byte * log, byte * cart, bool bMapFovea);
	int logmap32F( float * cart, float * log, bool bMapFovea);
	int invmap32F( float * log, float * cart, bool bMapFovea );
	int logmap32F_C3(float * cart, float * log, bool bMapFovea);
	int invmap32F_C3(float * log, float * cart, bool bMapFovea);
	int logmap32F_AC4( float * cart, float * log, bool bMapFovea );
	int invmap32F_AC4( float * log, float * cart, bool bMapFovea);
};

#endif //CLOGPOLARSENSOR_H