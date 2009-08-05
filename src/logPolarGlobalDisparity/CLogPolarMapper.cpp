// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino (VisLab/ISR/IST)
 */

#include "CLogPolarMapper.h"



void CLogPolarMapper::Create()
{
	create_logpolar_lut();
};
void CLogPolarMapper::Destroy()
{
	destroy_logpolar_lut();
};

/// computes the number of image lines to allocate, including extra space to store fovea pixels
int CLogPolarMapper::GetImageBufferLines()
{
	if(!m_bValidData || logpolar_has_changed || imageplane_has_changed)
		throw "Invalid Object";
	int extra_lines = (m_iFoveaPixels == 0) ? 0 : (m_iFoveaPixels-1)/m_iEccentr + 1;
	return m_iAngles + extra_lines;
}

/*
int compute_warp_field_2( CHomography * h, CImg * wf2);
int compute_warp_field_1(CHomography * h, CImg * wf1);
int get_fov(CImg * fov);
int get_log(CImg * log);
int invmap(CImg * log, CImg * fov, CImg *cart);
int logmap(CImg * cart);
*/

//int compute_warp_fields(/*[in]*/ float *h, /*[out]*/ float *wf1, /*[out]*/ float *wf2);

int CLogPolarMapper::logmap8U( byte * cart, byte * log, bool bMapFovea)
{
	_logmap( cart, log, bMapFovea);
    return 0;
};
int CLogPolarMapper::invmap8U( byte * log, byte * cart, bool bMapFovea )
{
	_invmap(log, cart, bMapFovea);
    return 0;
};
int CLogPolarMapper::logmap8U_C3(byte * cart, byte * log, bool bMapFovea)
{
	_logmap(cart,log,3,bMapFovea);
    return 0;
};
int CLogPolarMapper::invmap8U_C3(byte * log, byte * cart, bool bMapFovea)
{
	_invmap(log,cart,3, bMapFovea);
    return 0;
};
int CLogPolarMapper::logmap8U_AC4( byte * cart, byte * log, bool bMapFovea )
{
	_logmap( cart, log, 4, bMapFovea);
    return 0;
};
int CLogPolarMapper::invmap8U_AC4( byte * log, byte * cart, bool bMapFovea)
{
	_invmap(log,cart,4,bMapFovea);
    return 0;
};

int CLogPolarMapper::logmap32F( float * cart, float * log, bool bMapFovea)
{
	_logmap( cart, log, bMapFovea);
    return 0;
};
int CLogPolarMapper::invmap32F( float * log, float * cart, bool bMapFovea )
{
	_invmap(log, cart, bMapFovea);
    return 0;
};
int CLogPolarMapper::logmap32F_C3(float * cart, float * log, bool bMapFovea)
{
	_logmap(cart,log,3,bMapFovea);
    return 0;
};
int CLogPolarMapper::invmap32F_C3(float * log, float * cart, bool bMapFovea)
{
	_invmap(log,cart,3, bMapFovea);
    return 0;
};
int CLogPolarMapper::logmap32F_AC4( float * cart, float * log, bool bMapFovea )
{
	_logmap( cart, log, 4, bMapFovea);
    return 0;
};
int CLogPolarMapper::invmap32F_AC4( float * log, float * cart, bool bMapFovea)
{
	_invmap(log,cart,4,bMapFovea);
    return 0;
};

