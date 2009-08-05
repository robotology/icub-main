// AUTHOR:  Alexandre Bernardino - ISR/IST
// FILE:    AtrousGabor.h
// VERSION: 1.0
// DATE:    17/07/04
// CONTACT: alex@isr.ist.utl.pt
// LICENSE: GPL
// DESCRIPTION: Defines a class to compute approximations to Gabor and Gausssian 
//              filtering using the "a trous" algorithm to speed up computations.
// LIMITATIONS: Only works for a pre-defined set of scales

#pragma once

//#define USE_IPP

#ifdef USE_IPP
	#include <ipp.h>
//	#include "ippmerged.h"
//	#pragma comment(lib,"ippcorel")
//	#pragma comment(lib,"ippimerged")
//	#pragma comment(lib,"ippsmerged")
	typedef Ipp32f* IMAGE_PTR;
#else
	typedef float* IMAGE_PTR;
#endif

typedef IMAGE_PTR* IMAGE_PTR_VEC;

#define PREDEFINED_SCALES 9

class AtrousGabor
{
	//base filter coefficients for 'a trous algorithm
	static const float fa;
	static const float fb;
	static const float fc;
	//scale values for first few levels of atrous filters
	static const float atrous_scales[PREDEFINED_SCALES];
private:
	long m_i_lines;
	long m_i_cols;
	long m_i_linesext;
	long m_i_colsext;
	int m_i_strideext;
	long m_i_stridepix;
	long m_i_border;
	long m_i_scales;
	long m_i_wavelengths;
	long m_i_kernels;
	long m_i_orientations;
	double *m_scales;
	double *m_wavelengths;
	double *m_orientations;
	double *m_dc_gain;
	double pi;
	bool **m_scale_wav_table;
	IMAGE_PTR input;
	IMAGE_PTR even;
	IMAGE_PTR odd;
	IMAGE_PTR temp;
	IMAGE_PTR_VEC gausslevel;
	IMAGE_PTR_VEC laplevel;
	IMAGE_PTR_VEC reallevel;
	IMAGE_PTR_VEC imaglevel;
	IMAGE_PTR_VEC gaborlevel;
	IMAGE_PTR_VEC sine;
	IMAGE_PTR_VEC cosine;
	bool m_bAllocated;
public:
	long GetLines(){return m_i_lines;};
	long GetCols(){return m_i_cols;};
	long GetLinesEx(){return m_i_linesext;};
	long GetColsEx(){return m_i_colsext;};
	long GetStridePix(){return m_i_stridepix;};
	long GetStrideEx(){return m_i_strideext;};
	long GetBorder(){return m_i_border;};
	long GetNScales(){return m_i_scales;};
	long GetNWavs(){return m_i_wavelengths;};
	long GetNOrients(){return m_i_orientations;};
	long GetNKernels(){return m_i_kernels;};
	void GetScales(double *scales);
	void GetWavelengths(double *wavelengths);
	void GetOrientations(double *orientations);
	void GetScaleWavTable(bool *scalewavtable);

	AtrousGabor(void);
	virtual ~AtrousGabor(void);
	IMAGE_PTR AccessGaussianLevel(int level);
	IMAGE_PTR AccessLaplacianLevel(int level);
	IMAGE_PTR AccessGaborRealLevel(int level);
	IMAGE_PTR AccessGaborImagLevel(int level);
	IMAGE_PTR AccessGaborEnergyLevel(int level);
	IMAGE_PTR AccessGaussianBuffer(int level);
	IMAGE_PTR AccessLaplacianBuffer(int level);
	IMAGE_PTR AccessGaborRealBuffer(int level);
	IMAGE_PTR AccessGaborImagBuffer(int level);
	IMAGE_PTR AccessGaborEnergyBuffer(int level);
	bool Init();
	bool ProcessImage(IMAGE_PTR in);
	bool AllocateResources(long lines, long cols, long nlevels, long norients, 
		double *orients, long nwavs, double *wavs, bool *sw_table );
	bool FreeResources();
	bool IsAllocated();
private:
	void _image_prod(IMAGE_PTR i1, IMAGE_PTR i2, IMAGE_PTR out);
	void _const_prod(IMAGE_PTR in, IMAGE_PTR out, float gain);
	void _image_add(IMAGE_PTR i1, IMAGE_PTR i2, IMAGE_PTR out);
	void _image_sub(IMAGE_PTR i1, IMAGE_PTR i2, IMAGE_PTR out);
	void _image_sqrt(IMAGE_PTR in, IMAGE_PTR out);
	void _process_level(IMAGE_PTR in, IMAGE_PTR out, int level);
};
