// AUTHOR:  Alexandre Bernardino - ISR/IST
// FILE:    AtrousGabor.cpp
// VERSION: 1.0
// DATE:    17/07/04
// CONTACT: alex@isr.ist.utl.pt
// LICENSE: GPL
// DESCRIPTION: Defines a class to compute approximations to Gabor and Gausssian 
//              filtering using the "a trous" algorithm to speed up computations.
// LIMITATIONS: Only works for a pre-defined set of scales


#include "iCub/AtrousGabor.h"
//#include <math.h>
//#include <memory.h>
//#include <malloc.h>
#include <string.h>
#include <stdlib.h>
#include <cmath>


/*********************************************************************************
Initializing class static variables
**********************************************************************************/

//base filter coefficients for 'a trous algorithm

const float AtrousGabor::fa = 0.40f;
const float AtrousGabor::fb = 0.25f;
const float AtrousGabor::fc = 0.05f;

//scale values for first few levels of atrous filters
const float AtrousGabor::atrous_scales[PREDEFINED_SCALES] = { 
		0.94868329805051f,
		2.12132034355964f,
		4.34741302385683f,
		8.74642784226795f,
		17.51856158478772f,
		35.04996433664378f,
		70.10634778677320f,
		140.2159049466215f,
		280.4334145568249f,
	}; // the following scales are approx. multiplied by powers of 2*/

/*********************************************************************************
Constructor():
	Sets all variables to 0		
**********************************************************************************/
AtrousGabor::AtrousGabor(void)
{
	m_i_scales = 0;
	m_i_wavelengths = 0;
	m_i_orientations = 0;
	m_i_kernels = 0;
	m_i_lines = 0;
	m_i_cols = 0;
	m_i_linesext = 0;
	m_i_colsext = 0;
	m_i_strideext = 0;
	m_i_stridepix = 0;
	m_i_border = 0;
	sine = 0;
	cosine = 0;
	gausslevel = 0;
	laplevel = 0;
	reallevel = 0;
	imaglevel = 0;
	gaborlevel = 0;
	even = 0;
	odd = 0;
	temp = 0;
	input = 0;
	m_scales = 0;
	m_wavelengths = 0;
	m_orientations = 0;
	m_dc_gain = 0;
	m_scale_wav_table = 0;
	m_bAllocated = false;
}

/*********************************************************************************
Init():
	Initializes important parameters, variables and libraries.
	Should be called before start using the object.		
**********************************************************************************/
bool AtrousGabor::Init()
{
#ifdef USE_IPP
//	InitStatic(); //initialize Intel Performance Primitives Library
#endif
	return true;
}

/*********************************************************************************
Destructor():
	Releases all allocated resources and exits
**********************************************************************************/
AtrousGabor::~AtrousGabor(void)
{
	FreeResources();
}

/*********************************************************************************
IsAllocated():
	Check if memory resources have already been allocated		
**********************************************************************************/
bool AtrousGabor::IsAllocated()
{
	return m_bAllocated;
}

/*********************************************************************************
AllocateResources():
	- Creates internal structures and memory space for the required computations
	- Orientations are expressed in degrees and Wavelenght in pixels
	- If neither orientations nor wavelengths are specified, only computes 
	the gaussian levels. 
	- If orientation is specified but wavelenghts are not specified, computes 
	only gaussian and laplacian decompositions.

ARGUMENTS:
	lines - image height
	cols - image width
	nlevels - number of scales to compute 
	norient - number of orientations to compute (all will be computed)
	orient - values of orientation in degrees 
	nwavs - number of wavelengths specified (not all may be computed)
	wavs - values of wavelengths in pixels per cycle
	sw_table - boolean (0 or 1) matrix specifying which combinations 
				scale/wavelength to compute. A particular combination
				of scale i and wavelength j will be computed if 
				sw_table[i][j] = 1.


**********************************************************************************/

bool AtrousGabor::AllocateResources(long lines, long cols, long nlevels, long norients, double *orients, 
									long nwavs, double *wavs, bool *sw_table )
{
	int i,j,m,n;
	double wx, wy, angle;
	pi = 3.14159265358979323846;

	if( norients < 0 )
		throw "Invalid Argument";
	if( nwavs < 0 )
		throw "Invalid Argument";
	if( nlevels < 1 )
		throw "Invalid Argument";
	if(IsAllocated())
		FreeResources();

	m_i_orientations = norients;
	m_i_scales = nlevels;
	m_i_wavelengths = nwavs;

	m_scales = (double*)malloc(m_i_scales*sizeof(double));
	for( i = 0; i < m_i_scales; i++ )
	{
		if(i < PREDEFINED_SCALES)
			m_scales[i] = atrous_scales[i];
		else
			m_scales[i] = 2*m_scales[i-1];
	}
		

	if((m_i_wavelengths != 0) && (m_i_orientations != 0))
	{
		m_wavelengths = (double*)malloc(m_i_wavelengths*sizeof(double));
		m_orientations = (double*)malloc(m_i_orientations*sizeof(double));
		m_dc_gain = (double*)malloc(m_i_orientations*m_i_wavelengths*m_i_scales*sizeof(double));
		m_scale_wav_table = (bool**)malloc(m_i_scales*sizeof(bool*));
		for( i = 0; i < m_i_scales; i++ )
			m_scale_wav_table[i] = (bool*)calloc(m_i_wavelengths,sizeof(bool));

		for(i=0;i<m_i_scales;i++)
			for(j=0;j<m_i_wavelengths;j++)
				m_scale_wav_table[i][j] = sw_table[i*m_i_scales+j];

		m_i_kernels = 0;
		for(i=0;i<m_i_scales;i++)
			for(j=0;j<m_i_wavelengths;j++)
				if(m_scale_wav_table[i][j])
					m_i_kernels++;

		for( i = 0; i < m_i_orientations; i++ )
			m_orientations[i] = orients[i]*pi/180.0f;

		for( i = 0; i < m_i_wavelengths; i++ )
			m_wavelengths[i] = wavs[i];
	}

	double maxscale;
	maxscale = m_scales[m_i_scales-1];
	int minimgsize = (int)(5*maxscale);
	
	//value to extend the image margins to deal with border effect
	//m_i_border = (long)maxscale*2;
	long dim0 = 5; //coordinate this with _process_level
	m_i_border = (long)pow(2.0,(double)m_i_scales-1)*(dim0-1)+1; //size of the largest filter
	//m_i_border = 0;

	m_i_lines = lines;
	m_i_cols = cols;
	m_i_linesext = lines + 2*m_i_border;
	m_i_colsext = cols + 2*m_i_border;

	gausslevel = (IMAGE_PTR_VEC)malloc(m_i_scales*sizeof(IMAGE_PTR));
	if(m_i_orientations != 0)
	{
		laplevel = (IMAGE_PTR_VEC)malloc(m_i_scales*sizeof(IMAGE_PTR));
	}
	if((m_i_orientations != 0 )&&(m_i_wavelengths != 0))
	{
		sine = (IMAGE_PTR_VEC)malloc(m_i_orientations*m_i_wavelengths*sizeof(IMAGE_PTR));
		cosine = (IMAGE_PTR_VEC)malloc(m_i_orientations*m_i_wavelengths*sizeof(IMAGE_PTR));
		reallevel = (IMAGE_PTR_VEC)malloc(m_i_orientations*m_i_kernels*sizeof(IMAGE_PTR));
		imaglevel = (IMAGE_PTR_VEC)malloc(m_i_orientations*m_i_kernels*sizeof(IMAGE_PTR));
		gaborlevel = (IMAGE_PTR_VEC)malloc(m_i_orientations*m_i_kernels*sizeof(IMAGE_PTR));
		//computation of kernels dc values
		double step, tempval;
		for( j = 0; j < m_i_orientations; j++ )
		{
			for( i = 0; i < m_i_wavelengths; i++ )
			{
				wx = 2*pi/m_wavelengths[i]*cos(m_orientations[j]);
				wy = 2*pi/m_wavelengths[i]*sin(m_orientations[j]);
				tempval = (2*fc*cos(2*wx)+2*fb*cos(wx)+fa)*(2*fc*cos(2*wy)+2*fb*cos(wy)+fa);
				m_dc_gain[j*m_i_wavelengths*m_i_scales + i*m_i_scales + 0] = tempval;
				for( m = 1; m < m_i_scales; m++)
				{
					step = pow(2.0f,m);
					tempval = m_dc_gain[j*m_i_wavelengths*m_i_scales + i*m_i_scales + m-1]*(2*fc*cos(2*step*wx)+2*fb*cos(step*wx)+fa)*(2*fc*cos(2*step*wy)+2*fb*cos(step*wy)+fa);
					m_dc_gain[j*m_i_wavelengths*m_i_scales + i*m_i_scales + m] = tempval;
				}
			}			
		}
	}
	
	
#ifdef USE_IPP
	input = ippiMalloc_32f_C1(m_i_colsext, m_i_linesext, &m_i_strideext);
	temp = ippiMalloc_32f_C1(m_i_colsext, m_i_linesext, &m_i_strideext); 
	for( i = 0; i < m_i_scales; i++ )
	{
		gausslevel[i] = ippiMalloc_32f_C1(m_i_colsext, m_i_linesext, &m_i_strideext); 
	}
	m_i_stridepix = m_i_strideext/4;
	
	if(m_i_orientations != 0)
	{
		for( i = 0; i < m_i_scales; i++ )
		{
			laplevel[i] = ippiMalloc_32f_C1(m_i_colsext, m_i_linesext, &m_i_strideext); 
		}
	}
	if((m_i_orientations != 0)&&(m_i_wavelengths != 0))
	{
		even = ippiMalloc_32f_C1(m_i_colsext, m_i_linesext, &m_i_strideext); 
		odd = ippiMalloc_32f_C1(m_i_colsext, m_i_linesext, &m_i_strideext); 
	
		for( i = 0; i < m_i_orientations*m_i_wavelengths; i++ )
		{
			sine[i] = ippiMalloc_32f_C1(m_i_colsext, m_i_linesext, &m_i_strideext); 
			cosine[i] = ippiMalloc_32f_C1(m_i_colsext, m_i_linesext, &m_i_strideext); 
		}
		for( i = 0; i < m_i_orientations*m_i_kernels; i++ )
		{
			reallevel[i] = ippiMalloc_32f_C1(m_i_colsext, m_i_linesext, &m_i_strideext); 
			imaglevel[i] = ippiMalloc_32f_C1(m_i_colsext, m_i_linesext, &m_i_strideext); 
			gaborlevel[i] = ippiMalloc_32f_C1(m_i_colsext, m_i_linesext, &m_i_strideext); 
		}
		
		for( j = 0; j < m_i_orientations; j++ )
		{
			for( i = 0; i < m_i_wavelengths; i++ )
			{
				wx = 2*pi/m_wavelengths[i]*cos(m_orientations[j]);
				wy = 2*pi/m_wavelengths[i]*sin(-m_orientations[j]);
				for( m = 0; m < m_i_linesext; m++)
				{
					for( n = 0; n < m_i_colsext; n++ )
					{
						angle = wx*(n+1) + wy*(m+1);
						cosine[j*m_i_wavelengths+i][m*m_i_stridepix+n] = (float)cos(angle);
						sine[j*m_i_wavelengths+i][m*m_i_stridepix+n] = (float)sin(angle);
					}
				}
			}
		}
	}

#else
	m_i_stridepix = m_i_colsext;
	m_i_strideext = m_i_colsext*4;

	input = (float*)malloc(m_i_linesext*m_i_colsext*sizeof(float)); 
	temp = (float*)malloc(m_i_linesext*m_i_colsext*sizeof(float));
	for( i = 0; i < m_i_scales; i++ )
	{
		gausslevel[i] = (float*)malloc(m_i_linesext*m_i_colsext*sizeof(float));		
	}
	if(m_i_orientations != 0)
	{
		for( i = 0; i < m_i_scales; i++ )
		{
			laplevel[i] = (float*)malloc(m_i_linesext*m_i_colsext*sizeof(float));		
		}
	}


	if((m_i_orientations != 0)&&(m_i_wavelengths != 0))
	{
		even = (float*)malloc(m_i_linesext*m_i_colsext*sizeof(float));
		odd = (float*)malloc(m_i_linesext*m_i_colsext*sizeof(float));
		for( j = 0; j < m_i_orientations; j++ )
		{
			for( i = 0; i < m_i_wavelengths; i++ )
			{
				sine[j*m_i_wavelengths+i] = (float*)malloc(m_i_linesext*m_i_colsext*sizeof(float));
				cosine[j*m_i_wavelengths+i] = (float*)malloc(m_i_linesext*m_i_colsext*sizeof(float));

				wx = 2*pi/m_wavelengths[i]*cos(m_orientations[j]);
				wy = 2*pi/m_wavelengths[i]*sin(m_orientations[j]);
				for( m = 0; m < m_i_linesext; m++)
				{
					for( n = 0; n < m_i_colsext; n++ )
					{
						angle = wx*(n+1) + wy*(m+1);
						cosine[j*m_i_wavelengths+i][m*m_i_colsext+n] = (float)cos(angle);
						sine[j*m_i_wavelengths+i][m*m_i_colsext+n] = (float)sin(angle);
					}
				}
			}
		}
		for( i = 0; i < m_i_orientations*m_i_kernels; i++ )
		{
			reallevel[i] = (float*)malloc(m_i_linesext*m_i_colsext*sizeof(float));
			imaglevel[i] = (float*)malloc(m_i_linesext*m_i_colsext*sizeof(float));
			gaborlevel[i] = (float*)malloc(m_i_linesext*m_i_colsext*sizeof(float));
		}
	}
#endif
	m_bAllocated = true;
	return true;
}

/*********************************************************************************
FreeResources():
	Deletes internal structures and memory space allocated by AllocateResources
	Careful: Depends on the parameters 'm_i_orientations', 'm_i_scales',
				'm_i_kernels', 'm_i_wavelengths'. Do not change this values
				after calling AllocateResources
**********************************************************************************/
bool AtrousGabor::FreeResources()
{
	int i;
	m_bAllocated = false;

	if(m_scale_wav_table != 0)
	{
		for(i = 0; i < m_i_scales; i++ )
			free(m_scale_wav_table[i]);
		free(m_scale_wav_table);
		m_scale_wav_table = 0;
	}
	if(m_scales != 0)
	{
		free(m_scales);
		m_scales = 0;
	}
	if(m_orientations != 0)
	{
		free(m_orientations);
		m_orientations = 0;
	}
	if(m_wavelengths != 0)
	{
		free(m_wavelengths);
		m_wavelengths = 0;
	}
	if(m_dc_gain != 0)
	{
		free(m_dc_gain);
		m_dc_gain = 0;
	}
#ifdef USE_IPP
	if(gausslevel != 0)
	{
		for(i = 0; i < m_i_scales; i++)
			ippiFree(gausslevel[i]);
		free(gausslevel);		
	}
	
	if(laplevel != 0)
	{
		for(i = 0; i < m_i_scales; i++)
			ippiFree(laplevel[i]);
		free(laplevel);		
	}

	if(reallevel != 0)
	{
		for(i = 0; i < m_i_orientations*m_i_kernels; i++)
			ippiFree(reallevel[i]);
		free(reallevel);		
	}

	if(imaglevel != 0)
	{
		for(i = 0; i < m_i_orientations*m_i_kernels; i++)
			ippiFree(imaglevel[i]);
		free(imaglevel);
	}

	if(gaborlevel != 0)
	{
		for(i = 0; i < m_i_orientations*m_i_kernels; i++)
			ippiFree(gaborlevel[i]);
		free(gaborlevel);
	}

	if(sine != 0)
	{
		for(i = 0; i < m_i_orientations*m_i_wavelengths; i++)
			ippiFree(sine[i]);
		free(sine);		
	}

	if(cosine != 0)
	{
		for(i = 0; i < m_i_orientations*m_i_wavelengths; i++)
			ippiFree(cosine[i]);
		free(cosine);		
	}

	if(input != 0) 
		ippiFree(input);
	
	if(even != 0) 
		ippiFree(even);
	
	if(odd != 0) 
		ippiFree(odd);
		
	if(temp != 0) 
		ippiFree(temp);	
#else
	if(gausslevel != 0)
	{
		for(i = 0; i < m_i_scales; i++)
			free(gausslevel[i]);
		free(gausslevel);		
	}
	
	if(laplevel != 0)
	{
		for(i = 0; i < m_i_scales; i++)
			free(laplevel[i]);
		free(laplevel);		
	}

	if(reallevel != 0)
	{
		for(i = 0; i < m_i_orientations*m_i_kernels; i++)
			free(reallevel[i]);
		free(reallevel);		
	}

	if(imaglevel != 0)
	{
		for(i = 0; i < m_i_orientations*m_i_kernels; i++)
			free(imaglevel[i]);
		free(imaglevel);
	}

	if(gaborlevel != 0)
	{
		for(i = 0; i < m_i_orientations*m_i_kernels; i++)
			free(gaborlevel[i]);
		free(gaborlevel);
	}

	if(sine != 0)
	{
		for(i = 0; i < m_i_orientations*m_i_wavelengths; i++)
			free(sine[i]);
		free(sine);		
	}

	if(cosine != 0)
	{
		for(i = 0; i < m_i_orientations*m_i_wavelengths; i++)
			free(cosine[i]);
		free(cosine);		
	}

	if(input != 0) 
		free(input);
	
	if(even != 0) 
		free(even);
	
	if(odd != 0) 
		free(odd);
		
	if(temp != 0) 
		free(temp);	
#endif
	gausslevel = 0;
	laplevel = 0;
	reallevel = 0;
	imaglevel = 0;
	gaborlevel = 0;
	sine = 0;
	cosine = 0;
	input = 0;
	even = 0;
	odd = 0;
	temp = 0;
	return true;
}

/*********************************************************************************
ProcessImage():
	Computes the response of the gabor filters (real part, imaginary part and energy) 
	for all configured scales, orientations and wavelengths. 
	As a by product also computes the gaussian and laplacial levels
**********************************************************************************/
bool AtrousGabor::ProcessImage(IMAGE_PTR in)
{	
	if(!IsAllocated())
		throw "Resources not allocated";

	int i,j,w,s,k;

	//copy image to internal buffer
	for( i = 0; i < m_i_lines; i++ )
		memcpy(input + (m_i_border+i)*m_i_stridepix + m_i_border, in + m_i_cols*i, m_i_cols*sizeof(float) );


	//extending boundaries
	//left and rigth boudaries
	for( i = m_i_border; i < m_i_lines + m_i_border; i++ )
	{
		for( j = 0; j < m_i_border; j++)
			input[i*m_i_stridepix+j] = input[i*m_i_stridepix + m_i_border];
		for( j = m_i_cols + m_i_border; j < m_i_colsext; j++)
			input[i*m_i_stridepix+j] = input[i*m_i_stridepix + m_i_cols + m_i_border - 1];
	}
	//top and bottom boudaries
	for( i = 0; i < m_i_border; i++ )
	{		
		memcpy(input + m_i_stridepix*i, input + m_i_stridepix*m_i_border, m_i_strideext);
	}
	for( i = m_i_lines + m_i_border; i < m_i_linesext; i++ )
	{		
		memcpy(input + m_i_stridepix*i, input + m_i_stridepix*(m_i_linesext-m_i_border-1), m_i_strideext);
	}

	//computing the gaussian levels
	_process_level(input, gausslevel[0], 1);
	for( i = 1; i < m_i_scales; i++ )
	{
		_process_level(gausslevel[i-1], gausslevel[i], i+1);
	}
	if( m_i_orientations > 0) //compute the laplacian levels
	{
		_image_sub(input, gausslevel[0],laplevel[0]);
		for( i = 1; i < m_i_scales; i++)
		{
			_image_sub(gausslevel[i-1],gausslevel[i],laplevel[i]);
		}
	}
	if( m_i_orientations == 0 )
		return true;
	if( m_i_wavelengths == 0 )
		return true;
	//computing the even levels
	k = 0;
	for( j = 0; j < m_i_orientations; j++ )
	{
		for(w = 0; w < m_i_wavelengths; w++)
		{
			//what are the first and last scales to compute
			int lastscale = 0;
			int firstscale = 0;
			for( s = 0; s < m_i_scales; s++)
				if(m_scale_wav_table[s][w])
					lastscale = s;
			for( s = m_i_scales-1; s >= 0; s--)
				if(m_scale_wav_table[s][w])
					firstscale = s;
			//computing the even image
			_image_prod(input,cosine[j*m_i_wavelengths+w],even);

	
			for(s = 0; s < firstscale; s++)
				_process_level(even,even,s+1);
			_process_level(even,reallevel[k++],s+1);
			
			for(s=firstscale+1;s<=lastscale;s++)
			{
				if(m_scale_wav_table[s][w])
				{
					if(m_scale_wav_table[s-1][w])
						_process_level(reallevel[k-1], reallevel[k], s+1);
					else
						_process_level(even, reallevel[k], s+1);
					k++;
				}
				else
				{
					if(m_scale_wav_table[s-1][w])
						_process_level(reallevel[k-1], even, s+1);
					else
						_process_level(even, even, s+1);
				}
			}
		}
	}

	//computing the odd levels
	k = 0;
	for( j = 0; j < m_i_orientations; j++ )
	{
		for(w = 0; w < m_i_wavelengths; w++)
		{
			//what are the first and last scales to compute
			int lastscale = 0;
			int firstscale = 0;
			for( s = 0; s < m_i_scales; s++)
				if(m_scale_wav_table[s][w])
					lastscale = s;
			for( s = m_i_scales-1; s >= 0; s--)
				if(m_scale_wav_table[s][w])
					firstscale = s;
			//computing the even image
			_image_prod(input,sine[j*m_i_wavelengths+w],odd);

	
			for(s = 0; s < firstscale; s++)
				_process_level(odd,odd,s+1);
			_process_level(odd,imaglevel[k++],s+1);
			
			for(s=firstscale+1;s<=lastscale;s++)
			{
				if(m_scale_wav_table[s][w])
				{
					if(m_scale_wav_table[s-1][w])
						_process_level(imaglevel[k-1], imaglevel[k], s+1);
					else
						_process_level(odd, imaglevel[k], s+1);
					k++;
				}
				else
				{
					if(m_scale_wav_table[s-1][w])
						_process_level(imaglevel[k-1], odd, s+1);
					else
						_process_level(odd, odd, s+1);
				}
			}
		}
	}

	//computing real, imag and gabor energy levels
	k = 0;
	//float realtemp, imagtemp, sinetemp, cosinetemp;
	for( j = 0; j < m_i_orientations; j++ )
	{
		for(w = 0; w < m_i_wavelengths; w++)
		{	
			for( s = 0;  s < m_i_scales; s++ )
			{
				if(m_scale_wav_table[s][w])
				{
					_image_prod(reallevel[k],cosine[j*m_i_wavelengths+w], even );
					_image_prod(imaglevel[k],sine[j*m_i_wavelengths+w], odd );	
					_image_prod(reallevel[k],sine[j*m_i_wavelengths+w], input );
					_image_prod(imaglevel[k],cosine[j*m_i_wavelengths+w], temp );

					_image_sub(input,temp,imaglevel[k]);
					_const_prod(gausslevel[w],temp,(float)m_dc_gain[j*m_i_wavelengths*m_i_scales + w*m_i_scales + s]);
					_image_add(even,odd,reallevel[k]);
					_image_sub(reallevel[k],temp,reallevel[k]);

					_image_prod(reallevel[k],reallevel[k],even);
					_image_prod(imaglevel[k],imaglevel[k],odd);
					_image_add(even,odd,temp);
					_image_sqrt(temp,gaborlevel[k]);

					k++;
				}
			}
		}
	}
	return true;
}


/*********************************************************************************
GetScales():
	Returns the scale values configured for this object	
**********************************************************************************/
void AtrousGabor::GetScales(double *scales)
{
	for(int i = 0; i < m_i_scales; i++)
		scales[i] = m_scales[i];
}
/*********************************************************************************
GetWavelengths():
	Returns the wavelenght values configured for this object	
**********************************************************************************/
void AtrousGabor::GetWavelengths(double *wavelengths)
{
	for(int i = 0; i < m_i_wavelengths; i++)
		wavelengths[i] = m_wavelengths[i];
}
/*********************************************************************************
GetOrientations():
	Returns the orientation values configured for this object	
**********************************************************************************/
void AtrousGabor::GetOrientations(double *orientations)
{
	for(int i = 0; i < m_i_orientations; i++)
		orientations[i] = 180.0/pi*m_orientations[i];
}

/*********************************************************************************
GetScaleWavTable():
	Returns the table with computable scale/wavelenght combinations
**********************************************************************************/
void AtrousGabor::GetScaleWavTable(bool *scalewavtable)
{
	for(int i = 0; i < m_i_scales; i++)
		for(int j = 0; j < m_i_wavelengths; j++)
			scalewavtable[i*m_i_wavelengths+j] = m_scale_wav_table[i][j];
}


IMAGE_PTR AtrousGabor::AccessGaussianLevel(int level)
{	
	if(!IsAllocated())
		throw "Resources not allocated";

	if( (level < 0) || (level >= m_i_scales))
		throw "Invalid Argument";

	return gausslevel[level] + m_i_border*m_i_stridepix + m_i_border;
}
IMAGE_PTR AtrousGabor::AccessLaplacianLevel(int level)
{
	if(!IsAllocated())
		throw "Resources not allocated";

	if(m_i_orientations == 0)
		throw "Object not configured for laplacian computations";

	if( (level < 0) || (level >= m_i_scales))
		throw "Invalid Argument";

	return laplevel[level] + m_i_border*m_i_stridepix + m_i_border;
}
IMAGE_PTR AtrousGabor::AccessGaborRealLevel(int level)
{
	if((m_i_orientations == 0)||(m_i_wavelengths == 0) )
		throw "Object not configured for gabor computations";

	if(!IsAllocated())
		throw "Resources not allocated";

	if( (level < 0) || (level >= m_i_orientations*m_i_kernels))
		throw "Invalid Argument";

	return reallevel[level]  + m_i_border*m_i_stridepix + m_i_border;
}
IMAGE_PTR AtrousGabor::AccessGaborImagLevel(int level)
{
	if((m_i_orientations == 0)||(m_i_wavelengths == 0) )
		throw "Object not configured for gabor computations";

	if(!IsAllocated())
		throw "Resources not allocated";

	if( (level < 0) || (level >= m_i_orientations*m_i_kernels))
		throw "Invalid Argument";

	return imaglevel[level]  + m_i_border*m_i_stridepix + m_i_border;
}
IMAGE_PTR AtrousGabor::AccessGaborEnergyLevel(int level)
{
	if((m_i_orientations == 0)||(m_i_wavelengths == 0) )
		throw "Object not configured for gabor computations";

	if(!IsAllocated())
		throw "Resources not allocated";

	if( (level < 0) || (level >= m_i_orientations*m_i_kernels))
		throw "Invalid Argument";

	return gaborlevel[level]  + m_i_border*m_i_stridepix + m_i_border;
}

IMAGE_PTR AtrousGabor::AccessGaussianBuffer(int level)
{	
	if(!IsAllocated())
		throw "Resources not allocated";

	if( (level < 0) || (level >= m_i_scales))
		throw "Invalid Argument";

	return gausslevel[level];
}
IMAGE_PTR AtrousGabor::AccessLaplacianBuffer(int level)
{
	if(!IsAllocated())
		throw "Resources not allocated";

	if(m_i_orientations == 0)
		throw "Object not configured for laplacian computations";

	if( (level < 0) || (level >= m_i_scales))
		throw "Invalid Argument";

	return laplevel[level];
}
IMAGE_PTR AtrousGabor::AccessGaborRealBuffer(int level)
{
	if((m_i_orientations == 0)||(m_i_wavelengths == 0) )
		throw "Object not configured for gabor computations";

	if(!IsAllocated())
		throw "Resources not allocated";

	if( (level < 0) || (level >= m_i_orientations*m_i_kernels))
		throw "Invalid Argument";

	return reallevel[level];
}
IMAGE_PTR AtrousGabor::AccessGaborImagBuffer(int level)
{
	if((m_i_orientations == 0)||(m_i_wavelengths == 0) )
		throw "Object not configured for gabor computations";

	if(!IsAllocated())
		throw "Resources not allocated";

	if( (level < 0) || (level >= m_i_orientations*m_i_kernels))
		throw "Invalid Argument";

	return imaglevel[level];
}
IMAGE_PTR AtrousGabor::AccessGaborEnergyBuffer(int level)
{
	if((m_i_orientations == 0)||(m_i_wavelengths == 0) )
		throw "Object not configured for gabor computations";

	if(!IsAllocated())
		throw "Resources not allocated";

	if( (level < 0) || (level >= m_i_orientations*m_i_kernels))
		throw "Invalid Argument";

	return gaborlevel[level];
}



void AtrousGabor::_process_level( IMAGE_PTR in, IMAGE_PTR out, int level) // can be used in place
{
	const int filtersize = 5;
	const int filtercenter = 2;
	int r,c; //loop counters

	int step = (int)pow(2.0f,level-1); //size of the increments between non zero filter coefs
	int dim = step*(filtersize-1)+1;  //dimension of the FIR filter at this scale
	int mid = dim/2;  //filter center

#ifdef USE_IPP
	int stride;
	IppiSize size = {m_i_colsext, m_i_linesext};
	IppiSize horsize = {m_i_colsext-2*mid, m_i_linesext};
	IppiSize versize = {m_i_colsext, m_i_linesext-2*mid};
	Ipp32f *filter = ippiMalloc_32f_C1(dim,1,&stride);
	ippiSet_32f_C1R(0.0f, temp, m_i_strideext, size);
	for( r = 0; r < dim; r++ )
		filter[r] = 0.0f;
	filter[0] = filter[dim-1] = (Ipp32f)fc;
	filter[step] = filter[dim-step-1] = (Ipp32f)fb;
	filter[2*step] = (Ipp32f)fa;
	ippiFilterRow_32f_C1R( in + mid, m_i_strideext, temp+mid, m_i_strideext, horsize, filter, dim, mid);
	ippiFilterColumn_32f_C1R( temp + mid*m_i_stridepix, m_i_strideext, 
		out + mid*m_i_stridepix, m_i_strideext, versize, filter, dim, mid);
	ippiFree(filter);
#else
	
	int height = m_i_linesext;
	int width = m_i_colsext;
	int ws = m_i_stridepix;
	//horizontal filtering
	for(r=0; r < height; r++ )
	{
		//filtering the left border
		for(c = 0; c < step; c++)
			temp[r*ws+c] = fc*(in[r*ws]+in[r*ws+c+2*step]) +
							  fb*(in[r*ws]+in[r*ws+c+step]) + 
							  fa*in[r*ws+c];
			
		for(c = step; c < 2*step; c++)
			temp[r*ws+c] = fc*(in[r*ws]+in[r*ws+c+2*step]) +
							  fb*(in[r*ws+c-step]+in[r*ws+c+step]) + 
							  fa*in[r*ws+c];

		//filtering the valid part
		for(c = 2*step; c < width - 2*step; c++)
			temp[r*ws+c] = fc*(in[r*ws+c-2*step]+in[r*ws+c+2*step]) +
							  fb*(in[r*ws+c-step]+in[r*ws+c+step]) + 
							  fa*in[r*ws+c];

		//filtering the right border
		for(c = width-2*step; c < width-step; c++)
			temp[r*ws+c] = fc*(in[r*ws+c-2*step]+in[r*ws+width-1]) +
							  fb*(in[r*ws+c-step]+in[r*ws+c+step]) + 
							  fa*in[r*ws+c];
			
		for(c = width-step; c < width; c++)
			temp[r*ws+c] = fc*(in[r*ws+c-2*step]+in[r*ws+width-1]) +
							  fb*(in[r*ws+c-step]+in[r*ws+width-1]) + 
							  fa*in[r*ws+c];
	}
	//vertical filtering
	for(c=0; c < width; c++ )
	{
		//filtering the top border
		for(r = 0; r < step; r++)
			out[r*ws+c] = fc*(temp[c]+temp[(r+2*step)*ws+c]) + 
							 fb*(temp[c]+temp[(r+step)*ws+c]) + 
							 fa*temp[r*ws+c];

		for(r = step; r < 2*step; r++)
			out[r*ws+c] = fc*(temp[c]+temp[(r+2*step)*ws+c]) + 
							 fb*(temp[(r-step)*ws+c]+temp[(r+step)*ws+c]) + 
							 fa*temp[r*ws+c];

		
		//filtering the valid part
		for(r = 2*step; r < height - 2*step; r++)
			out[r*ws+c] = fc*(temp[(r-2*step)*ws+c]+temp[(r+2*step)*ws+c]) + 
							 fb*(temp[(r-step)*ws+c]+temp[(r+step)*ws+c]) + 
							 fa*temp[r*ws+c];

		//filtering the right border
		for(r=height-2*step; r < height-step; r++)
			out[r*ws+c] = fc*(temp[(r-2*step)*ws+c]+temp[(height-1)*ws+c]) + 
							 fb*(temp[(r-step)*ws+c]+temp[(r+step)*ws+c]) + 
							 fa*temp[r*ws+c];

		for(r=height-step; r < height; r++)
			out[r*ws+c] = fc*(temp[(r-2*step)*ws+c]+temp[(height-1)*ws+c]) + 
							 fb*(temp[(r-step)*ws+c]+temp[(height-1)*ws+c]) + 
							 fa*temp[r*ws+c];
	}
#endif
	return;
}


void AtrousGabor::_image_prod(IMAGE_PTR i1, IMAGE_PTR i2, IMAGE_PTR out)
{
#ifdef USE_IPP		
	IppiSize roiSize = {m_i_colsext, m_i_linesext};
	ippiMul_32f_C1R(i1, m_i_strideext, i2, m_i_strideext, out, m_i_strideext, roiSize);
#else
	int i, j;
	for(i = 0; i < m_i_linesext; i++)
		for(j = 0; j < m_i_colsext; j++)
			out[i*m_i_stridepix+j] = i1[i*m_i_stridepix+j]*i2[i*m_i_stridepix+j];
#endif
}

void AtrousGabor::_image_add(IMAGE_PTR i1, IMAGE_PTR i2, IMAGE_PTR out)
{
#ifdef USE_IPP		
	IppiSize roiSize = {m_i_colsext, m_i_linesext};
	ippiAdd_32f_C1R(i1, m_i_strideext, i2, m_i_strideext, out, m_i_strideext, roiSize);
#else
	int i, j;
	for(i = 0; i < m_i_linesext; i++)
		for(j = 0; j < m_i_colsext; j++)
			out[i*m_i_stridepix+j] = i1[i*m_i_stridepix+j] + i2[i*m_i_stridepix+j];
#endif
}

void AtrousGabor::_image_sub(IMAGE_PTR i1, IMAGE_PTR i2, IMAGE_PTR out)
{
#ifdef USE_IPP		
	IppiSize roiSize = {m_i_colsext, m_i_linesext};
	ippiSub_32f_C1R(i2, m_i_strideext, i1, m_i_strideext, out, m_i_strideext, roiSize);
#else
	int i, j;
	for(i = 0; i < m_i_linesext; i++)
		for(j = 0; j < m_i_colsext; j++)
			out[i*m_i_stridepix+j] = i1[i*m_i_stridepix+j] - i2[i*m_i_stridepix+j];
#endif
}

void AtrousGabor::_const_prod(IMAGE_PTR in, IMAGE_PTR out, float gain)
{
#ifdef USE_IPP		
	IppiSize roiSize = {m_i_colsext, m_i_linesext};
	ippiMulC_32f_C1R(in, m_i_strideext, gain, out, m_i_strideext, roiSize);
#else
	int i, j;
	for(i = 0; i < m_i_linesext; i++)
		for(j = 0; j < m_i_colsext; j++)
			out[i*m_i_stridepix+j] = gain*in[i*m_i_stridepix+j];
#endif
}

void AtrousGabor::_image_sqrt(IMAGE_PTR in, IMAGE_PTR out)
{
#ifdef USE_IPP		
	IppiSize roiSize = {m_i_colsext, m_i_linesext};
	ippiSqrt_32f_C1R(in, m_i_strideext, out, m_i_strideext, roiSize);
#else
	int i, j;
	for(i = 0; i < m_i_linesext; i++)
		for(j = 0; j < m_i_colsext; j++)
			out[i*m_i_stridepix+j] = (float)sqrt(in[i*m_i_stridepix+j]);
#endif
}

