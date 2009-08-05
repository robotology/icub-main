#ifndef __CLOGPOLARSENSOR_H_
#define __CLOGPOLARSENSOR_H_

///
///		Template based class for logpolar mapping with generic types.
/// 
///		The logpolar image is an array each line corresponds to a radial scan and each column is a angular scan.
///     The last lines are filled with the pixels from the fovea, sequentially.
///			
///     The logpolar image creation is made with a look-up-table (LUT) 
///		that associates cartesian pixels to logpolar cells. 
///     
///		Before invoking the mapping functions, the LUT must be created.
///

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino (VisLab/ISR/IST)
 */

#include "CLogPolarParams.h"


#include "CImagePlaneParams.h"

class CLogPolarSensor : public CLogPolarParams, public CImagePlaneParams
{
protected:

	typedef struct list_elem
	{
		 int	cart_index;			//index pixel in cartesian image
		 struct list_elem *next;
	}
	tLPSListElem;
	tLPSListElem **m_pix_index_list; //temporary structure to build m_pix_index_array; dynamically built

	bool m_bValidData;
	int  m_iFoveaPixels;
//	float pi;
	int *m_num_pix_in_cell;	  //accumulator - number of cartesian pixel falling in log-polar cell
	int  m_iNumPixels;        //total number of used cartesian pixels
	int *m_pix_index_array;   //array of indexes of cartesian pixels corresponding to log-polar cell
							  //it is organized sequentially. Size is m_iNumPixels
	int *m_fovea_index_array; //array of indexes of cartesian pixels corresponding to fovea pixels
							  //it is organized sequentially. Size is m_iFoveaPixels
	

public:
	CLogPolarSensor()
		: //pi(3.14159265358979323846f),
		 m_iFoveaPixels(0)
		, m_bValidData(false)
		, m_iNumPixels(0)
		, m_num_pix_in_cell(0)
		, m_pix_index_array(0)
		, m_fovea_index_array(0)
	{
	}

	virtual ~CLogPolarSensor();
	

	void destroy_logpolar_lut();
	
	// assume logpolar and imageplane parameters are appropriately initialized
	// invoke compute_params_* functions and set base class properties
	// 
	void create_logpolar_lut();
	
	int get_fovea_pix();
	
	///
    /// Template functions to map images of arbitrary types
    /// USE WITH CARE: These functions do not check the validity of the arguments.
    /// There is the danger of memory space violations.
    /// 

    template <class inType, class outType>
    void _logmap(inType *cart, outType *log, bool bMapFovea)
    {
	    if(!m_bValidData)
		    throw "Object not created. Requires allocation";
	    if(imageplane_has_changed || logpolar_has_changed)
		    throw "Object has changed. Requires reallocation";
	    int i, j, k, index, np, counter = 0;
	    float temp;
	    for(i = 0; i < m_iAngles; i++) {
		    for( j = 0; j < m_iEccentr; j++) {
			    np = m_num_pix_in_cell[ i*m_iEccentr + j ];
			    if(np == 0)
				    np = 1;
			    temp = 0.0f;
			    for(k = 0; k < np; k++) {
				    index = m_pix_index_array[ counter++ ];
				    temp += cart[ index ];
			    }
			    log[ i*m_iEccentr + j ] = (outType)(temp/np);
		    }
	    }

	    if( bMapFovea )
	    {
		    int sz = m_iAngles*m_iEccentr;
		    for(i = 0; i < m_iFoveaPixels; i++ )
		    {
			    index = m_fovea_index_array[i];
			    log[sz+i] = (outType)(cart[index]);
		    }
	    }
    }

    template <class inType, class outType>
    void _invmap(inType *log, outType *cart, bool bMapFovea)
    {
	    if(!m_bValidData)
		    throw "Object not created. Requires allocation";
	    if(imageplane_has_changed || logpolar_has_changed)
		    throw "Object has changed. Requires reallocation";
	    int i, j, k, index, np, counter=0;
	    for(i = 0; i < m_iAngles; i++) {
		    for(j = 0; j < m_iEccentr; j++) {
			    np = m_num_pix_in_cell[ i*m_iEccentr + j ];
			    if(np == 0)
				    np = 1;
			    for(k = 0; k < np; k++) {
				    index = m_pix_index_array[ counter++ ];
				    cart[ index ] = (outType)log[ i*m_iEccentr + j ];
			    }
		    }
	    }
	    if( bMapFovea )
	    {
		    int sz = m_iAngles*m_iEccentr;
		    for( i = 0; i < m_iFoveaPixels; i++ )
		    {
			    index = m_fovea_index_array[i];
			    cart[index] = (outType)(log[sz+i]);
		    }
	    }
    }

    //OVERLOADING FOR INTERLACED COLOR IMAGES
    template <class inType, class outType>
    void _logmap(inType *cart, outType *log, int nChannels, bool bMapFovea)
    {
	    if(!m_bValidData)
		    throw "Object not created. Requires allocation";
	    if(imageplane_has_changed || logpolar_has_changed)
		    throw "Object has changed. Requires reallocation";
	    int i, j, k, c, index, np, counter = 0;
	    float temp[10];
	    if(nChannels > 10)
		    throw "Invalid Argument";
	    for(i = 0; i < m_iAngles; i++) {
		    for(j = 0; j < m_iEccentr; j++) {
			    np = m_num_pix_in_cell[ i*m_iEccentr + j ];
			    if(np == 0)
				    np = 1;
			    for(c = 0; c < nChannels; c++) {
				    temp[c] = 0.0;
			    }
			    for(k = 0; k < np; k++) {
				    index = m_pix_index_array[ counter++ ];
				    for(c = 0; c < nChannels; c++) {
					    temp[c] += cart[ index*nChannels + c ];
				    }
			    }
			    for(c = 0; c < nChannels; c++) {
				    log[ (i*m_iEccentr+j)*nChannels + c ] = (outType)(temp[c]/np);
			    }
		    }
	    }

	    if( bMapFovea )
	    {
		    int sz = m_iAngles*m_iEccentr;
		    for(i = 0; i < m_iFoveaPixels; i++ )
		    {
			    index = m_fovea_index_array[i];
			    //fov[i] = cart[index];
			    for(c = 0; c < nChannels; c++) {
				    log[(sz+i)*nChannels+c] = cart[ index*nChannels + c ];
			    }
		    }
	    }
    }

    template <class inType, class outType>
    void _invmap(inType *log, outType *cart, int nChannels, bool bMapFovea)
    {
	    if(!m_bValidData)
		    throw "Object not created. Requires allocation";
	    if(imageplane_has_changed || logpolar_has_changed)
		    throw "Object has changed. Requires reallocation";
	    int i, j, k, c, index, np, counter = 0;
	    for(i = 0; i < m_iAngles; i++) {
		    for(j = 0; j < m_iEccentr; j++) {
			    np = m_num_pix_in_cell[ i*m_iEccentr + j ];
			    if(np == 0)
				    np = 1;
			    for(k = 0; k < np; k++) {
				    index = m_pix_index_array[ counter++ ];
				    for(c = 0; c < nChannels; c++) {
					    cart[ index*nChannels + c ] = (outType)log[ (i*m_iEccentr+j)*nChannels + c ];
				    }
			    }
		    }
	    }

	    if( bMapFovea )
	    {
		    int sz = m_iAngles*m_iEccentr;
		    for( i = 0; i < m_iFoveaPixels; i++ )
		    {
			    index = m_fovea_index_array[i];
			    for(c = 0; c < nChannels; c++) {
				    cart[index*nChannels + c] = (outType)(log[(sz+i)*nChannels+c]);
			    }
		    }
	    }
    }
};



#endif /* __CLOGPOLARSENSOR_H_ */
