// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino (VisLab/ISR/IST)
 */

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

#include "CLogPolarSensor.h"

CLogPolarSensor::~CLogPolarSensor()
{
	destroy_logpolar_lut();
}

void CLogPolarSensor::destroy_logpolar_lut()
{
	if(m_num_pix_in_cell)
	{
		free(m_num_pix_in_cell);
		m_num_pix_in_cell = 0;
	}
	if(m_pix_index_array)
	{
		free(m_pix_index_array);
		m_pix_index_array = 0;
	}
	if(m_fovea_index_array)
	{
		free(m_fovea_index_array);
		m_fovea_index_array = 0;
	}
	m_bValidData = false;
}


// assume logpolar and imageplane parameters are appropriately initialized
// invoke compute_params_* functions and set base class properties
// 
void CLogPolarSensor::create_logpolar_lut()
{
	double xi, yi, xm, ym, xp, yp, cr, ca;
	int i,j; //general integer for-loop indexes
	int dr, da, dx, dy;
	tLPSListElem *ptr, *temp, *fovea_list = NULL;

	if(m_bValidData)
		destroy_logpolar_lut();

	// memory allocation
	m_num_pix_in_cell = (int*)calloc(m_iAngles*m_iEccentr, sizeof(int));
	if(m_num_pix_in_cell == NULL) {
		return;
	}

	m_pix_index_list = (tLPSListElem **)calloc( m_iAngles*m_iEccentr, sizeof(tLPSListElem*) );
	if( m_pix_index_list == NULL )	{
		return;
	}

	m_iNumPixels = 0;
	m_iFoveaPixels = 0;
	for( i = 0; i < m_iAngles*m_iEccentr; i++)		{
		m_num_pix_in_cell[i] = 0;
		m_pix_index_list[i] = NULL;
	}

	// filling luts
	for( i = 0 ; i < m_i_image_frame_lines; i++ )		{
		for( j = 0; j < m_i_image_frame_columns ; j++ )			{
			//measurements in the center of the pixel
			image_frame_to_pixel(j+0.5,i+0.5, &xm, &ym);
			pixel_to_image_plane(xm, ym, &xp, &yp);
			map_coordinates(xp, yp, &cr, &ca );
			dr = (int)cr;
			da = (int)ca;
			if(dr < 0)    //belongs to fovea
			{
				m_iFoveaPixels++;
				ptr = (tLPSListElem*)calloc(1,sizeof(tLPSListElem));
				if( ptr == NULL )	{	 //out of memory
					return;
				}
				ptr->cart_index = i*m_i_image_frame_columns + j;
				// insert at beginning of fovea list
				ptr->next = fovea_list;
				fovea_list = ptr;
			}

			if( (dr >= 0) && (dr < m_iEccentr))
			{
				m_iNumPixels++;
				m_num_pix_in_cell[ da*m_iEccentr + dr ] += 1;
				//ptr = (tLPSListElem*)calloc(1,sizeof(tLPSListElem));
                ptr = new tLPSListElem;

				if( ptr == NULL )	{	 //out of memory
					return;
				}
				ptr->cart_index = i*m_i_image_frame_columns + j;
				// insert at beginning of list
				ptr->next = m_pix_index_list[ da*m_iEccentr + dr];
				m_pix_index_list[da*m_iEccentr + dr] = ptr;
			}
		}
	}


	//dealing with empty cells
	//bool use_left, use_top; // flags for deciding interpolating pixels
	for( i = 0 ; i < m_iAngles ; i++ ) {
		for( j = 0 ; j < m_iEccentr ; j++ )  {
			if( m_num_pix_in_cell[ i*m_iEccentr + j ] == 0) {
				//compute the closest cartesian pixel
				cr = j+0.5f;
				ca = i+0.5f;
				invmap_coordinates( cr , ca , &xm , &ym );
				image_plane_to_pixel(xm, ym, &xp, &yp);
				pixel_to_image_frame(xp, yp, &xi, &yi);

				dx = (int)xi;
				dy = (int)yi;

				ptr=(tLPSListElem*)calloc(1,sizeof(tLPSListElem));
				if( ptr == NULL ) {  //out of memory
					throw "Memory allocation error";
				}
				ptr->cart_index = dy*m_i_image_frame_columns + dx;
				ptr->next = m_pix_index_list[ i*m_iEccentr + j ];
				m_pix_index_list[ i*m_iEccentr + j] = ptr;
				m_iNumPixels++;
			}
		}
	}
	// constructing sparse matrices
	m_pix_index_array = (int*)calloc(m_iNumPixels, sizeof(int));
	if(m_pix_index_array == NULL) {
		throw "Memory allocation error";
	}

	int counter = 0;
	for(i = 0; i < m_iAngles; i++) {
		for(j = 0; j < m_iEccentr; j++) {
			ptr = m_pix_index_list[ i*m_iEccentr + j ];
			while(ptr != NULL) {
				m_pix_index_array[counter++] = ptr->cart_index;
				temp = ptr;
				ptr = ptr->next;
				free(temp);
			}
		}
	}
	free(m_pix_index_list);

	m_fovea_index_array = (int*)calloc(m_iFoveaPixels, sizeof(int));
	if(m_fovea_index_array == NULL) {
		throw "Memory allocation error";
	}

	ptr = fovea_list;
	for( i = 0; i < m_iFoveaPixels; i++ )
	{
		m_fovea_index_array[i] = ptr->cart_index;
		temp = ptr;
		ptr = ptr->next;
		free(temp);
	}
	m_bValidData = true;
	imageplane_has_changed = false;
	logpolar_has_changed = false;
}

int CLogPolarSensor::get_fovea_pix()
{
	return m_iFoveaPixels;
}
