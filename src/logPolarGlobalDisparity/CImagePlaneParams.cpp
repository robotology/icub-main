// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino (VisLab/ISR/IST)
 */

#include "CImagePlaneParams.h"


void CImagePlaneParams::set_image_plane_params(int lines, int cols, double x0, double y0, double skew, double width, double height )
{
	put_image_frame_lines(lines);
	put_image_frame_columns(cols);
	if(width == 0.0f)
		put_image_plane_width((double)cols);
	else
		put_image_plane_width(width);
	
	if(height == 0.0f)
		put_image_plane_height((double)lines);
	else
		put_image_plane_height(height);
	
	put_principal_point_x(x0);
	put_principal_point_y(y0);
	put_skew(skew);
}

int CImagePlaneParams::get_image_frame_lines()
{
	return m_i_image_frame_lines;
}

void CImagePlaneParams::put_image_frame_lines(int newVal)
{
	if( newVal <= 0) 
		throw "Invalid Argument";
	if( m_i_image_frame_lines != newVal )
	{
		m_i_image_frame_lines = newVal;
		//update pixel size dimension
		m_f_pixel_y_size = m_f_image_plane_height/(double)m_i_image_frame_lines;
		imageplane_has_changed = true;
	}
}

int CImagePlaneParams::get_image_frame_columns()
{
	return m_i_image_frame_columns;
}

void CImagePlaneParams::put_image_frame_columns(int newVal)
{
	if( newVal <= 0)
		throw "Invalid Argument";
	if(m_i_image_frame_columns != newVal)
	{
		m_i_image_frame_columns = newVal;
		//update pixel size dimension
		m_f_pixel_x_size = m_f_image_plane_width/(double)m_i_image_frame_columns;
		imageplane_has_changed = true;
	}
}

double CImagePlaneParams::get_image_plane_width()
{
	return m_f_image_plane_width;
}

void CImagePlaneParams::put_image_plane_width(double newVal)  
{
	if( newVal <= 0.0f )
		throw "Invalid Argument";

	if(m_f_image_plane_width != newVal)
	{
		m_f_image_plane_width = newVal;
		//update pixel size dimension
		m_f_pixel_x_size = m_f_image_plane_width/(double)m_i_image_frame_columns;
		imageplane_has_changed = true;
	}
}

double CImagePlaneParams::get_image_plane_height()
{
	return m_f_image_plane_height;
}

void CImagePlaneParams::put_image_plane_height(double newVal)  
{
	if( newVal <= 0.0f )
		throw "Invalid Argument";
	if(m_f_image_plane_height != newVal)
	{
		m_f_image_plane_height = newVal;
		//update pixel size dimension
		m_f_pixel_y_size = m_f_image_plane_height/(double)m_i_image_frame_lines;
		imageplane_has_changed = true;
	}
}


////////////////////////////////////////////////
////////////////////////////////////////////////
// Pixel Size parameters cannot be user changed
// They are determined by image plane and image array sizes

double CImagePlaneParams::get_pixel_x_size()
{
	return m_f_pixel_x_size;
}

double CImagePlaneParams::get_pixel_y_size()
{
	return m_f_pixel_y_size;
}

double CImagePlaneParams::get_principal_point_x()
{
	return m_f_principal_point_x;
}

void CImagePlaneParams::put_principal_point_x(double newVal)
{
	if(m_f_principal_point_x != newVal)
	{
		m_f_principal_point_x = newVal;
		imageplane_has_changed = true;
	}
}

double CImagePlaneParams::get_principal_point_y()
{
	return m_f_principal_point_y;
}

void CImagePlaneParams::put_principal_point_y(double newVal)
{
	if(m_f_principal_point_y != newVal)
	{
		m_f_principal_point_y = newVal;
		imageplane_has_changed = true;
	}
}

double CImagePlaneParams::get_skew()
{
	return m_f_skew;
}

void CImagePlaneParams::put_skew(double newVal)
{
	if(m_f_skew != newVal)
	{
		m_f_skew = newVal;
		imageplane_has_changed = true;
	}
}


///////////////////////////////////////////////////////////////
/// Operations

void CImagePlaneParams::image_frame_to_pixel(double col, double lin, double *px, double *py)
{		
	// center and invert y axis
	
	*px = col - m_i_image_frame_columns/2.0f;
	*py = m_i_image_frame_lines/2.0f - lin;
}

void CImagePlaneParams::pixel_to_image_frame(double px, double py, double *col, double *lin)
{		
	*col = px + m_i_image_frame_columns/2.0f;
	*lin = m_i_image_frame_lines/2.0f - py;
}


void CImagePlaneParams::image_plane_to_pixel(double xi, double yi, double *px, double *py)
{	
	*px = xi/m_f_pixel_x_size;
	*py = yi/m_f_pixel_y_size;
}

void CImagePlaneParams::pixel_to_image_plane(double px, double py, double *xi, double *yi)
{			
	*xi = m_f_pixel_x_size*px;
	*yi = m_f_pixel_y_size*py;		
}

void CImagePlaneParams::metric_projection_to_image_plane(double x, double y, double *xi, double *yi)
{				
	*xi = x + m_f_principal_point_x + m_f_skew*y;
	*yi = y + m_f_principal_point_y;
}

void CImagePlaneParams::image_plane_to_metric_projection(double xi, double yi, double *x, double *y)
{	
	*y = yi - m_f_principal_point_y;
	*x = xi + m_f_principal_point_x - m_f_skew*(*y);
}
