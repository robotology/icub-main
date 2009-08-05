// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino (VisLab/ISR/IST)
 */

#ifndef __CIMAGEPLANEPARAMS_H_
#define __CIMAGEPLANEPARAMS_H_

class CImagePlaneParams 
{
protected:
	int  m_i_image_frame_columns;
	int  m_i_image_frame_lines;
	double m_f_image_plane_width;
	double m_f_image_plane_height;	
	double m_f_pixel_x_size;
	double m_f_pixel_y_size;	
	double m_f_principal_point_x;
	double m_f_principal_point_y;
	double m_f_skew;
	bool imageplane_has_changed;
public:
	CImagePlaneParams()
		: m_f_pixel_x_size(1.0f)					//default
		, m_f_pixel_y_size(1.0f)					//default
		, m_f_image_plane_width(128.0f)				//default
		, m_f_image_plane_height(128.0f)			//default
		, m_i_image_frame_lines(128)				//default
		, m_i_image_frame_columns(128)				//default
		, m_f_principal_point_x(0.0f)				//default
		, m_f_principal_point_y(0.0f)				//default
		, m_f_skew(0.0f)							//default
		, imageplane_has_changed(false)
	{		
	}

	virtual ~CImagePlaneParams()
	{
	}

	void set_image_plane_params(int lines, int cols, double x0 = 0.0f, double y0 = 0.0f, double skew = 0.0f, double width = 0.0f , double height = 0.0f  );
	
	int get_image_frame_lines();
	
	void put_image_frame_lines(int newVal);
	
	int get_image_frame_columns();
	
	void put_image_frame_columns(int newVal);
	
	double get_image_plane_width();
	
	void put_image_plane_width(double newVal);
	
	double get_image_plane_height();
	
	void put_image_plane_height(double newVal);
	
	////////////////////////////////////////////////
	////////////////////////////////////////////////
	// Pixel Size parameters cannot be user changed
	// They are determined by image plane and image array sizes

	double get_pixel_x_size();
	
	double get_pixel_y_size();
	
	double get_principal_point_x();
	
	void put_principal_point_x(double newVal);
	
	double get_principal_point_y();
	
	void put_principal_point_y(double newVal);
	
	double get_skew();
	
	void put_skew(double newVal);
	

	///////////////////////////////////////////////////////////////
	/// Operations

	void image_frame_to_pixel(double col, double lin, double *px, double *py);
	
	void pixel_to_image_frame(double px, double py, double *col, double *lin);


	void image_plane_to_pixel(double xi, double yi, double *px, double *py);
	
	void pixel_to_image_plane(double px, double py, double *xi, double *yi);
	
	void metric_projection_to_image_plane(double x, double y, double *xi, double *yi);
	
	void image_plane_to_metric_projection(double xi, double yi, double *x, double *y);
};

#endif //__CIMAGEPLANE_H_
