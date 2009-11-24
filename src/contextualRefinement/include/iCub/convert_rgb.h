/**
 * @ingroup icub_primatevision_utils
 *
 * \defgroup icub_primatevision_utils_convert_rgb Convert_RGB
 *
 * 
 * A class that processes input RGBA images, and converts them to separate Y, U, and V, and YUVA images, and to a RGB QImage.
 *
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at \in src/primateVision/utils/convert_rgb/convert_rgb.h
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#ifndef CONVERT_RGB_H
#define CONVERT_RGB_H

#include <ipp.h>
//#include <qimage.h>

//using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {

     /** 
       *  A class that processes input RGBA images, and converts them to separate Y, U, and V, and YUVA images, and to a RGB QImage.
       */
      class Convert_RGB
      {
	
      public:

	/** Constructor.
	 * @param imsize Input image width and height for memory allocation.
	 */
	Convert_RGB(IppiSize imsize);

	/** Destructor.
	 */
	~Convert_RGB();

	/** Processing initiator.
	 * @param rgba_orig Pointer to 4-channel RGBA input image.
	 * @param psb4 Step in bytes through the input image.
	 */
	void proc(Ipp8u *rgba_orig,int psb4);

	/** Memory width return function.
	 * @return Step in bytes through the 8u single channel output image.
	 */
	int get_psb(){return psb;}

	/** Memory width return function.
	 * @return Step in bytes through the 8u 4-channel output image.
	 */
	int get_psb4(){return psb_4;}

	/** Access to the resulting Y intensity channel.
	 * @return Pointer to the planar Y channel.
	 */
	Ipp8u* get_y(){return y_orig;}
	
	/** Access to the resulting U chrominance channel.
	 * @return Pointer to the planar U channel.
	 */
	Ipp8u* get_u(){return u_orig;}

	/** Access to the resulting V chrominance channel.
	 * @return Pointer to the planar V channel.
	 */
	Ipp8u* get_v(){return v_orig;}

	/** Access to a YUVA 4-channel image.
	 * @return Pointer to the YUVA 4-channel image.
	 */
	Ipp8u* get_yuva(){return yuva_orig;}

	/** Access to a 4-channel colour image in the QImage format.
	 * @return Pointer to the colour image in QImage format.
	 */
//	QImage* get_qrgb(){return qrgb;}
	
	
      private:
	Ipp8u *yuva_orig;
	Ipp8u *y_orig,*u_orig,*v_orig,*tmp;
	//QImage *qrgb;
	IppiSize srcsize;
	int width,height;
	int psb;
	int psb_4;
	Ipp8u** pyuva;
      };

    }
  }
}
#endif
