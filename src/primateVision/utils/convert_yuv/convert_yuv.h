/**
 * @ingroup icub_primatevision_utils
 *
 * \defgroup icub_primatevision_utils_convert_yuv Convert_YUV
 *
 * 
 * A class that processes input images in the YUV422 format (typically the low-latency format native to cameras), and converts them to separate R, G, and B, and RGBA images, and to a RGB QImage.
 *
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at \in src/primateVision/utils/convert_yuv/convert_yuv.h
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#ifndef CONVERT_YUV_H
#define CONVERT_YUV_H

#include <ipp.h>
#include <libmmx.h>
#include <qimage.h>

using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {
      
      /** A class that processes input images in the YUV422 format and converts them to separate R, G, and B, an RGBA image, and a colour QImage. */
      class Convert_YUV
      {
	
      public:

	/** Constructor.
	 * @param imsize Input image width and height for memory allocation.
	 */
	Convert_YUV(IppiSize imsize);

	/** Destructor.
	 */
	~Convert_YUV();

	/** Processing initiator.
	 * @param yuyv_orig Pointer to 4-channel YUV422 input image.
	 * @param psb4 Step in bytes through the input image.
	 */
	void proc(Ipp8u* yuyv_orig);

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

	/** Access to a RGBA 4-channel image.
	 * @return Pointer to the RGBA 4-channel image.
	 */
	Ipp8u* get_rgba(){return qrgb->bits();}

	/** Memory width return function.
	 * @return Step in bytes through the 8u single channel output image.
	 */
	int get_psb(){return psb;}

	/** Memory width return function.
	 * @return Step in bytes through the 8u 4-channel output image.
	 */
	int get_psb4(){return psb_4;}

	/** Access to a 4-channel colour image in the QImage format.
	 * @return Pointer to the colour image in QImage format.
	 */
	QImage* get_qrgb(){return qrgb;}
	
      private:
	Ipp8u *yuva_orig,*uv_orig,*r_orig,*g_orig,*b_orig,*a_orig;
	Ipp8u *y_orig,*u_orig,*v_orig;
	QImage *qrgb;
	IppiSize srcsize;
	int width,height;
	Ipp8u** pyuv;
	Ipp8u** prgb;
	int psb,psb_4;
	const Ipp8u** psrc;
      };

    }
  }
}
#endif
