/**
 * @ingroup icub_primatevision_utils
 *
 * \defgroup icub_primatevision_utils_canny Canny
 *
 * 
 * A processing class interface to the IPP MMX/SSE optimised implementation of the Canny edge detector.
 *
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at \in src/primateVision/utils/canny/canny.cpp
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef __CANNY_H
#define __CANNY_H

#include <ipp.h>

using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {


     /** 
       * A processing class that performs Canny edge detection.
       */
      class Canny
      {
      public:

	/** Constructor.
	 * @param imsize Input image width and height for memory allocation.
	 */
	Canny(IppiSize imsize);

	/** Destructor.
	 */
	~Canny();

	/** Processing initiator.
	 * @param im Pointer to input image.
	 * @param psb_in Step in bytes through the input image.
	 * @param TL Lower Canny hysteresis threshold.
	 * @param TU Upper Canny hysteresis threshold.
	 */
	void proc(Ipp8u* im,int psb_in,double TL,double TU);

	/** Output access function.
	 * @return Pointer to the resulting Canny edge map.
	 */
	Ipp8u* get_edgemap(){return edge_map;}

	/** Memory width return function.
	 * @return Step in bytes through the output image.
	 */
	int get_psb(){return psb;}
	

      private:
	int psb_i;
	int psb;
	int psb_s;
	Ipp8u* new_im; /**< Pointer to input image. */
	Ipp16s* pSDx; /**< Pointer to internally calculated horizontal Sobel map. */
	Ipp16s* pSDy; /**< Pointer to internally calculated vertical Sobel map. */
	int bufsize; /**< Internal IPP processing memory. */
	Ipp8u *buf_h,*buf_v,*pbuf;
	IppiSize isize;/**< Image width and height */
	Ipp8u *edge_map;/**< Pointer to output edge map. */
	
      };
      
    }
  }
}
#endif
