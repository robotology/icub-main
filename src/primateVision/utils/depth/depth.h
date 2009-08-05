/**
 * @ingroup icub_primatevision_utils
 *
 * \defgroup icub_primatevision_utils_depth Depth
 *
 * 
 * A class that processes input Y channel epipolar rectified stereo images, and produces an image-frame disparity map. The current version uses the StereoBM implementation in libCV (ver>1.1pre). It is implemented to operate on IPP images as used in the PrimateVision framework, and produce compatible output.
 *
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at \in src/primateVision/utils/depth/depth.h
 */

/*
 * Copyright (C) 2003-2009 Andrew Dankers. All rights reserved.
 * 
 */


#ifndef __DEPTH_H
#define __DEPTH_H

#include <ipp.h>
#include <cvaux.h>
#include <cv.h>

using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {


     /** 
       *  A class that processes input Y channel epipolar rectified stereo images, and produces an image-frame disparity map.
       */
      class Depth
      {
	
      public:

	/** Constructor.
	 * @param imsize Input image width and height for memory allocation.
	 * @param filterSize CV BM algorithm filterSize parameter.
	 * @param filterCap  CV BM algorithm filterCap parameter.
	 * @param windowSize CV BM algorithm windowSize parameter.
	 * @param minDisparity CV BM algorithm minDisparity parameter.
	 * @param numDisparities CV BM algorithm numDisparities parameter.
	 * @param threshold CV BM algorithm threshold parameter.
	 * @param uniqueness CV BM algorithm uniqueness parameter.
	 */
	Depth(IppiSize imsize,
	      int filterSize,
	      int filterCap,
	      int windowSize,
	      int minDisparity,
	      int numDisparities,
	      int threshold,
	      int uniqueness
	      );

	/** Destructor.
	 */
	~Depth();

	/** Processing initiator.
	 * @param im_l Pointer to left input image.
	 * @param im_r Pointer to right input image.
	 * @param psb_8u Step in bytes through the input image.
	 */
	void proc(Ipp8u*im_l,Ipp8u*im_r,int psb_8u);
	
	/** Access to the disparity output map.
	 * @return Pointer to the disparity map where entries range from 0 to numdisparities (minDisparity represented as 0).
	 */
	Ipp8u* get_disp(){return disp_ret;}

	/** Memory width return function.
	 * @return Step in bytes through the output image.
	 */
	int get_psb(){return psb_o;}
	
	
      private:
	void calc_disp();
	
	//memory:
	Ipp8u *disp_ret;
	IplImage *imgLeft,*imgRight;
	CvMat *disp,*vdisp;
	
	//params:
	CvStereoBMState* state;
	int numDisparities;
	
	//image sizes:
	IppiSize fsize;
	
	//image memwidths:
	int psb_o;
	
      };
      
    }
  }
}
#endif
