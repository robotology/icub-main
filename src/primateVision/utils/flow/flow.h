/**
 * @ingroup icub_primatevision_utils
 *
 * \defgroup icub_primatevision_utils_flow Flow
 *
 * 
 * A class that processes input images and head status parameters to produce mosaic-frame optical flow output maps for flow in the x, and y directions. The current version uses a custom MMX SAD implementation situates in the SpTmp_MMX library. It is implemented to operate on IPP images as used in the PrimateVision framework, and produce compatible output.
 *
 *
 * If the head parameters are set to zero, image-frame optical flow is obtained. If the head parameters are provided, the class removes the effect of deliberate camera motions in the optical flow calculation, yielding output maps that quantify real scene flow only.  
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at \in src/primateVision/utils/flow/flow.h
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef __FLOW_H
#define __FLOW_H

#include <ipp.h>
#include <libmmx.h>
#include <sptmpmmx.h>


using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {
      
      /** 
       *  A class that processes input images and head status parameters to produce mosaic-frame optical flow output maps for flow in the x, and y directions.
       */
      
      class Flow
      {
      public:

	/** Constructor.
	 * @param imsize Input image width and height for memory allocation.
	 * @param flow_scale IMage downsampling for processing speed-up.
	 */
	Flow(IppiSize imsize, int flow_scale);

	/** Destructor.
	 */
	~Flow();

	/** Processing initiator.
	 * @param im Pointer to input image.
	 * @param psb_8u Step in bytes through the input image.
	 * @param newx Horizontal mosaic position of image.
	 * @param newy Vertical mosaic position of image.
	 */
	void proc(Ipp8u* im,int psb_8u,int newx,int newy);

	/** Access to flow output, x-components.
	 * @return Flow x-components map.
	 */
	Ipp8u* get_fx(){return fx;}
	
	/** Access to flow output, y-components.
	 * @return Flow y-components map.
	 */
	Ipp8u* get_fy(){return fy;}

	/** Memory width return function.
	 * @return Step in bytes through the output images.
	 */
	int get_psb(){return psb;}
	
      private:
	int width,height;
	int swidth,sheight;
	int flow_scale;
	int oldx,oldy;
	int newx,newy;
	
	Ipp8u *new_shrunk,*old_shrunk;
	Ipp8u *new_shrunk_c,*old_shrunk_c;
	Ipp8u *old_im,*new_im;
	Ipp8u *flow;
	Ipp8u *fx;
	Ipp8u *fy;
	Ipp8u *flow_x,*flow_y;
	
	MMXMatrix *old_m;
	MMXMatrix *new_m;
	MMXMatrix *flow_m;
	
	IppiRect froi;
	IppiRect sroi;
	IppiSize fullsize;
	IppiSize scalesize;
	IppiSize scalesize_b;
	IppiSize rsize;
	
	IppiRect new_roi;
	IppiRect old_roi;
	
	int dx,dy;
  
	int psb_b,psb,psb_s;	
      };

    }
  }
}
#endif
