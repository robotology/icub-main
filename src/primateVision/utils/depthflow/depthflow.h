/**
 * @ingroup icub_primatevision_utils
 *
 * \defgroup icub_primatevision_utils_depthflow Depthflow
 *
 * 
 * A class that processes input Y channel epipolar rectified stereo images, and produces an image-frame disparity map, and uses head status parameters to produce a mosaic-frame disparity map, and an absolute depth map. Conseccutive depth maps are analysed to obtain a depthflow map, that is, the flow of visual surfaces in the depth direction. It is implemented to operate on IPP images as used in the PrimateVision framework, and produce compatible output.
 *
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at \in src/primateVision/utils/depthflow/depthflow.h
 */




/*
 * Copyright (C) 2003-2009 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef __DEPTHFLOW___H
#define __DEPTHFLOW___H


#include <depth.h>




using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {


     /** 
       * A convenient storage structure for 3D points.
       */
      struct point3D{
	float x;
	float y;
	float z;
      };


      /**A class that produces an image-frame disparity map, a mosaic-frame disparity map, and an absolute depth map. Conseccutive depth maps are analysed to obtain a depthflow map. */
      class Depthflow
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
	 * @param baseline Stereo head baseline length (m).
	 * @param focus Camera focus length (pixels).
	 */
	Depthflow(IppiSize imsize,
		  int filterSize,
		  int filterCap,
		  int windowSize,
		  int minDisparity,
		  int numDisparities,
		  int threshold,
		  int uniqueness,
		  double baseline,
		  int focus);
	
	/** Destructor.
	 */
	~Depthflow();

	/** Processing initiator, accepting 8u images.
	 * @param im_l Pointer to left input image.
	 * @param im_r Pointer to right input image.
	 * @param psb_8u Step in bytes through the input image.
	 * @param ixl Horizontal mosaic position of left image.
	 * @param iyl Vartical mosaic position of left image.
	 * @param ixr Horizontal mosaic position of right image.
	 * @param iyr Vartical mosaic position of right image.
	 */
	void proc(Ipp8u*im_l,Ipp8u*im_r,int psb_8u,int ixl,int iyl,int ixr,int iyr);

	/** Access to the output disparity map.
	 * @return Pointer to the output disparity map.
	 */
	Ipp8u*  get_disp(){return disp_ret;}

	/** Access to the output depth map.
	 * @return Pointer to the output depth map.
	 */
	Ipp32f* get_depth(){return depth_ret;}

	/** Access to the output depthflow map.
	 * @return Pointer to the output depthflow map.
	 */
	Ipp32f* get_depthflow(){return depthflow_ret;}

	/** Memory width return function.
	 * @return Step in bytes through the 8u output image.
	 */
	int     get_psb(){return psb;}
		
	/** Memory width return function.
	 * @return Step in bytes through the 32f output image.
	 */
	int     get_psb_32f(){return psb_f;}

	/** Access to horizontal position of output map.
	 * @return Horizontal position of output map.
	 */
	int     get_px(){return px;}

	/** Access to vertical position of output map.
	 * @return Vertical position of output map.
	 */
	int     get_py(){return py;}

	/** Access to left-right image offset in mosaic.
	 * @return Left-right image offset in mosaic.
	 */
	int     get_hd(){return hd;}

	/** Access to minimum measurable depth for current head geometry.
	 * @return Minimum measurable depth for current head geometry.
	 */
	double  get_mind(){return mind;}
	
	/** Access to maximum measurable depth for current head geometry.
	 * @return Maximum measurable depth for current head geometry.
	 */
	double  get_maxd(){return maxd;}
	
	
	
      private:
	void disp2depth(Ipp8u*disp_,int psb_,Ipp32f*depth_,int psbf_,IppiSize s_);
	
	Ipp8u  *im_l_tmp,*im_r_tmp,*disp_ret;
	Ipp32f *depth_ret,*depth_old,*depth_new;
	Ipp32f *depthflow,*depthflow_ret;
	
	Depth *dmap;
	
	double baseline,mind,maxd;
	int d_range,d_offset,focus;
	int width,height;
	int old_px,old_py,dx,dy;
	int hd,px,py;
	
	IppiSize dsize;
	IppiSize fullsize;
	IppiSize tmpsize;
	
	int psb,psb_f;
	
      };

    }
  }
}
#endif
