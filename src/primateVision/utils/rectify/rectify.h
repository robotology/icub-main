/**
 * @ingroup icub_primatevision_utils
 *
 * \defgroup icub_primatevision_utils_rectify Rectify
 *
 * 
 * A class that calculates epipolar rectification parameters based upon head status, and barrel rectifies stereo input images according to configuration settings (obtained via MATLAB camera rectification) before applying the epipolar rectification transformation. 
 *
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at \in src/primateVision/utils/rectify/rectify.h
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 *
 */

#ifndef RECTIFY_H
#define RECTIFY_H

#ifdef __cplusplus
extern "C"{
#endif

#include <matrix2.h>
#include <oldnames.h>
#include <ipp.h>

#ifdef __cplusplus
}
#endif


using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {


     /** 
       * A processing class that performs barrel and epipolar rectification for a single camera.
       */
      class Rectify
      {
	
      public:
	
	/** Constructor.
	 * @param imsize Input image width and height for memory allocation.
	 * @param brecFile Name of Matlab intrinsic camera parameter file to load for barrel rectification parameters.
	 * @param tilt_offset Tilt Offset in degrees between left and right cameras; one is the reference and must be zero.
	 */
	Rectify(bool clear, char* brecFile,IppiSize imsize, double tilt_offset);

	/** Destructor.
	 */
	~Rectify();

	
	/** Processing initiator. First applies barrel rectification, then determines epipolar rectification parameters based on geometry, then applies epipolar rectifying transformation.
	 * @param ang_t Tilt angle of this camera.
	 * @param ang_v Verge angle of this camera.
	 */
	void proc(double ang_t, double ang_v, double ang_roll);
	
	/** Barrel rectifies an image.
	 * @param im Pointer to input image to be barrel rectified.
	 * @param psb_in Step in bytes through input image.
	 * @param brect_im Pointer to location to write barrel rectified image.
	 * @param psb_out Step in bytes through output image.
	 */
	void  barrel_rect(Ipp8u *im,int psb_in, Ipp8u *brect_im,int psb_out);
	
	/** Epipolar rectifies an image based on parameters calculated from geomatry.
	 * @param im Pointer to input image to be epipolar rectified.
	 * @param psb_in Step in bytes through input image.
	 * @param brect_im Pointer to location to write epipolar rectified image.
	 * @param psb_out Step in bytes through output image.
	 */
	void epipolar_rect(Ipp8u *im,int psb_in, Ipp8u *erect_im,int psb_out);

	/**  Focus return function.
	 * @return Camera focus length, in pixels.
	 */
	int get_focus(){return (int) ((fc1+fc2)/2.0);}

	/** Mosaic horizontal augmentation coordinate access.
	 * @return Horizontal mosaic position to augment rectified image.
	 */
	int get_ix(){return ix;}
		
	/** Mosaic vertical augmentation coordinate access.
	 * @return Vertical mosaic position to augment rectified image.
	 */
	int get_iy(){return iy;}
	
  
      private:
	//EPIPOLAR RECT SUPPORT FUNCS:
	void P2T (MAT *Po_, MAT *Pn_, MAT *H_,MAT *T_);
	void shift_origin(MAT *A_, int x, int y);
	void transf(MAT *A_, double a, double b,VEC*result);
	
	
	VEC *uO,*uA,*uB,*uC,*uD;
	MAT *Po,*T,*A,*An,*Rt,*Pn,*Rtn,*H;
	bool mod;
	double r,p,y,tx,ty,tz,rn,pn,yn_,txn,tyn,tzn;
	int ix,iy;
	int width,height;
	IppiSize srcsize;
	IppiRect srcroi;
	double Tc[3][3];
	double tilt_offset;
	Ipp32f *xMap, *yMap;
	Ipp32f fc1,fc2,cc1,cc2,kc1,kc2,pc1,pc2;
	Ipp8u *buffer;
	int xStep, yStep, buflen;
    bool clear;
	
      };

    }
  }
}
#endif
