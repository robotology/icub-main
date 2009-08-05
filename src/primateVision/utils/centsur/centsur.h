/**
 * @ingroup icub_primatevision_utils
 *
 * \defgroup icub_primatevision_utils_centsur CentSur
 *
 * 
 * An implementation modelling the centre-surround response in retinal ganglions, used for construction of spatial uniqueness maps. Based on the difference-of-Gaussians pyramid approach of Itti. A single Gaussian pyramid is created. Neighbouring pyramid entries are subtracted (eg Pyramid level 0 - Pyramid level 1, 1-2, 2-3 ...etc), and so are 2nd neighbours (eg 1-3,2-4,3-5..etc), to obtain spatial uniqueness at various spatial scales.  All resultant subtraction results (the difference of Gaussian maps) are summated to yield the centre-surround map output. 
 *
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at \in src/primateVision/utils/centsur/centsur.h
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef CENTSUR_H
#define CENTSUR_H

#include <ipp.h>

using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {


     /** 
       * A processing class that performs centre-surround uniqueness analysis.
       */
      class CentSur
      {
	
      public:

	/** Constructor.
	 * @param imsize Input image width and height for memory allocation.
	 * @param nscale Number of pyramid scales.
	 * @param sigma Gaussian sigma of base pyramid.
	 */
	CentSur(IppiSize imsize, int nscale, double sigma = 1.0);

	/** Destructor.
	 */
	~CentSur();
		
	/** Processing initiator, accepting 8u images.
	 * @param im_8u Pointer to 8u input image.
	 * @param psb_8u Step in bytes through the input image.
	 */
	void proc_im_8u(Ipp8u* im_8u, int psb_8u);
		
	/** Processing initiator, accepting 32f images.
	 * @param im_32f Pointer to 32f input image.
	 * @param psb_32f Step in bytes through the input image.
	 */
	void proc_im_32f(Ipp32f* im_32f, int psb_32f);
	
	/** Access to scale levels in the Gaussian Pyramid.
	 * @param s Scale, where base level is 0.
	 * @return Pointer to the requester Gaussian level.
	 */
	Ipp32f* get_gauss(int s){return gauss[s];}

	/** Access to scale levels in the difference-of-Gaussian Pyramid.
	 * @param s Scale, where base level is 0.
	 * @return Pointer to the requester difference-of-Gaussian level.
	 */
	Ipp32f* get_pyramid(int s){return pyramid[s];}

	/** Access to the 32f centre-surround output.
	 * @return Pointer to the 32f centre-surround output image.
	 */
	Ipp32f* get_centsur_32f(){return cs_tot_32f;} 
		
	/** Access to the 8u centre-surround output.
	 * @return Pointer to the normalised 8u centre-surround output image.
	 */
	Ipp8u*  get_centsur_norm8u(){return cs_tot_8u;}

	/** Memory width return function.
	 * @return Step in bytes through the 8u output image.
	 */
	int get_psb_8u(){return psb_8u;}
	
	/** Memory width return function.
	 * @return Step in bytes through the 32f output image.
	 */
	int get_psb_32f(){return psb_32f;}
	
      private:
	void make_pyramid(Ipp32f* im_in, int pin32_);
	
	Ipp32f **pyramid,**pyramid_gauss,**gauss,*cs_tot_32f,*tmp_im_32f,*im_in_32f;
	Ipp8u *cs_tot_8u,*pbuf;
	int *psb_p,pbufsize,psb_8u,psb_32f,ngauss;
	IppiSize srcsize,*psize;
	IppiRect *proi;
	double sd,su,sigma;
	
      };

    }
  }
}
#endif
