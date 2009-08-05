/** 
 *  \ingroup icub_primatevision_nzdfserver
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef __NZDOPT_H__
#define __NZDOPT_H__

#include <ipp.h>
#include <stdlib.h>
#include <coord.h>


using namespace std; 

namespace iCub {
  namespace contrib {
    namespace primateVision {
      
     /** 
       * The neighbourhood structure.
       */
      const struct Coord NEIGHBORS[] = { Coord(1, 0), Coord(0, -1) };
#define NEIGHBOR_NUM (sizeof(NEIGHBORS) / sizeof(Coord))

      /** 
       * A modification of the utils/multiclass class specifically tailored for near-zero disparity filtering.
       */
      class NZDFOpt
      {
      public:
	
	struct Parameters
	{
	  int iter_max;
	  bool randomize_every_iteration;
	  int intensity_smoothness_3sigmaon2;
	  int disparity_penalty;
	  int zd_penalty;
	  int smoothness_penalty;
	  int bland_dog_thresh;
	};
	
	/** Constructor.
	 */
	NZDFOpt(IppiSize im_size,int psb,Parameters *pars_);
	/** Destructor.
	 */
	~NZDFOpt();
	/** Returns pointer to output segmentation.
	 * @return out Binary segmentation output image.
	 */
	Ipp8u* get_nzdf(){return out;};

	/** Returns step in bytes through output image.
	 * @return Step in bytes through output image.
	 */
	int get_psb(){return psb;};
	
	/** Main processing sequence.
	 * @param iml Left input image.
	 * @param imr Right input image.
	 * @param imd Input depth map.
	 */
	void proc(Ipp8u*iml,Ipp8u*imr,Ipp8u*imd,Ipp8u*dog_l,Ipp8u*dog_r);
	
      private:
	int likelihood(Coord c, int d);
	int prior_smoothness(Coord p, Coord np, int d, int nd);
	int compute_energy();
	void clear();
	void expand(int a);/* computes the minimum a-expansion configuration */
	void* create_list(Coord c,Ipp8u *im, int w, int*list);
	bool is_tex(int*list);
	double cmp_rank(int*l1, int*l2);
	void generate_permutation(int *buf, int n);
	
	int nmaps;
	int len_nv;
	int psb_in,psb;
	IppiSize im_size;
	Coord im_sz;
	Ipp8u *im_l,*im_r,*im_d,*dog_l,*dog_r; 
	Ipp8u *out;  
	Parameters *params;
	int E;
	void **ptr_im;
	
      };

    }
  }
}
#endif 
