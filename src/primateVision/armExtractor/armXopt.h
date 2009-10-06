
/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef __ARMXOPT_H__
#define __ARMXOPT_H__

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
      class ArmXOpt
      {
      public:
	
	struct Parameters
	{
	  int iter_max;
	  bool randomize_every_iteration;
	  int int_smoothness_3sigmaon2;
	  int bland_dog_thresh;
	  
	  int neighbour_smoothness_penalty;
	  int int_smoothness_penalty;
	  int disp_smoothness_penalty;
	  int dmap_bg_smoothness_penalty;

	  int nzd_dmap_data_penalty;
	  int nzd_dmap_bg_data_penalty; 
	  int nzd_dmap_fg_data_penalty;
	  int nzd_fb_data_penalty;

	  int nnzd_zd_data_penalty;
	  int nnzd_dmap_data_penalty;
	  int nnzd_dmap_bg_data_penalty; 
	  int nnzd_dmap_fg_data_penalty;
	  int nnzd_fb_data_penalty;

	  int sim_data_penalty;
	  int rad_data_penalty;
	};
	
	/** Constructor.
	 */
	ArmXOpt(IppiSize im_size,int psb,Parameters *pars_);
	/** Destructor.
	 */
	~ArmXOpt();
	/** Returns pointer to output segmentation.
	 * @return out Binary segmentation output image.
	 */
	Ipp8u* get_out(){return out;};

	/** Returns step in bytes through output image.
	 * @return Step in bytes through output image.
	 */
	int get_psb(){return psb;};
	
	/** Main processing sequence.
	 * @param iml Left input image.
	 * @param imr Right input image.
	 * @param imd Input depth map.
	 */
	void proc(Ipp8u*iml,Ipp8u*imr,Ipp8u*imd,Ipp8u*dog_l,Ipp8u*dog_r,Ipp8u*p_sim,Ipp8u*p_feedback);
	
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
	Ipp8u *im_l,*im_r,*im_d,*dog_l,*dog_r,*p_sim,*p_feedback; 
	Ipp8u *out;  
	Parameters *params;
	int E;
	void **ptr_im;
	
      };

    }
  }
}
#endif 
