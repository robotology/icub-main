/**
 * @ingroup icub_primatevision_utils
 *
 * \defgroup icub_primatevision_utils_multiclass MultiClass
 *
 * 
 * A processing class that, given multiple (numClasses) class probability maps, and an image, outputs a multiclass segmentation map.
 *
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at \in src/primateVision/utils/multiclass/multiclass.h
 */

#ifndef __MULTCL_H__
#define __MULTCL_H__

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
       * A processing class that performs multi-class segmentation.
       */
      class MultiClass
      {
      public:

	/*
	 * A structure to accommodate input parameters.
	 */
	struct Parameters
	{
	  int iter_max;	
	  bool randomize_every_iteration;
	  
	  int smoothness_penalty;
	  int smoothness_penalty_base;
	  int smoothness_3sigmaon2;
	  int data_penalty;
	};
	
	/** Constructor.
	 * @param imsize Input image width and height for memory allocation.
	 * @param psb_in Step in bytes through input images.
	 * @param numClasses Number of classes in output, and number of class probability maps provided.
	 * @param params Input parameters.
	 */
	MultiClass(IppiSize imsize,int psb_in,int numClasses,Parameters *params);
	
	/** Destructor.
	 */
	~MultiClass();
	
	/** Access to the classification output.
	 * @return Pointer to the output classification image.
	 */
	Ipp8u* get_class(){return out;};

	/** Memory width return function.
	 * @return Step in bytes through the output image.
	 */
	int get_psb(){return psb;};

	/** Processing initiator.
	 * @param im_in Pointer to input image to be used for edge smoothness.
	 * @param pMaps Reference to the array of pointers to the input class probability maps.
	 */
	void proc(Ipp8u* im_in, Ipp8u** pMaps);
	
      private:
	int likelihood(Coord c, int d);
	int prior_intensity_smoothness(Coord p, Coord np, int d, int nd);
	void generate_permutation(int *buf, int n);
	int compute_energy();
	void clear();
	void expand(int a);/* computes the minimum a-expansion configuration */
	
	int nmaps;
	int len_nv;
	int psb_in,psb;
	IppiSize im_size;
	Coord im_sz;
	Ipp8u *im, **prob; 
	Ipp8u *out;  
	Parameters *params;
	int E;
	void **ptr_im;
	
      };

    }
  }
}
#endif 
      
