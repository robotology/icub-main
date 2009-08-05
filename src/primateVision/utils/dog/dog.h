/**
 * @ingroup icub_primatevision_utils
 *
 * \defgroup icub_primatevision_utils_dog DoG
 *
 * 
 * A class that processes an input image and produces complex (on and off-centre) difference-of-Gaussian output maps. The magnitude difference-of-Gaussian maps can be used in a DoG pyramid (such as that in CentSur), or for texture detection. 
 *
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at \in src/primateVision/utils/dog/dog.h
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#ifndef __DOG_H
#define __DOG_H

#include <ipp.h>

#define PAD_BORD 8


using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {


     /** 
       * A processing class that constructs on- and off-centre difference-of-Gaussian maps.
       */
      class DoG
      {
	
      public:

	/** Constructor.
	 * @param imsize Input image width and height for memory allocation.
	 */
	DoG(IppiSize imsize);

	/** Destructor.
	 */
	~DoG();

	/** Processing initiator.
	 * @param im Pointer to input image.
	 * @param psb_8u Step in bytes through the input image.
	 */
	void proc(Ipp8u* im, int psb_8u);

	/** Access to the on-centre output.
	 * @return Pointer to the on-centre output image.
	 */
	Ipp8u* get_dog_on(){return out_dog_on;}   //on-centre
	
	/** Access to the off-centre output.
	 * @return Pointer to the off-centre output image.
	 */
	Ipp8u* get_dog_off(){return out_dog_off;} //off-centre

	/** Access to the magnitude output.
	 * @return Pointer to the on/off-centre output image.
	 */
	Ipp8u* get_dog_onoff(){return out_dog_onoff;} //absolute difference
	
	/** Memory width return function.
	 * @return Step in bytes through the output image.
	 */
	int get_psb(){return psb_o;}
	
      private:
	Ipp32f *dog;
	Ipp32f *dog_on;
	Ipp32f *dog_off;
	Ipp32f *dog_onoff;
	Ipp32f *tmp1;
	Ipp32f *tmp2;
	Ipp32f *tmp3;
	Ipp32f *in_pad;
	Ipp8u  *in_pad_8u;
	
	Ipp8u *out_dog_on;
	Ipp8u *out_dog_off;
	Ipp8u *out_dog_onoff;
	
	int width,height;
	int psb_o,psb_pad,psb_pad_8u;
	IppiSize srcsize,psize;
	
      };

    }
  }
}
#endif
