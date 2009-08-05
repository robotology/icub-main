/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef CCSSST_H
#define CCSSST_H

#include <ipp.h>
#include <qthread.h>
#include "centsur.h"

using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {

      /** 
       * A thread wrapper class placed around the util/centsur class.
       */
      class CentsurT : public QThread
      {
	
      public:
	/** Constructor.
	 * @param srcsize Image width and height.
	 * @param ns Number of log-Gabor scales to be processed.
	 * @param ncsscale Number of centsur scales.
	 * @param sig Sigma for centsur processing.
	 */
	CentsurT(IppiSize srcsize,
		 int ns,
		 int ncsscale,
		 double sig);
	/** Destructor.
	 */
	~CentsurT();
	/** Main event loop.
	 */
	void run();
	
	//centsur returns:
	/** Returns pointer to 32f output image.
	 */
	Ipp32f* get_cs_tot_32f(){   return tot_32f;}
	/** Returns pointer to normalised 8u output image.
	 */
	Ipp8u* get_cs_tot_norm8u(){ return tot_8u;}
	/** Returns step in bytes through 32f output.
	 */
	int get_psb_32f(){          return centsur->get_psb_32f();}
	/** Returns step in bytes through 8u output.
	 */
	int get_psb_8u(){           return centsur->get_psb_8u();}
	/** Processing semaphore.
	 */
	bool is_done(){             return done;}
	
	//thread initiator:
	/** Thread initiator
	 *  @param newims Pointer to set of images to be processed.
	 *  @param p_i Number of images in the set to be processed.
	 */
	void proc_im_32f(Ipp32f**newims, int p_i);
	
      private:
	Ipp32f**images;  /**< Pointer to input images.*/
	CentSur*centsur; /**< Pointer to instantiation of centsur processing class.*/
	bool done;   /**< Processing completion flag.*/
	int psb_32f;  /**< Step in bytes through internal 32f images.*/
	int psb_8u;  /**< Step in bytes through internal 8u images.*/
	Ipp32f*tot_32f; /**< Pointer to summated centsur accross scales, 32f.*/
	Ipp8u*tot_8u; /**< Pointer to summated centsur accross scales, 8u.*/
	IppiSize srcsize; /**< Input/output width and height.*/
	int nscale; /**< Number of centsur spatial scales processed for each log-Gabor orientation.*/
	
      };

    }
  }
}
#endif

