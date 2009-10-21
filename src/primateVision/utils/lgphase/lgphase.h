/**
 * @ingroup icub_primatevision_utils
 *
 * \defgroup icub_primatevision_utils_lgphase LGPhase
 *
 * 
 * A class that processes an input image with banks of spatially tuned complex log-Gabor kernels for a set of specified orientations. Orientation energy, phase congruency, and other phase-based statistics are obtained.
 *
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at \in src/primateVision/utils/lgphase/lgphase.h
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 * Implementation inspired by work of Peter Kovesi.
 *
 */


#ifndef LGPHASE_H
#define LGPHASE_H

#include <ipp.h>

using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {

     /** 
       * A processing class that performs phase analysis.
       */
      class LGPhase
      {
	
      public:
	
	/** Constructor.
	 * @param imsize Image width and height.
	 * @param psb_in  Step in bytes through 8u input image.
	 * @param nscales Number of scales processed for this orientation.
	 * @param norient Total number of log-Gabor orientations to be processed.
	 * @param thisorient Which of the 'norient' orientations this thread is processing
	 * @param minWaveLength Minimum wavelength.
	 * @param mult mult.
	 * @param sigmaOnf sigmaOnf.
	 * @param dThetaOnSigma dThetaOnSigma.
	 * @param k k.
	 * @param cutOff cutOff.
	 * @param g g.
	 */
	LGPhase(IppiSize imsize, 
		int psb_in,
		int nscales, 
		int norients, 
		int thisorient,
		int minWaveLength, 
		double mult, 
		double sigmaOnf, 
		double dThetaOnSigma,
		double k,
		double cutOff,
		int g);
	
	/** Destructor.
	 */
	~LGPhase();
	
	/** Processing initiator, accepting 8u images.
	 * @param im_in Pointer to complex input image.
	 */
	void proc(Ipp32fc *im_in);
	
	/** Access to 32f combined magnitude of orientation energy over all scales.
	 * @return Returns pointer to 32f combined magnitude of orientation energy over all scales.
	 */
	Ipp32f* get_sumAn_32f(){return sumAn;}

	/** Access to 32f combined real orientation energy components over all scales.
	 * @return Returns pointer to 32f combined real orientation energy components over all scales.
	 */
	Ipp32f* get_sumE_32f(){return sumE;}

	/** Access to 32f combined imaginary orientation energy components over all scales.
	 * @return Returns pointer to 32f combined imaginary orientation energy components over all scales.
	 */
	Ipp32f* get_sumO_32f(){return sumO;}

	/** Access to 32f image with maximal magnitude orientation energy.
	 * @return Returns pointer to 32f image with maximal magnitude orientation energy.
	 */	
	Ipp32f* get_maxAn_32f(){return maxAn;}

	/** Access to 32f magnitude of orientation energy over all scales.
	 * @return Returns pointer to 32f magnitude of orientation energy over all scales.
	 * @param s scale.
	 */
	Ipp32f* get_A_EO_32f(int scale){return A_E0[scale];}

	/** Access to 32f real component of orientation energy image for a particular scale.
	 *  @return Returns pointer to 32f real component of orientation energy image for a particular scale.
	 * @param s scale.
	 */
	Ipp32f* get_R_EO_32f(int scale){return R_E0[scale];}

	/** Access to 32f imaginary component of orientation energy image for a particular scale.
	 *  @return Returns pointer to 32f imaginary component of orientation energy image for a particular scale.
	 * @param s scale.
	 */
	Ipp32f* get_I_EO_32f(int scale){return I_E0[scale];}

	/** Access to noise threshold.
	 * @return Returns noise threshold.
	 */
	double  get_T(){return T;}

	/** Access to step in bytes through 32f output.
	 * @return Returns step in bytes through 32f output.
	 */
	int get_psb_32f(){return psb_32f;}
	

      private:
	void quadshift_32f(IppiSize qs, Ipp32f*im);
	void makelowpassfilter(IppiSize slfp, Ipp32f*im,double cutoff,int n);
	void conv_32f_to_8u(Ipp32f*im_i,int p4_,Ipp8u*im_o,int p1_,IppiSize ssize_);
	double getMedian(Ipp32f*im,int psb_,IppiSize isize);
	
	double 
	  T,
	  minWaveLength,
	  mult,
	  sigmaOnf,
	  dThetaOnSigma,
	  cutOff,
	  k,
	  EM_n;  
	
	int 
	  psb_in,
	  psb_32f,
	  psb_32fc,
	  nscale,
	  norient,
	  thisorient,
	  g;
	
	Ipp32fc 
	  **filterArray,
	  **logGabor,
	  **E0,
	  *spread,
	  *tmp_32fc;
	
	Ipp32f 
	  **R_E0,
	  **I_E0,
	  **A_E0,
	  **ifftFilterArray,
	  *A2_E0, 
	  *radius,
	  *theta,
	  *sintheta,
	  *costheta,
	  *dtheta,
	  *lp,
	  *sumE,
	  *sumO,
	  *sumAn,
	  *tmp1_32f,
	  *EstSumAn2, 
	  *EstSumAiAj,
	  *maxAn;
	
	Ipp8u 
	  *pbuf;
	
	IppiSize srcsize;
	IppiDFTSpec_C_32fc *pDFTSpec;
	
	
	double medianE2n;
	double meanE2n;
	double noisePower;
	double EstNoiseEnergy;
	double EstNoiseEnergy2;
	double EstNoiseEnergySigma;
	double sumEstSumAiAj;
	double sumEstSumAn2;
	double tau;
      };

    }
  }
}
#endif

