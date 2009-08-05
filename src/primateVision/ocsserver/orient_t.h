/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef OCSST_H
#define OCSST_H

#include <ipp.h>
#include <qthread.h>
#include "lgphase.h"

using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {

      /** 
       * A thread wrapper class placed around the utils/LGPhase class.
       */
      class OrientT : public QThread
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
	OrientT(
		IppiSize imsize, 
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
		int g
		);
	/** Destructor.
	 */
	~OrientT();
	/** Main event loop.
	 */
	void run();
	
	//orient returns:
	/** Access to 32f combined magnitude of orientation energy over all scales.
	 * @return Returns pointer to 32f combined magnitude of orientation energy over all scales.
	 */
	Ipp32f* get_sumAn_32f(){return orient->get_sumAn_32f();}

	/** Access to 32f combined real orientation energy components over all scales.
	 * @return Returns pointer to 32f combined real orientation energy components over all scales.
	 */
	Ipp32f* get_sumE_32f(){return orient->get_sumE_32f();}

	/** Access to 32f combined imaginary orientation energy components over all scales.
	 * @return Returns pointer to 32f combined imaginary orientation energy components over all scales.
	 */
	Ipp32f* get_sumO_32f(){return orient->get_sumO_32f();}

	/** Access to 32f image with maximal magnitude orientation energy.
	 * @return Returns pointer to 32f image with maximal magnitude orientation energy.
	 */	
	Ipp32f* get_maxAn_32f(){return orient->get_maxAn_32f();}

	/** Access to 32f magnitude of orientation energy image for a particular scale.
	 * @return Returns pointer to 32f combined real orientation energy components over all scales.
	 * @param s scale.
	 */
	Ipp32f* get_A_EO_32f(int scale){return orient->get_A_EO_32f(scale);}

	/** Access to 32f real component of orientation energy image for a particular scale.
	 *  @return Returns pointer to 32f real component of orientation energy image for a particular scale.
	 * @param s scale.
	 */
	Ipp32f* get_R_EO_32f(int scale){return orient->get_R_EO_32f(scale);}

	/** Access to 32f imaginary component of orientation energy image for a particular scale.
	 *  @return Returns pointer to 32f imaginary component of orientation energy image for a particular scale.
	 * @param s scale.
	 */
	Ipp32f* get_I_EO_32f(int scale){return orient->get_I_EO_32f(scale);}

	/** Access to noise threshold.
	 * @return Returns noise threshold.
	 */
	double  get_T(){return orient->get_T();}

	/** Access to step in bytes through 32f output.
	 * @return Returns step in bytes through 32f output.
	 */
	int get_psb_32f(){return orient->get_psb_32f();}


	/** Thread initiator
	 *  @param newim Pointer input image to be processed.
	 */
	void proc(Ipp32fc*newim);
	
      private:
	Ipp32fc*imagefft; /**< Fourier transform of input image. */
	int psb_32fc;     /**< step in bytes through 32fc complex internal images. */
	LGPhase*orient;    /**< Pointer to an instantiation of the LGPhase utility class. */
	bool done;       /** Processing completion flag. */
	
      };

    }
  }
}

#endif

