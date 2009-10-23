/**
 * \ingoup icub_primatevision_demos_objman
 */

 /*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#ifndef REC_KAL_H
#define REC_KAL_H

#include <stdio.h>
#include <iCub/kalman.h>
#include <yarp/math/Math.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BinPortable.h>


using namespace ctrl;
using namespace yarp;
using namespace yarp::math;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;



namespace iCub {
  namespace contrib {
    namespace primateVision {

      /** 
       * A Kalman filter wrapper.
       *
       */
      class Kal : public RateThread
      {
	
      public:
	/** Constructor.
	 * @param period The command cycle period.
	 * @param X_ Initial x.
	 * @param Y_ Initial y.
	 * @param Z_ Initial z.
	 * @param proc_noise_cov Process noise covariance estimate.
	 * @param meas_noise_cov Measurement noise covariance estimate.
	 */
	Kal(int period,double X_,double Y_,double Z_,double proc_noise_cov, double meas_noise_cov);

	/** Update after measurement.
	 * @param x_ Measurement x.
	 * @param y_ Measurement y.
	 * @param z_ Measurement z.
	 * @return Returns Kalman filtered Vector of 3D position.
	 */
	Vector update(double x_,double y_,double z_);
	
	/** Processing events occurring once every period.
	 */
	virtual void run();
	/** RateThread initialiser.
	 */
	virtual bool threadInit(){return true;}
	//virtual void afterStart(bool s);
	
	/** Clean-up.
	 */
	virtual void threadRelease(){delete kalPos;}
	
	
      private:
	Matrix kalA,kalH,kalQ,kalR;
	Matrix kalP0;
	Vector kalx0;
	Kalman* kalPos;
	Vector estX;
	bool done;
	
      };
      
    }
  }
}
#endif
