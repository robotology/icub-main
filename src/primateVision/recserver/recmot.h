/**
 * \ingoup icub_primatevision_recserver
 */

 /*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#ifndef REC_MOT_H
#define REC_MOT_H

#include <yarp/os/Property.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BinPortable.h>
#include "recio.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;




namespace iCub {
  namespace contrib {
    namespace primateVision {




      /** 
       * A class that handles recserver motion requests from clients obtained over
       * the motion reqest port, implements a basic VOR, and forwards motion commands to the iCub control axes. Implements 
       * a velocity controller based on a RateThread.
       */
      class RecHandleMotionRequest : public RateThread 
      {
	
      public:
	/** Constructor.
	 * @param period The command cycle period.
	 */
	RecHandleMotionRequest(int period);
	/** Processing events occurring once every period.
	 */
	virtual void run();
	/** RateThread initialiser.
	 */
	virtual bool threadInit(){return true;}
	//virtual void afterStart(bool s);
	
	/** Clean-up.
	 */
	virtual void threadRelease(){vel->stop();}
	
	
	double pix2degx; /**< Conversion factor, pixels to degrees for horizontal axis. */
	double pix2degy; /**< Conversion factor, pixels to degrees for vertical axis. */
	double k_vel_vers;    /**< Proportional velocity gain. */
	double k_vel_verg;    /**< Proportional velocity gain. */
	double k_vel_tilt;    /**< Proportional velocity gain. */
	double k_vel_roll;    /**< Proportional velocity gain. */
	double k_vel_pitch;    /**< Proportional velocity gain. */
	double k_vel_yaw;    /**< Proportional velocity gain. */
	bool   motion;   /**< Motion enable flag. */
	bool   fake;     /**< Fake/real Recserver flag. */
	double *enc;     /**< Pointer to encoder status. */
	double enc_tmp[6];/**< Encoder status cache. */
	IVelocityControl *vel; /**< Pointer velocity controller ports. */
	IEncoders *encs; /**< Pointer encoder status ports. */

	bool vor_on;
	double vor_k_rol;
	double vor_k_pan;
	double vor_k_tlt;
	double vor_d_rol;
	double vor_d_pan;
	double vor_d_tlt;

	BufferedPort<BinPortable<RecMotionRequest> > *inPort_mot; /**< Yarp port for receiving motion requests. */
	BufferedPort<yarp::sig::Vector> *inPort_inertial;
	
	
	
      private:
	double angles[6]; /**< Axis angles. */
	double vels[6];   /**< Axis velocities. */
	bool relative;    /**< Relative/absolute motion flag. */
	int suspend;      /**< Suspend period (in loop cycles). */
	int locked_to;    /**< Command acceptance locking. */
	BinPortable<RecMotionRequest> *rmq; /**< Motion request container received over port. */

	//VOR stuff:
	Vector *gyro;
	double vor_vels[3];
	double gyro_vel[3];

      };

    }
  }
}
#endif
