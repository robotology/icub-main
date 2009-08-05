/** 
 *  \ingroup icub_primatevision_vorserver
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#ifndef VORSERVER_H
#define VORSERVER_H

#include <yarp/os/Property.h>
#include <qthread.h>
//#include <yarp/os/RateThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/os/BinPortable.h>
#include <yarp/sig/Vector.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

namespace iCub {
  namespace contrib {
    namespace primateVision {



      /** 
       * A basic VOR.
       */
      class VORServer : public QThread
      {
	
      public:
	/** Constructor.
	 * @param period The command cycle period.
	 */
	VORServer(string*cfg_);
	~VORServer();

	/** Processing events occurring once every period.
	 */
	void run();
	/** RateThread initialiser.
	 */
	//virtual bool threadInit(){return true;}
	//virtual void afterStart(bool s);
	
	/** Clean-up.
	 */
	void stop();
	
		
      private:
	string*cfg;
	IVelocityControl *vel_ctrl;
      };      
      
    }
  }
}




#endif
