/** 
 *  \ingroup icub_primatevision_recserver
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#ifndef RECSERVER_H
#define RECSERVER_H

#include <qthread.h>
#include <qapplication.h>
#include <string.h>
#include "recmot.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace std; 

namespace iCub {
  namespace contrib {
    namespace primateVision {



      /** 
       * A processing class that 1) barrel rectifies images obtained from the iCub cameras,
       * and 2) uses head encoder status to determine epipolar rectifying paramerters
       * that are then 3) applied to the barrel rectified images to obtain epipolar
       * rectified images.
       */
      class RecServer : public QThread
      {
	
      public:
	
	/** Constructor.
	 * @param c A string referencing the config file.
	 */
	RecServer(string *c);
	
	/** Destructor.
	 */
	~RecServer();
	
	/** Main event loop.
	 */
	void run();
	
	/** Clean-up and exit.
	 */
	virtual void exit();
	
      private:
	string *cfg; /**< A string referencing the config file. */
	RecHandleMotionRequest *motion_handler; /**< Handler class for motion requests arriving on motion request port. */
	
      };
      
      
      
      
      
      
      /**
       * A class that responds to client RecServer probes, 
       * such that the clients can obtain server parameters 
       * essential to their operation.
       */
      class RecReplyParamProbe : public PortReader {
	
	/** Port reader for server parameter probes.
	 *  @param connection Pointer to Yarp2 ConnectionReader.
	 *  @return Returns true upon completion.
	 */
	virtual bool read(ConnectionReader& connection) {
	  Bottle foo;
	  BinPortable<RecServerParams> outBin;
	  outBin.content() = reply;
	  bool ok = foo.read(connection);
	  if (!ok){printf("read fail!\n"); return false;}
	  ConnectionWriter *returnToSender = connection.getWriter();
	  if (returnToSender!=NULL) {
	    outBin.write(*returnToSender);
	  }
	  return true;
	}
	
      public:
	
	/** Reply sent to the probing client. Contains server
	 *  parameters that the client uses for initialisation. */
	RecServerParams reply; 
	
      };
      
      
      
      
    }
  }
}




#endif
