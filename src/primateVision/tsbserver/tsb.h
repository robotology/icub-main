/** 
 *  \ingroup icub_primatevision_tsbserver
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 */

#ifndef TSBSERVER_H 
#define TSBSERVER_H

#include <qthread.h>
#include <ipp.h>
#include <string.h>
//MY INCLUDES
#include "tsbio.h"

using namespace std;


namespace iCub {
  namespace contrib {
    namespace primateVision {

      /** 
       * A processing class that constructs an image-frame task-depentent spatial bias based on a radial mosaic-frame task-dependent spatial bias and the current gaze status, and sends the results to clients over an output port.
       */
      class TSBServer : public QThread
      {
	
      public:
	/** Constructor.
	 * @param c A string referencing the config file.
	 */
	TSBServer(string*c_);

	/** Destructor.
	 */
	~TSBServer();

	/** Main event loop.
	 */
	void run();
	
      private:
	string *cfg; /**< A string referencing the config file. */
	
      };


      /**
       * A class that responds to client TSBServer probes, 
       * such that the clients can obtain server parameters 
       * essential to their operation.
       */
      class TSBReplyParamProbe : public PortReader {
	
	/** Port reader for server parameter probes.
	 *  @param connection Pointer to Yarp2 ConnectionReader.
	 *  @return Returns true upon completion.
	 */
	virtual bool read(ConnectionReader& connection) {
	  Bottle foo;
	  BinPortable<TSBServerParams> outBin;
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
	TSBServerParams reply;
      
      };

    }
  }
}
#endif
 
