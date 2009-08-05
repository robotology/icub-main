/** 
 *  \ingroup icub_primatevision_iorserver
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef IORSERVER_H 
#define IORSERVER_H

#include <qthread.h>
#include <qapplication.h>
#include <ipp.h>
#include <string.h>
//MY INCLUDES
#include "iorio.h"


using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {

      /** 
       * A processing class that receives flow data for a single camera over a port, calculates propagated inhibition-of-return, and sends the results to clients over an output port.
       */
      class IORServer : public QThread
      {
	
      public:
	
	/** Constructor.
	 * @param c A string referencing the config file.
	 */
	IORServer(string*c_);

	/** Destructor.
	 */
	~IORServer();

	/** Main event loop.
	 */
	void run();
	
      private:
	string *cfg; /**< A string referencing the config file. */
	
      };
 


      /**
       * A class that responds to client IORServer probes, 
       * such that the clients can obtain server parameters 
       * essential to their operation.
       */
      class IORReplyParamProbe : public PortReader {
		
	/** Port reader for server parameter probes.
	 *  @param connection Pointer to Yarp2 ConnectionReader.
	 *  @return Returns true upon completion.
	 */
	virtual bool read(ConnectionReader& connection) {
	  Bottle foo;
	  BinPortable<IORServerParams> outBin;
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
	IORServerParams reply;
      };
            
    }
  }
}
#endif
 
