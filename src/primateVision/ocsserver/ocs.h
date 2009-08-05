/** 
 *  \ingroup icub_primatevision_ocsserver
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 */

#ifndef OCSSERVER_H
#define OCSSERVER_H

#include <qthread.h>
#include <ipp.h>
#include <string.h>

#include "ocsio.h"

using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {

      /** 
       * A processing class that receives a single channel input image over a port, performs log-Gabor pyramid processing, and sends resultant maps to clients over an output port.
       */
      class OCSServer : public QThread
      {
	
      public:

	/** Constructor.
	 * @param c A string referencing the config file.
	 */
	OCSServer(string*c_);

	/** Destructor.
	 */
	~OCSServer();

	/** Main event loop.
	 */
	void run();
	
      private:
	string *cfg; /**< A string referencing the config file. */

      };



      /**
       * A class that responds to client OCSServer probes, 
       * such that the clients can obtain server parameters 
       * essential to their operation.
       */
      class OCSReplyParamProbe : public PortReader {

	/** Port reader for server parameter probes.
	 *  @param connection Pointer to Yarp2 ConnectionReader.
	 *  @return Returns true upon completion.
	 */
	virtual bool read(ConnectionReader& connection) {
	  Bottle foo;
	  BinPortable<OCSServerParams> outBin;
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
	OCSServerParams reply;
	
      };
      
	      
    }
  }
}
#endif
