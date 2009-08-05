/** 
 *  \ingroup icub_primatevision_depthflowserver
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef DEPTHFLOWSERVER_H
#define DEPTHFLOWSERVER_H

#include <qthread.h>
#include <string.h>
//MY INCLUDES
#include "depthflowsio.h"

using namespace std;


namespace iCub {
  namespace contrib {
    namespace primateVision {
      
      /** 
       * A processing class that receives stereo intensity channel input image over a port, performs depth and depthflow processing, and sends the results to clients over an output port.
       */
      class DepthflowServer : public QThread
      {
	
      public:
	/** Constructor.
	 * @param c A string referencing the config file.
	 */
	DepthflowServer(string*c_);

	/** Destructor.
	 */
	~DepthflowServer();

	/** Main event loop.
	 */
	void run();
	
      private:
	string *cfg; /**< A string referencing the config file. */
	
      };



      /**
       * A class that responds to client YCSServer probes, 
       * such that the clients can obtain server parameters 
       * essential to their operation.
       */
      class DepthflowReplyParamProbe : public PortReader {
	
	/** Port reader for server parameter probes.
	 *  @param connection Pointer to Yarp2 ConnectionReader.
	 *  @return Returns true upon completion.
	 */
	virtual bool read(ConnectionReader& connection) {
	  Bottle foo;
	  BinPortable<DepthflowServerParams> outBin;
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
	DepthflowServerParams reply;
      };

    }
  }
}
#endif
 
