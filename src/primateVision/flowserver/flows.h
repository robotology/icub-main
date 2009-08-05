/** 
 *  \ingroup icub_primatevision_ycsserver
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 */

#ifndef FLOWSERVER_H
#define FLOWSERVER_H

#include <qthread.h>
#include <ipp.h>
#include <string.h>
//MY INCLUDES
#include "flowsio.h"


using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {

      /** 
       * A processing class that receives a intensity channel input image over a port, performs optical flow processing, and sends the results to clients over an output port.
       */
      class FlowServer : public QThread
      {
	
      public:

	/** Constructor.
	 * @param c A string referencing the config file.
	 */
	FlowServer(string*c_);
	
	/** Destructor.
	 */
	~FlowServer();

	/** Main event loop.
	 */
	void run();
	
      private:
	string *cfg; /**< A string referencing the config file. */
	
      };



      /**
       * A class that responds to client FlowServer probes, 
       * such that the clients can obtain server parameters 
       * essential to their operation.
       */
      class FlowReplyParamProbe : public PortReader {

	/** Port reader for server parameter probes.
	 *  @param connection Pointer to Yarp2 ConnectionReader.
	 *  @return Returns true upon completion.
	 */
	virtual bool read(ConnectionReader& connection) {
	  Bottle foo;
	  BinPortable<FlowServerParams> outBin;
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
	FlowServerParams reply;
	
      };
      
      
    }
  }
}
#endif
 
