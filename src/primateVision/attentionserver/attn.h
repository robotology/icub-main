/** 
 *  \ingroup icub_primatevision_attnserver
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef ATTNSERVER_H 
#define ATTNSERVER_H

#include <qthread.h>
#include <ipp.h>
#include <string.h>
//MY INCLUDES
#include "attnio.h"


using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {


      /** 
       * A processing class that receives saliency, inhibition-of-retuen, and task-dependent spatial bias data over ports, constructs an attentional fixation map, moderates the map to provide suitable attentional peaks,  and sends the results to clients over an output port.
       */
      class AttnServer : public QThread
      {
	
      public:
		
	/** Constructor.
	 * @param c A string referencing the config file.
	 */
	AttnServer(string*c_);

	/** Destructor.
	 */
	~AttnServer();

	/** Main event loop.
	 */
	void run();
	
      private:
	string *cfg; /**< A string referencing the config file. */
	
      };



      /**
       * A class that responds to client AttnServer probes, 
       * such that the clients can obtain server parameters 
       * essential to their operation.
       */
      class AttnReplyParamProbe : public PortReader {
	
	/** Port reader for server parameter probes.
	 *  @param connection Pointer to Yarp2 ConnectionReader.
	 *  @return Returns true upon completion.
	 */
	virtual bool read(ConnectionReader& connection) {
	  Bottle foo;
	  BinPortable<AttnServerParams> outBin;
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
	AttnServerParams reply;
      };

    }
  }
}
#endif
 
