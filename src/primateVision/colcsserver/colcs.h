/** 
 *  \ingroup icub_primatevision_colcsserver
 */

/*
 * Copyright (C) 2007 Andrew Dankers
 * 
 */


#ifndef COLCS_H
#define COLCS_H

#include <qthread.h>
#include <ipp.h>
#include <string>

#include "colcsio.h"

using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {

      
      /** 
       * A processing class that receives a two chrominance channel input images over ports, performs Difference-of-Gaussian pyramid centre-surround processing, and sends the combined results to clients over an output port.
       */
      

      class ColCSServer : public QThread
      {
	
      public:
	
	/** Constructor.
	 * @param c A string referencing the config file.
	 */
	ColCSServer(string*c_);

	/** Destructor.
	 */
	~ColCSServer();

	/** Main event loop.
	 */
	void run();
	
      private:
	string *cfg; /**< A string referencing the config file. */
	
      };
 
      /**
       * A class that responds to client ColCSServer probes, 
       * such that the clients can obtain server parameters 
       * essential to their operation.
       */
      class ColCSReplyParamProbe : public PortReader {
	
	/** Port reader for server parameter probes.
	 *  @param connection Pointer to Yarp2 ConnectionReader.
	 *  @return Returns true upon completion.
	 */
	virtual bool read(ConnectionReader& connection) {
	  Bottle foo;
	  BinPortable<ColCSServerParams> outBin;
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
	ColCSServerParams reply;
	
      };
      

    }
  }
}

#endif
 


