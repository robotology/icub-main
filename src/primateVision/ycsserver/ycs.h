/** 
 *  \ingroup icub_primatevision_ycsserver
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 */


#ifndef YCSSERVER_H
#define YCSSERVER_H

#include <qthread.h>
#include <ipp.h>
#include <string>
//MY INCLUDES
#include "ycsio.h"

using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {


     /** 
       * A processing class that receives a single intensity channel input image over a port, performs Difference-of-Gaussian pyramid centre-surround processing, and sends the results to clients over an output port.
       */
      class YCSServer : public QThread
      {
	
      public:
	
	/** Constructor.
	 * @param c A string referencing the config file.
	 */
	YCSServer(string*c_);

	/** Destructor.
	 */
	~YCSServer();

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
      class YCSReplyParamProbe : public PortReader {
	
	/** Port reader for server parameter probes.
	 *  @param connection Pointer to Yarp2 ConnectionReader.
	 *  @return Returns true upon completion.
	 */
	virtual bool read(ConnectionReader& connection) {
	  Bottle foo;
	  BinPortable<YCSServerParams> outBin;
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
	YCSServerParams reply;
      };
          
    }
  }
}

#endif
 
