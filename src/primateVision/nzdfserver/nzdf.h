/** 
 *  \ingroup icub_primatevision_nzdfserver
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef NZDFSERVER_H
#define NZDFSERVER_H

#include <qthread.h>
#include <ipp.h>
#include <string>
//MY INCLUDES
#include "nzdfopt.h"
#include "nzdfio.h"

using namespace std; 

namespace iCub {
  namespace contrib {
    namespace primateVision {

      /** 
       * A processing class that receives stereo input images over ports, performs spatial (stereo) near-zero disparity filtering, and sends the results to clients over an output port.
       */
      class NZDFServer : public QThread
      {
	
      public:
	/** Constructor.
	 * @param c A string referencing the config file.
	 */
	NZDFServer(string*c_);

	/** Destructor.
	 */
	~NZDFServer();

	/** Main event loop.
	 */
	void run();
	
      private:
	string *cfg;/**< A string referencing the config file. */
		
	/** A method to obtain the area, centre-of-gravity, and a measure of spread of a binary image.
	 * @param im Pointer to image data.
	 * @param p Step in bytes through input image.
	 * @param s Input image width, height.
	 * @param parea A pointer to the destination memory space to write the area.
	 * @param pdx A pointer to the destination memory space to write the cog x.
	 * @param pdy A pointer to the destination memory space to write the cog y.
	 * @param pspread A pointer to the destination memory space to write the spread.
	 */
	void getAreaCoGSpread(Ipp8u*im_, int p_,IppiSize s_, int*parea,int*pdx,int*pdy,int*pspread);
      };
      
      
      /**
       * A class that responds to client NZDFServer probes, 
       * such that the clients can obtain server parameters 
       * essential to their operation.
       */
      class NZDFReplyParamProbe : public PortReader {

	/** Port reader for server parameter probes.
	 *  @param connection Pointer to Yarp2 ConnectionReader.
	 *  @return Returns true upon completion.
	 */
	virtual bool read(ConnectionReader& connection) {
	  BinPortable<NZDFServerParams> outBin;
	  outBin.content() = reply;
	  ConnectionWriter *returnToSender = connection.getWriter();
	  if (returnToSender!=NULL) {
	      outBin.write(*returnToSender);
	  }
	}

      public:

	/** Reply sent to the probing client. Contains server
	 *  parameters that the client uses for initialisation. */
	NZDFServerParams reply;
      };
      
    }
  }
}
#endif
 
