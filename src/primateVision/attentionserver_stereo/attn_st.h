/** 
 *  \ingroup icub_primatevision_attnserver_stereo
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef ATTNSERVER_ST_H 
#define ATTNSERVER_ST_H

#include <qthread.h>
#include <ipp.h>
#include <string.h>
//MY INCLUDES
#include "attnio_st.h"


using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {
      
      /** 
       * A processing class that receives saliency, inhibition-of-retuen, and task-dependent spatial bias data for both left and right cameras over ports, constructs an attentional fixation map, moderates the map to provide suitable attentional peaks, cross-checks stereo target, and sends the results to clients over an output port.
       */
      class AttnServer_Stereo : public QThread
      {
	
      public:
	
	/** Constructor.
	 * @param c A string referencing the config file.
	 */
	AttnServer_Stereo(string*c_);
	
	/** Destructor.
	 */
	~AttnServer_Stereo();

	/** Main event loop.
	 */
	void run();
	
      private:
	string *cfg; /**< A string referencing the config file. */

	/** Stereo cross-checking function.
	 * @param ar Right attention map.
	 * @param al Left attention map.
	 * @param x_r Position of checked location in primary image x.
	 * @param y_r Position of checked location in primary image y.
	 * @param psb Step in bytes through the image.
	 * @param yoff Vertical offset between left and right images.
	 * @return Cross-check success.
	 */
	bool crosscheck(Ipp8u*ar,int x_r,int y_r,Ipp8u*al,int*x_l,int yoff,int psb);
	
	//X-CHECK STUFF:
	Ipp32f*xres;
	IppiSize xisize,xtsize,srcsize;
	int psbxres;
	Ipp32f xtm_sim;
	
      };


      /**
       * A class that responds to client AttnServer_Stereo probes, 
       * such that the clients can obtain server parameters 
       * essential to their operation.
       */
      class AttnStReplyParamProbe : public PortReader {
	
	/** Port reader for server parameter probes.
	 *  @param connection Pointer to Yarp2 ConnectionReader.
	 *  @return Returns true upon completion.
	 */
	virtual bool read(ConnectionReader& connection) {
	  Bottle foo;
	  BinPortable<AttnStServerParams> outBin;
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
	AttnStServerParams reply;
      };

    }
  }
}
#endif
