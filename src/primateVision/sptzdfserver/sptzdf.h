/** 
 *  \ingroup icub_primatevision_sptzdfserver
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef SPTZDFSERVER_H
#define SPTZDFSERVER_H

#include <qthread.h>
#include <ipp.h>
#include <string>
//MY INCLUDES
#include <multiclass.h>
#include "sptzdfio.h"

using namespace std; 

namespace iCub {
  namespace contrib {
    namespace primateVision {

      /** 
       * A processing class that receives stereo input images over ports, performs spatio-temporal zero disparity filtering, and sends the results to clients over an output port.
       */
      class SpTZDFServer : public QThread
      {
	
      public:
	/** Constructor.
	 * @param c A string referencing the config file.
	 */
	SpTZDFServer(string*c_);
	
	/** Destructor.
	 */
	~SpTZDFServer();
	
	/** Main event loop.
	 */
	void run();
	
      private:
	string *cfg; /**< A string referencing the config file. */


	/** A method to populate a list encoding the rank transform.
	 * @param c Image coordinates.
	 * @param im Pointer to image data.
	 * @param w Step in bytes through input image.
	 * @param list Pointer to list to be populated with Rank transform.
	 */
	void get_rank(Coord c,Ipp8u *im, int w, int*list);

	/** A method to compare two rank lists.
	 * @param l1 Rank transform list 1.
	 * @param l2 Rank transform list 2.
	 * @return Probablistic similarity of l1 and l2.
	 */
	double cmp_rank(int*l1, int*l2);
	


	/** A method to populate a list encoding the NDT transform.
	 * @param c Image coordinates.
	 * @param im Pointer to image data.
	 * @param w Step in bytes through input image.
	 * @param list Pointer to list to be populated with NDT transform.
	 */
	void get_ndt(Coord c,Ipp8u *im, int w, int*list);

	/** A method to compare two NDT lists.
	 * @param l1 NDT transform list 1.
	 * @param l2 NDT transform list 2.
	 * @return Probablistic similarity of l1 and l2.
	 */
	double cmp_ndt(int*l1, int*l2);


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
       * A class that responds to client SPTZDFServer probes, 
       * such that the clients can obtain server parameters 
       * essential to their operation.
       */
      class SpTZDFReplyParamProbe : public PortReader {
	
	/** Port reader for server parameter probes.
	 *  @param connection Pointer to Yarp2 ConnectionReader.
	 *  @return Returns true upon completion.
	 */
	virtual bool read(ConnectionReader& connection) {
	  BinPortable<SpTZDFServerParams> outBin;
	  outBin.content() = reply;
	  ConnectionWriter *returnToSender = connection.getWriter();
	  if (returnToSender!=NULL) {
	    outBin.write(*returnToSender);
	  }
	}

      public:

	/** Reply sent to the probing client. Contains server
	 *  parameters that the client uses for initialisation. */
	SpTZDFServerParams reply;
      };
      

    }
  }
}
#endif
 
