/** 
 *  \ingroup icub_primatevision_zdfserver
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef ZDFSERVER_H
#define ZDFSERVER_H

#include <qthread.h>
#include <qapplication.h>
#include <ipp.h>
#include <string>
//MY INCLUDES
#include <multiclass.h>
#include "zdfio.h"

using namespace std; 

namespace iCub {
  namespace contrib {
    namespace primateVision {


      /** 
       * A processing class that receives stereo input images over ports, performs spatial (stereo) zero disparity filtering, and sends the results to clients over an output port.
       */
      class ZDFServer : public QThread
      {
	
      public:
	/** Constructor.
	 * @param c A string referencing the config file.
	 */
	ZDFServer(string*c_);

	/** Destructor.
	 */
	~ZDFServer();

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
	void getAreaCoGSpread(Ipp8u*im, int p,IppiSize s, int*parea,int*pdx,int*pdy,int*pspread);


      };



      /**
       * A class that responds to client ZDFServer probes, 
       * such that the clients can obtain server parameters 
       * essential to their operation.
       */
      class ZDFReplyParamProbe : public PortReader {

	/** Port reader for server parameter probes.
	 *  @param connection Pointer to Yarp2 ConnectionReader.
	 *  @return Returns true upon completion.
	 */
	virtual bool read(ConnectionReader& connection) {
	  BinPortable<ZDFServerParams> outBin;
	  outBin.content() = reply;
	  ConnectionWriter *returnToSender = connection.getWriter();
	  if (returnToSender!=NULL) {
      outBin.write(*returnToSender);
	  }
	}

      public:

	/** Reply sent to the probing client. Contains server
	 *  parameters that the client uses for initialisation. */
	ZDFServerParams reply;
      };

    }
  }
}
#endif
 
